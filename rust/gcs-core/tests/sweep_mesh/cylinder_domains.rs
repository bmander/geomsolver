//! Explicit chart adapter for the Phase 2 fixture, NOT a general cylinder
//! detector or an oracle mesher. Domains come from the source cylinder's wall,
//! end disks, circular rim edges and rigid x rotation. Visibility events are
//! solved here before tessellation. The independent radial oracle is not used.
use gcs_core::solid::swept_boundary::shared::Patch;
use std::{collections::BTreeMap,f64::consts::FRAC_PI_2};

#[derive(Default)]
struct Graph { corners: BTreeMap<String,u32>, edges: BTreeMap<String,u32>, patches: Vec<Patch<'static>> }
impl Graph {
    fn add(&mut self,names: [String;4],source: u32,flip: bool,trim: Option<(usize,String)>,evaluate: impl Fn([f64;2]) -> [f64;3] + 'static) {
        let mut corners = names.map(|s| { let next = self.corners.len() as u32; *self.corners.entry(s).or_insert(next) });
        let mut edges = std::array::from_fn(|k| {
            let (a,b) = (corners[k],corners[(k+1)%4]); let next = self.edges.len() as u32;
            let key = trim.as_ref().filter(|(side,_)| *side == k).map(|(_,key)| key.clone())
                .unwrap_or_else(|| format!("ends/{}/{}",a.min(b),a.max(b)));
            *self.edges.entry(key).or_insert(next)
        });
        if flip { corners = [corners[1],corners[0],corners[3],corners[2]]; edges = [edges[0],edges[3],edges[2],edges[1]]; }
        self.patches.push(Patch {id:self.patches.len() as u32,source,corners,edges,minimum_levels:if source == 4 || source == 5 {[0,2]} else {[0,0]},
            evaluate:Box::new(move |[u,v]| evaluate([u,if flip {1.-v} else {v}]))});
    }
}
fn reflect([x,y,z]: [f64;3],q: usize) -> [f64;3] {
    [x,if q == 1 || q == 2 {-y} else {y},if q >= 2 {-z} else {z}]
}
fn turn([x,y,z]: [f64;3],theta: f64) -> [f64;3] {
    let (s,c) = theta.sin_cos(); [x,c*y-s*z,s*y+c*z]
}
fn corner(station: usize,q: usize,kind: usize) -> String {
    let name = match kind {
        0 if station == 0 || station == 3 => "axis".into(),
        0 => format!("y{}",usize::from(q == 1 || q == 2)),
        1 => format!("rim{q}"),
        // At both visibility events the top domain contracts to the z axis.
        _ => format!("z{}",usize::from(q >= 2)),
    };
    format!("{station}/{name}")
}

/// Fixture range excludes the change of arrangement at 0 and 45 degrees.
/// `warp` perturbs interior parameter sampling, leaving all edge parameters fixed.
pub(super) fn domains(h: f64,warp: f64) -> Vec<Patch<'static>> {
    assert!(h > 0. && h < std::f64::consts::FRAC_PI_4 && warp.abs() < 0.2);
    // Opposed rim sweeps first coincide at y=0 when a=tan(h).
    let event = h.tan().acos(); let stations = [-FRAC_PI_2,-event,event,FRAC_PI_2];
    let mut g = Graph::default();
    for strip in 0..3 { for q in 0..4 {
        let (lo,hi) = (stations[strip],stations[strip+1]);
        for branch in 0..if strip == 1 {3} else {2} {
            let names = [corner(strip,q,branch),corner(strip,q,branch+1),
                corner(strip+1,q,branch+1),corner(strip+1,q,branch)];
            // Explicit analytic support IDs, NOT the old sampled sheet numbers:
            // 0/1 endpoint cylinder wall, 2/3 endpoint disk, 4/5 circular rim,
            // 6/7 smooth-wall contact at the two x extrema.
            let source = match branch {
                0 => if q%2 == 0 {0} else {1},
                1 => if q < 2 {4} else {5},
                _ => if q%2 == 0 {2} else {3},
            };
            let trim = (strip == 1 && branch > 0).then(|| (if branch == 1 {1} else {3},format!("trim/{q}")));
            g.add(names,source,q%2 == 1,trim,move |[u,v]| {
                let u = u+warp*(std::f64::consts::PI*u).sin()*v*(1.-v);
                let alpha = lo+(hi-lo)*u; let x = 3.+alpha.sin();
                let a = if alpha.abs() == FRAC_PI_2 {0.} else {alpha.cos()};
                let p = match branch {
                    // Kept endpoint-wall region, cut at z=a tan(h) and z=1.
                    0 => turn([x,a,a*h.tan()+(1.-a*h.tan())*v],-h),
                    // Edge-generator orbit. When opposite rim sheets coincide,
                    // own only the domain up to y=0 (theta=atan(a)).
                    1 => turn([x,a,1.],-h+(h+a.atan().min(h))*v),
                    // Endpoint disk, visible only between a and tan(h).
                    _ => turn([x,a+(h.tan()-a)*v,1.],h),
                };
                reflect(p,q)
            });
        }
    } }
    // The extrema are swept wall generators, not arbitrary hole-filling fans.
    // Each chart has a declared collapsed radial edge at the axis.
    for station in [0,3] { for q in 0..4 {
        let names = [corner(station,q,0),corner(station,q,0),corner(station,q,2),corner(station,q,1)];
        g.add(names,if station == 0 {6} else {7},(station == 3) ^ (q%2 == 1),None,move |[r,v]| {
            reflect(turn([if station == 0 {2.} else {4.},0.,r],-h+h*v),q)
        });
    } }
    g.patches
}
