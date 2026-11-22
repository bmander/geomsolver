//! **An upright tool of revolution under a motion keeping its plane** (#880 T1f,
//! `brep::swept::upright`): brep-bench's bull nose carried without turning along a cubic that
//! crosses itself and turns tighter than the mill, cut from its block as the bench calls it. Its
//! volume is pinned to the reading made apart from the build — the area each height's region
//! encloses (`region::SweptRegion`, the disc of the mill's radius there), integrated over the
//! mill's corner by Gauss–Legendre on the torus's angle (`cad-kernel`'s `bull_probe`) — and held
//! within 1e-5 of cadrs's; the mesh's volume agrees with the exact one; and the solid's points
//! agree with the sweep's definition, read with no kernel at all: a point is swept where it stands
//! within the mill's radius at its height of the curve.
use crate::brep::mesh_volume;
use crate::common::bench::bull_nose::*;
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::cut_from;
use gcs_core::brep::swept::member::Solid;
use gcs_core::brep::topo::Brep;

/// The groove's volume read apart from the build (`bull_probe 32 16384`: 32 panels of four
/// Gauss–Legendre points over the corner, each region's loops as polylines of 16384 points a
/// piece, read to 1e-6).
const READ: f64 = 10956.986456893;
/// cadrs's (the bench's peer): its faces 10 B-spline, 2 cylinder, 7 plane, 2 torus and 11 other.
const CADRS: f64 = 10956.942404803389;

/// The bench's call, with the curve's poles `control`.
fn built(control: [[f64;3];4]) -> (Brep,f64) {
    let (stock,tool,path,roll,bar,tol) = case(control);
    let b = cut_from(&stock,&tool,&path,roll,bar,tol).unwrap_or_else(|e| panic!("the bull nose: {e:?}"));
    b.check(tol).unwrap_or_else(|e| panic!("the bull nose's solid: {e}"));
    (b,tol)
}

/// **The sweep's definition**: the curve's points at 1024 places, and the most it moves between
/// two.
struct Definition { curve: gcs_core::brep::nurbs::BSpline,at: Vec<[f64;2]>,step: f64 }

/// How many places the definition reads the curve at.
const PLACES: usize = 1024;

impl Definition {
    fn new(control: [[f64;3];4]) -> Definition {
        let curve = curve(control);
        let at: Vec<[f64;2]> = (0..=PLACES).map(|i| { let q = curve.point(i as f64/PLACES as f64); [q[0],q[1]] }).collect();
        let step = at.windows(2).map(|w| (w[1][0]-w[0][0]).hypot(w[1][1]-w[0][1])).fold(0.,f64::max);
        Definition {curve,at,step}
    }

    /// **Whether `p` is swept**: within the mill's height, and within its radius there of the
    /// curve — the curve's nearest point the nearest place, settled by a golden section about
    /// every place nearer than its neighbours that could stand nearer than the radius.
    fn swept(&self,p: [f64;3]) -> bool {
        let z = p[2];
        let [_,_,z0] = FOOT;
        if z < z0 || z > TOP { return false }
        let corner = z0+CORNER;
        let r = if z >= corner { RADIUS } else { RADIUS-CORNER+(CORNER*CORNER-(z-corner)*(z-corner)).max(0.).sqrt() };
        let d = |t: f64| { let q = self.curve.point(t); (q[0]-p[0]).hypot(q[1]-p[1]) };
        let ds: Vec<f64> = self.at.iter().map(|q| (q[0]-p[0]).hypot(q[1]-p[1])).collect();
        let best = ds.iter().copied().fold(f64::INFINITY,f64::min);
        if best <= r { return true }
        for i in 0..=PLACES {
            // (a least of the distance between two places lies beside a place nearer than both
            // its neighbours, or one of the ends)
            let least = (i == 0 || ds[i] <= ds[i-1]) && (i == PLACES || ds[i] <= ds[i+1]);
            if !least || ds[i]-self.step > r { continue }
            let (mut lo,mut hi) = ((i.max(1)-1) as f64/PLACES as f64,((i+1).min(PLACES)) as f64/PLACES as f64);
            for _ in 0..60 {
                let (a,b) = (lo+(hi-lo)/3.,hi-(hi-lo)/3.);
                if d(a) <= d(b) { hi = b } else { lo = a }
            }
            if d(0.5*(lo+hi)) <= r { return true }
        }
        false
    }
}

/// The bar the built solid is meshed within, its points read by their distance from it.
const BODY: f64 = 0.01;

/// The built solid against the definition at `n` seeded points of the box about the groove (the
/// mill's reach about the curve's poles' box, down to its foot): a point is the solid's where it is
/// the block's and not swept, one within twice its mesh's bar of the boundary left out; and the
/// groove's volume counted (swept points of the block), with its standard error.
fn agrees(b: &Brep,control: [[f64;3];4],n: usize) -> (f64,f64) {
    let body = Solid::new(b,BODY,1.).unwrap();
    let definition = Definition::new(control);
    let (lo,hi) = ([-12.5,-2.5,-1.5],[32.5,17.5,0.]);
    let mut rng = gcs_core::rng::Rng::new(0x6b0);
    let (mut inside,mut held) = (0,0);
    for _ in 0..n {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
        let cut = definition.swept(p);
        let stock = (0..3).all(|k| p[k] >= STOCK_LO[k] && p[k] <= STOCK_HI[k]);
        if cut && stock { inside += 1 }
        let d = body.signed_distance(p);
        if d.abs() < 2.*BODY { continue }
        assert_eq!(d < 0.,stock && !cut,"at {p:?}: the solid's distance {d}, the definition's sweep {cut}, in the block {stock}");
        held += 1;
    }
    assert!(held > 9*n/10,"{held} of {n} points held");
    let boxed: f64 = (0..3).map(|k| hi[k]-lo[k]).product();
    let f = inside as f64/n as f64;
    (boxed*f,boxed*(f*(1.-f)/n as f64).sqrt())
}

#[test]
fn the_bull_nose_along_a_looping_spline_is_its_bench_volume() {
    let (b,_) = built(CONTROL);
    let v = volume(&b);
    assert!((v-READ).abs() <= 2e-6,"{v} against the regions' reading {READ}");
    assert!((v-CADRS).abs() <= 1e-5*CADRS,"{v} against cadrs's {CADRS}");
    // the mesh's own volume, within a tenth of its sag over its area (a pcurve on a fold's wrong
    // leaf checks and reads its volume far off: both readings held)
    let sag = 1e-3;
    let mesh = gcs_core::brep::mesh::mesh(&b,sag,0.1).unwrap();
    let area: f64 = mesh.tris.iter().map(|t| {
        let [a,b,c] = t.map(|i| mesh.pts[i as usize]);
        0.5*gcs_core::space::norm(gcs_core::space::cross(gcs_core::space::sub(b,a),gcs_core::space::sub(c,a)))
    }).sum();
    let vm = mesh_volume(&mesh);
    assert!((vm-v).abs() <= 0.1*sag*area,"the mesh encloses {vm}, the solid {v} (sag {sag} over area {area})");
}

#[test]
fn the_bull_nose_along_a_looping_spline_agrees_with_its_definition() {
    let (b,_) = built(CONTROL);
    let (groove,err) = agrees(&b,CONTROL,20_000);
    let want = 11200.-volume(&b);
    eprintln!("the groove counted {groove} ± {err} against {want}");
    assert!((groove-want).abs() <= 4.*err,"the groove counted {groove} ± {err} against {want}");
}

#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, the bull nose built five times over")]
fn the_bull_nose_builds_with_its_poles_moved() {
    let v = volume(&built(CONTROL).0);
    for (pole,axis,by) in [(1,0,None),(2,0,None),(1,0,Some(1e-6)),(2,1,Some(1e-6)),(3,0,Some(-1e-6)),(0,1,Some(1e-6))] {
        let mut control = CONTROL;
        let x = control[pole][axis];
        control[pole][axis] = by.map_or(f64::from_bits(x.to_bits()+1),|d| x+d);
        let moved = volume(&built(control).0);
        // (the groove's walls are some 2 × 42 × 1.5 square units: a pole moved by `d` moves the
        // volume by no more than a few hundred times it; an ulp by nothing past the rounding)
        let room = by.map_or(1e-9*v,|d| 300.*d.abs());
        assert!((moved-v).abs() <= room,"pole {pole} moved by {by:?}: {moved} against {v}");
    }
}

#[test]
fn the_bull_nose_s_closed_sweep_agrees_with_membership() {
    let (_,tool,path,roll,bar,tol) = case(CONTROL);
    let sweep = gcs_core::brep::swept::upright::swept(&tool,&path,roll,bar,tol).unwrap().expect("the road reads the bull nose");
    crate::swept_closed::agrees_over(&sweep,&tool,&path,roll);
}

#[test]
fn k2_refuses_by_name_a_flat_end_its_translation_touches_whole() {
    // (K2's pencil reads one root a half-plane; a flat end square to the motion's every velocity
    // has `n · v` nought all over it — refused at the face before any fit, where the bull nose's
    // sheet was fitted to 1.9 and no closer)
    use gcs_core::brep::swept::closed::Unbuilt;
    use gcs_core::brep::swept::patches::{Revolved,fitted};
    let (_,mill,path,roll,_,tol) = case(CONTROL);
    let cylinder = gcs_core::brep::build::prism(&gcs_core::brep::build::disc(FOOT,[0.,0.,1.],RADIUS),0.,TOP-FOOT[2]).unwrap();
    for (name,tool) in [("the bull nose",mill),("a flat-ended cylinder",cylinder)] {
        let revolved = Revolved::read(&tool).unwrap_or_else(|e| panic!("{name}: {e:?}"));
        match fitted(&revolved,&path,roll,0.1*tol) {
            Err(Unbuilt::Curved {at}) => assert!((at[2]-FOOT[2]).abs() <= 1e-12 || (at[2]-TOP).abs() <= 1e-12,
                "{name}: refused at {at:?}, on neither flat end"),
            other => panic!("{name}: {:?}",other.map(|_| "a sheet fitted")),
        }
    }
}
