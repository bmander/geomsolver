//! **A solid of revolution carried square to its axis, swept closed** (#721 K1, a closed form where
//! one stands: #880 P2): a tool every face of which turns about one line — a bull-nose or a
//! ball-end mill, a drill, a disc — every slice of it square to that line a disc about it, moved
//! along a path of slides square to the line and turns about lines along it (a mill run along a
//! line, an arc and a line), the line kept upright, every piece taking up the path's direction where
//! the one before left it.
//!
//! Each point of the tool keeps its height along the line, and at each height the tool is a disc
//! of the radius `R(z)` its meridian has there, so at each height the sweep is the region within
//! `R(z)` of the axis's path: a band along the path and a disc at each end. The band is the tool's
//! axial section square to the path, carried along it: under a slide its prism, under a turn its
//! revolution about the turn's line (whose section in the half-plane through both lines is the
//! band's, `|ρ − d| ≤ R(z)`, where the tool is narrower than its distance `d` from that line);
//! beyond the path's ends, the tool's halves there, its meridian turned half a turn. The pieces meet
//! on their section planes, which bound nothing: they are sewn along them into one solid
//! (`sew::sewn`), nothing met in space, and a blank is cut by it once.
//!
//! Read before anything is built, and declined (none, for the other roads) where it does not hold:
//! a face of the tool no surface of revolution about one line, a slice no disc (a ring, a meridian
//! doubling back up the line), a piece no slide square to the line or turn about a line along it, a
//! turn about a line the tool reaches, a path whose direction breaks where two pieces meet, a whole
//! turn, or a path coming back within the tool's reach of itself — the band then laps over itself,
//! the sewn boundary no solid's. That last is read as a ball's tube is (`canal::first_return`), from
//! the axis's path sampled: two of its points half a turn of the tool's radius or more apart along
//! it must stand farther apart than the tool is wide, by the samples' spacing. **Judged before it is
//! returned**, as every closed build is (`closed::judge`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{prism,revolve,Profile};
use crate::geom::Surface;
use crate::planar::{Region,Seg};
use crate::recipe::region_profile;
use crate::section::Sectioned;
use crate::topo::Brep;
use crate::trajectory::{Screw,Trajectory};
use crate::util::space::{cross,dot,norm,sub,unit};
use super::closed::{whole_turn,Unbuilt};
use super::member::{Solid,Swept};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// The most points the axis's path is read at for whether it comes back on itself.
const SAMPLES: usize = 4096;

/// The tool's meridian: its region in `(r, z)` off its line — `r` from the line, `z` along it from
/// `origin` — and the steps of its outline from the line at its foot up to the line at its top.
struct Meridian { origin: V,axis: V,region: Region,chain: Vec<Seg>,widest: f64 }

/// One piece of the path: its twist, its stretch of the roll, and where the tool's line stands
/// as it takes the piece up and as it leaves it.
struct Leg { screw: Screw,over: [f64;2],from: V,to: V }

impl Leg {
    /// The length the tool's line runs over the leg.
    fn length(&self) -> f64 {
        if self.screw.is_slide() { norm(sub(self.to,self.from)) } else { self.screw.radius(self.from)*self.angle().abs() }
    }
    /// The angle a turn turns through.
    fn angle(&self) -> f64 { self.screw.ratio*(self.over[1]-self.over[0]) }
    /// The point the tool's line passes at `t` of the leg, from 0 to 1.
    fn at(&self,t: f64) -> V { self.screw.carry(self.from,t*(self.over[1]-self.over[0])) }
    /// The direction the tool's line moves where it stands at `p`.
    fn ahead(&self,p: V) -> V { unit(self.screw.velocity(p)) }
}

/// `x` within `eps` of zero.
fn nil(x: f64,eps: f64) -> bool { x.abs() <= eps }

/// Whether some angle `w + 2πk` lies strictly inside `(lo, hi)`.
fn passes(w: f64,lo: f64,hi: f64) -> bool {
    let w = lo+(w-lo).rem_euclid(TAU);
    (w > lo+1e-12 && w < hi-1e-12) || w+TAU < hi-1e-12
}

/// The path's twists over `roll`, each a slide or a turn short of a whole turn: read off its
/// structure alone, before the tool is.
fn twists(path: &Trajectory,[a,b]: [f64;2]) -> Option<Vec<(Screw,[f64;2])>> {
    let twist = |p: &Trajectory| match p { Trajectory::Twist(s) | Trajectory::Placed {twist:s,..} => Some(*s),_ => None };
    let out: Vec<(Screw,[f64;2])> = match path {
        Trajectory::Pieces(pieces) => pieces.pieces().iter().filter(|p| p.over[1].min(b) > p.over[0].max(a))
            .map(|p| twist(&p.path).map(|s| (s,[p.over[0].max(a),p.over[1].min(b)]))).collect::<Option<_>>()?,
        _ => vec![(twist(path)?,[a,b])],
    };
    (!out.is_empty() && out.iter().all(|(s,over)| s.is_slide() || (s.is_turn() && !whole_turn(s,*over)))).then_some(out)
}

/// The tool's meridian, where it is one revolution's (`Sectioned::meridian`) and every slice of
/// it square to its line is a disc about it (or a point at its ends); none otherwise.
fn meridian(tool: &Brep,eps: f64) -> Option<Meridian> {
    let sectioned = Sectioned::of_boundary(tool).ok()?;
    let region = sectioned.meridian()?.clone();
    let [ref l] = region.loops[..] else { return None };
    if l.iter().any(|s| s.chart.is_some()) { return None }
    let segs: Vec<Seg> = l.iter().map(|s| s.seg).collect();
    let on_line = |s: &Seg| matches!(*s,Seg::Line {a,b} if nil(a[0],eps) && nil(b[0],eps));
    // the loop begun just past its stretch along the line (its material on its left, so that
    // stretch runs down it and the rest up from its foot)
    let n = segs.len();
    let first = (0..n).find(|&k| on_line(&segs[k]) && !on_line(&segs[(k+1)%n]))?;
    let chain: Vec<Seg> = (1..n).map(|k| segs[(first+k)%n]).take_while(|s| !on_line(s)).collect();
    if chain.is_empty() || chain.len()+segs.iter().filter(|s| on_line(s)).count() != n { return None }
    // up from the foot without turning back, off the line but at its two ends
    let (foot,top) = (chain[0].point(0.),chain[chain.len()-1].point(1.));
    if !(nil(foot[0],eps) && nil(top[0],eps) && top[1] > foot[1]+eps) { return None }
    let mut widest: f64 = 0.;
    for (k,s) in chain.iter().enumerate() {
        let (p,q) = (s.point(0.),s.point(1.));
        if q[1] < p[1]-eps || (k > 0 && p[0] <= eps) || (k+1 < chain.len() && q[0] <= eps) { return None }
        widest = widest.max(p[0]).max(q[0]);
        match *s {
            Seg::Line {..} => {}
            Seg::Arc {c,r,a0,sweep} => {
                // its height turns back a quarter turn either side of the horizontal, its radius
                // is least half a turn from its greatest
                let (lo,hi) = (a0.min(a0+sweep),a0.max(a0+sweep));
                if passes(0.5*PI,lo,hi) || passes(-0.5*PI,lo,hi) { return None }
                if passes(PI,lo,hi) && c[0]-r <= eps { return None }
                if passes(0.,lo,hi) { widest = widest.max(c[0]+r) }
            }
            Seg::Conic {..} => return None,
        }
    }
    Some(Meridian {origin:sectioned.origin,axis:unit(sectioned.axis),region,chain,widest})
}

/// A meridian step mirrored across the line: `(r, z)` at `(−r, z)`, an arc's angle `w` at `π − w`.
fn mirrored(s: &Seg) -> Seg {
    match *s {
        Seg::Line {a,b} => Seg::Line {a:[-a[0],a[1]],b:[-b[0],b[1]]},
        Seg::Arc {c,r,a0,sweep} => Seg::Arc {c:[-c[0],c[1]],r,a0:PI-a0,sweep:-sweep},
        Seg::Conic {..} => unreachable!("a meridian has no conic (`meridian`)"),
    }
}

/// The tool's axial section where its line is `(o, a)`, in the plane holding it and `x`: its
/// meridian and its mirror image, a level step from the line and its image one step across it.
fn section(m: &Meridian,o: V,a: V,x: V,eps: f64) -> Profile {
    let k = m.chain.len();
    let level = |s: &Seg| match *s { Seg::Line {a: p,b: q} => (p[1]-q[1]).abs() <= eps,_ => false };
    let (foot,top) = (level(&m.chain[0]),k > 1 && level(&m.chain[k-1]));
    let across = |p: [f64;2]| Seg::Line {a:[-p[0],p[1]],b:p};
    let inner = &m.chain[usize::from(foot)..k-usize::from(top)];
    let steps: Vec<Seg> = foot.then(|| across(m.chain[0].point(1.))).into_iter()
        .chain(inner.iter().flat_map(|s| [*s,mirrored(s)]))
        .chain(top.then(|| across(m.chain[k-1].point(0.)))).collect();
    region_profile(&Region::plain(vec![steps]),o,x,a,&|_| String::new())
}

/// The tool's half beyond its section plane square to `ahead` where its line is `(o, a)`: its
/// meridian on the side `a × ahead` turned half a turn about the line, back against `ahead` where
/// `back`, else on along it.
fn half(m: &Meridian,o: V,a: V,ahead: V,back: bool) -> Result<Brep,String> {
    // a quarter turn about `a` takes `x = a × ahead` to `−ahead`
    revolve(&region_profile(&m.region,o,unit(cross(a,ahead)),a,&|_| String::new()),o,a,if back { PI } else { -PI })
}

/// **The closed solid `tool` sweeps under `path` over `roll`, or `blank` cut by it, where the tool is
/// a solid of revolution carried square to its axis** (the module's method), built to `tol` and
/// judged by membership read on the tool meshed within `bar`; none where the module's reading does
/// not hold.
pub fn swept(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Option<Brep>,Unbuilt> {
    let roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let eps = 1e-9*(1.+tool.size());
    // the motion read first, then the tool
    let Some(twists) = twists(path,roll) else { return Ok(None) };
    let Some(m) = meridian(tool,eps) else { return Ok(None) };
    // the tool's line where each piece takes it up and leaves it, kept upright: each slide square
    // to it, each turn about a line along it that the tool stands clear of
    let start = path.pose(roll[0])?;
    let a = unit(start.vector(m.axis));
    let mut legs: Vec<Leg> = Vec::with_capacity(twists.len());
    for (screw,over) in twists {
        let (from,to) = (path.pose(over[0])?.point(m.origin),path.pose(over[1])?.point(m.origin));
        let l = unit(screw.axis);
        let upright = if screw.is_slide() { dot(l,a).abs() <= 1e-9 && norm(sub(to,from)) > eps }
            else { norm(cross(l,a)) <= 1e-9 && screw.radius(from) > m.widest+eps };
        if !upright { return Ok(None) }
        legs.push(Leg {screw,over,from,to});
    }
    // the direction kept where two pieces meet, and the path never back within the tool's width
    if legs.windows(2).any(|w| dot(w[0].ahead(w[0].to),w[1].ahead(w[1].from)) < 1.-1e-9) { return Ok(None) }
    let length: f64 = legs.iter().map(Leg::length).sum();
    let h = (0.125*m.widest).max(length/SAMPLES as f64);
    let (mut points,mut along) = (Vec::new(),Vec::new());
    for l in &legs {
        let (len,n) = (l.length(),((l.length()/h).ceil() as usize).max(1));
        let s0 = along.last().copied().unwrap_or(0.);
        for k in usize::from(!points.is_empty())..=n { points.push(l.at(k as f64/n as f64)); along.push(s0+len*k as f64/n as f64); }
    }
    if super::canal::first_return(&points,&along,m.widest).is_some() { return Ok(None) }
    // the pieces, each with the section planes its caps lie in
    let (first,last) = (&legs[0],&legs[legs.len()-1]);
    let mut pieces: Vec<(Brep,Vec<(V,V)>)> = vec![(half(&m,first.from,a,first.ahead(first.from),true)?,vec![(first.from,first.ahead(first.from))])];
    for l in &legs {
        let (enter,leave) = (l.ahead(l.from),l.ahead(l.to));
        // (the section is its own mirror image across the line, so either side of it serves)
        let x = unit(cross(a,enter));
        let solid = if l.screw.is_slide() { prism(&section(&m,l.from,a,x,eps),0.,l.length())? }
            else { revolve(&section(&m,l.from,a,x,eps),l.screw.origin,unit(l.screw.axis),l.angle())? };
        pieces.push((solid,vec![(l.from,enter),(l.to,leave)]));
    }
    pieces.push((half(&m,last.to,a,last.ahead(last.to),false)?,vec![(last.to,last.ahead(last.to))]));
    // every cap of every piece dropped — each lies on a section plane another piece's cap lies on
    // too, and bounds nothing — and the rest sewn into one boundary
    let reach = tool.size();
    let open: Vec<Brep> = pieces.into_iter().map(|(mut b,planes)| {
        let faces = std::mem::take(&mut b.faces);
        let cap = |f: &crate::topo::Face| match &f.surface {
            Surface::Plane(fr) => planes.iter().any(|&(o,n)| norm(cross(fr.z,n)) <= 1e-9 && dot(sub(fr.o,o),n).abs() <= 1e2*eps
                && f.loops.iter().flatten().all(|c| norm(sub(b.vertices[b.ends(c)[0] as usize].p,o)) <= reach)),
            _ => false,
        };
        let faces = faces.into_iter().filter(|f| !cap(f)).collect();
        Brep {faces,..b}
    }).collect();
    let groove = crate::sew::sewn(&open,tol)?;
    crate::util::meter::go_on()?;
    // (each checked as it is made: the sewn boundary by `sewn`, the cut by the Boolean)
    let out = match blank {
        Some(blank) => crate::boolean::combined(blank,&groove,crate::boolean::Op::Cut,tol)?,
        None => groove,
    };
    // **judged by its definition before it is returned** (#777 V2), over the blank or a box
    // holding the sweep
    let solid = Solid::new(tool,bar,1.)?;
    let held;
    let within = match blank {
        Some(blank) => blank,
        None => { let (lo,hi) = out.bounds(); held = super::closed::cube_holding(&[lo,hi],tol)?.0; &held }
    };
    super::closed::judge(&out,&Swept {tool:&solid,path,roll},within,blank,&[],tol)?;
    Ok(Some(out))
}
