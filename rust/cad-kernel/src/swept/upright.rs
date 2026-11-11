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
//! the sewn boundary no solid's. That last is read from the axis's path sampled: two of its points
//! farther apart along it than half a turn of its tightest bend must stand farther apart than the
//! tool is wide, by the samples' spacing, the most a point between samples may come nearer; nearer
//! along it than that the path cannot come back, its bends wider than the tool. **Judged before it
//! is returned**, as every closed build is (`closed::judge`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{prism,revolve,Profile,ProfileEdge};
use crate::geom::{Frame,Surface};
use crate::planar::Seg;
use crate::section::Sectioned;
use crate::topo::Brep;
use crate::trajectory::{Pose,Screw,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use super::member::{Solid,Swept};
use std::f64::consts::{PI,TAU};

type V = [f64;3];
type P = [f64;2];

/// The most points the axis's path is read at for whether it comes back on itself.
const SAMPLES: usize = 4096;

/// The tool's meridian: the steps of its outline in `(r, z)` off its line — `r` from the line,
/// `z` along it from `origin` — from the line at its foot up to the line at its top, and its line.
struct Meridian { origin: V,axis: V,chain: Vec<Seg>,widest: f64 }

/// One piece of the path over its stretch of the roll: its twist, the tool's line where it takes it
/// up and where it leaves it, the direction the line moves at each end, and the angle a turn turns
/// through (`ratio` by the stretch).
struct Leg { screw: Screw,from: (V,V),to: (V,V),ahead: [V;2],angle: f64 }

/// `x` within `eps` of zero.
fn nil(x: f64,eps: f64) -> bool { x.abs() <= eps }

/// Whether some angle `w + 2πk` lies strictly inside `(lo, hi)`.
fn passes(w: f64,lo: f64,hi: f64) -> bool {
    let w = lo+(w-lo).rem_euclid(TAU);
    (w > lo+1e-12 && w < hi-1e-12) || w+TAU < hi-1e-12
}

/// The tool's meridian, where every face of it turns about one line and every slice of it square
/// to that line is a disc about it (or a point at its ends); none otherwise.
fn meridian(tool: &Brep,eps: f64) -> Option<Meridian> {
    let sectioned = Sectioned::of_boundary(tool).ok()?;
    if !sectioned.about_its_line() { return None }
    let (origin,axis) = (sectioned.origin,unit(sectioned.axis));
    let region = sectioned.section(Frame::about(origin,axis).x).ok()?;
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
    Some(Meridian {origin,axis,chain,widest})
}

/// A point `(r, z)` of the meridian in space where the tool's line is `(o, a)`, `r` along `x`.
fn lift(o: V,a: V,x: V,p: P) -> V { add(o,add(scale(x,p[0]),scale(a,p[1]))) }

/// A meridian step carried into space where the tool's line is `(o, a)`, `r` along `x` — or, where
/// `mirror`, along `−x`. An arc's frame is `(x, a)`, about `x × a`.
fn edge(s: &Seg,o: V,a: V,x: V,mirror: bool) -> ProfileEdge {
    let at = |p: P| lift(o,a,x,[if mirror { -p[0] } else { p[0] },p[1]]);
    match *s {
        Seg::Line {a: p,b: q} => ProfileEdge::Line {a:at(p),b:at(q)},
        Seg::Arc {c,r,a0,sweep} => {
            // mirrored, the angle `w` is `π − w`
            let (w0,w1) = (a0.min(a0+sweep),a0.max(a0+sweep));
            let span = if mirror { [PI-w1,PI-w0] } else { [w0,w1] };
            ProfileEdge::Arc {frame:Frame {o:at(c),x,y:a,z:cross(x,a)},r,span:Some(span)}
        }
        Seg::Conic {..} => unreachable!("a meridian has no conic (`meridian`)"),
    }
}

/// The tool's axial section where its line is `(o, a)`, in the plane holding it and `x`: its
/// meridian and its mirror image, a level step from the line and its image one step across it.
fn section(m: &Meridian,o: V,a: V,x: V,eps: f64) -> Profile {
    let k = m.chain.len();
    let level = |s: &Seg| match *s { Seg::Line {a: p,b: q} => (p[1]-q[1]).abs() <= eps,_ => false };
    let (foot,top) = (level(&m.chain[0]),k > 1 && level(&m.chain[k-1]));
    let mut edges = Vec::new();
    if foot {
        let q = m.chain[0].point(1.);
        edges.push(ProfileEdge::Line {a:lift(o,a,x,[-q[0],q[1]]),b:lift(o,a,x,q)});
    }
    for s in &m.chain[usize::from(foot)..k-usize::from(top)] { edges.push(edge(s,o,a,x,false)); edges.push(edge(s,o,a,x,true)); }
    if top {
        let p = m.chain[k-1].point(0.);
        edges.push(ProfileEdge::Line {a:lift(o,a,x,p),b:lift(o,a,x,[-p[0],p[1]])});
    }
    Profile {origin:o,normal:cross(x,a),loops:vec![edges],names:Vec::new()}
}

/// The tool's half beyond its section plane square to `ahead` where its line is `(o, a)`: its
/// meridian on the side `a × ahead` turned half a turn about the line, back against `ahead` where
/// `back`, else on along it.
fn half(m: &Meridian,o: V,a: V,ahead: V,back: bool) -> Result<Brep,String> {
    let x = unit(cross(a,ahead));
    let mut edges: Vec<ProfileEdge> = m.chain.iter().map(|s| edge(s,o,a,x,false)).collect();
    let (foot,top) = (m.chain[0].point(0.),m.chain[m.chain.len()-1].point(1.));
    edges.push(ProfileEdge::Line {a:lift(o,a,x,[0.,top[1]]),b:lift(o,a,x,[0.,foot[1]])});
    let profile = Profile {origin:o,normal:cross(x,a),loops:vec![edges],names:Vec::new()};
    // a quarter turn about `a` takes `x = a × ahead` to `−ahead`
    revolve(&profile,o,a,if back { PI } else { -PI })
}

impl Leg {
    /// Under a turn: the foot of the tool's line on the turn's (at the tool's origin's height), the
    /// line's offset from it where the leg begins, and the angle it turns through.
    fn turned(&self) -> (V,V,f64) {
        let a = unit(self.screw.axis);
        let r = sub(self.from.0,self.screw.origin);
        let p = sub(r,scale(a,dot(r,a)));
        (sub(self.from.0,p),p,self.angle)
    }
    /// The length the tool's line runs over the leg.
    fn length(&self) -> f64 {
        if self.screw.is_slide() { norm(sub(self.to.0,self.from.0)) } else { let (_,r,w) = self.turned(); norm(r)*w.abs() }
    }
    /// The point the tool's line passes at `t` of the leg, from 0 to 1.
    fn at(&self,t: f64) -> V {
        if self.screw.is_slide() { return add(self.from.0,scale(sub(self.to.0,self.from.0),t)) }
        let (c,r,w) = self.turned();
        let (s,cs) = (w*t).dsin_cos();
        add(c,add(scale(r,cs),scale(cross(unit(self.screw.axis),r),s)))
    }
}

/// The path's pieces over `roll`, each a slide square to the tool's line or a turn about a line
/// along it short of a whole turn, read where the tool stands; none for any other motion.
fn legs(m: &Meridian,path: &Trajectory,[a,b]: [f64;2],eps: f64) -> Option<Vec<Leg>> {
    let twist = |p: &Trajectory| match p { Trajectory::Twist(s) | Trajectory::Placed {twist:s,..} => Some(*s),_ => None };
    let stretches: Vec<(Screw,[f64;2])> = match path {
        Trajectory::Pieces(pieces) => pieces.pieces().iter().filter(|p| p.over[1].min(b) > p.over[0].max(a))
            .map(|p| twist(&p.path).map(|s| (s,[p.over[0].max(a),p.over[1].min(b)]))).collect::<Option<_>>()?,
        _ => vec![(twist(path)?,[a,b])],
    };
    if stretches.is_empty() { return None }
    let line = |q: &Pose| (q.point(m.origin),unit(q.vector(m.axis)));
    stretches.into_iter().map(|(screw,[lo,hi])| {
        let (from,to) = (line(&path.pose(lo).ok()?),line(&path.pose(hi).ok()?));
        let a = from.1;
        if norm(cross(a,to.1)) > 1e-9 { return None }
        let ahead = if screw.is_slide() {
            let d = sub(to.0,from.0);
            if !(norm(d) > eps) { return None }
            let d = unit(d);
            if norm(cross(d,unit(screw.axis))) > 1e-9 || dot(d,a).abs() > 1e-9 { return None }
            [d,d]
        } else if screw.is_turn() {
            let l = unit(screw.axis);
            if norm(cross(l,a)) > 1e-9 || (screw.ratio*(hi-lo)).abs() >= TAU*(1.-1e-12) { return None }
            let round = |p: V| { let r = sub(p,screw.origin); unit(scale(cross(l,sub(r,scale(l,dot(r,l)))),screw.ratio.signum())) };
            [round(from.0),round(to.0)]
        } else { return None };
        Some(Leg {screw,from,to,ahead,angle:screw.ratio*(hi-lo)})
    }).collect()
}

/// Whether the axis's path over `legs` comes back within `width` of itself (the module's reading),
/// `bend` the radius of its tightest turn.
fn laps(legs: &[Leg],width: f64,bend: f64) -> bool {
    if !bend.is_finite() { return false }
    let length: f64 = legs.iter().map(Leg::length).sum();
    let h = (0.125*width).max(length/SAMPLES as f64);
    let mut pts: Vec<(V,f64)> = Vec::new();
    let mut along = 0.;
    for l in legs {
        let len = l.length();
        let n = ((len/h).ceil() as usize).max(1);
        pts.extend((0..=n).map(|k| (l.at(k as f64/n as f64),along+len*k as f64/n as f64)));
        along += len;
    }
    let (apart,near) = (PI*bend,width+h);
    (0..pts.len()).any(|i| pts[i+1..].iter().any(|&(q,s)| s-pts[i].1 >= apart && norm(sub(q,pts[i].0)) < near))
}

/// **The closed solid `tool` sweeps under `path` over `roll`, or `blank` cut by it, where the tool is
/// a solid of revolution carried square to its axis** (the module's method), built to `tol` and
/// judged by membership read on the tool meshed within `bar`; none where the module's reading does
/// not hold.
pub fn swept(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Option<Brep>,Unbuilt> {
    let roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let eps = 1e-9*(1.+tool.size());
    let Some(m) = meridian(tool,eps) else { return Ok(None) };
    let Some(legs) = legs(&m,path,roll,eps) else { return Ok(None) };
    // each turn about a line the tool stands clear of, and the path's direction kept where two
    // pieces meet
    let mut bend = f64::INFINITY;
    for l in legs.iter().filter(|l| l.screw.is_turn()) {
        let d = norm(l.turned().1);
        if !(d > m.widest+eps) { return Ok(None) }
        bend = bend.min(d);
    }
    if legs.windows(2).any(|w| dot(w[0].ahead[1],w[1].ahead[0]) < 1.-1e-9) { return Ok(None) }
    if laps(&legs,2.*m.widest,bend) { return Ok(None) }
    // the pieces, each with the section planes its caps lie in
    let mut pieces: Vec<(Brep,Vec<(V,V)>)> = Vec::new();
    let (first,last) = (&legs[0],&legs[legs.len()-1]);
    pieces.push((half(&m,first.from.0,first.from.1,first.ahead[0],true)?,vec![(first.from.0,first.ahead[0])]));
    for l in &legs {
        let a = l.from.1;
        let solid = if l.screw.is_slide() {
            prism(&section(&m,l.from.0,a,unit(cross(a,l.ahead[0])),eps),0.,l.length())?
        } else {
            let (_,r,w) = l.turned();
            revolve(&section(&m,l.from.0,a,unit(r),eps),l.screw.origin,unit(l.screw.axis),w)?
        };
        pieces.push((solid,vec![(l.from.0,l.ahead[0]),(l.to.0,l.ahead[1])]));
    }
    pieces.push((half(&m,last.to.0,last.to.1,last.ahead[1],false)?,vec![(last.to.0,last.ahead[1])]));
    // every cap of every piece dropped — each lies on a section plane another piece's cap lies on
    // too, and bounds nothing — and the rest sewn into one boundary
    let reach = tool.size();
    let open: Vec<Brep> = pieces.into_iter().map(|(b,planes)| {
        let cap = |f: &crate::topo::Face| match &f.surface {
            Surface::Plane(fr) => planes.iter().any(|&(o,n)| norm(cross(fr.z,n)) <= 1e-9 && dot(sub(fr.o,o),n).abs() <= 1e2*eps
                && f.loops.iter().flatten().all(|c| norm(sub(b.vertices[b.ends(c)[0] as usize].p,o)) <= reach)),
            _ => false,
        };
        let faces = b.faces.iter().filter(|f| !cap(f)).cloned().collect();
        Brep {faces,..b}
    }).collect();
    let groove = crate::sew::sewn(&open,tol)?;
    crate::util::meter::go_on()?;
    let out = match blank {
        Some(blank) => crate::boolean::combined(blank,&groove,crate::boolean::Op::Cut,tol)?,
        None => groove,
    };
    out.check(tol)?;
    // **judged by its definition before it is returned** (#777 V2), over the blank or a box
    // holding the sweep
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    match blank {
        Some(blank) => super::closed::judge(&out,&swept,blank,Some(blank),&[],tol)?,
        None => {
            let (lo,hi) = out.bounds();
            let (domain,..) = super::closed::cube_holding(&[lo,hi],tol)?;
            super::closed::judge(&out,&swept,&domain,None,&[],tol)?;
        }
    }
    Ok(Some(out))
}
