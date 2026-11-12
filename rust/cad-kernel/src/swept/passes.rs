//! **A tool of revolution under a program of passes** (a milling program, `brep-bench`'s `cnc/`):
//! a tool whose every section square to its axis is a disc — a flat, bull-nose or ball-end mill, a
//! drill — carried along a path of slides and turns, each move's sweep built in closed form and the
//! moves taken from the blank one after another (or united, with no blank), with no arrangement of
//! candidates and no membership read cell by cell. A program of hundreds of moves is hundreds of
//! Booleans, each local to its move.
//!
//! The tool is read off its own B-rep as a solid of revolution (`Revolution`): every face a plane
//! square to one axis or a cylinder, cone, ball or torus about it, their meridians one chain from the
//! axis out and back to it, rising all the way (so each section is a disc), its volume by Pappus the
//! B-rep's own. Each move is read where it takes the tool up, and is one of:
//!
//! - **a slide square to the axis**: everything within the tool's radius at each height of the
//!   segment its axis slides along — the tool's half behind the slide at its start (its meridian
//!   turned half a turn), the section through its axis square to the slide (its meridian and its
//!   mirror image) slid the slide's length, and its half ahead at its end, united. Exact: a point
//!   of the sweep is within the radius at its height of some point of the segment, nearest either
//!   an end (the halves) or a point between (the section's prism);
//! - **a turn about a line along the axis** that clears the tool (more than its reach from its
//!   axis): the same, the section through both lines turned about the turn's (each circle about the
//!   turn's line meets a disc of the tool in one arc about that section);
//! - **a slide along the axis**, where the tool's radius never narrows up its meridian: the tool
//!   stretched, its meridian's side carried up the slide's length.
//!
//! Anything else (a screw, a slide aslant the axis, a turn about another line, a tool no such
//! revolution) is not this class's (`Ok(None)`), and the closed class's arrangement is asked. What
//! the class builds is checked as a solid and judged by the sweep's definition before it is
//! returned (`closed::judge`), as every closed build is.
use crate::boolean::{boolean,face_parameters,Op};
use crate::build::{prism,revolve,Profile,ProfileEdge};
use crate::geom::{Frame,Surface,V};
use crate::topo::Brep;
use crate::trajectory::{Pose,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub};
use super::closed::{cube_holding,judge,Unbuilt};
use super::member::{Solid,Swept};
use std::f64::consts::{PI,TAU};

/// A piece of a meridian in `(ρ, h)`: radius from the axis and height along it.
#[derive(Clone,Copy,Debug)]
enum Seg {
    Line([f64;2],[f64;2]),
    /// The arc of radius `r` about `c` from angle `a[0]` to `a[1]` (from `+ρ` toward `+h`).
    Arc { c: [f64;2],r: f64,a: [f64;2] },
}

impl Seg {
    fn at(&self,s: f64) -> [f64;2] {
        match *self {
            Seg::Line(p,q) => [p[0]+(q[0]-p[0])*s,p[1]+(q[1]-p[1])*s],
            Seg::Arc {c,r,a} => { let t = a[0]+(a[1]-a[0])*s; [c[0]+r*t.cos(),c[1]+r*t.sin()] }
        }
    }
    fn ends(&self) -> [[f64;2];2] { [self.at(0.),self.at(1.)] }
    fn reversed(&self) -> Seg {
        match *self { Seg::Line(p,q) => Seg::Line(q,p),Seg::Arc {c,r,a} => Seg::Arc {c,r,a:[a[1],a[0]]} }
    }
    /// `∫ ρ² dh` along the piece, as it runs: π times it is the volume its sweep about the axis
    /// encloses with the axis (Pappus, by Green).
    fn moment(&self) -> f64 {
        match *self {
            Seg::Line(p,q) => (p[0]*p[0]+p[0]*q[0]+q[0]*q[0])/3.*(q[1]-p[1]),
            Seg::Arc {c,r,a} => {
                // ρ = c₀ + r cos t, dh = r cos t dt: c₀² r cos t + 2 c₀ r² cos² t + r³ cos³ t, in
                // closed form
                let f = |t: f64| { let (s,co) = t.sin_cos();
                    c[0]*c[0]*r*s+2.*c[0]*r*r*(0.5*t+0.5*s*co)+r*r*r*(s-s*s*s/3.) };
                f(a[1])-f(a[0])
            }
        }
    }
}

/// The tool read as a solid of revolution: a point `o` of its axis and the axis `a`, and its
/// meridian, one chain from the axis at its foot out and up and back to the axis at its top,
/// rising all the way; its reach (the most radius), and whether its radius never narrows up the
/// chain short of its top's cap (`widening`).
#[derive(Clone,Debug)]
struct Revolution { o: V,a: V,chain: Vec<Seg>,reach: f64,top: f64,widening: bool }

/// The tool `b` as a solid of revolution, or none where it is no such solid (a face no plane square
/// to one axis or round face about it, a meridian no one rising chain, or a volume not its
/// meridian's).
fn revolution(b: &Brep,tol: f64) -> Option<Revolution> {
    let (o,a) = b.faces.iter().find_map(|f| match &f.surface {
        Surface::Cylinder(fr,_) | Surface::Cone(fr,..) | Surface::Torus(fr,..) => Some((fr.o,unit(fr.z))),
        _ => None,
    })?;
    let along = |f: &Frame| norm(cross(f.z,a)) <= 1e-9 && norm(cross(sub(f.o,o),a)) <= tol;
    let rh = |p: V| { let d = sub(p,o); let h = dot(d,a); [norm(sub(d,scale(a,h))),h] };
    let mut segs: Vec<Seg> = Vec::new();
    for (fi,f) in b.faces.iter().enumerate() {
        let [_,[v0,v1]] = face_parameters(b,fi);
        let seg = match &f.surface {
            Surface::Plane(fr) => {
                if norm(cross(fr.z,a)) > 1e-9 { return None }
                // its rims' radii, every one at the plane's height: a disc where they are one circle
                let rims: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|c| {
                    let e = &b.edges[c.edge as usize];
                    (0..3).map(move |k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/2.,&b.vertices))
                }).map(rh).collect();
                let h = rh(fr.o)[1];
                if rims.iter().any(|q| (q[1]-h).abs() > 10.*tol) { return None }
                let (lo,hi) = rims.iter().fold((f64::INFINITY,0_f64),|(l,m),q| (l.min(q[0]),m.max(q[0])));
                if hi <= tol { continue }
                let lo = if f.loops.len() == 1 && hi-lo <= 10.*tol { 0. } else { lo };
                Seg::Line([lo,h],[hi,h])
            }
            Surface::Cylinder(fr,r) => {
                if !along(fr) { return None }
                let s = dot(fr.z,a);
                let h = rh(fr.o)[1];
                Seg::Line([*r,h+s*v0],[*r,h+s*v1])
            }
            Surface::Cone(fr,r,al) => {
                if !along(fr) { return None }
                let s = dot(fr.z,a);
                let h = rh(fr.o)[1];
                let at = |v: f64| [r+v*al.sin(),h+s*v*al.cos()];
                Seg::Line(at(v0),at(v1))
            }
            Surface::Sphere(fr,r) => {
                if norm(cross(fr.z,a)) > 1e-9 || norm(cross(sub(fr.o,o),a)) > tol { return None }
                let s = dot(fr.z,a);
                Seg::Arc {c:[0.,rh(fr.o)[1]],r:*r,a:[s*v0,s*v1]}
            }
            Surface::Torus(fr,big,r) => {
                if !along(fr) { return None }
                let s = dot(fr.z,a);
                Seg::Arc {c:[*big,rh(fr.o)[1]],r:*r,a:[s*v0,s*v1]}
            }
            _ => return None,
        };
        // a face cut in two (a seam) is one piece of the meridian
        let same = |x: &Seg| { let ([p,q],[r,s]) = (x.ends(),seg.ends());
            let near = |u: [f64;2],v: [f64;2]| (u[0]-v[0]).hypot(u[1]-v[1]) <= 10.*tol;
            (near(p,r) && near(q,s)) || (near(p,s) && near(q,r)) };
        if !segs.iter().any(same) { segs.push(seg) }
    }
    // the chain, from the foot on the axis
    let near = |u: [f64;2],v: [f64;2]| (u[0]-v[0]).hypot(u[1]-v[1]) <= 100.*tol;
    let (k,_) = segs.iter().enumerate().flat_map(|(k,s)| s.ends().into_iter().map(move |p| (k,p)))
        .filter(|(_,p)| p[0].abs() <= 100.*tol).min_by(|x,y| x.1[1].total_cmp(&y.1[1]))?;
    let first = if segs[k].ends()[0][0].abs() <= 100.*tol && segs[k].ends()[0][1] <= segs[k].ends()[1][1] { segs[k] } else { segs[k].reversed() };
    let mut chain = vec![first];
    let mut used = vec![false;segs.len()];
    used[k] = true;
    loop {
        let end = chain.last().unwrap().ends()[1];
        if end[0].abs() <= 100.*tol && chain.len() > 1 { break }
        let (j,s) = segs.iter().enumerate().filter(|(j,_)| !used[*j]).find_map(|(j,s)| {
            let [p,q] = s.ends();
            if near(p,end) { Some((j,*s)) } else if near(q,end) { Some((j,s.reversed())) } else { None }
        })?;
        used[j] = true;
        chain.push(s);
    }
    if used.iter().any(|u| !u) { return None }
    // rising all the way, so that each section is a disc; and whether it widens to its top's cap
    let mut last = chain[0].ends()[0];
    let mut widening = true;
    let mut reach = 0_f64;
    let top = chain.last().unwrap().ends()[1][1];
    for s in &chain {
        for i in 1..=8 {
            let p = s.at(i as f64/8.);
            if p[1] < last[1]-10.*tol || p[0] < -10.*tol { return None }
            if p[0] < last[0]-10.*tol && p[1] < top-10.*tol { widening = false }
            reach = reach.max(p[0]);
            last = p;
        }
    }
    // its volume its meridian's, by Pappus
    let v = PI*chain.iter().map(Seg::moment).sum::<f64>().abs();
    let own = crate::props::volume(b);
    if !((v-own).abs() <= 1e-9*own.abs().max(1e-300)) { return None }
    Some(Revolution {o,a,chain,reach,top,widening})
}

fn unit(v: V) -> V { scale(v,1./norm(v)) }

impl Revolution {
    /// The chain's pieces as profile edges in the half-plane of `w` (a unit square to the axis)
    /// about the axis through `c`.
    fn edges(&self,chain: &[Seg],c: V,a: V,w: V) -> Vec<ProfileEdge> {
        let at = |p: [f64;2]| add(c,add(scale(w,p[0]),scale(a,p[1])));
        chain.iter().map(|s| match *s {
            Seg::Line(p,q) => ProfileEdge::Line {a:at(p),b:at(q)},
            Seg::Arc {c:k,r,a:t} => ProfileEdge::Arc {frame:Frame::new(at(k),cross(w,a),w),r,
                span:Some([t[0].min(t[1]),t[0].max(t[1])])},
        }).collect()
    }

    /// The meridian about the axis through `c` along `a` in the half-plane of `w`, closed down the
    /// axis: `chain` (the tool's own, or one stretched).
    fn meridian(&self,chain: &[Seg],c: V,a: V,w: V) -> Profile {
        let mut edges = self.edges(chain,c,a,w);
        let [top,foot] = [chain.last().unwrap().ends()[1],chain[0].ends()[0]];
        let at = |p: [f64;2]| add(c,add(scale(w,p[0]),scale(a,p[1])));
        edges.push(ProfileEdge::Line {a:at([0.,top[1]]),b:at([0.,foot[1]])});
        Profile {origin:c,normal:cross(w,a),loops:vec![edges],names:Vec::new()}
    }

    /// The section through the axis through `c` along `a`, in the plane of `w`: the meridian and
    /// its mirror image, its normal `w × a`; a level line at the foot or the top across the axis
    /// one edge.
    fn section(&self,c: V,a: V,w: V) -> Profile {
        let mut chain = self.chain.clone();
        let mut mirror: Vec<Seg> = chain.iter().map(|s| match *s {
            Seg::Line(p,q) => Seg::Line([-p[0],p[1]],[-q[0],q[1]]),
            Seg::Arc {c,r,a} => Seg::Arc {c:[-c[0],c[1]],r,a:[PI-a[0],PI-a[1]]},
        }).collect();
        // a level piece from the axis at the foot or the top and its mirror image are one line
        // across the axis
        let level = |s: &Seg| matches!(s,Seg::Line(p,q) if (p[1]-q[1]).abs() <= 1e-12*(1.+p[1].abs()));
        let mut joined = Vec::new();
        if level(&chain[0]) && chain.len() > 1 {
            let [x,y] = [mirror[0].ends()[1],chain[0].ends()[1]];
            joined.push(Seg::Line(x,y));
            chain.remove(0);
            mirror.remove(0);
        }
        if level(chain.last().unwrap()) && chain.len() > 1 {
            let [x,y] = [chain.last().unwrap().ends()[0],mirror.last().unwrap().ends()[0]];
            joined.push(Seg::Line(x,y));
            chain.pop();
            mirror.pop();
        }
        let mut edges = self.edges(&chain,c,a,w);
        edges.extend(self.edges(&mirror,c,a,w));
        let at = |p: [f64;2]| add(c,add(scale(w,p[0]),scale(a,p[1])));
        for s in joined { if let Seg::Line(p,q) = s { edges.push(ProfileEdge::Line {a:at(p),b:at(q)}) } }
        Profile {origin:c,normal:cross(w,a),loops:vec![edges],names:Vec::new()}
    }

    /// The chain stretched up `by`: its pieces up to where it first reaches its reach, then its side
    /// at the reach up to `by` past its top, and its top's cap across to the axis there.
    fn stretched(&self,by: f64) -> Vec<Seg> {
        let eps = 1e-9*(1.+self.reach);
        let mut out = Vec::new();
        for s in &self.chain {
            out.push(*s);
            if s.ends()[1][0] >= self.reach-eps { break }
        }
        let h = out.last().unwrap().ends()[1][1];
        out.push(Seg::Line([self.reach,h],[self.reach,self.top+by]));
        out.push(Seg::Line([self.reach,self.top+by],[0.,self.top+by]));
        out
    }
}

/// One move of the program: a twist over its stretch of the roll, from the pose it takes the tool
/// up at.
struct Move { screw: crate::trajectory::Screw,over: [f64;2] }

/// The moves of `path` over `roll`, each a constant twist; none where some piece is no twist.
fn moves(path: &Trajectory,roll: [f64;2]) -> Option<Vec<Move>> {
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let one = |p: &Trajectory| match p { Trajectory::Twist(s) | Trajectory::Placed {twist:s,..} => Some(*s),_ => None };
    match path {
        Trajectory::Pieces(pieces) => pieces.pieces().iter().filter_map(|piece| {
            let (lo,hi) = (piece.over[0].max(a),piece.over[1].min(b));
            (hi > lo).then(|| one(&piece.path).map(|screw| Move {screw,over:[lo,hi]}))
        }).collect(),
        p => Some(vec![Move {screw:one(p)?,over:[a,b]}]),
    }
}

/// **The sweep of `tool` under `path` over `roll`** cut from `blank`, or closed where there is
/// none, where the tool is a solid of revolution and every move one this class takes (the
/// module's); `Ok(None)` where not, for the closed class to ask its arrangement.
pub fn built(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64,blank: Option<&Brep>) -> Result<Option<Brep>,Unbuilt> {
    let Some(rev) = revolution(tool,tol) else { return Ok(None) };
    let Some(moves) = moves(path,roll) else { return Ok(None) };
    let size = tool.size().max(blank.map_or(0.,Brep::size));
    let eps = 1e-9*(1.+size);
    let clock = crate::util::clock::Instant::now();
    let lap = |what: &str| if crate::util::meter::profiling() { eprintln!("passes: {what} at {:?}",clock.elapsed()) };
    // each move's sweep: a solid of its own (a slide along the axis, a whole turn), or a leg square
    // to the axis, its two half-pieces glued at the plane through the axis
    let mut items: Vec<Item> = Vec::new();
    let pose = |t: f64| path.pose(t).map_err(Unbuilt::from);
    let axis_at = |p: &Pose| (p.point(rev.o),unit(p.vector(rev.a)));
    for m in &moves {
        let (p0,p1) = (pose(m.over[0])?,pose(m.over[1])?);
        let ((c0,a),(c1,_)) = (axis_at(&p0),axis_at(&p1));
        let s = m.screw;
        if s.is_slide() {
            let d = sub(c1,c0);
            let l = norm(d);
            if l <= eps { continue }
            let u = scale(d,1./l);
            if norm(cross(u,a)) <= 1e-12 {
                // along the axis: the tool stretched from its lower pose
                if !rev.widening { return Ok(None) }
                let low = if dot(u,a) > 0. { c0 } else { c1 };
                let w = unit(any_square(a));
                items.push(Item::Solid(revolve(&rev.meridian(&rev.stretched(l),low,a,w),low,a,TAU)?));
            } else if dot(u,a).abs() <= 1e-12 {
                // square to the axis: each half-plane's meridian slid the slide's length
                let w = cross(a,u);
                // (each half's own normal, `w × a`, which its arcs' frames share: along the slide
                // for one half and against it for the other, swept back along it there)
                let half = |w: V| -> Result<Brep,Unbuilt> {
                    let p = rev.meridian(&rev.chain,c0,a,w);
                    Ok(if dot(p.normal,u) > 0. { prism(&p,0.,l)? } else { prism(&p,-l,0.)? })
                };
                let piece = joined(half(w)?,half(scale(w,-1.))?,tol)?;
                items.push(Item::Leg(Leg {a,c0,t0:u,c1,t1:u,piece}));
            } else {
                return Ok(None)
            }
        } else if s.is_turn() {
            let k = unit(s.axis);
            if norm(cross(k,a)) > 1e-12 { return Ok(None) }
            let off = sub(c0,s.origin);
            let radial = sub(off,scale(k,dot(off,k)));
            let d = norm(radial);
            if d <= rev.reach+1e-6*(1.+size) { return Ok(None) }
            let e = scale(radial,1./d);
            let angle = s.ratio*(m.over[1]-m.over[0]);
            if angle.abs() <= 1e-15 { continue }
            let turned = |w: V,by: f64| -> Result<Brep,Unbuilt> {
                Ok(revolve(&rev.meridian(&rev.chain,c0,a,w),s.origin,scale(k,angle.signum()),by)?)
            };
            if angle.abs() >= TAU-1e-12 {
                items.push(Item::Solid(revolve(&rev.section(c0,a,e),s.origin,k,TAU)?));
                continue
            }
            let piece = joined(turned(e,angle.abs())?,turned(scale(e,-1.),angle.abs())?,tol)?;
            let off1 = sub(c1,s.origin);
            let e1 = unit(sub(off1,scale(k,dot(off1,k))));
            let t0 = scale(cross(k,e),angle.signum());
            let t1 = scale(cross(k,e1),angle.signum());
            items.push(Item::Leg(Leg {a,c0,t0,c1,t1,piece}));
        } else {
            return Ok(None)
        }
    }
    lap("moves read");
    // the legs in runs, each run one tube: closed on itself where it ends as it began, else capped by
    // the tool's half behind its start and its half ahead of its end
    let mut solids: Vec<Brep> = Vec::new();
    let mut run: Vec<Leg> = Vec::new();
    let follows = |x: &Leg,y: &Leg| norm(sub(x.c1,y.c0)) <= eps && norm(sub(x.t1,y.t0)) <= 1e-9;
    for item in items {
        match item {
            Item::Solid(b) => {
                if !run.is_empty() { solids.extend(tubes(&rev,std::mem::take(&mut run),eps,tol)?) }
                solids.push(b);
            }
            Item::Leg(l) => {
                if run.last().is_some_and(|last| !follows(last,&l)) { solids.extend(tubes(&rev,std::mem::take(&mut run),eps,tol)?) }
                run.push(l);
            }
        }
    }
    if !run.is_empty() { solids.extend(tubes(&rev,run,eps,tol)?) }
    lap("tubes glued");
    let held: Vec<V> = solids.iter().flat_map(|s| { let (lo,hi) = s.bounds(); [lo,hi] }).collect();
    // united, neighbours in the program first (`united`), and taken from the blank at once — only
    // those reaching it, where there is one
    let out = match blank {
        Some(b) => {
            let bb = b.bounds();
            let reaching: Vec<Brep> = solids.into_iter().filter(|s| crate::boolean::overlap(&s.bounds(),&bb,eps)).collect();
            if reaching.is_empty() { b.clone() } else {
                let all = united(reaching,tol)?;
                lap("united");
                crate::boolean::combined(b,&all,Op::Cut,tol)?
            }
        }
        None => {
            if solids.is_empty() { return Err(Unbuilt::Failed("the program moves the tool nowhere".into())) }
            united(solids,tol)?
        }
    };
    lap("cut");
    out.check(tol)?;
    lap("checked");
    // **judged by its definition before it is returned** (#777 V2): over the blank, or a box
    // holding every move's sweep
    let solid = Solid::new(tool,bar,1.)?;
    let swept = Swept {tool:&solid,path,roll};
    match blank {
        Some(b) => judge(&out,&swept,b,Some(b),&[],tol)?,
        None => {
            let (domain,..) = cube_holding(&held,tol)?;
            judge(&out,&swept,&domain,None,&[],tol)?
        }
    }
    lap("judged");
    Ok(Some(out))
}

/// A move square to the axis: where it takes the axis up and leaves it, the way the axis heads at
/// each, and its piece, the section through the axis square to the heading carried along.
struct Leg { a: V,c0: V,t0: V,c1: V,t1: V,piece: Brep }

/// A move's sweep: a solid of its own, or a leg of a run.
enum Item { Solid(Brep),Leg(Leg) }

/// Two solids sharing faces, glued there (`boolean::glued`), else united.
fn joined(a: Brep,b: Brep,tol: f64) -> Result<Brep,Unbuilt> {
    match crate::boolean::glued(&a,&b,tol) {
        Some(g) => Ok(g),
        None => Ok(boolean(&a,&b,Op::Union,tol)?),
    }
}

/// **A run of legs, each beginning as the last ended, as tubes**: the legs glued end to end, a run
/// that comes back onto itself parted where a leg's box meets that of one before it but its
/// neighbour (so that nothing glued meets anything but at the faces it is glued at); each tube
/// closed on itself where it ends where and as it began, else capped by the tool's half behind its
/// start and its half ahead of its end — the halves' flat faces the section, so glued too. Its
/// coplanar faces made one.
fn tubes(rev: &Revolution,run: Vec<Leg>,eps: f64,tol: f64) -> Result<Vec<Brep>,Unbuilt> {
    let boxes: Vec<_> = run.iter().map(|l| l.piece.bounds()).collect();
    let shrunk = |(lo,hi): (V,V)| -> (V,V) { (lo.map(|x| x+eps),hi.map(|x| x-eps)) };
    let meet = |i: usize,j: usize| crate::boolean::overlap(&shrunk(boxes[i]),&shrunk(boxes[j]),0.);
    let closes = |x: &Leg,y: &Leg| norm(sub(x.c1,y.c0)) <= eps && norm(sub(x.t1,y.t0)) <= 1e-9;
    // the runs parted: each a stretch of legs, none of whose boxes meets another's but its neighbours'
    // (and the first's and the last's, where the whole run closes)
    let whole = run.len() > 1 && closes(&run[run.len()-1],&run[0]);
    let mut parts: Vec<std::ops::Range<usize>> = Vec::new();
    let mut from = 0;
    for j in 0..run.len() {
        let clash = (from..j.saturating_sub(1)).any(|i| meet(i,j) && !(whole && from == 0 && i == 0 && j == run.len()-1));
        if clash { parts.push(from..j); from = j; }
    }
    parts.push(from..run.len());
    let closed = whole && parts.len() == 1;
    let mut out = Vec::new();
    let mut legs: Vec<Option<Leg>> = run.into_iter().map(Some).collect();
    for part in parts {
        let first = part.start;
        let last = part.end-1;
        let (a,c0,t0) = { let l = legs[first].as_ref().unwrap(); (l.a,l.c0,l.t0) };
        let (c1,t1) = { let l = legs[last].as_ref().unwrap(); (l.c1,l.t1) };
        let pieces: Vec<Brep> = part.map(|i| legs[i].take().unwrap().piece).collect();
        let glue = |ps: &[Brep]| -> Result<Brep,Unbuilt> {
            let mut it = ps.iter().cloned();
            let mut g = it.next().unwrap();
            for p in it { g = joined(g,p,tol)? }
            Ok(g)
        };
        let tube = if closed {
            let mid = pieces.len()/2;
            joined(glue(&pieces[..mid])?,glue(&pieces[mid..])?,tol)?
        } else {
            let back = revolve(&rev.meridian(&rev.chain,c0,a,cross(a,t0)),c0,a,PI)?;
            let front = revolve(&rev.meridian(&rev.chain,c1,a,cross(t1,a)),c1,a,PI)?;
            let mut g = back;
            for p in pieces { g = joined(g,p,tol)? }
            joined(g,front,tol)?
        };
        out.push(crate::boolean::planes_joined(&tube,tol).map(|b| b.compacted()).unwrap_or(tube));
    }
    Ok(out)
}

/// **Solids united as a balanced tree**: each in turn with its neighbour in the program, then those
/// unions with theirs, and so on — each union local, its two operands near one another, and every
/// pair at a level united on its own core. The cost of a run of Booleans each against the whole is
/// the moves times the part; this is the part's size times the levels.
fn united(mut solids: Vec<Brep>,tol: f64) -> Result<Brep,Unbuilt> {
    while solids.len() > 1 {
        crate::util::meter::go_on()?;
        let pairs: Vec<&[Brep]> = solids.chunks(2).collect();
        let next = crate::util::par::map(&pairs,|pair| -> Result<Brep,String> {
            match pair { [a,b] => boolean(a,b,Op::Union,tol),[a] => Ok(a.clone()),_ => unreachable!() }
        });
        solids = next.into_iter().collect::<Result<_,_>>()?;
    }
    Ok(solids.pop().expect("a solid to unite"))
}

/// A vector square to `a`.
fn any_square(a: V) -> V {
    let x = if a[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
    sub(x,scale(a,dot(x,a)))
}
