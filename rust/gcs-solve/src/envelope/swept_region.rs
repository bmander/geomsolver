//! The region a planar tool sweeps (#606): under a motion that turns a plane in itself, a tool
//! whose section in that plane is a region of lines and arcs (prisms' sections, united and
//! intersected) sweeps `R = ∪ₜ poseₜ(section)` over its roll, and a blank cut by the sweep keeps
//! what lies outside `R`.  `R`'s boundary is read here, within a box about the blank's shadow, as
//! loops of exact pieces:
//!
//! 1. **Candidates.**  Every piece of the section that can be on `R`'s boundary, as a curve of the
//!    roll: a line's contact (the foot of the perpendicular from the instant centre), each side of
//!    an arc's (where its radius points at the instant centre), a corner's path, and — where the
//!    roll is no whole period — the section at each end pose.  A point of one is valid only where
//!    it lies on the section's boundary at its own pose, so an edge inside another member of a
//!    union, or outside one of an intersection, is never a candidate there.
//! 2. **Kept where outside every pose.**  A candidate's image is kept only where it lies outside the
//!    section at every pose of the roll (the section's signed distance, carried back, read on a
//!    grid of the roll and bounded between by the point's speed, `envelope::planar`'s test turned
//!    round).  That is the meaning of the cut: a second contact, a branch of the envelope another
//!    pose removes, a trim at a concave corner — each is dropped by it.
//! 3. **Linked into loops.**  Each kept stretch, ends found by bisection, is walked with `R` on its
//!    left (the tool's outward normal at its contact on its right); a stretch ends where it meets
//!    another (two crossing, solved exactly), where it runs on into another (an edge's contact
//!    reaching its corner, at the same roll), where the roll ends, or where it leaves the box,
//!    which then carries the loop round its sides counter-clockwise to the next stretch entering.
//!    A loop counter-clockwise bounds a piece of `R`, one clockwise a hole in it.
//!
//! What the loops do not decide is refused naming why and where (`Condition::Kept`): a stretch with
//! no partner, a fold, a crossing.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::motion::Family;
use crate::solid::admission::Condition;
use crate::solid::surface::Edge;

type P = [f64;2];
type V = [f64;3];

fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1] }
fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn add(a: P,b: P) -> P { [a[0]+b[0],a[1]+b[1]] }
fn scale(a: P,k: f64) -> P { [a[0]*k,a[1]*k] }
fn length(a: P) -> f64 { a[0].dhypot(a[1]) }
fn dist(a: P,b: P) -> f64 { length(sub(a,b)) }
fn unit(a: P) -> P { let l = length(a); if l > 0. { scale(a,1./l) } else { a } }
/// A quarter turn counter-clockwise.
fn left(a: P) -> P { [-a[1],a[0]] }
/// A quarter turn clockwise: the outward normal of a walk with its material on its left.
fn right(a: P) -> P { [a[1],-a[0]] }

/// Why no region: the row, what was found, and a point of the plane where.
#[derive(Clone,Debug)]
pub struct Refusal { pub row: Condition,pub message: String,pub witness: Option<P> }

fn refuse(row: Condition,message: impl Into<String>,witness: Option<P>) -> Refusal { Refusal {row,message:message.into(),witness} }

fn debug() -> bool { std::env::var_os("SOLVENT_REGION_DEBUG").is_some() }

/// The plane the motion keeps, in space: a point of it, two unit axes in it and its unit normal.
#[derive(Clone,Copy,Debug)]
pub struct Plane { pub o: V,pub u: V,pub v: V,pub n: V }

impl Plane {
    /// The point at `p` of the plane, `h` along its normal.
    pub fn world(&self,p: P,h: f64) -> V { std::array::from_fn(|k| self.o[k]+p[0]*self.u[k]+p[1]*self.v[k]+h*self.n[k]) }
    /// A point of space's shadow on the plane.
    pub fn page(&self,w: V) -> P { let d = crate::space::sub(w,self.o); [crate::space::dot(d,self.u),crate::space::dot(d,self.v)] }
    /// How far along the normal a point of space stands from the plane.
    pub fn height(&self,w: V) -> f64 { crate::space::dot(crate::space::sub(w,self.o),self.n) }
    fn vector(&self,w: V) -> P { [crate::space::dot(w,self.u),crate::space::dot(w,self.v)] }
}

/// How a section's members combine: a member, the union of some, or what they hold in common.
#[derive(Clone,Debug)]
pub enum Tree { Member(usize),Union(Vec<Tree>),Common(Vec<Tree>) }

/// The tool's section at one height, in the plane's coordinates: each member's loops walked with
/// its material on the left (none where the member does not reach the height), and how they
/// combine.
#[derive(Clone,Debug)]
pub struct Section { pub(crate) members: Vec<Option<Vec<Vec<Edge>>>>,pub tree: Tree }

impl Section {
    /// A member's signed distance at `p`: its loops' nearest edge, signed by ray parity; nothing
    /// (+∞) where the member is absent.
    fn member(&self,m: usize,p: P) -> f64 {
        let Some(loops) = &self.members[m] else { return f64::INFINITY };
        let (mut best,mut crossings) = (f64::INFINITY,0);
        for e in loops.iter().flatten() {
            best = best.min(e.distance(p));
            crossings += e.crossings(p);
        }
        if crossings%2 == 1 { -best } else { best }
    }

    fn tree(&self,t: &Tree,p: P) -> f64 {
        match t {
            Tree::Member(m) => self.member(*m,p),
            Tree::Union(ts) => ts.iter().map(|t| self.tree(t,p)).fold(f64::INFINITY,f64::min),
            Tree::Common(ts) => ts.iter().map(|t| self.tree(t,p)).fold(f64::NEG_INFINITY,f64::max),
        }
    }

    /// The section's field at `p`: negative inside, one-Lipschitz, zero on its boundary.
    pub fn value(&self,p: P) -> f64 { self.tree(&self.tree,p) }

    /// Whether no member reaches this height.
    pub fn is_empty(&self) -> bool { self.members.iter().all(Option::is_none) }
}

/// The motion at one roll, in the plane's coordinates: `x ↦ A x + c`, its turning rate `ω` and the
/// velocity of the tool's origin turned back into the tool, `b = Aᵀ ċ` (so a tool point `x` moves
/// at `ω J x + b` in the tool's own frame).
#[derive(Clone,Copy,Debug)]
struct Rigid { a: [[f64;2];2],c: P,omega: f64,b: P }

impl Rigid {
    fn point(&self,x: P) -> P { [self.a[0][0]*x[0]+self.a[0][1]*x[1]+self.c[0],self.a[1][0]*x[0]+self.a[1][1]*x[1]+self.c[1]] }
    fn vector(&self,v: P) -> P { [self.a[0][0]*v[0]+self.a[0][1]*v[1],self.a[1][0]*v[0]+self.a[1][1]*v[1]] }
}

/// A pose turned back: `y ↦ Aᵀ(y − c)`.
#[derive(Clone,Copy,Debug)]
struct Back { a: [[f64;2];2],c: P }

impl Back {
    fn point(&self,y: P) -> P { let d = sub(y,self.c); [self.a[0][0]*d[0]+self.a[1][0]*d[1],self.a[0][1]*d[0]+self.a[1][1]*d[1]] }
}

/// What a point of the region is solved from, read once off the solved sketch: the section, the
/// motion and the plane, the roll and a length the tolerances are taken against.
#[derive(Clone,Debug)]
pub struct Setting { pub section: Section,pub family: Family,pub plane: Plane,pub roll: [f64;2],pub scale: f64 }

impl Setting {
    /// The motion at `theta`, refused where it carries the plane out of itself or turns it over.
    fn rigid(&self,theta: f64) -> Result<Rigid,Refusal> {
        let m = self.family.at(theta).map_err(|e| refuse(Condition::Plane,e,None))?;
        let pl = &self.plane;
        let o = pl.o;
        let c = pl.page(m.point(o));
        let a0 = pl.vector(m.vector(pl.u));
        let a1 = pl.vector(m.vector(pl.v));
        let a = [[a0[0],a1[0]],[a0[1],a1[1]]];
        let dc = pl.vector(m.velocity(o));
        let du = sub(pl.vector(m.velocity(crate::space::add(o,pl.u))),dc);
        // ω from Aᵀ Ȧ = ω J: the first column of Ȧ against A's second
        let omega = cross(a0,du);
        let b = [a[0][0]*dc[0]+a[1][0]*dc[1],a[0][1]*dc[0]+a[1][1]*dc[1]];
        let n = pl.n;
        let off = crate::space::dot(crate::space::sub(m.point(o),o),n);
        let turn = crate::space::dot(m.vector(n),n);
        if off.abs() > 1e-9*self.scale || crate::space::dot(m.velocity(o),n).abs() > 1e-9*self.scale || turn < 1.-1e-9 {
            return Err(refuse(Condition::Plane,format!("`{}` carries the tool out of its plane, or turns it over",self.family.name),None))
        }
        Ok(Rigid {a,c,omega,b})
    }

    /// The pose at `theta` turned back, read off the pose alone.
    fn back(&self,theta: f64) -> Option<Back> {
        let m = self.family.pose_at(theta).ok()?;
        let pl = &self.plane;
        let c = pl.page(m.point(pl.o));
        let a0 = pl.vector(m.vector(pl.u));
        let a1 = pl.vector(m.vector(pl.v));
        Some(Back {a:[[a0[0],a1[0]],[a0[1],a1[1]]],c})
    }

    fn tolerance(&self) -> f64 { 1e-6*(1.+self.scale) }

    /// Points of the section's boundary, `per` an edge, and each a hair inside it.
    fn samples(&self,per: usize) -> Vec<(P,P)> {
        let h = 1e-4*(1.+self.scale);
        self.section.members.iter().flatten().flatten().flatten().flat_map(|e| (0..per).map(move |k| {
            let f = (k as f64+0.5)/per as f64;
            let (x,y) = (e.at(f-1e-6),e.at(f+1e-6));
            let p = e.at(f);
            (p,add(p,scale(unit(left(sub(y,x))),h)))
        })).filter(|&(p,q)| self.section.value(p).abs() <= 1e-9*(1.+self.scale) && self.section.value(q) < 0.).collect()
    }

    /// Whether the section stands where it stood at roll `t0` again at `t1`: the motion between
    /// the two takes it onto itself (a whole period of the pair as the observer sees it, the tool's
    /// own symmetry counted), read at points of its boundary and a hair inside them.
    pub fn alike(&self,t0: f64,t1: f64) -> bool {
        let (Some(b0),Some(m1)) = (self.back(t0),self.family.pose_at(t1).ok()) else { return false };
        let pl = &self.plane;
        let carried = |x: P| b0.point(pl.page(m1.point(pl.world(x,0.))));
        let near = 1e-7*(1.+self.scale);
        let samples = self.samples(8);
        !samples.is_empty() && samples.iter().all(|&(p,q)| self.section.value(carried(p)).abs() <= near && self.section.value(carried(q)) < 0.)
    }

    /// The least roll after `t0`, a degree at a time and within `turns` turns, at which the section
    /// stands again where it stood at `t0` (`alike`).
    pub fn period(&self,t0: f64,turns: f64) -> Option<f64> {
        let step = std::f64::consts::PI/180.;
        let last = (turns*360.) as usize;
        (1..=last).find(|&k| self.alike(t0,t0+step*k as f64)).map(|k| t0+step*k as f64)
    }

    /// The section's points (boundary and a grid inside it) carried by the pose at `t`, in the
    /// plane: what the tool covers at one pose.
    pub fn covered(&self,t: f64,grid: usize) -> Vec<P> {
        let Some(m) = self.family.pose_at(t).ok() else { return Vec::new() };
        let pl = &self.plane;
        let mut inside: Vec<P> = self.samples(16).into_iter().map(|(_,q)| q).collect();
        // a grid over the section's own box
        let (lo,hi) = self.section.members.iter().flatten().flatten().flatten().flat_map(|e| (0..=8).map(move |k| e.at(k as f64/8.)))
            .fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(lo,hi),p| ([lo[0].min(p[0]),lo[1].min(p[1])],[hi[0].max(p[0]),hi[1].max(p[1])]));
        for i in 0..=grid { for j in 0..=grid {
            let p = [lo[0]+(hi[0]-lo[0])*i as f64/grid as f64,lo[1]+(hi[1]-lo[1])*j as f64/grid as f64];
            if self.section.value(p) < 0. { inside.push(p) }
        } }
        inside.into_iter().map(|x| pl.page(m.point(pl.world(x,0.)))).collect()
    }
}

/// A curve the region's boundary may run along, in the tool's frame: a line's contact, one side of
/// an arc's, a corner's path (each a curve of the roll), or an edge of the section held at a pose
/// (a curve of the edge's own parameter, from 0 to 1).
#[derive(Clone,Debug)]
enum Kind {
    /// From `a` along unit `d` for `length`, the material on its left.
    Line { a: P,d: P,length: f64 },
    /// The arc's edge, and which side of its centre the contact stands (the radius toward the
    /// instant centre, or away).
    Arc { edge: Edge,side: f64 },
    /// A corner, a unit direction into the section there, and the outward normals of the two
    /// sides meeting at it: it is in contact only while the line from the instant centre to it
    /// lies within its fan of normals — its velocity crossing one side's normal outward and the
    /// other's inward — and joins a side's contact exactly where that one reaches it.
    Corner { x: P,inward: P,normals: [P;2] },
    /// An edge at the pose of roll `at`.
    Fixed { edge: Edge,at: f64 },
}

/// One point of a candidate: where it is in the plane, and the section's outward normal there.
#[derive(Clone,Copy,Debug)]
struct Hit { at: P,outward: P }

impl Kind {
    /// The candidate's domain: the roll, or an edge's parameter.
    fn domain(&self,roll: [f64;2]) -> [f64;2] { if let Kind::Fixed {..} = self { [0.,1.] } else { roll } }

    /// The candidate at `u`: none where its contact leaves its edge, or is no point of the section's
    /// boundary at its own pose (`on`, the section's tolerance).
    fn at(&self,s: &Setting,u: f64,on: f64) -> Option<Hit> {
        let (x,outward,pose) = match self {
            Kind::Fixed {edge,at} => {
                let x = edge.at(u);
                let t = match *edge {
                    Edge::Line {a,b,..} => unit(sub(b,a)),
                    Edge::Arc {start,sweep,..} => { let (sn,cs) = (start+u*sweep).dsin_cos(); scale([-sn,cs],sweep.signum()) }
                    Edge::Conic {..} => return None,
                };
                (x,right(t),s.rigid(*at).ok()?)
            }
            _ => {
                let r = s.rigid(u).ok()?;
                let (w,b) = (r.omega,r.b);
                match self {
                    Kind::Line {a,d,length} => {
                        if !(w.abs() > 1e-12) { return None }
                        let nu = right(*d);
                        // ν·(ω J x + b) = 0 along x = a + s d, ν·J d being −1
                        let sv = dot(nu,add(scale(left(*a),w),b))/w;
                        if !(0. ..=*length).contains(&sv) { return None }
                        (add(*a,scale(*d,sv)),nu,r)
                    }
                    Kind::Arc {edge,side} => {
                        let Edge::Arc {center,radius,start,sweep,..} = *edge else { return None };
                        // e ⟂ (ω J c + b): along ω c − J b
                        let e = sub(scale(center,w),left(b));
                        if !(length(e) > 0.) { return None }
                        let e = scale(unit(e),*side);
                        let angle = e[1].datan2(e[0]);
                        if ((angle-start)*sweep.signum()).rem_euclid(std::f64::consts::TAU) > sweep.abs() { return None }
                        (add(center,scale(e,radius)),scale(e,sweep.signum()),r)
                    }
                    Kind::Corner {x,inward,normals} => {
                        let v = add(scale(left(*x),w),b);
                        if !(dot(v,normals[0])*dot(v,normals[1]) < 0.) { return None }
                        (*x,scale(*inward,-1.),r)
                    }
                    Kind::Fixed {..} => unreachable!("read above"),
                }
            }
        };
        if s.section.value(x).abs() > on { return None }
        Some(Hit {at:pose.point(x),outward:unit(pose.vector(outward))})
    }
}

/// One piece of a loop: a stretch of a candidate between two of its parameters (walked from `from`
/// to `to`, either way), or a straight step along the box's side.
#[derive(Clone,Copy,Debug)]
pub enum Piece { Run { candidate: usize,from: f64,to: f64 },Side { a: P,b: P } }

/// A loop of the region's boundary within the box, the region on its left: its pieces in order,
/// each corner (the start of piece `k` at `corners[k]`) where they meet, and the area it bounds
/// (positive counter-clockwise: an outer boundary of the region; negative: a hole in it).
#[derive(Clone,Debug)]
pub struct Loop { pub pieces: Vec<Piece>,pub corners: Vec<P>,pub area: f64 }

/// The region's boundary within the box: its loops, and what they are read from.
#[derive(Clone,Debug)]
pub struct SweptRegion { pub setting: Setting,candidates: Vec<Kind>,pub loops: Vec<Loop>,pub bounds: [P;2],
    /// How many candidates were read, and kept stretches found.
    pub read: [usize;2] }

/// Why a stretch of a candidate ends: its roll or parameter ends, its contact leaves its edge (or
/// the section's boundary), another pose covers it, or it leaves the box.
#[derive(Clone,Copy,Debug,PartialEq)]
enum End { Domain,Valid,Kept,Box }

/// A kept stretch: its candidate, its ends' parameters and why each is one, walked from the first.
#[derive(Clone,Copy,Debug)]
struct Run { candidate: usize,ends: [f64;2],why: [End;2] }

/// Samples of the roll per degree, and of a fixed edge.
const PER_DEGREE: f64 = 4.;
const EDGE_SAMPLES: usize = 400;
/// Roll steps a turn of the keeping test's grid takes, and the least stretch it halves to.
const KEEP_PER_TURN: f64 = 1440.;
const KEEP_STEP: f64 = 1e-3;

impl SweptRegion {
    /// The boundary of the region `setting`'s section sweeps over its roll, within `bounds` (the
    /// plane's least and greatest corner of a box about the blank's shadow). `whole` says the roll
    /// is a whole period of the motion, so the section's end poses bound nothing.
    pub fn read(setting: Setting,bounds: [P;2],whole: bool) -> Result<SweptRegion,Refusal> {
        let clock = crate::clock::Instant::now();
        let s = &setting;
        let [a,b] = s.roll;
        if !(a < b) { return Err(refuse(Condition::Period,"the roll increases",None)) }
        // the motion keeps the plane at every grid pose
        let steps = (((b-a)/std::f64::consts::TAU*KEEP_PER_TURN).ceil() as usize).max(64);
        let times: Vec<f64> = (0..=steps).map(|j| a+(b-a)*j as f64/steps as f64).collect();
        for &t in times.iter().step_by(16) { s.rigid(t)?; }
        let backs: Vec<Back> = times.iter().map(|&t| s.back(t).ok_or_else(|| refuse(Condition::Plane,"the motion cannot be read",None)))
            .collect::<Result<_,_>>()?;
        let keep = Keep {setting:s,backs:&backs,times:&times,tol:s.tolerance(),
            domain:crate::interval::Interval::new(a,b).map_err(|_| refuse(Condition::Period,"the roll is no interval",None))?};
        let candidates = candidates(s,whole);
        let on = 1e-9*(1.+s.scale);
        let inside = |p: P| (0..2).all(|k| p[k] >= bounds[0][k] && p[k] <= bounds[1][k]);
        // each candidate sampled over its domain: where it is valid, inside the box and kept
        let state = |k: usize,u: f64| -> Result<(),End> {
            let Some(h) = candidates[k].at(s,u,on) else { return Err(End::Valid) };
            if !inside(h.at) { return Err(End::Box) }
            if !keep.outside(h.at) { return Err(End::Kept) }
            Ok(())
        };
        let samples = |k: usize| -> usize {
            if let Kind::Fixed {..} = candidates[k] { EDGE_SAMPLES } else { (((b-a).to_degrees()*PER_DEGREE).ceil() as usize).max(64) }
        };
        let marks: Vec<Vec<Result<(),End>>> = crate::par::indices(candidates.len(),|k| {
            let [u0,u1] = candidates[k].domain(s.roll);
            let n = samples(k);
            (0..=n).map(|j| state(k,u0+(u1-u0)*j as f64/n as f64)).collect()
        });
        // the kept stretches, their ends found by bisection to the last bit
        let mut runs: Vec<Run> = Vec::new();
        for (k,m) in marks.iter().enumerate() {
            let [u0,u1] = candidates[k].domain(s.roll);
            let n = m.len()-1;
            let u = |j: usize| u0+(u1-u0)*j as f64/n as f64;
            let mut j = 0;
            while j <= n {
                if m[j].is_err() { j += 1; continue }
                let first = j;
                while j < n && m[j+1].is_ok() { j += 1 }
                let last = j;
                j += 1;
                let edge = |inner: f64,off: f64| -> (f64,End) {
                    let (x,y) = crate::roots::bisect(inner,off,|x,y| 0.5*(x+y),|x,y| { let m = 0.5*(x+y); m != x && m != y },
                        |t| state(k,t).is_ok());
                    (x,state(k,y).err().unwrap_or(End::Kept))
                };
                let lo = if first == 0 { (u0,End::Domain) } else { edge(u(first),u(first-1)) };
                let hi = if last == n { (u1,End::Domain) } else { edge(u(last),u(last+1)) };
                runs.push(Run {candidate:k,ends:[lo.0,hi.0],why:[lo.1,hi.1]});
            }
        }
        if debug() { eprintln!("region: {} candidates, {} runs ({:?})",candidates.len(),runs.len(),clock.elapsed()); }
        let point = |k: usize,u: f64| candidates[k].at(s,u,f64::INFINITY);
        // each stretch walked with the region on its left: the outward normal on its right
        for r in &mut runs {
            let mid = 0.5*(r.ends[0]+r.ends[1]);
            let h = 1e-6*(r.ends[1]-r.ends[0]).abs().max(1e-12);
            let (Some(m),Some(x),Some(y)) = (point(r.candidate,mid),point(r.candidate,mid-h),point(r.candidate,mid+h)) else { continue };
            if dot(sub(y.at,x.at),left(m.outward)) < 0. { r.ends.swap(0,1); r.why.swap(0,1); }
        }
        // a fold: a stretch turning back against its own walk
        for r in &runs {
            let n = 32;
            let at: Vec<Hit> = (0..=n).filter_map(|j| point(r.candidate,r.ends[0]+(r.ends[1]-r.ends[0])*j as f64/n as f64)).collect();
            for w in at.windows(2) {
                if dot(sub(w[1].at,w[0].at),left(w[0].outward)) < -1e-9*(1.+s.scale) {
                    return Err(refuse(Condition::Kept,"the swept region's boundary folds back on itself",Some(w[0].at)))
                }
            }
        }
        let ends = |r: &Run| -> Option<[P;2]> { Some([point(r.candidate,r.ends[0])?.at,point(r.candidate,r.ends[1])?.at]) };
        let mut at: Vec<[P;2]> = runs.iter().map(|r| ends(r).ok_or_else(|| refuse(Condition::Kept,"a kept stretch cannot be read at its end",None)))
            .collect::<Result<_,_>>()?;
        if debug() {
            for (r,p) in runs.iter().zip(&at) {
                eprintln!("region: run of {:?} from {:.9} ({:?}) at {:?} to {:.9} ({:?}) at {:?}",candidates[r.candidate],r.ends[0],r.why[0],p[0],
                    r.ends[1],r.why[1],p[1]);
            }
        }
        // each stretch's end joined to the start of the next: where they meet exactly (two
        // crossing, solved together), run on into each other or the box carries one to the next
        let join = 1e-7*(1.+s.scale);
        let loose = 1e-3*(1.+s.scale);
        let mut next: Vec<Option<usize>> = vec![None;runs.len()];
        let mut taken = vec![false;runs.len()];
        let mut pairs: Vec<(f64,usize,usize)> = Vec::new();
        for i in 0..runs.len() {
            if runs[i].why[1] == End::Box { continue }
            for j in 0..runs.len() {
                if runs[j].why[0] == End::Box { continue }
                let d = dist(at[i][1],at[j][0]);
                if d <= loose { pairs.push((d,i,j)) }
            }
        }
        pairs.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
        // an edge held at an end pose joined to the contact the roll ends at: tangent there, so the
        // keeping test finds its end only roughly; the contact is on the edge, and is that end
        let tangent = |fixed: &Run,other: &Run,end: usize,p: P| -> Option<f64> {
            let Kind::Fixed {edge,at} = &candidates[fixed.candidate] else { return None };
            if other.why[1-end] != End::Domain || matches!(candidates[other.candidate],Kind::Fixed {..}) { return None }
            let x = s.back(*at)?.point(p);
            (edge.distance(x) <= join).then(|| parameter(edge,x))
        };
        for (d,i,j) in pairs {
            if next[i].is_some() || taken[j] { continue }
            if let Some(u) = tangent(&runs[j],&runs[i],0,at[i][1]) {
                runs[j].ends[0] = u; at[j][0] = at[i][1];
            } else if let Some(u) = tangent(&runs[i],&runs[j],1,at[j][0]) {
                runs[i].ends[1] = u; at[i][1] = at[j][0];
            } else if d > join {
                // two stretches crossing: where their curves meet, solved for both parameters
                let Some((ui,uj,p)) = meet(&point,runs[i].candidate,runs[i].ends[1],runs[j].candidate,runs[j].ends[0],loose) else { continue };
                runs[i].ends[1] = ui; runs[j].ends[0] = uj;
                at[i][1] = p; at[j][0] = p;
            }
            next[i] = Some(j); taken[j] = true;
        }
        // the box's sides, counter-clockwise from one stretch leaving to the next entering
        let [lo,hi] = bounds;
        let (w,h) = (hi[0]-lo[0],hi[1]-lo[1]);
        let perimeter = 2.*(w+h);
        let around = |p: P| -> f64 {
            let d = [(p[1]-lo[1]).abs(),(p[0]-hi[0]).abs(),(p[1]-hi[1]).abs(),(p[0]-lo[0]).abs()];
            let side = (0..4).min_by(|&a,&b| d[a].total_cmp(&d[b])).expect("four sides");
            match side { 0 => p[0]-lo[0],1 => w+p[1]-lo[1],2 => w+h+hi[0]-p[0],_ => 2.*w+h+hi[1]-p[1] }
        };
        let box_corners = [(w,[hi[0],lo[1]]),(w+h,hi),(2.*w+h,[lo[0],hi[1]]),(perimeter,lo)];
        let entering: Vec<(f64,usize)> = (0..runs.len()).filter(|&j| runs[j].why[0] == End::Box).map(|j| (around(at[j][0]),j)).collect();
        let mut sides: Vec<Option<Vec<P>>> = vec![None;runs.len()];
        for i in 0..runs.len() {
            if runs[i].why[1] != End::Box { continue }
            let from = around(at[i][1]);
            let ahead = |x: f64| (x-from).rem_euclid(perimeter);
            let Some(&(to,j)) = entering.iter().filter(|(_,j)| !taken[*j]).min_by(|x,y| ahead(x.0).total_cmp(&ahead(y.0))) else {
                return Err(refuse(Condition::Kept,"the swept region's boundary leaves the box and never comes back into it",Some(at[i][1])))
            };
            let mut path = vec![at[i][1]];
            let mut corners: Vec<(f64,P)> = box_corners.iter().flat_map(|&(c,p)| [(c,p),(c+perimeter,p)]).filter(|&(c,_)| c > from && c-from < ahead(to)).collect();
            corners.sort_by(|x,y| x.0.total_cmp(&y.0));
            path.extend(corners.into_iter().map(|(_,p)| p));
            path.push(at[j][0]);
            sides[i] = Some(path);
            next[i] = Some(j); taken[j] = true;
        }
        // the loops, walked from each stretch not yet in one
        let mut loops: Vec<Loop> = Vec::new();
        let mut used = vec![false;runs.len()];
        for first in 0..runs.len() {
            if used[first] { continue }
            let mut pieces: Vec<Piece> = Vec::new();
            let mut corners: Vec<P> = Vec::new();
            let mut i = first;
            loop {
                used[i] = true;
                let r = runs[i];
                corners.push(at[i][0]);
                pieces.push(Piece::Run {candidate:r.candidate,from:r.ends[0],to:r.ends[1]});
                if let Some(path) = &sides[i] {
                    for w in path.windows(2) {
                        if dist(w[0],w[1]) > join { corners.push(w[0]); pieces.push(Piece::Side {a:w[0],b:w[1]}); }
                    }
                }
                let Some(j) = next[i] else {
                    return Err(refuse(Condition::Kept,"the swept region's boundary does not close: a kept stretch meets no other where it \
                        ends",Some(at[i][1])))
                };
                if j == first { break }
                if used[j] { return Err(refuse(Condition::Kept,"the swept region's boundary branches",Some(at[j][0]))) }
                i = j;
            }
            // a sliver a few rounding errors long (where three sides meet at an end pose) is no piece:
            // its neighbours meet at its start
            let tiny = 1e-6*(1.+s.scale);
            let mut k = 0;
            while pieces.len() > 3 && k < pieces.len() {
                let n = pieces.len();
                if dist(corners[k],corners[(k+1)%n]) > tiny { k += 1; continue }
                pieces.remove(k);
                // the last piece's end is the first's start: that one starts where the sliver did
                if k == n-1 { corners[0] = corners[k]; corners.pop(); } else { corners.remove(k+1); }
            }
            loops.push(Loop {pieces,corners,area:0.});
        }
        // a box the region holds whole, where no stretch reaches its sides
        if loops.iter().all(|l| l.pieces.iter().all(|p| matches!(p,Piece::Run {..}))) && !keep.outside(lo) {
            let c = [lo,[hi[0],lo[1]],hi,[lo[0],hi[1]]];
            loops.push(Loop {pieces:(0..4).map(|k| Piece::Side {a:c[k],b:c[(k+1)%4]}).collect(),corners:c.to_vec(),area:0.});
        }
        let read = [candidates.len(),runs.len()];
        let mut region = SweptRegion {setting,candidates,loops,bounds,read};
        for k in 0..region.loops.len() {
            let pts = region.polyline(k,16);
            region.loops[k].area = pts.iter().zip(pts.iter().cycle().skip(1)).map(|(p,q)| cross(*p,*q)).sum::<f64>()/2.;
        }
        region.crossings()?;
        if debug() {
            for l in &region.loops { eprintln!("region: a loop of {} pieces, area {:.6}",l.pieces.len(),l.area); }
            eprintln!("region: {} loops ({:?})",region.loops.len(),clock.elapsed());
        }
        Ok(region)
    }

    /// Piece `piece` of loop `l` at fraction `f` from its start: exact on a candidate (the corners
    /// at its ends), straight along the box.
    pub fn point(&self,l: usize,piece: usize,f: f64) -> Option<P> {
        let lp = &self.loops[l];
        let n = lp.pieces.len();
        if f <= 0. { return Some(lp.corners[piece]) }
        if f >= 1. { return Some(lp.corners[(piece+1)%n]) }
        match lp.pieces[piece] {
            Piece::Run {candidate,from,to} => self.candidates[candidate].at(&self.setting,from+(to-from)*f,f64::INFINITY).map(|h| h.at),
            Piece::Side {a,b} => Some(add(a,scale(sub(b,a),f))),
        }
    }

    /// Loop `l` as points, `per` a piece, in order round it.
    pub fn polyline(&self,l: usize,per: usize) -> Vec<P> {
        let n = self.loops[l].pieces.len();
        (0..n).flat_map(|k| (0..per).filter_map(move |j| self.point(l,k,j as f64/per as f64))).collect()
    }

    /// Whether a point of the plane lies inside the swept region, by the loops' ray parity; the
    /// region is read within the box only.
    pub fn holds(&self,p: P,per: usize) -> bool {
        let mut crossings = 0;
        for l in 0..self.loops.len() {
            let pts = self.polyline(l,per);
            for (x,y) in pts.iter().zip(pts.iter().cycle().skip(1)) {
                if (x[1] > p[1]) != (y[1] > p[1]) && p[0] < x[0]+(p[1]-x[1])*(y[0]-x[0])/(y[1]-x[1]) { crossings += 1 }
            }
        }
        crossings%2 == 1
    }

    /// No two chords of the loops meet but at a shared corner.
    fn crossings(&self) -> Result<(),Refusal> {
        let per = 24;
        let chords: Vec<(P,P)> = (0..self.loops.len()).flat_map(|l| {
            let pts = self.polyline(l,per);
            let n = pts.len();
            (0..n).map(move |k| (pts[k],pts[(k+1)%n])).collect::<Vec<_>>()
        }).filter(|(a,b)| dist(*a,*b) > 0.).collect();
        let corners: Vec<P> = self.loops.iter().flat_map(|l| l.corners.iter().copied()).collect();
        let near = 1e-6*(1.+self.setting.scale);
        let boxed = |(a,b): (P,P)| crate::bvh::Bounds {lo:[a[0].min(b[0]),a[1].min(b[1])],hi:[a[0].max(b[0]),a[1].max(b[1])]};
        let tree = crate::bvh::Bvh::new(chords.iter().map(|&c| boxed(c)));
        for i in 0..chords.len() {
            let mut crossed = None;
            tree.query(|b| b.overlaps(boxed(chords[i])),|j| {
                if crossed.is_some() || j <= i { return }
                let ((a,b),(c,d)) = (chords[i],chords[j]);
                if [a,b].iter().any(|&p| dist(p,c) <= near || dist(p,d) <= near) { return }
                let (d1,d2) = (cross(sub(b,a),sub(c,a)),cross(sub(b,a),sub(d,a)));
                let (d3,d4) = (cross(sub(d,c),sub(a,c)),cross(sub(d,c),sub(b,c)));
                let at_corner = || corners.iter().any(|&k| [a,b,c,d].iter().any(|&p| dist(p,k) <= 1e3*near));
                if d1*d2 < 0. && d3*d4 < 0. && !at_corner() { crossed = Some(a) }
            });
            if let Some(p) = crossed { return Err(refuse(Condition::Kept,"the swept region's boundary crosses itself",Some(p))) }
        }
        Ok(())
    }
}

/// Where candidates `i` near parameter `ui` and `j` near `uj` meet: Newton on both parameters,
/// kept within `reach` of where it started.
fn meet(point: &dyn Fn(usize,f64) -> Option<Hit>,i: usize,ui: f64,j: usize,uj: f64,reach: f64) -> Option<(f64,f64,P)> {
    let (mut x,mut y) = (ui,uj);
    let start = point(i,x)?.at;
    for _ in 0..40 {
        let (p,q) = (point(i,x)?.at,point(j,y)?.at);
        let f = sub(p,q);
        if length(f) <= 1e-13*(1.+length(p)) { return Some((x,y,scale(add(p,q),0.5))) }
        let (hx,hy) = (1e-7*(1.+x.abs()),1e-7*(1.+y.abs()));
        let dp = scale(sub(point(i,x+hx)?.at,point(i,x-hx)?.at),0.5/hx);
        let dq = scale(sub(point(j,y+hy)?.at,point(j,y-hy)?.at),0.5/hy);
        // p(x) − q(y) = 0: [dp, −dq] [δx, δy] = −f
        let det = cross(dp,scale(dq,-1.));
        if !(det.abs() > 1e-300) { return None }
        let dx = cross(scale(f,-1.),scale(dq,-1.))/det;
        let dy = cross(dp,scale(f,-1.))/det;
        x += dx; y += dy;
        if dist(point(i,x)?.at,start) > reach { return None }
    }
    let (p,q) = (point(i,x)?.at,point(j,y)?.at);
    (dist(p,q) <= 1e-9*(1.+length(p))).then(|| (x,y,scale(add(p,q),0.5)))
}

/// Every curve the section's boundary may sweep: each member's edges (an arc's two sides), each
/// corner — a member's, or where two members' edges cross on the section's boundary — and, where
/// the roll is no whole period, every edge held at each end pose.
fn candidates(s: &Setting,whole: bool) -> Vec<Kind> {
    let section = &s.section;
    let on = 1e-9*(1.+s.scale);
    let edges: Vec<(usize,&Edge)> = section.members.iter().enumerate()
        .flat_map(|(m,l)| l.iter().flatten().flatten().map(move |e| (m,e))).collect();
    let mut out: Vec<Kind> = Vec::new();
    let mut corners: Vec<(P,[P;2])> = Vec::new();
    for loops in section.members.iter().flatten() {
        for l in loops {
            for (k,e) in l.iter().enumerate() {
                match *e {
                    Edge::Line {a,b,..} => out.push(Kind::Line {a,d:unit(sub(b,a)),length:dist(a,b)}),
                    Edge::Arc {..} => for side in [1.,-1.] { out.push(Kind::Arc {edge:e.clone(),side}) },
                    Edge::Conic {..} => {}
                }
                // a vertex where the sides go on tangent (a whole circle's seam) is no corner
                let before = &l[(k+l.len()-1)%l.len()];
                let (ta,tb) = (tangent(before,before.at(1.)),tangent(e,e.at(0.)));
                if cross(ta,tb).abs() <= 1e-9 && dot(ta,tb) > 0. { continue }
                corners.push((e.at(0.),[right(ta),right(tb)]));
            }
        }
    }
    // where two members' edges cross on the section's boundary: a corner of an intersection
    for i in 0..edges.len() {
        for j in i+1..edges.len() {
            if edges[i].0 == edges[j].0 { continue }
            for p in crossing(edges[i].1,edges[j].1) {
                if section.value(p).abs() <= 1e3*on { corners.push((p,[right(tangent(edges[i].1,p)),right(tangent(edges[j].1,p))])) }
            }
        }
    }
    let size = 1e-9*(1.+s.scale);
    let mut seen: Vec<P> = Vec::new();
    for (x,_) in corners {
        if seen.iter().any(|&y| dist(x,y) <= size) { continue }
        seen.push(x);
        if let Some((inward,normals)) = fan(section,x,1e-6*(1.+s.scale)) { out.push(Kind::Corner {x,inward,normals}) }
    }
    if !whole {
        for at in s.roll {
            for (_,e) in &edges { out.push(Kind::Fixed {edge:(*e).clone(),at}) }
        }
    }
    out
}

/// An edge's unit tangent, the way it is walked, at its point `p`.
fn tangent(e: &Edge,p: P) -> P {
    match *e {
        Edge::Line {a,b,..} => unit(sub(b,a)),
        Edge::Arc {center,sweep,..} => scale(unit(left(sub(p,center))),sweep.signum()),
        Edge::Conic {..} => [1.,0.],
    }
}

/// The parameter of edge `e` (0 to 1 along it) nearest `x`.
fn parameter(e: &Edge,x: P) -> f64 {
    match *e {
        Edge::Line {a,b,..} => { let d = sub(b,a); (dot(sub(x,a),d)/dot(d,d)).clamp(0.,1.) }
        Edge::Arc {center,start,sweep,..} => {
            let angle = (x[1]-center[1]).datan2(x[0]-center[0]);
            let turned = ((angle-start)*sweep.signum()).rem_euclid(std::f64::consts::TAU);
            (turned/sweep.abs()).clamp(0.,1.)
        }
        Edge::Conic {..} => 0.,
    }
}

/// The section's two sides at a corner `x`, read on a circle of radius `r` about it (so a corner
/// where members meet is read as the section has it, whichever edges make it): a unit direction
/// into the section, and each side's outward normal. None where the section does not turn there.
fn fan(section: &Section,x: P,r: f64) -> Option<(P,[P;2])> {
    let n = 720;
    let dir = |a: f64| { let (sn,cs) = a.dsin_cos(); [cs,sn] };
    let inside = |a: f64| section.value(add(x,scale(dir(a),r))) < 0.;
    let step = std::f64::consts::TAU/n as f64;
    let marks: Vec<bool> = (0..n).map(|k| inside(k as f64*step)).collect();
    let changes: Vec<usize> = (0..n).filter(|&k| marks[k] != marks[(k+1)%n]).collect();
    let [c0,c1] = changes[..] else { return None };
    // each side's direction, bisected onto where the section's boundary leaves the corner
    let side = |k: usize| {
        let (a,b) = crate::roots::bisect(k as f64*step,(k+1) as f64*step,|a,b| 0.5*(a+b),|a,b| b-a > 1e-12,|a| inside(a) == marks[k]);
        0.5*(a+b)
    };
    // the material runs counter-clockwise from `first` to `second`
    let (first,second) = if marks[(c0+1)%n] { (side(c0),side(c1)) } else { (side(c1),side(c0)) };
    let (t1,t2) = (dir(first),dir(second));
    let turn = (second-first).rem_euclid(std::f64::consts::TAU);
    if (turn-std::f64::consts::PI).abs() < 1e-6 { return None }
    let mid = dir(first+turn/2.);
    Some((mid,[right(t1),left(t2)]))
}

/// Where two edges cross (a line's or an arc's carrier met within both).
fn crossing(a: &Edge,b: &Edge) -> Vec<P> {
    let carrier = |e: &Edge| -> Option<(P,P,f64)> { match *e {
        Edge::Line {a,b,..} => Some((a,sub(b,a),0.)),
        Edge::Arc {center,radius,..} => Some((center,[0.,0.],radius)),
        Edge::Conic {..} => None,
    } };
    let (Some(x),Some(y)) = (carrier(a),carrier(b)) else { return Vec::new() };
    let mut points: Vec<P> = Vec::new();
    match (x.2 == 0.,y.2 == 0.) {
        (true,true) => {
            let w = cross(x.1,y.1);
            if w.abs() > 1e-15 { points.push(add(x.0,scale(x.1,cross(sub(y.0,x.0),y.1)/w))) }
        }
        (true,false) | (false,true) => {
            let ((p,d,_),(c,_,r)) = if x.2 == 0. { (x,y) } else { (y,x) };
            let d = unit(d);
            let f = sub(p,c);
            let (bb,k) = (2.*dot(f,d),dot(f,f)-r*r);
            let disc = bb*bb-4.*k;
            if disc >= 0. { let q = disc.sqrt(); points.extend([add(p,scale(d,(-bb-q)/2.)),add(p,scale(d,(-bb+q)/2.))]) }
        }
        (false,false) => {
            let (c0,r0,c1,r1) = (x.0,x.2,y.0,y.2);
            let d = sub(c1,c0);
            let l = length(d);
            if l > 0. {
                let aa = (r0*r0-r1*r1+l*l)/(2.*l);
                let h2 = r0*r0-aa*aa;
                if h2 >= 0. {
                    let m = add(c0,scale(d,aa/l));
                    let n = scale(left(d),h2.sqrt()/l);
                    points.extend([add(m,n),sub(m,n)]);
                }
            }
        }
    }
    // within both edges: a point on an edge is no farther from it than rounding
    points.into_iter().filter(|&p| a.distance(p) <= 1e-9*(1.+length(p)) && b.distance(p) <= 1e-9*(1.+length(p))).collect()
}

/// What decides that a point of the plane is kept: outside the section at every pose of the roll.
struct Keep<'a> { setting: &'a Setting,backs: &'a [Back],times: &'a [f64],tol: f64,domain: crate::interval::Interval }

impl Keep<'_> {
    /// Whether `y` lies outside the section at every pose, to `tol`: the section's signed distance
    /// at `y` carried back, read on the grid's poses and between two readings bounded below by
    /// the lesser less the point's speed times half the step (the distance is one-Lipschitz), the
    /// stretch halved where that bound is not yet above `-tol` — down to `KEEP_STEP` of the roll,
    /// where both readings within it are taken for the stretch: at a contact the distance is zero
    /// and flat, and a Lipschitz bound would halve to the roundoff to clear it.
    fn outside(&self,y: P) -> bool {
        let s = self.setting;
        let world = s.plane.world(y,0.);
        let Ok(speed) = s.family.inverse_point_speed_bound(world,self.domain) else { return false };
        let read = |b: &Back| s.section.value(b.point(y));
        let n = self.backs.len();
        let stride = 32.min(n-1);
        let mut marks: Vec<usize> = (0..n).step_by(stride).collect();
        if *marks.last().expect("a mark") != n-1 { marks.push(n-1) }
        let mut values = Vec::with_capacity(marks.len());
        for &j in &marks {
            let v = read(&self.backs[j]);
            if v < -self.tol { return false }
            values.push(v);
        }
        let mut stack: Vec<(f64,f64,f64,f64)> = marks.windows(2).zip(values.windows(2))
            .map(|(m,v)| (self.times[m[0]],self.times[m[1]],v[0],v[1])).collect();
        while let Some((a,b,va,vb)) = stack.pop() {
            if va.min(vb)-speed*(b-a)/2. >= -self.tol { continue }
            if b-a <= KEEP_STEP { continue }
            let m = 0.5*(a+b);
            let Some(back) = s.back(m) else { return false };
            let vm = read(&back);
            if vm < -self.tol { return false }
            stack.push((a,m,va,vm));
            stack.push((m,b,vm,vb));
        }
        true
    }
}
