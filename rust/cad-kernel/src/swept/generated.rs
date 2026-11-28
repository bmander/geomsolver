//! **A thread hobbing a wheel: each tool point touches once, in closed form** (brep-bench's worm
//! wheel): the candidates of the sweep of a tool a screw carries onto itself — a worm's thread, its
//! section a closed loop of generators screwed about its axis and cut off square to it at its ends
//! — turned about that axis while it is seen from a turn, over a roll after which both stand as
//! they began.
//!
//! **The thread's turn is a slide.** Turning a thread by `φ` about its axis moves it as sliding it
//! by `−kφ` along the axis does (`k` its rise a radian): the screw carries the endless thread `T∞`
//! onto itself. So wherever its ends play no part, the worm turning at `ρ` and advancing at `α`
//! sweeps what `T∞` slid at `δ = α − kρ` does, and the motion seen from the observer's turn `O` is
//! `M(t) = O(t)⁻¹ · slide(t δ)`: a rack's against the blank it cuts. Under it the velocity of the
//! tool point `x` is affine in the roll, `V_t(x) = V₀(x) + t (V₁(x) − V₀(x))`, so `n · V_t(x)`
//! has one root (`generating`'s chart, a line in the roll): **every point of a face touches the
//! sweep's boundary at one instant**, read from two twists, and the face's generated sheet is
//! `G(x) = M(t(x)) x` — exact at every point, with nothing traced and no root sought.
//!
//! - **A face** of `T∞` (a generator screwed) generates the sheet `G(u, φ)` over the generator's
//!   run `u` and the screw's turn `φ`.
//! - **A convex edge** (a corner of the section screwed) bounds the sweep between the instants its
//!   two faces touch at its point, `n₁ · V` and `n₂ · V` of opposite signs there: its fan is the
//!   point carried over them, `F(φ, λ) = M((1 − λ) t₁ + λ t₂) e(φ)`, meeting each face's sheet
//!   along that face's rail, tangent there.
//!
//! **One rack tooth cuts one tooth space**: the screw's next turn is `T∞` slid by its lead `L`, so
//! `G(u, φ + 2π) = O(L/δ) · G(u, φ)`, the same sheet turned on by one pitch of the wheel. The
//! section's elements at one turn `φ` are one tooth of the rack, read over the one window of `φ`
//! whose sheets reach the blank (`Reach`) in each period, and every copy is that one turned. The
//! elements reaching the blank, in the section's order, are one candidate: their sheets fitted on
//! one grid of `φ`, each rail between two of them one curve, so the candidate is closed along its
//! rails by construction and no meeting between its faces is sought (the trim meets no two faces
//! of one candidate). Where one copy's turns about the observer's axis within the reach span less
//! than a pitch, by `CLEAR` of it either way, its sector holds it alone (`Sector`): the caller
//! trims that one copy within the sector and patterns what it cuts, the copies being the turn's.
//!
//! **Where it holds, and what it asks.** The thread's faces screws of one line and rise, its ends
//! planes square to its axis (`Thread::read`); the motion a turn (or a screw) about that line seen
//! from a turn, the roll whole turns of the observer and whole leads of the slide, so the sweep is
//! the endless motion's (`Rack::read`); and the ends of the thread clear of the blank's reach,
//! read in closed form: the cylinder about the thread's axis holding it meets the reach — a
//! cylinder about the observer's axis, square to it — within the thread's length
//! (`Rack::ends_clear`). Any other is not this road's (`None`), and the general road is asked. A
//! concave edge in the reach (the sheets of its two faces crossing, not meeting along a rail) is
//! refused by name; a sheet that folds within the reach is left to the trim's membership and the
//! judge, which refuse it.
use crate::geom::{off_axis,Carry,Curve,Frame,Rigid,Surface};
use crate::nurbs::{interpolate,BSpline,Net};
use crate::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::trajectory::{Screw,Trajectory,Twist};
use crate::util::space::{cross,distance,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use super::trim::Reach;
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];

/// A generator of the thread's section: its curve, run from `t[0]` to `t[1]` the way the section's
/// loop goes, and the sign that turns `X_u × X_φ` outward.
#[derive(Clone,Debug)]
struct Generator { curve: Curve,t: [f64;2],sign: f64 }

/// **A tool a screw carries onto itself, cut off at its ends** (`read`): the screw's line (through
/// `centre` along the unit `axis`) and its `rise` along the axis a radian of turn; its section's
/// generators in the order its loop runs, corner to corner; the span of its ends along the axis
/// from `centre`; and how far it reaches from the axis.
#[derive(Clone,Debug)]
pub struct Thread { centre: V,axis: V,rise: f64,generators: Vec<Generator>,ends: [f64;2],reach: f64 }

impl Thread {
    /// **The thread `tool` is, read from its B-rep**: every face either a screw's sheet of one
    /// generator (`Surface::Blend` under `Carry::Screw`), all on one line at one rise, or a plane
    /// square to that line (an end); the generators joined end to end into one closed loop, the
    /// thread's section. None for any other tool.
    pub fn read(tool: &Brep) -> Option<Thread> {
        let eps = 1e-9*(1.+tool.size());
        let mut screw: Option<(V,V,f64)> = None;
        let mut found: Vec<Generator> = Vec::new();
        let mut caps: Vec<(V,V)> = Vec::new();
        for f in &tool.faces {
            match &f.surface {
                Surface::Blend(_,b) => {
                    let Carry::Screw {center,axis,angle,advance} = b.carry else { return None };
                    if angle == 0. || advance == 0. { return None }
                    // (an axis the other way round turns the angle and the advance both: the rise a
                    // radian along the axis the same)
                    let (a,rise) = (unit(axis),advance/angle);
                    match screw {
                        None => screw = Some((center,a,rise)),
                        Some((c0,a0,r0)) => {
                            if (dot(a,a0).abs()-1.).abs() > 1e-12 || off_axis(center,c0,a0) > eps
                                || (rise-r0).abs() > 1e-12*(1.+r0.abs()) { return None }
                        }
                    }
                    let (p,q) = (b.a.point(b.ta[0]),b.a.point(b.ta[1]));
                    if found.iter().any(|g| {
                        let (x,y) = (g.curve.point(g.t[0]),g.curve.point(g.t[1]));
                        (distance(x,p) <= eps && distance(y,q) <= eps) || (distance(x,q) <= eps && distance(y,p) <= eps)
                    }) { continue }
                    // the sign turning `X_u × X_φ` outward, read where the face's own surface is
                    let (centre,a,rise) = screw.expect("a screw read");
                    let carry = Carry::Screw {center:centre,axis:a,angle:1.,advance:rise};
                    let (s,phi) = (0.5*(b.ta[0]+b.ta[1]),0.5*angle);
                    let (x0,dx) = b.a.d1(s);
                    let x = carry.at(x0,phi);
                    let raw = cross(carry.dir(scale(dx,b.ta[1]-b.ta[0]),phi),carry.rate(x));
                    let n = f.surface.normal([0.5,0.5])?;
                    let outward = if f.reversed { scale(n,-1.) } else { n };
                    let sign = dot(raw,outward).signum();
                    if sign == 0. { return None }
                    found.push(Generator {curve:b.a.clone(),t:b.ta,sign});
                }
                Surface::Plane(frame) => caps.push((frame.o,frame.z)),
                _ => return None,
            }
        }
        let (centre,axis,rise) = screw?;
        // the ends: planes square to the axis
        let mut ends = [f64::INFINITY,f64::NEG_INFINITY];
        for (o,z) in caps {
            if (dot(unit(z),axis).abs()-1.).abs() > 1e-9 { return None }
            let h = dot(sub(o,centre),axis);
            ends = [ends[0].min(h),ends[1].max(h)];
        }
        if !(ends[0] < ends[1]) { return None }
        // the section's loop: each generator run on from where the last ended
        let mut generators = vec![found.swap_remove(0)];
        while !found.is_empty() {
            let last = generators.last().expect("a generator");
            let end = last.curve.point(last.t[1]);
            let k = found.iter().position(|g| distance(g.curve.point(g.t[0]),end) <= eps || distance(g.curve.point(g.t[1]),end) <= eps)?;
            let mut g = found.swap_remove(k);
            if distance(g.curve.point(g.t[0]),end) > eps { g.t = [g.t[1],g.t[0]]; g.sign = -g.sign; }
            generators.push(g);
        }
        let (first,last) = (&generators[0],generators.last().expect("a generator"));
        if generators.len() < 2 || distance(first.curve.point(first.t[0]),last.curve.point(last.t[1])) > eps { return None }
        let reach = generators.iter().map(|g| match &g.curve {
            Curve::Line {..} => off_axis(g.curve.point(g.t[0]),centre,axis).max(off_axis(g.curve.point(g.t[1]),centre,axis)),
            c => { let (lo,hi) = c.bounds(g.t); super::closed::corners_of((lo,hi)).into_iter().map(|p| off_axis(p,centre,axis)).fold(0.,f64::max) }
        }).fold(0.,f64::max);
        Some(Thread {centre,axis,rise,generators,ends,reach})
    }

    /// The screw carrying the thread onto itself, `φ` its turn in radians.
    fn screw(&self) -> Carry { Carry::Screw {center:self.centre,axis:self.axis,angle:1.,advance:self.rise} }

    /// Generator `g`'s point at the fraction `u` of its run, turned by `φ`: the point, its rate
    /// along `u`, and the thread's outward normal there.
    fn at(&self,g: usize,u: f64,phi: f64) -> (V,V,V) {
        let gen = &self.generators[g];
        let (p,dp) = gen.curve.d1(gen.t[0]+u*(gen.t[1]-gen.t[0]));
        let c = self.screw();
        let x = c.at(p,phi);
        let xu = c.dir(scale(dp,gen.t[1]-gen.t[0]),phi);
        (x,xu,scale(unit(cross(xu,c.rate(x))),gen.sign))
    }

    /// Corner `k` of the section (where generator `k` starts) turned by `φ`.
    fn corner(&self,k: usize,phi: f64) -> V {
        let g = &self.generators[k];
        self.screw().at(g.curve.point(g.t[0]),phi)
    }
}

/// **The motion a rack's** (`read`): the thread's turn read as its slide, seen from the observer's
/// turn (`path`, its pose `O(t)⁻¹ · slide(t δ)`), the twists it moves the tool by at the roll's two
/// ends (`n · V` affine between them), the roll between the period's copies (`step`, `L/δ`) and how
/// many distinct copies the observer's turn makes of one (`copies`).
pub struct Rack { path: Trajectory,observer: Screw,ends: [(f64,Twist);2],step: f64,copies: usize }

impl Rack {
    /// The rack `path` is to `thread` over `roll`: `path` a screw about the thread's line seen from
    /// a turn, the roll whole turns of that turn and whole leads of the slide the screw is to the
    /// thread, the slide moving the thread. None for any other motion.
    pub fn read(thread: &Thread,path: &Trajectory,roll: [f64;2]) -> Result<Option<Rack>,Unbuilt> {
        let Trajectory::Relative {source,observer} = path else { return Ok(None) };
        let (Trajectory::Twist(s),Trajectory::Twist(o)) = (&**source,&**observer) else { return Ok(None) };
        let eps = 1e-9*(1.+norm(thread.centre)+thread.reach);
        let along = dot(unit(s.axis),thread.axis);
        if (along.abs()-1.).abs() > 1e-12 || off_axis(s.origin,thread.centre,thread.axis) > eps { return Ok(None) }
        // the screw's turn read as the slide it is to the thread
        let delta = s.advance/TAU*along-thread.rise*s.ratio*along;
        if delta == 0. || o.ratio == 0. || o.advance != 0. { return Ok(None) }
        let length = (roll[1]-roll[0]).abs();
        let lead = TAU*thread.rise;
        let whole = |x: f64| x >= 0.5 && (x-x.round()).abs() <= 1e-9*x.max(1.);
        // (the roll whole turns of the observer and whole leads of the slide: the endless motion's sweep)
        if !whole(o.ratio.abs()*length/TAU) || !whole(length*delta.abs()/lead.abs()) { return Ok(None) }
        let step = lead/delta;
        let per = TAU/(o.ratio*step).abs();
        if !whole(per) { return Ok(None) }
        let slide = Screw {origin:thread.centre,axis:thread.axis,ratio:0.,advance:TAU*delta};
        let path = Trajectory::Relative {source:Box::new(Trajectory::Twist(slide)),observer:Box::new(Trajectory::Twist(*o))};
        let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
        let ends = [(a,Twist::body(path.at(a)?)),(b,Twist::body(path.at(b)?))];
        Ok(Some(Rack {path,observer:*o,ends,step,copies:per.round() as usize}))
    }

    /// **The instant the point `x` with outward normal `n` touches**: the root of `n · V_t(x)`,
    /// affine in `t`; none where it does not change (the point slides along itself throughout).
    fn contact(&self,x: V,n: V) -> Option<f64> {
        let [(a,wa),(b,wb)] = &self.ends;
        let (fa,fb) = (dot(n,wa.velocity(x)),dot(n,wb.velocity(x)));
        let t = a+(b-a)*fa/(fa-fb);
        t.is_finite().then_some(t)
    }

    /// `x` carried to the instant `t`.
    fn carried(&self,x: V,t: f64) -> Result<V,Unbuilt> { Ok(self.path.pose(t)?.point(x)) }

    /// The `m`th copy of the period's sheets: the observer's pose `m` steps on.
    fn copy(&self,m: usize) -> Result<Rigid,Unbuilt> {
        let q = Trajectory::Twist(self.observer).pose(m as f64*self.step)?;
        Ok(Rigid {r:q.r,t:q.p})
    }

    /// **Whether the thread's ends play no part within `reach`**: the cylinder about the thread's
    /// axis holding it, met with the reach — a cylinder about the observer's axis, square to the
    /// thread's — stands within the thread's length, so wherever the slid thread enters the reach
    /// the endless one is the same. Read in closed form: in the plane square to the observer's
    /// axis, the thread's band about its axis met with the reach's disc, at the height where the
    /// thread's cylinder is widest within the reach's span.
    fn ends_clear(&self,thread: &Thread,reach: &Reach) -> bool {
        let Reach::Turned {o,z,r,span} = *reach else { return false };
        let (c,a) = (thread.centre,thread.axis);
        if dot(a,z).abs() > 1e-12 { return false }
        // the thread's axis's height along the observer's, and the height within the span nearest it
        let h = dot(sub(c,o),z);
        let near = h.clamp(span[0],span[1]);
        let wide = thread.reach*thread.reach-(near-h)*(near-h);
        if wide < 0. { return true }
        let wide = wide.sqrt();
        // in the plane: the axis's offset from the observer's across it, and the nearest offset in the band
        let across = unit(cross(z,a));
        let e = dot(sub(c,o),across);
        let m = if e-wide > 0. { e-wide } else if e+wide < 0. { e+wide } else { 0. };
        if m.abs() > r { return true }
        let s = (r*r-m*m).sqrt();
        let at = dot(sub(c,o),a);
        let [lo,hi] = thread.ends;
        // (the reach's chord along the thread's axis, from the thread's centre)
        -s-at >= lo && s-at <= hi
    }
}

/// An element of the thread's section: a face (generator `k` screwed) or the fan of corner `k`
/// (where generator `k` starts).
#[derive(Clone,Copy,Debug,PartialEq)]
enum Element { Face(usize),Fan(usize) }

/// What a corner of the section does at one turn `φ`: its point, the instants its two faces touch
/// there (the face before it, then the face it starts), and where each carries it.
#[derive(Clone,Copy,Debug)]
struct Corner { point: V,times: [f64;2],rails: [V;2] }

/// The road's reading at one turn: each corner's.
fn corners(thread: &Thread,rack: &Rack,phi: f64) -> Result<Vec<Corner>,Unbuilt> {
    let m = thread.generators.len();
    (0..m).map(|k| {
        let point = thread.corner(k,phi);
        let before = thread.at((k+m-1)%m,1.,phi).2;
        let after = thread.at(k,0.,phi).2;
        let times = [rack.contact(point,before),rack.contact(point,after)];
        let [Some(t0),Some(t1)] = times else { return Err(Unbuilt::Curved {at:point}) };
        Ok(Corner {point,times:[t0,t1],rails:[rack.carried(point,t0)?,rack.carried(point,t1)?]})
    }).collect()
}

/// Whether the corner `k` is convex at `φ`: the face it starts leaves it behind the face before it.
fn convex(thread: &Thread,k: usize,phi: f64) -> bool {
    let m = thread.generators.len();
    let before = thread.at((k+m-1)%m,1.,phi).2;
    let into = thread.at(k,0.,phi).1;
    dot(before,into) < 0.
}

/// **Element `el`'s sheet at `w` across it and turn `φ`**, `at` the corners there: its rails
/// exactly the corners' (so two elements sharing one read the same points there).
fn sheet_point(thread: &Thread,rack: &Rack,el: Element,w: f64,phi: f64,at: &[Corner]) -> Result<V,Unbuilt> {
    let m = thread.generators.len();
    match el {
        Element::Face(k) => {
            if w == 0. { return Ok(at[k].rails[1]) }
            if w == 1. { return Ok(at[(k+1)%m].rails[0]) }
            let (x,_,n) = thread.at(k,w,phi);
            let t = rack.contact(x,n).ok_or(Unbuilt::Curved {at:x})?;
            rack.carried(x,t)
        }
        Element::Fan(k) => {
            if w == 0. { return Ok(at[k].rails[0]) }
            if w == 1. { return Ok(at[k].rails[1]) }
            let [t0,t1] = at[k].times;
            rack.carried(at[k].point,t0+w*(t1-t0))
        }
    }
}

/// The degree the sheets are fitted at, both ways.
const DEGREE: usize = 5;

/// How many turns of one period the window is looked for at, and how many points across each
/// element at each.
const LOOKS: usize = 720;
const ACROSS: usize = 12;

/// **The candidates of the swept solid within `reach`** (see the module), or None where the tool
/// is no thread, the motion no rack to it, or its ends play a part within the reach: one candidate
/// per copy of the period, each the section's elements that reach the reach, their sheets fitted
/// within `tol / 4` on one grid of the turn — or, where one copy's sector holds it apart, that one
/// copy and its sector. No meeting is given: the elements of one candidate share their rails, and
/// two copies meet nowhere the trim does not find.
pub fn candidates(tool: &Brep,path: &Trajectory,roll: [f64;2],reach: &Reach,tol: f64) -> Result<Option<Generated>,Unbuilt> {
    let Some(thread) = Thread::read(tool) else { return Ok(None) };
    let Some(rack) = Rack::read(&thread,path,roll)? else { return Ok(None) };
    if !rack.ends_clear(&thread,reach) {
        if crate::debug() { eprintln!("generated: the thread's ends reach the blank: not the rack's road") }
        return Ok(None)
    }
    let m = thread.generators.len();
    let elements: Vec<Element> = (0..m).flat_map(|k| [Element::Fan(k),Element::Face(k)]).collect();
    let inside = |p: V| match *reach {
        Reach::Turned {o,z,r,span} | Reach::Sector {o,z,r,span,..} => { let h = dot(sub(p,o),z); off_axis(p,o,z) <= r && h >= span[0] && h <= span[1] }
        Reach::Box(lo,hi) => (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k]),
    };
    // **the window**: the turns of one period at which some element's sheet reaches the reach, and
    // which elements do
    let mut looked = vec![false;LOOKS];
    let mut reaching = vec![false;elements.len()];
    for (j,seen) in looked.iter_mut().enumerate() {
        let phi = -PI+TAU*j as f64/LOOKS as f64;
        // (a turn at which a corner touches at no instant — its faces' normals square to the
        // motion's rate there, the point sliding along itself throughout — is no turn that reaches)
        let Ok(at) = corners(&thread,&rack,phi) else { continue };
        for (e,&el) in elements.iter().enumerate() {
            if let Element::Fan(k) = el { if !convex(&thread,k,phi) { continue } }
            for i in 0..=ACROSS {
                if sheet_point(&thread,&rack,el,i as f64/ACROSS as f64,phi,&at).is_ok_and(|p| inside(p)) { *seen = true; reaching[e] = true; }
            }
        }
    }
    if looked.iter().all(|&x| x) || !looked.iter().any(|&x| x) {
        if crate::debug() { eprintln!("generated: the thread reaches the blank at {} of {LOOKS} turns: no window",looked.iter().filter(|&&x| x).count()) }
        return Ok(None)
    }
    // the one run of turns reaching it, round the period
    let start = (0..LOOKS).find(|&j| looked[j] && !looked[(j+LOOKS-1)%LOOKS]).expect("a run begins");
    let len = (0..LOOKS).take_while(|&i| looked[(start+i)%LOOKS]).count();
    if (0..LOOKS).filter(|&j| looked[j]).count() != len {
        return Err(Unbuilt::Failed("the thread's sheets reach the blank in two runs of one period's turns".into()))
    }
    let dphi = TAU/LOOKS as f64;
    let spare = 2.+0.05*len as f64;
    let window = [-PI+dphi*(start as f64-spare),-PI+dphi*((start+len) as f64-1.+spare)];
    if window[1]-window[0] >= TAU-2.*dphi { return Err(Unbuilt::Failed("the thread's window spans its period".into())) }
    // **the runs of elements reaching it**, round the section: a fan between two faces reaching it
    // reaches it too
    let n = elements.len();
    for e in 0..n {
        if let Element::Fan(_) = elements[e] { if reaching[(e+n-1)%n] && reaching[(e+1)%n] { reaching[e] = true } }
    }
    for (e,&el) in elements.iter().enumerate() {
        if let (Element::Fan(k),true) = (el,reaching[e]) {
            if !convex(&thread,k,0.5*(window[0]+window[1])) {
                return Err(Unbuilt::Curved {at:thread.corner(k,0.5*(window[0]+window[1]))})
            }
        }
    }
    let runs: Vec<Vec<Element>> = if reaching.iter().all(|&x| x) { vec![elements.clone()] } else {
        let first = (0..n).find(|&e| reaching[e] && !reaching[(e+n-1)%n]).expect("a run begins");
        let mut out: Vec<Vec<Element>> = Vec::new();
        for i in 0..n {
            let e = (first+i)%n;
            if !reaching[e] { continue }
            if i > 0 && reaching[(e+n-1)%n] { out.last_mut().expect("a run").push(elements[e]) } else { out.push(vec![elements[e]]) }
        }
        out
    };
    let closed = runs.len() == 1 && runs[0].len() == n;
    if crate::debug() { eprintln!("generated: window [{:.4}, {:.4}] of the turn, runs {runs:?}, {} copies",window[0],window[1],rack.copies) }
    let mut chains: Vec<Brep> = Vec::new();
    for run in &runs {
        let nets = fitted(&thread,&rack,run,window,closed,0.25*tol)?;
        chains.push(chain(nets,closed,run)?);
    }
    // **one copy's sector, where the copies stand apart**: the turns about the observer's axis the
    // period's sheets take within the reach, read at their exact points; where they span less than
    // a pitch by a margin either way, the sector a pitch wide about their middle holds the one copy
    // and its sides stand in the material between it and its neighbours
    if let Some(sector) = sector_of(&thread,&rack,&runs,window,reach,&inside)? {
        if crate::debug() { eprintln!("generated: one copy within the sector from {:.6} by {:.6}",sector.from,sector.pitch) }
        return Ok(Some(Generated {candidates:chains,given:Vec::new(),sector:Some(sector)}))
    }
    let mut out = Vec::new();
    for c in 0..rack.copies {
        let r = rack.copy(c)?;
        out.extend(chains.iter().map(|b| b.moved(&r)));
    }
    Ok(Some(Generated {candidates:out,given:Vec::new(),sector:None}))
}

/// **What the rack's road found** (`candidates`): the candidates and their meetings known — every
/// copy's, or, where one copy's sector holds it apart from the others (`sector`), the one copy's.
pub struct Generated { pub candidates: Vec<Brep>,pub given: Vec<crate::boolean::Given>,pub sector: Option<Sector> }

/// **One copy's sector about the observer's axis** (`Reach::Sector`'s): its turns from the unit `x`
/// square to the axis, from `from` by `pitch`, and how many copies the whole is.
#[derive(Clone,Copy,Debug)]
pub struct Sector { pub x: V,pub from: f64,pub pitch: f64,pub count: usize }

/// The share of a pitch a copy's turns must stand clear of each side of its sector by.
const CLEAR: f64 = 0.04;

/// How finely the copy's sheets are read for the turns they take: along the turn, and across each
/// element.
const SPREAD: [usize;2] = [384,48];

/// **The sector holding one copy** (`Sector`), or none where the copies' turns within the reach
/// overlap or come within `CLEAR` of a pitch of one another: the sheets' points within the reach,
/// read exactly on a grid of the window, their turns about the observer's axis from the first.
fn sector_of(thread: &Thread,rack: &Rack,runs: &[Vec<Element>],window: [f64;2],reach: &Reach,inside: &dyn Fn(V) -> bool)
    -> Result<Option<Sector>,Unbuilt> {
    let Reach::Turned {o,z,..} = *reach else { return Ok(None) };
    if rack.copies < 2 { return Ok(None) }
    let pitch = TAU/rack.copies as f64;
    // (the observer's axis must be the reach's: the copies turn about it)
    if (dot(unit(rack.observer.axis),z).abs()-1.).abs() > 1e-12 || off_axis(rack.observer.origin,o,z) > 1e-9*(1.+thread.reach) { return Ok(None) }
    let x = { let q = sub(thread.centre,o); let q = sub(q,scale(z,dot(q,z))); if norm(q) > 0. { unit(q) } else { return Ok(None) } };
    let y = cross(z,x);
    let mut turns: Vec<f64> = Vec::new();
    for j in 0..=SPREAD[0] {
        let phi = window[0]+(window[1]-window[0])*j as f64/SPREAD[0] as f64;
        let Ok(at) = corners(thread,rack,phi) else { continue };
        for &el in runs.iter().flatten() {
            for i in 0..=SPREAD[1] {
                let Ok(p) = sheet_point(thread,rack,el,i as f64/SPREAD[1] as f64,phi,&at) else { continue };
                if !inside(p) { continue }
                let q = sub(p,o);
                turns.push(dot(q,y).atan2(dot(q,x)));
            }
        }
    }
    let Some(&first) = turns.first() else { return Ok(None) };
    let (lo,hi) = turns.iter().map(|&a| (a-first+PI).rem_euclid(TAU)-PI).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),a| (l.min(a),h.max(a)));
    if hi-lo > pitch*(1.-2.*CLEAR) {
        if crate::debug() { eprintln!("generated: a copy within the reach spans {:.6} of a pitch {pitch:.6}: no sector",hi-lo) }
        return Ok(None)
    }
    Ok(Some(Sector {x,from:first+0.5*(lo+hi)-0.5*pitch,pitch,count:rack.copies}))
}

/// **The run's sheets fitted on one grid of the turn** over `window`, each within `tol`: each
/// element's rows across it interpolated at every turn of the grid, through its exact points (its
/// rails the corners' points, shared with its neighbours), then their poles along the turn; the
/// grid across each element and the grid of the turn doubled where a point between them misses.
/// Each two neighbours' shared rail is then one curve, laid as both nets' edge row.
fn fitted(thread: &Thread,rack: &Rack,run: &[Element],window: [f64;2],closed: bool,tol: f64) -> Result<Vec<Net>,Unbuilt> {
    let mut across = vec![8usize;run.len()];
    let mut along = 16usize;
    let phi = |s: f64| window[0]+s*(window[1]-window[0]);
    let refuse = |why: &str| Unbuilt::Failed(format!("the thread's generated sheets: {why}"));
    loop {
        let tv: Vec<f64> = (0..=along).map(|j| j as f64/along as f64).collect();
        let mids: Vec<f64> = tv.windows(2).map(|w| 0.5*(w[0]+w[1])).collect();
        let at: Vec<Vec<Corner>> = crate::util::par::map(&tv,|&s| corners(thread,rack,phi(s))).into_iter().collect::<Result<_,_>>()?;
        let at_mid: Vec<Vec<Corner>> = crate::util::par::map(&mids,|&s| corners(thread,rack,phi(s))).into_iter().collect::<Result<_,_>>()?;
        let mut nets = Vec::with_capacity(run.len());
        let mut grow_along = false;
        let mut grown = false;
        for (e,&el) in run.iter().enumerate() {
            let tu: Vec<f64> = (0..=across[e]).map(|i| i as f64/across[e] as f64).collect();
            let rows: Vec<BSpline> = tv.iter().zip(&at).map(|(&s,c)| {
                let pts = tu.iter().map(|&w| sheet_point(thread,rack,el,w,phi(s),c)).collect::<Result<Vec<V>,_>>()?;
                interpolate(&pts,&tu,DEGREE).ok_or_else(|| refuse("a row not interpolated"))
            }).collect::<Result<_,_>>()?;
            let cols: Vec<BSpline> = (0..rows[0].poles.len()).map(|i| {
                interpolate(&rows.iter().map(|r| r.poles[i]).collect::<Vec<_>>(),&tv,DEGREE).ok_or_else(|| refuse("a column not interpolated"))
            }).collect::<Result<_,_>>()?;
            let net = Net {du:rows[0].degree,dv:cols[0].degree,uknots:rows[0].knots.clone(),vknots:cols[0].knots.clone(),
                poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
            // how far it misses between its points across, and between its rows along
            let mut miss_across = 0f64;
            for (&s,c) in tv.iter().zip(&at) {
                for w in tu.windows(2) {
                    let u = 0.5*(w[0]+w[1]);
                    miss_across = miss_across.max(distance(net.point(u,s),sheet_point(thread,rack,el,u,phi(s),c)?));
                }
            }
            let mut miss_along = 0f64;
            for (&s,c) in mids.iter().zip(&at_mid) {
                for &u in &tu { miss_along = miss_along.max(distance(net.point(u,s),sheet_point(thread,rack,el,u,phi(s),c)?)) }
            }
            if crate::debug() { eprintln!("generated: {el:?} on {} × {along}: misses {miss_across:.2e} across, {miss_along:.2e} along",across[e]) }
            if miss_across > tol {
                if across[e] >= 512 { return Err(refuse("a sheet not fitted across within the tolerance")) }
                across[e] *= 2;
                grown = true;
            }
            if miss_along > tol { grow_along = true }
            nets.push(net);
        }
        if grow_along {
            if along >= 1024 { return Err(refuse("a sheet not fitted along the turn within the tolerance")) }
            along *= 2;
            grown = true;
        }
        if grown { continue }
        // each shared rail one curve, the edge row of both its nets
        let rails = if closed { run.len() } else { run.len()-1 };
        for r in 0..rails {
            let (a,b) = (r,(r+1)%run.len());
            let last = nets[a].poles.len()-1;
            let rail = nets[a].poles[last].clone();
            nets[b].poles[0] = rail;
        }
        return Ok(nets)
    }
}

/// **The run's sheets as one candidate**, each a face of its net, the faces in the run's order and
/// each two neighbours sharing their rail as one edge (a closed run's last sharing its first's).
fn chain(nets: Vec<Net>,closed: bool,run: &[Element]) -> Result<Brep,Unbuilt> {
    let refuse = |why: String| Unbuilt::Failed(format!("the thread's generated sheets: {why}"));
    for net in &nets { net.check().map_err(refuse)? }
    let q = nets.len();
    let mut b = Brep::default();
    // the rails: the first net's first row, then each net's last row (a closed run's last its first's)
    let rows: Vec<BSpline> = std::iter::once(nets[0].row(0)).chain(nets.iter().map(|n| n.row(n.poles.len()-1))).collect();
    let rails = if closed { q } else { q+1 };
    let mut bottom = Vec::with_capacity(rails);
    let mut top = Vec::with_capacity(rails);
    let mut edges = Vec::with_capacity(rails);
    for row in rows.iter().take(rails) {
        let [v0,v1] = row.domain();
        let (p0,p1) = (row.point(v0),row.point(v1));
        let (x,y) = (b.vertex(p0),b.vertex(p1));
        edges.push(b.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(row.clone()))),[v0,v1],[x,y]));
        bottom.push(x);
        top.push(y);
    }
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    for (i,net) in nets.into_iter().enumerate() {
        let (l,r) = (i,if closed { (i+1)%q } else { i+1 });
        let [[u0,u1],[v0,v1]] = net.domain();
        let (last_v,lo,hi) = (net.poles[0].len()-1,net.column(0),net.column(net.poles[0].len()-1));
        let _ = last_v;
        let e0 = b.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(lo))),[u0,u1],[bottom[l],bottom[r]]);
        let e2 = b.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(hi))),[u0,u1],[top[l],top[r]]);
        let lp = vec![
            Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
            Coedge {edge:edges[r],reversed:false,pcurve:line([u1,v0],[u1,v1])},
            Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
            Coedge {edge:edges[l],reversed:true,pcurve:line([u0,v0],[u0,v1])},
        ];
        let name = match run[i] { Element::Face(k) => format!("generated face {k}"),Element::Fan(k) => format!("generated fan {k}") };
        b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],name});
    }
    Ok(b)
}
