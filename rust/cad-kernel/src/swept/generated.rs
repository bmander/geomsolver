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
//! (`Thread::ends_clear`). Any other is not this road's (`None`), and the general road is asked.
//! A concave edge in the reach (the sheets of its two faces crossing, not meeting along a rail) and
//! a whole section reaching it are refused by name; a sheet that folds within the reach is left to
//! the trim's membership and the judge, which refuse it.
use crate::geom::{off_axis,Carry,Curve,Rigid,Surface};
use crate::nurbs::{interpolate,BSpline,Net};
use crate::topo::Brep;
use crate::trajectory::{Screw,Trajectory,Twist};
use crate::util::space::{cross,distance,dot,norm,scale,sub,unit};
use super::closed::Unbuilt;
use super::trim::Reach;
use std::f64::consts::{PI,TAU};

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
        // (every face a screw's sheet or a plane, read before the tool's size is: K2 asks this of
        // every tool it is given)
        if !tool.faces.iter().all(|f| match &f.surface {
            Surface::Blend(_,b) => matches!(b.carry,Carry::Screw {..}),
            s => matches!(s,Surface::Plane(_)),
        }) { return None }
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
                    let &mut (centre,a0,r0) = screw.get_or_insert((center,a,rise));
                    if (dot(a,a0).abs()-1.).abs() > 1e-12 || off_axis(center,centre,a0) > eps
                        || (rise-r0).abs() > 1e-12*(1.+r0.abs()) { return None }
                    let (p,q) = (b.a.point(b.ta[0]),b.a.point(b.ta[1]));
                    if found.iter().any(|g| {
                        let (x,y) = (g.curve.point(g.t[0]),g.curve.point(g.t[1]));
                        (distance(x,p) <= eps && distance(y,q) <= eps) || (distance(x,q) <= eps && distance(y,p) <= eps)
                    }) { continue }
                    // the sign turning `X_u × X_φ` outward, read where the face's own surface is
                    let carry = Carry::Screw {center:centre,axis:a0,angle:1.,advance:r0};
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
                _ => unreachable!("every face read a screw's or a plane"),
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

    /// **Whether the thread's ends play no part within the reach** — the cylinder of radius `r`
    /// about the line through `o` along the unit `z`, over `span` along it — under the observer's
    /// turn about that line: the cylinder about the thread's axis holding it, met with the reach,
    /// stands within the thread's length, so wherever the slid thread enters the reach the endless
    /// one is the same. Read in closed form: in the plane square to the observer's axis, the
    /// thread's band about its axis met with the reach's disc, at the height where the thread's
    /// cylinder is widest within the reach's span.
    fn ends_clear(&self,o: V,z: V,r: f64,span: [f64;2]) -> bool {
        let (c,a) = (self.centre,self.axis);
        if dot(a,z).abs() > 1e-12 { return false }
        // the thread's axis's height along the observer's, and the height within the span nearest it
        let h = dot(sub(c,o),z);
        let near = h.clamp(span[0],span[1]);
        let wide = self.reach*self.reach-(near-h)*(near-h);
        if wide < 0. { return true }
        let wide = wide.sqrt();
        // in the plane: the axis's offset from the observer's across it, and the nearest offset in the band
        let e = dot(sub(c,o),unit(cross(z,a)));
        let m = if e-wide > 0. { e-wide } else if e+wide < 0. { e+wide } else { 0. };
        if m.abs() > r { return true }
        let s = (r*r-m*m).sqrt();
        let at = dot(sub(c,o),a);
        let [lo,hi] = self.ends;
        // (the reach's chord along the thread's axis, from the thread's centre)
        -s-at >= lo && s-at <= hi
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

}

/// An element of the thread's section: a face (generator `k` screwed) or the fan of corner `k`
/// (where generator `k` starts).
#[derive(Clone,Copy,Debug,PartialEq)]
enum Element { Face(usize),Fan(usize) }

/// What a corner of the section does at one turn `φ`: its point, the instants its two faces touch
/// there (the face before it, then the face it starts), where each carries it, and whether it is
/// convex — the face it starts leaving it behind the face before it.
#[derive(Clone,Copy,Debug)]
struct Corner { point: V,times: [f64;2],rails: [V;2],convex: bool }

/// The road's reading at one turn: each corner's.
fn corners(thread: &Thread,rack: &Rack,phi: f64) -> Result<Vec<Corner>,Unbuilt> {
    let m = thread.generators.len();
    (0..m).map(|k| {
        let before = thread.at((k+m-1)%m,1.,phi).2;
        let (point,into,after) = thread.at(k,0.,phi);
        let [Some(t0),Some(t1)] = [rack.contact(point,before),rack.contact(point,after)] else { return Err(Unbuilt::Curved {at:point}) };
        Ok(Corner {point,times:[t0,t1],rails:[rack.carried(point,t0)?,rack.carried(point,t1)?],convex:dot(before,into) < 0.})
    }).collect()
}

/// **Element `el`'s sheet at `w` across it and turn `φ`**, `at` the corners there: its rails
/// exactly the corners' (so two elements sharing one read the same points there).
fn sheet_point(thread: &Thread,rack: &Rack,el: Element,w: f64,phi: f64,at: &[Corner]) -> Result<V,Unbuilt> {
    let m = thread.generators.len();
    match el {
        Element::Face(k) if w == 0. => Ok(at[k].rails[1]),
        Element::Face(k) if w == 1. => Ok(at[(k+1)%m].rails[0]),
        Element::Face(k) => {
            let (x,_,n) = thread.at(k,w,phi);
            let t = rack.contact(x,n).ok_or(Unbuilt::Curved {at:x})?;
            rack.carried(x,t)
        }
        Element::Fan(k) if w == 0. => Ok(at[k].rails[0]),
        Element::Fan(k) if w == 1. => Ok(at[k].rails[1]),
        Element::Fan(k) => { let [t0,t1] = at[k].times; rack.carried(at[k].point,t0+w*(t1-t0)) }
    }
}

/// **Element `el`'s points within the reach at turn `φ`**, `count` + 1 of them across it read
/// exactly (none of a concave corner's fan, which has none).
fn reached<'a>(thread: &'a Thread,rack: &'a Rack,el: Element,phi: f64,at: &'a [Corner],count: usize,inside: &'a dyn Fn(V) -> bool)
    -> impl Iterator<Item=V> + 'a {
    let fan_of_concave = matches!(el,Element::Fan(k) if !at[k].convex);
    (0..=count).filter(move |_| !fan_of_concave)
        .filter_map(move |i| sheet_point(thread,rack,el,i as f64/count as f64,phi,at).ok()).filter(move |&p| inside(p))
}

/// The degree the sheets are fitted at, both ways.
const DEGREE: usize = 5;

/// How many turns of one period the window is looked for at, and how many points across each
/// element at each.
const LOOKS: usize = 720;
const ACROSS: usize = 12;

/// **The candidates of the swept solid within `reach`** (see the module), or None where the tool
/// is no thread, the motion no rack to it, its ends play a part within the reach, or the reach is no
/// cylinder about the observer's axis: one candidate per copy of the period, each the section's
/// elements that reach the reach, their sheets fitted within `tol / 4` on one grid of the turn — or,
/// where one copy's sector holds it apart, that one copy and its sector. No meeting is given: the
/// elements of one candidate share their rails, and two copies meet nowhere the trim does not find.
pub fn candidates(tool: &Brep,path: &Trajectory,roll: [f64;2],reach: &Reach,tol: f64) -> Result<Option<Generated>,Unbuilt> {
    let Reach::Turned {o,z,r,span} = *reach else { return Ok(None) };
    let Some(thread) = Thread::read(tool) else { return Ok(None) };
    let Some(rack) = Rack::read(&thread,path,roll)? else { return Ok(None) };
    if !thread.ends_clear(o,z,r,span) {
        if crate::debug() { eprintln!("generated: the thread's ends reach the blank: not the rack's road") }
        return Ok(None)
    }
    let m = thread.generators.len();
    let elements: Vec<Element> = (0..m).flat_map(|k| [Element::Fan(k),Element::Face(k)]).collect();
    let inside = |p: V| { let h = dot(sub(p,o),z); off_axis(p,o,z) <= r && h >= span[0] && h <= span[1] };
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
            if reached(&thread,&rack,el,phi,&at,ACROSS,&inside).next().is_some() { *seen = true; reaching[e] = true; }
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
    // reaches it too; a concave corner's (its two faces' sheets crossing, not meeting along a rail)
    // and a whole section's (a tube, no tooth of a rack) are refused by name
    let n = elements.len();
    let middle = corners(&thread,&rack,0.5*(window[0]+window[1]))?;
    for e in 0..n {
        let Element::Fan(k) = elements[e] else { continue };
        if reaching[(e+n-1)%n] && reaching[(e+1)%n] { reaching[e] = true }
        if reaching[e] && !middle[k].convex { return Err(Unbuilt::Curved {at:middle[k].point}) }
    }
    if reaching.iter().all(|&x| x) { return Err(Unbuilt::Failed("the thread's whole section reaches the blank".into())) }
    let first = (0..n).find(|&e| reaching[e] && !reaching[(e+n-1)%n]).expect("a run begins");
    let mut runs: Vec<Vec<Element>> = Vec::new();
    for i in 0..n {
        let e = (first+i)%n;
        if !reaching[e] { continue }
        if i > 0 && reaching[(e+n-1)%n] { runs.last_mut().expect("a run").push(elements[e]) } else { runs.push(vec![elements[e]]) }
    }
    if crate::debug() { eprintln!("generated: window [{:.4}, {:.4}] of the turn, runs {runs:?}, {} copies",window[0],window[1],rack.copies) }
    let chains: Vec<Brep> = runs.iter().map(|run| {
        let nets = fitted(&thread,&rack,run,window,0.25*tol)?;
        let named = nets.into_iter().zip(run).map(|(net,el)| (net,match el {
            Element::Face(k) => format!("generated face {k}"),Element::Fan(k) => format!("generated fan {k}") })).collect();
        crate::build::sheets(named).map_err(|e| unfitted(&e))
    }).collect::<Result<_,_>>()?;
    // **one copy's sector, where the copies stand apart**: the turns about the observer's axis the
    // period's sheets take within the reach, read at their exact points; where they span less than
    // a pitch by a margin either way, the sector a pitch wide about their middle holds the one copy
    // and its sides stand in the material between it and its neighbours
    if let Some(sector) = sector_of(&thread,&rack,&runs,window,reach,&inside)? {
        if crate::debug() { eprintln!("generated: one copy within {sector:?}") }
        return Ok(Some(Generated {candidates:chains,sector:Some((sector,rack.copies))}))
    }
    let mut out = Vec::new();
    for c in 0..rack.copies {
        let r = rack.copy(c)?;
        out.extend(chains.iter().map(|b| b.moved(&r)));
    }
    Ok(Some(Generated {candidates:out,sector:None}))
}

/// **What the rack's road found** (`candidates`): the candidates, no meeting between them known —
/// every copy's, or, where one copy's sector holds it apart from the others (`sector`), the one
/// copy's, with that sector (`Reach::Sector`) and how many copies the whole is.
pub struct Generated { pub candidates: Vec<Brep>,pub sector: Option<(Reach,usize)> }

/// The share of a pitch a copy's turns must stand clear of each side of its sector by.
const CLEAR: f64 = 0.04;

/// How finely the copy's sheets are read for the turns they take: along the turn, and across each
/// element.
const SPREAD: [usize;2] = [384,48];

/// **The sector of `reach` holding one copy** (`Reach::Sector`), or none where the copies' turns
/// within the reach overlap or come within `CLEAR` of a pitch of one another: the sheets' points
/// within the reach, read exactly on a grid of the window, their turns about the observer's axis
/// from the first.
fn sector_of(thread: &Thread,rack: &Rack,runs: &[Vec<Element>],window: [f64;2],reach: &Reach,inside: &dyn Fn(V) -> bool)
    -> Result<Option<Reach>,Unbuilt> {
    let Reach::Turned {o,z,r,span} = *reach else { return Ok(None) };
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
            turns.extend(reached(thread,rack,el,phi,&at,SPREAD[1],inside).map(|p| { let q = sub(p,o); dot(q,y).atan2(dot(q,x)) }));
        }
    }
    let Some(&first) = turns.first() else { return Ok(None) };
    let (lo,hi) = turns.iter().map(|&a| (a-first+PI).rem_euclid(TAU)-PI).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),a| (l.min(a),h.max(a)));
    if hi-lo > pitch*(1.-2.*CLEAR) {
        if crate::debug() { eprintln!("generated: a copy within the reach spans {:.6} of a pitch {pitch:.6}: no sector",hi-lo) }
        return Ok(None)
    }
    Ok(Some(Reach::Sector {o,z,x,r,span,from:first+0.5*(lo+hi)-0.5*pitch,pitch}))
}

/// The road's refusal of a sheet it could not fit or lay.
fn unfitted(why: &str) -> Unbuilt { Unbuilt::Failed(format!("the thread's generated sheets: {why}")) }

/// **The run's sheets fitted on one grid of the turn** over `window`, each within `tol`: each
/// element's rows across it interpolated at every turn of the grid, through its exact points (its
/// rails the corners' points, shared with its neighbours), then their poles along the turn
/// (`nurbs::net_of_rows`); the grid across each element and the grid of the turn doubled where a
/// point between them misses. Each two neighbours' shared rail is then one curve, laid as both
/// nets' edge row.
fn fitted(thread: &Thread,rack: &Rack,run: &[Element],window: [f64;2],tol: f64) -> Result<Vec<Net>,Unbuilt> {
    let mut across = vec![8usize;run.len()];
    let mut along = 16usize;
    let phi = |s: f64| window[0]+s*(window[1]-window[0]);
    loop {
        // the corners at every turn of the grid and halfway between (the misses along read there)
        let halves: Vec<f64> = (0..=2*along).map(|j| j as f64/(2*along) as f64).collect();
        let at: Vec<Vec<Corner>> = crate::util::par::map(&halves,|&s| corners(thread,rack,phi(s))).into_iter().collect::<Result<_,_>>()?;
        let tv: Vec<f64> = halves.iter().step_by(2).copied().collect();
        let mut nets = Vec::with_capacity(run.len());
        let mut grown = false;
        let mut grow_along = false;
        for (e,&el) in run.iter().enumerate() {
            let tu: Vec<f64> = (0..=across[e]).map(|i| i as f64/across[e] as f64).collect();
            let rows: Vec<BSpline> = tv.iter().zip(at.iter().step_by(2)).map(|(&s,c)| {
                let pts = tu.iter().map(|&w| sheet_point(thread,rack,el,w,phi(s),c)).collect::<Result<Vec<V>,_>>()?;
                interpolate(&pts,&tu,DEGREE).ok_or_else(|| unfitted("a row not interpolated"))
            }).collect::<Result<_,_>>()?;
            let net = crate::nurbs::net_of_rows(&rows,&tv,DEGREE).ok_or_else(|| unfitted("a column not interpolated"))?;
            // how far it misses at the points `us` across, at the turns of the grid `rows` (even or odd)
            let miss = |us: &mut dyn Iterator<Item=f64>,odd: usize| -> Result<f64,Unbuilt> {
                let us: Vec<f64> = us.collect();
                let mut most = 0f64;
                for (j,c) in at.iter().enumerate().skip(odd).step_by(2) {
                    let s = halves[j];
                    for &u in &us { most = most.max(distance(net.point(u,s),sheet_point(thread,rack,el,u,phi(s),c)?)) }
                }
                Ok(most)
            };
            let miss_across = miss(&mut tu.windows(2).map(|w| 0.5*(w[0]+w[1])),0)?;
            let miss_along = miss(&mut tu.iter().copied(),1)?;
            if crate::debug() { eprintln!("generated: {el:?} on {} × {along}: misses {miss_across:.2e} across, {miss_along:.2e} along",across[e]) }
            if miss_across > tol {
                if across[e] >= 512 { return Err(unfitted("a sheet not fitted across within the tolerance")) }
                across[e] *= 2;
                grown = true;
            }
            grow_along |= miss_along > tol;
            nets.push(net);
        }
        if grow_along {
            if along >= 1024 { return Err(unfitted("a sheet not fitted along the turn within the tolerance")) }
            along *= 2;
            grown = true;
        }
        if grown { continue }
        // each shared rail one curve, the edge row of both its nets
        for r in 1..run.len() {
            let rail = nets[r-1].poles.last().expect("a row of poles").clone();
            nets[r].poles[0] = rail;
        }
        return Ok(nets)
    }
}
