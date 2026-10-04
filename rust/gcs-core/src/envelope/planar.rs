//! The inner envelope of a closed planar curve under a planar motion (#65): over a whole period,
//! the boundary of what the moving curve encloses at every pose — a Wankel rotor's flanks inside
//! its housing's bore. Read after the solve, as the motion is.
//!
//! The moving curve is `X(s, θ) = M(θ)·C(s)` in the curve's own view, and a point of the envelope
//! is where it moves along itself: `F(s, θ) = ∂X/∂s × ∂X/∂θ = 0`. For each `s` every root `θ` is
//! found (brackets on a grid of the roll, then regula falsi) and its image kept only where the point lies
//! inside the curve at every pose of the period (branch and bound on the roll against the
//! source's signed distance, its speed bounded by the motion's). The kept images are linked by
//! continuity in `s` into pieces; a branch whose image stands still as `s` runs (the apex the
//! bore is the path of) is a corner. The pieces must close into one loop through corners, turn
//! one way along themselves (no fold) and cross nothing: each refusal names its row and a point.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::Sketch;
use crate::motion::Family;
use crate::solid::PlanarField;
use std::fmt;

type P = [f64;2];

fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn dist(a: P,b: P) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

pub use crate::solid::admission::Condition;

/// Why no envelope: the row, what was found, and a point of the curve's view where it was.
#[derive(Clone,Debug)]
pub struct Refusal { pub row: Condition,pub message: String,pub witness: Option<P> }

impl fmt::Display for Refusal {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"{}: {}",self.row.code(),self.message)?;
        if let Some(w) = self.witness { write!(f," (near ({:.6}, {:.6}))",w[0],w[1])?; }
        Ok(())
    }
}

fn debug() -> bool { std::env::var_os("SOLVENT_ENVELOPE_DEBUG").is_some() }

fn refuse(row: Condition,message: impl Into<String>,witness: Option<P>) -> Refusal { Refusal {row,message:message.into(),witness} }

/// A planar motion read in one view's page coordinates, `x ↦ A x + c`, with its rate per radian.
#[derive(Clone,Copy,Debug)]
struct Rigid { a: [[f64;2];2],c: P,da: [[f64;2];2],dc: P }

impl Rigid {
    fn point(&self,x: P) -> P { [self.a[0][0]*x[0]+self.a[0][1]*x[1]+self.c[0],self.a[1][0]*x[0]+self.a[1][1]*x[1]+self.c[1]] }
    fn vector(&self,v: P) -> P { [self.a[0][0]*v[0]+self.a[0][1]*v[1],self.a[1][0]*v[0]+self.a[1][1]*v[1]] }
    fn velocity(&self,x: P) -> P { [self.da[0][0]*x[0]+self.da[0][1]*x[1]+self.dc[0],self.da[1][0]*x[0]+self.da[1][1]*x[1]+self.dc[1]] }
    /// `y` carried back to where the motion took it from: `Aᵀ(y − c)`.
    fn back(&self,y: P) -> P { let d = sub(y,self.c); [self.a[0][0]*d[0]+self.a[1][0]*d[1],self.a[0][1]*d[0]+self.a[1][1]*d[1]] }
    /// The contact condition at the source's point `x` and tangent `t`.
    fn contact(&self,x: P,t: P) -> f64 { cross(self.vector(t),self.velocity(x)) }
}

/// A view's page and space, as the affine maps between them: what a pose is read through without
/// the sketch (which the threads reading contacts cannot share).
#[derive(Clone,Copy,Debug)]
struct Frame { o: [f64;3],cols: [[f64;3];2],at: P,rows: [[f64;3];2] }

impl Frame {
    fn of(sk: &Sketch,view: Option<usize>) -> Frame {
        let o = sk.world_in(view,(0.,0.));
        let cols = [crate::space::sub(sk.world_in(view,(1.,0.)),o),crate::space::sub(sk.world_in(view,(0.,1.)),o)];
        let (ax,ay) = sk.on_view_sheet([0.;3],view);
        let e = |k: usize| { let mut v = [0.;3]; v[k] = 1.; sk.page_vector_in(view,v) };
        let (ex,ey,ez) = (e(0),e(1),e(2));
        Frame {o,cols,at:[ax,ay],rows:[[ex.0,ey.0,ez.0],[ex.1,ey.1,ez.1]]}
    }
    fn world(&self,p: P) -> [f64;3] { std::array::from_fn(|k| self.o[k]+p[0]*self.cols[0][k]+p[1]*self.cols[1][k]) }
    fn vector(&self,v: [f64;3]) -> P { [crate::space::dot(self.rows[0],v),crate::space::dot(self.rows[1],v)] }
    fn page(&self,w: [f64;3]) -> P { let v = self.vector(w); [self.at[0]+v[0],self.at[1]+v[1]] }
    fn normal(&self) -> [f64;3] { crate::space::cross(self.cols[0],self.cols[1]) }
}

/// The source curve, read once off the solved sketch so that an envelope point is solved from it
/// with no sketch at hand (a meter reads one from many threads): a formula's tapes and the numbers
/// they are read with, or a generated profile's encoding (`generate.rs`: a Wankel's bore is its
/// apex's envelope under the rotor's motion) and the coordinates it is written over.
#[derive(Clone,Debug)]
enum Source {
    Formula { x: crate::tape::Tape,y: crate::tape::Tape,vars: Vec<f64> },
    Generated { consts: Vec<f64>,theta: Vec<f64> },
}

impl Source {
    fn of(sk: &Sketch,curve: usize) -> Result<Source,Refusal> {
        use crate::model::{CurveBody,EntKind,EntRef};
        let cv = &sk.curves[curve];
        match &sk.curve_defs[cv.def as usize].body {
            CurveBody::Exprs {x,y} => Ok(Source::Formula {x:x.clone(),y:y.clone(),vars:sk.curve_vars(curve,0.)}),
            // what a contact on it carries: the roll its root is chosen at, its numbers, its encoding
            CurveBody::Envelope(g) => {
                let consts = g.contact_consts(sk,curve);
                let theta = sk.entity_params(EntRef::new(EntKind::Curve,curve)).into_iter()
                    .map(|p| sk.params[p as usize].value).collect();
                Ok(Source::Generated {consts,theta})
            }
            CurveBody::Trace(_) => Err(refuse(Condition::Envelope,"the inner envelope is read of a curve of a computed point or a \
                generated profile, not yet of a traced curve",None)),
        }
    }

    /// The point at `s` in the view's page coordinates and its derivative per unit of `s`.
    fn at(&self,s: f64) -> Option<(P,P)> {
        let out = match self {
            Source::Formula {x,y,vars} => {
                let mut v = vars.clone();
                v[0] = s;
                let mut scratch = crate::tape::Scratch::new();
                let (a,b) = (x.eval(&v,&mut scratch),y.eval(&v,&mut scratch));
                ([a.v,b.v],[a.d[0],b.d[0]])
            }
            Source::Generated {consts,theta} => {
                let v = crate::generate::kernel_eval(consts,s,theta,1,false);
                if !v.ok { return None }
                ([v.x,v.y],[v.dx[0],v.dy[0]])
            }
        };
        out.0.iter().chain(&out.1).all(|v| v.is_finite()).then_some(out)
    }
}

/// The source, the motion and the view they are read in: what an envelope point is solved from.
#[derive(Clone,Debug)]
struct Setting { source: Source,frame: Frame,family: Family,scale: f64 }

impl Setting {
    /// The motion at `θ` in the view's page coordinates, refused where it leaves the plane.
    fn rigid(&self,theta: f64) -> Result<Rigid,Refusal> {
        let m = self.family.at(theta).map_err(|e| refuse(Condition::Envelope,e,None))?;
        let f = &self.frame;
        let (o,w) = (f.o,[crate::space::add(f.o,f.cols[0]),crate::space::add(f.o,f.cols[1])]);
        let a0 = f.vector(m.vector(f.cols[0]));
        let a1 = f.vector(m.vector(f.cols[1]));
        let c = f.page(m.point(o));
        let dc = f.vector(m.velocity(o));
        let d0 = sub(f.vector(m.velocity(w[0])),dc);
        let d1 = sub(f.vector(m.velocity(w[1])),dc);
        // the plane kept, and kept facing the same way: the moved origin on it, the turn a rotation
        let n = f.normal();
        let off = crate::space::dot(crate::space::sub(m.point(o),o),n);
        let turn = crate::space::dot(m.vector(n),n);
        if off.abs() > 1e-9*self.scale || crate::space::dot(m.velocity(o),n).abs() > 1e-9*self.scale || turn < 1.-1e-9 {
            return Err(refuse(Condition::Plane,format!("`{}` carries the curve out of its plane, or turns it over: \
                an envelope in the plane is of a motion of the plane",self.family.name),None));
        }
        Ok(Rigid {a:[[a0[0],a1[0]],[a0[1],a1[1]]],c,da:[[d0[0],d1[0]],[d0[1],d1[1]]],dc})
    }

    /// Where the motion at `θ` takes a point of the view back from: `Aᵀ(y − c)`, read off the pose
    /// alone (no derivative, no allocation), for the many readings of the keeping test.
    fn back(&self,theta: f64,y: P) -> Option<P> {
        let m = self.family.pose_at(theta).ok()?;
        let f = &self.frame;
        let w = m.inverse().point(f.world(y));
        Some(f.page(w))
    }

    fn source(&self,s: f64) -> Option<(P,P)> { self.source.at(s) }
}

/// One point of the envelope: the source's parameter and the roll it is touched at (both NaN at a
/// corner, which no one contact makes), and where it is in the view.
#[derive(Clone,Copy,Debug)]
pub struct Contact { pub s: f64,pub roll: f64,pub at: P }

/// The envelope read: its pieces in order round the loop, counter-clockwise, each running from the
/// corner it starts at to the one it ends at (those two its first and last contact).
#[derive(Clone,Debug)]
pub struct InnerEnvelope {
    pub pieces: Vec<Vec<Contact>>,
    setting: Setting,
}

/// How many roll steps a turn of the grid the roots are bracketed on takes, and how many samples of
/// the source parameter the envelope is found at.
const ROLL_STEPS_PER_TURN: f64 = 1440.;
const SOURCE_STEPS: usize = 1440;
/// The least stretch of the roll the keeping test halves to (radians): a point is read as kept
/// where no reading finds it outside and no stretch longer than this one might.
const KEEP_STEP: f64 = 1e-3;

impl InnerEnvelope {
    /// The inner envelope of closed curve `curve` under motion `motion` over `roll` (radians), a
    /// whole period of it, in the curve's view.
    pub fn read(sk: &Sketch,curve: usize,motion: usize,roll: [f64;2]) -> Result<InnerEnvelope,Refusal> {
        if !(roll[0] < roll[1]) { return Err(refuse(Condition::Period,"the roll increases",None)) }
        let clock = crate::clock::Instant::now();
        let family = Family::read(sk,motion).map_err(|e| refuse(Condition::Envelope,e,None))?;
        if !sk.curve_closed(curve) {
            return Err(refuse(Condition::Envelope,"the curve does not come back to where it started: an envelope \
                is of a closed curve",None));
        }
        let view = sk.curve_view(curve);
        let (s0,s1) = sk.curve_domain(curve);
        let (s0,s1) = (s0.min(s1),s0.max(s1));
        // the source loop as chords, its signed distance the test of what lies inside it
        let poly = sk.curve_polyline(curve);
        let scale = poly.iter().fold(0_f64,|m,p| m.max(dist([p.0,p.1],[poly[0].0,poly[0].1]))).max(1e-300);
        let setting = Setting {source:Source::of(sk,curve)?,frame:Frame::of(sk,view),family,scale};
        let slack = 1e-7*(1.+scale);
        let ring: Vec<P> = sk.curve_polyline_within(curve,slack).into_iter().map(|(_,q)| [q.0,q.1]).collect();
        let edges: Vec<crate::solid::surface::Edge> = ring.windows(2)
            .map(|w| crate::solid::surface::Edge::Line {a:w[0],b:w[1],axis:false}).collect();
        let field = PlanarField::from_loops(&[edges],0.)
            .map_err(|e| refuse(Condition::Envelope,format!("the curve is no simple loop ({e})"),None))?;
        // the roll's grid, and the whole period it must be
        let steps = (((roll[1]-roll[0])/std::f64::consts::TAU*ROLL_STEPS_PER_TURN).ceil() as usize).max(64);
        let times: Vec<f64> = (0..=steps).map(|j| roll[0]+(roll[1]-roll[0])*j as f64/steps as f64).collect();
        let grid: Vec<Rigid> = times.iter().map(|&t| setting.rigid(t)).collect::<Result<_,_>>()?;
        let (first,last) = (grid[0],grid[steps]);
        let apart = (0..2).flat_map(|i| (0..2).map(move |j| (i,j))).map(|(i,j)| (first.a[i][j]-last.a[i][j]).abs()*scale)
            .chain((0..2).map(|i| (first.c[i]-last.c[i]).abs())).fold(0.,f64::max);
        if apart > 1e-9*(1.+scale) {
            return Err(refuse(Condition::Period,format!("from {:.4} to {:.4} degrees `{}` does not bring the curve back: \
                its inner envelope is read over a whole period",roll[0].to_degrees(),roll[1].to_degrees(),setting.family.name),None))
        }
        let keep = Keep {field:&field,setting:&setting,grid:&grid,times:&times,tol:1e-6*(1.+scale),
            domain:crate::interval::Interval::new(roll[0],roll[1]).map_err(|_| refuse(Condition::Period,"the roll is no interval",None))?};
        // every kept contact at each sample of the source, distinct images only: the source read
        // here, the roots and the keeping on every core
        let ds = (s1-s0)/SOURCE_STEPS as f64;
        let samples: Vec<(f64,P,P)> = (0..SOURCE_STEPS).map(|k| {
            let s = s0+ds*(k as f64+0.5);
            let (x,t) = setting.source(s).ok_or_else(|| refuse(Condition::Envelope,
                format!("the curve cannot be read with its tangent at {s}"),None))?;
            Ok((s,x,t))
        }).collect::<Result<_,Refusal>>()?;
        let roots = crate::par::map(&samples,|&(s,x,t)| contacts(&setting,&grid,&times,s,x,t));
        let roots: Vec<Vec<Contact>> = roots.into_iter().collect::<Result<_,_>>()?;
        // each image once: a point touched at many samples (a corner, or a flank the curve's own
        // symmetry touches twice) is kept or not once, at the sample it is first touched at, and
        // how many samples touch it is what says it is a corner
        let still = 1e-6*(1.+scale);
        let mut grid = crate::space::Grid::new(4.*still);
        // each image: the contact first read, the sample it was read at, the last sample touching
        // it and how many do (the samples come in order, so the last says whether one is new)
        let mut images: Vec<(Contact,usize,usize,usize)> = Vec::new();
        for (k,cs) in roots.iter().enumerate() {
            for c in cs {
                let at = [c.at[0],c.at[1],0.];
                let mut same = None;
                grid.around(at,|u| if same.is_none() && dist(images[u as usize].0.at,c.at) <= still { same = Some(u as usize) });
                match same {
                    Some(u) => if images[u].2 != k { images[u].2 = k; images[u].3 += 1 },
                    None => { grid.insert(at,images.len() as u32); images.push((*c,k,k,1)); }
                }
            }
        }
        let kept = crate::par::map(&images,|&(c,..)| keep.inside(c));
        let mut found: Vec<Vec<Contact>> = vec![Vec::new();roots.len()];
        let mut corners: Vec<P> = Vec::new();
        for (&(c,k,_,touched),kept) in images.iter().zip(kept) {
            if !kept { continue }
            if touched >= 3 { corners.push(c.at) } else { found[k].push(c) }
        }
        if debug() { eprintln!("envelope: {} roots, {} images, corners {corners:?}",roots.iter().map(Vec::len).sum::<usize>(),images.len()); }
        if debug() { eprintln!("envelope: {} kept contacts at {} samples ({:?})",found.iter().map(Vec::len).sum::<usize>(),found.len(),clock.elapsed()); }
        let period = roll[1]-roll[0];
        let pieces = link(&found,period,s1-s0,scale);
        if debug() { eprintln!("envelope: {} chains ({:?})",pieces.len(),clock.elapsed()); }
        let pieces = close(&setting,pieces,corners,ds,scale)?;
        Ok(InnerEnvelope {pieces,setting})
    }

    /// Where a point of the envelope's view stands in space.
    pub fn world(&self,p: P) -> [f64;3] { self.setting.frame.world(p) }

    /// Where a point in space stands on the envelope's view (its shadow along the normal).
    pub fn page(&self,w: [f64;3]) -> P { self.setting.frame.page(w) }

    /// The unit normal of the envelope's plane, toward the viewer of its view.
    pub fn normal(&self) -> [f64;3] { let n = self.setting.frame.normal(); n.map(|x| x/crate::space::norm(n)) }

    /// The point a fraction `f` along piece `k` by its source parameter, solved onto the envelope
    /// from the nearest contact read: exact where a contact can be solved, the corners themselves at
    /// the piece's ends.
    pub fn point(&self,k: usize,f: f64) -> Option<P> {
        let piece = self.pieces.get(k)?;
        let n = piece.len();
        if n < 3 { return piece.get(if f < 0.5 { 0 } else { n-1 }).map(|c| c.at) }
        let (a,b) = (piece[1].s,piece[n-2].s);
        if f <= 0. { return Some(piece[0].at) }
        if f >= 1. { return Some(piece[n-1].at) }
        // the corners lie a hair past the first and last contacts; the stretch to them is straight
        let s = a+(b-a)*f;
        // the contacts' parameters run one way along the piece: the stretch holding `s` by halves
        let m = piece[1..n-1].partition_point(|c| if b >= a { c.s <= s } else { c.s >= s });
        let j = m.max(1).min(n.saturating_sub(3).max(1));
        let (lo,hi) = (piece[j],piece[j+1]);
        let w = if hi.s != lo.s { (s-lo.s)/(hi.s-lo.s) } else { 0. };
        let guess = lo.roll+w*(hi.roll-lo.roll);
        solve_at(&self.setting,s,guess,(hi.roll-lo.roll).abs().max(1e-6)).map(|c| c.at)
    }
}

/// The source's point at `s` touched near roll `guess`: Newton on the contact condition in the
/// roll, kept within `reach` of the guess.
fn solve_at(setting: &Setting,s: f64,guess: f64,reach: f64) -> Option<Contact> {
    let (x,t) = setting.source(s)?;
    let f = |theta: f64| setting.rigid(theta).ok().map(|r| r.contact(x,t));
    let mut theta = guess;
    for _ in 0..40 {
        let h = 1e-7*(1.+theta.abs());
        let (v,a,b) = (f(theta)?,f(theta+h)?,f(theta-h)?);
        let slope = (a-b)/(2.*h);
        if !(slope.abs() > 0.) { return None }
        let step = v/slope;
        theta -= step;
        if (theta-guess).abs() > 4.*reach { return None }
        if step.abs() <= 1e-14*(1.+theta.abs()) { break }
    }
    let r = setting.rigid(theta).ok()?;
    Some(Contact {s,roll:theta,at:r.point(x)})
}

/// Every root in the roll of the contact condition at source parameter `s`, with its image.
fn contacts(setting: &Setting,grid: &[Rigid],times: &[f64],s: f64,x: P,t: P) -> Result<Vec<Contact>,Refusal> {
    let values: Vec<f64> = grid.iter().map(|r| r.contact(x,t)).collect();
    let mut out = Vec::new();
    // the last pose is the first again: its roots are the first's
    for j in 0..grid.len()-1 {
        let (fa,fb) = (values[j],values[j+1]);
        let theta = if fa == 0. { times[j] } else if fa*fb < 0. {
            let f = |theta: f64| setting.rigid(theta).ok().map(|r| r.contact(x,t));
            crate::roots::bracketed_root(f,times[j],fa,times[j+1],fb,1e-15*(1.+times[j].abs()))
        } else { continue };
        let r = setting.rigid(theta)?;
        out.push(Contact {s,roll:theta,at:r.point(x)});
    }
    Ok(out)
}

/// What decides that a contact's image is kept: inside the curve at every pose of the period.
struct Keep<'a> { field: &'a PlanarField,setting: &'a Setting,grid: &'a [Rigid],times: &'a [f64],tol: f64,
    domain: crate::interval::Interval }

impl Keep<'_> {
    /// Whether `c`'s image lies within the curve at every pose, to `tol`: the signed distance of
    /// the image carried back read on the grid's poses, and between two readings bounded by the
    /// larger plus the image's speed times half the step (the distance is one-Lipschitz), the
    /// stretch halved where that bound is not yet below `tol` — down to `KEEP_STEP` of the roll,
    /// where both readings within it are taken for the stretch: at a contact the distance is
    /// zero and flat, and a Lipschitz bound would halve to the roundoff to clear it.
    fn inside(&self,c: Contact) -> bool {
        let world = self.setting.frame.world(c.at);
        let Ok(speed) = self.setting.family.inverse_point_speed_bound(world,self.domain) else { return false };
        let read = |r: &Rigid| self.field.value(r.back(c.at));
        let stride = 32.min(self.grid.len()-1);
        let mut marks: Vec<usize> = (0..self.grid.len()).step_by(stride).collect();
        if *marks.last().unwrap() != self.grid.len()-1 { marks.push(self.grid.len()-1) }
        let mut values = Vec::with_capacity(marks.len());
        for &j in &marks {
            let v = read(&self.grid[j]);
            if v > self.tol { return false }
            values.push(v);
        }
        // (from, to) in roll, with their readings
        let mut stack: Vec<(f64,f64,f64,f64)> = marks.windows(2).zip(values.windows(2))
            .map(|(m,v)| (self.times[m[0]],self.times[m[1]],v[0],v[1])).collect();
        while let Some((a,b,va,vb)) = stack.pop() {
            if va.max(vb)+speed*(b-a)/2. <= self.tol { continue }
            if b-a <= KEEP_STEP { continue }
            let m = 0.5*(a+b);
            let Some(q) = self.setting.back(m,c.at) else { return false };
            let vm = self.field.value(q);
            if vm > self.tol { return false }
            stack.push((a,m,va,vm));
            stack.push((m,b,vm,vb));
        }
        true
    }
}

/// The kept contacts linked into chains by continuity in the source parameter: each contact
/// joins the chain ending nearest it at the previous sample, in space and in roll (modulo the
/// period). The source is closed, so a chain reaching its last sample continues one starting at
/// its first, the source parameter carried on past the end of its interval (`span` further).
fn link(found: &[Vec<Contact>],period: f64,span: f64,scale: f64) -> Vec<Vec<Contact>> {
    struct Chain { contacts: Vec<Contact>,first: usize,last: usize }
    let n = found.len();
    let near = 0.05*scale;
    let turned = |a: f64,b: f64| { let d = (a-b).rem_euclid(period); d.min(period-d) };
    let follows = |a: &Contact,b: &Contact| dist(a.at,b.at) <= near && turned(a.roll,b.roll) <= 0.1;
    let mut chains: Vec<Chain> = Vec::new();
    let mut open: Vec<usize> = Vec::new();
    for (k,cs) in found.iter().enumerate() {
        let mut pairs: Vec<(f64,usize,usize)> = Vec::new();
        for (i,c) in cs.iter().enumerate() {
            for (o,&ch) in open.iter().enumerate() {
                let last = chains[ch].contacts.last().unwrap();
                if follows(last,c) { pairs.push((dist(last.at,c.at),i,o)) }
            }
        }
        pairs.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let (mut used_c,mut used_o) = (vec![false;cs.len()],vec![false;open.len()]);
        let mut next_open = Vec::new();
        for (_,i,o) in pairs {
            if used_c[i] || used_o[o] { continue }
            used_c[i] = true; used_o[o] = true;
            let ch = &mut chains[open[o]];
            ch.contacts.push(cs[i]);
            ch.last = k;
            next_open.push(open[o]);
        }
        for (i,c) in cs.iter().enumerate() {
            if used_c[i] { continue }
            chains.push(Chain {contacts:vec![*c],first:k,last:k});
            next_open.push(chains.len()-1);
        }
        open = next_open;
    }
    // across the source's seam
    let mut merged = vec![false;chains.len()];
    for e in 0..chains.len() {
        if chains[e].last != n-1 || merged[e] { continue }
        let tail = *chains[e].contacts.last().unwrap();
        let best = (0..chains.len()).filter(|&s| s != e && !merged[s] && chains[s].first == 0 && follows(&tail,&chains[s].contacts[0]))
            .min_by(|&a,&b| dist(chains[a].contacts[0].at,tail.at).total_cmp(&dist(chains[b].contacts[0].at,tail.at)));
        if let Some(s) = best {
            let next: Vec<Contact> = std::mem::take(&mut chains[s].contacts).into_iter()
                .map(|c| Contact {s:c.s+span,..c}).collect();
            chains[e].contacts.extend(next);
            chains[e].last = chains[s].last;
            merged[s] = true;
        }
    }
    chains.into_iter().zip(merged).filter(|(_,m)| !m).map(|(c,_)| c.contacts).collect()
}

/// The chains made one loop: a chain whose image stands still is a corner; every other is a piece,
/// marched on to the corners at its two ends, checked for a fold, and joined to them; the loop is
/// then checked for crossings and turned counter-clockwise.
fn close(setting: &Setting,chains: Vec<Vec<Contact>>,corners: Vec<P>,ds: f64,scale: f64)
    -> Result<Vec<Vec<Contact>>,Refusal> {
    // a moving chain cut where it passes a corner (two branches cross there, and linking by
    // nearness may carry one on into the other): the contacts near a corner dropped, to be
    // marched back to it exactly
    let zone = 1e-3*(1.+scale);
    let mut pieces: Vec<Vec<Contact>> = Vec::new();
    for c in chains {
        let mut run: Vec<Contact> = Vec::new();
        for x in c {
            if corners.iter().any(|&k| dist(k,x.at) <= zone) {
                if run.len() >= 2 { pieces.push(std::mem::take(&mut run)) } else { run.clear() }
            } else { run.push(x) }
        }
        if run.len() >= 2 { pieces.push(run) }
    }
    let join = 1e-5*(1.+scale);
    // each piece marched to the corner nearest each end, the source parameter stepped by halves
    // where a step would leave the branch, and doubled again where it does not: the root carried
    // from a kept one, so kept as it is (near a corner every reading is near zero, and the test
    // would halve the whole roll)
    for piece in &mut pieces {
        for forward in [true,false] {
            let end = if forward { *piece.last().unwrap() } else { piece[0] };
            let Some(&corner) = corners.iter().min_by(|a,b| dist(**a,end.at).total_cmp(&dist(**b,end.at))) else { continue };
            if dist(corner,end.at) > 0.05*scale { continue }
            let mut at = end;
            let mut h = ds/2.;
            while dist(at.at,corner) > 1e-7*(1.+scale) && h > 1e-13*ds {
                let s = if forward { at.s+h } else { at.s-h };
                let step = solve_at(setting,s,at.roll,0.05)
                    .filter(|c| dist(c.at,corner) < dist(at.at,corner) && dist(c.at,at.at) <= 2.*dist(at.at,corner));
                match step {
                    Some(c) => { at = c; if forward { piece.push(c) } else { piece.insert(0,c) } h = (2.*h).min(ds/2.); }
                    None => h /= 2.,
                }
            }
        }
    }
    if debug() { for p in &pieces { eprintln!("envelope: piece of {} from {:?} (s {}) to {:?} (s {})",p.len(),p[0].at,p[0].s,p.last().unwrap().at,p.last().unwrap().s); } }
    // a fold: the image turning back on itself along a piece
    for piece in &pieces {
        for w in piece.windows(3) {
            let (a,b) = (sub(w[1].at,w[0].at),sub(w[2].at,w[1].at));
            if a[0]*b[0]+a[1]*b[1] < 0. && dist(w[0].at,w[1].at) > 1e-9*scale && dist(w[1].at,w[2].at) > 1e-9*scale {
                return Err(refuse(Condition::Envelope,"the envelope folds back on itself: no rotor stays inside the curve \
                    along it",Some(w[1].at)))
            }
        }
    }
    if pieces.is_empty() { return Err(refuse(Condition::Envelope,"no contact is kept: the curve encloses nothing at every pose",None)) }
    // each piece's ends at corners, and the loop walked through them
    let end_corner = |p: P| corners.iter().position(|&k| dist(k,p) <= join);
    let mut ends: Vec<[usize;2]> = Vec::new();
    for piece in &pieces {
        let (a,b) = (piece[0].at,piece.last().unwrap().at);
        match (end_corner(a),end_corner(b)) {
            (Some(x),Some(y)) => ends.push([x,y]),
            (x,_) => return Err(refuse(Condition::Envelope,"the envelope does not close: a piece of it ends at no corner \
                of the curve",Some(if x.is_none() { a } else { b }))),
        }
    }
    for k in 0..corners.len() {
        let meets = ends.iter().flatten().filter(|&&c| c == k).count();
        if meets != 2 && meets != 0 {
            return Err(refuse(Condition::Envelope,format!("the envelope branches: {meets} pieces meet at one corner"),Some(corners[k])))
        }
    }
    let mut order: Vec<(usize,bool)> = vec![(0,true)];
    let mut used = vec![false;pieces.len()];
    used[0] = true;
    let mut at = ends[0][1];
    while at != ends[0][0] {
        let Some((k,forward)) = (0..pieces.len()).filter(|&k| !used[k])
            .find_map(|k| if ends[k][0] == at { Some((k,true)) } else if ends[k][1] == at { Some((k,false)) } else { None }) else {
            return Err(refuse(Condition::Envelope,"the envelope does not close into one loop",Some(corners[at])))
        };
        used[k] = true;
        order.push((k,forward));
        at = if forward { ends[k][1] } else { ends[k][0] };
    }
    if used.iter().any(|u| !u) {
        return Err(refuse(Condition::Envelope,"the envelope is more than one loop",None))
    }
    let mut out: Vec<Vec<Contact>> = order.into_iter().map(|(k,forward)| {
        let mut p = pieces[k].clone();
        if !forward { p.reverse() }
        let (a,b) = (ends[k][usize::from(!forward)],ends[k][usize::from(forward)]);
        p.insert(0,Contact {s:f64::NAN,roll:f64::NAN,at:corners[a]});
        p.push(Contact {s:f64::NAN,roll:f64::NAN,at:corners[b]});
        p
    }).collect();
    // crossing: no two chords of the loop meet but at a shared end
    let chords: Vec<(P,P)> = out.iter().flat_map(|p| p.windows(2).map(|w| (w[0].at,w[1].at))).filter(|(a,b)| dist(*a,*b) > 0.).collect();
    let n = chords.len();
    let boxed = |(a,b): (P,P)| crate::bvh::Bounds {lo:[a[0].min(b[0]),a[1].min(b[1])],hi:[a[0].max(b[0]),a[1].max(b[1])]};
    let tree = crate::bvh::Bvh::new(chords.iter().map(|&c| boxed(c)));
    for i in 0..n {
        let mut crossed = false;
        tree.query(|b| b.overlaps(boxed(chords[i])),|j| {
            if crossed || j < i+2 || (i == 0 && j == n-1) { return }
            let ((a,b),(c,d)) = (chords[i],chords[j]);
            let (d1,d2) = (cross(sub(b,a),sub(c,a)),cross(sub(b,a),sub(d,a)));
            let (d3,d4) = (cross(sub(d,c),sub(a,c)),cross(sub(d,c),sub(b,c)));
            // where two pieces meet, at a corner, their last chords may graze within the march's reach
            let at_corner = || corners.iter().any(|&k| [a,b,c,d].iter().any(|&p| dist(p,k) <= join));
            crossed = d1*d2 < 0. && d3*d4 < 0. && !at_corner();
        });
        if crossed { return Err(refuse(Condition::Envelope,"the envelope crosses itself",Some(chords[i].0))) }
    }
    // counter-clockwise
    let area: f64 = chords.iter().map(|(a,b)| cross(*a,*b)).sum();
    if area < 0. {
        out.reverse();
        for p in &mut out { p.reverse() }
    }
    Ok(out)
}
