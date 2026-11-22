//! **An upright tool of revolution under a motion keeping its plane, swept closed** (#880 T1f):
//! where every pose of the motion turns one plane in itself, the same face up, and the tool is a
//! convex solid of revolution whose axis stands along that plane's normal, each of the tool's
//! sections square to its axis is a disc centred on the axis, which a turn about the axis leaves
//! where it is. So the section at height `h` sweeps the disc of its radius `ρ(h)` carried along the
//! path `c` the axis's foot runs in the plane — `c ⊕ D(ρ)`, the points within `ρ` of the path —
//! and the swept solid is `{(p, h) : d(p) ≤ ρ(h)}`, `d` the distance from the path: its boundary
//! the graph of the meridian over the path's distance, which no reading of the tool's contact
//! under the motion's twist (K2's pencil, the contact's road) has to find. Under a translation
//! the tool's flat faces are touched whole (`n · v ≡ 0`) and its corner's meridians square to the
//! velocity touched whole; here they are what they are, a cap and the sheets below.
//!
//! **The sheets.** Each piece of the meridian (`patches::Revolved`, walked from the axis below
//! round to the axis above) is a slab over its own parameter `σ ∈ [0, 1]`, its radius `ρ(σ)` and
//! height `h(σ)`. At each `σ` the region `c ⊕ D(ρ(σ))` is read exactly (`region::SweptRegion`, the
//! disc its section): its boundary runs along the path's offsets `c(t) ± ρ N(t)` (each a
//! **side**, `N` the path's left normal) and along the disc at either end pose (each an **end**,
//! by its angle `φ` about the path's end). Over the slab each is a sheet in `(t, σ)` or `(φ, σ)`:
//! a side the offset lifted to `h(σ)`, an end the tool's own face posed at the roll's end. A sheet
//! is part of the boundary where no other instant comes nearer than `ρ` (the **cover**, read by
//! the path's distance, certified by its speed between samples), and its pieces are bounded by
//! where two sheets' points coincide (**double curves**: two instants' contacts at one point, the
//! region's corners lifted through the slab), traced exactly by continuation in `(p₁, p₂, σ)`.
//! A double curve ends on a rim (the slab's end), on a join (a side meeting its end's disc along
//! the end pose's meridian), where a third instant covers it (a **triple point**, solved exactly,
//! three curves meeting) or where its two instants merge (a **birth**: a swallowtail born on a side
//! where the path's radius of curvature passes `ρ`, its point found as the limit of the curve's
//! divided form). A curve turning back in `σ` (the apex of an island the slab leaves, the stock's
//! mound inside a loop) is traced through its turn.
//!
//! **The faces.** On each sheet the curves, its rims and joins, and from each birth a cut straight
//! across the slab to the rim the swallowtail opens away from (so no face meets itself along its
//! double curve), part its domain; the parts are walked as a planar graph and each kept where the
//! cover says it bounds the sweep. A side's face is a net fitted over its part of `(t, σ)` within a
//! tenth of the tolerance; an end's is the tool's face posed; a cap (a flat end) the region at its
//! height. The faces are sewn (`sew::sewn`), nothing intersected; the closed solid is judged by its
//! definition and cut from the blank.
//!
//! What the road does not read is refused by name: a double curve closing on itself, a cut from a
//! birth meeting another curve, a part of a sheet whose neighbours across a curve are both kept.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::util::fmath::{HALF_READ,THIRD_READ};
use crate::geom::{Curve,Frame,Surface};
use crate::nurbs::{BSpline,Net};
use crate::topo::{Brep,Coedge,Edge,EdgeCurve,Face,Pcurve};
use crate::trajectory::Trajectory;
use crate::util::space::{dot,sub};
use super::closed::Unbuilt;
use super::patches::{Element,Revolved};
use super::region::{Piece,Plane,SweptRegion};
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];
/// A point of a double curve: its two sheets' parameters and the slab's.
type X = [f64;3];

fn debug() -> bool { std::env::var_os("SOLVENT_UPRIGHT_DEBUG").is_some() }

fn p_add(a: P,b: P) -> P { [a[0]+b[0],a[1]+b[1]] }
fn p_sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn p_scale(a: P,k: f64) -> P { [a[0]*k,a[1]*k] }
fn p_dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1] }
fn p_len(a: P) -> f64 { a[0].dhypot(a[1]) }
fn left(a: P) -> P { [-a[1],a[0]] }

/// What a point of the region's boundary lies on: a side, the path's offset to its left (`1`) or
/// right (`-1`), by its roll; or the disc at the roll's start (`0`) or end (`1`), by its angle.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Sheet { Side(i8),End(u8) }

/// A piece of the meridian as a slab: its radius and height over `σ ∈ [0, 1]`, and their rates.
#[derive(Clone,Copy,Debug)]
struct Slab { element: Element,face: usize }

impl Slab {
    /// `(ρ, h, ρ', h')` at `σ`.
    fn at(&self,s: f64) -> [f64;4] {
        match self.element {
            Element::Line {a,b} => [a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1]),b[0]-a[0],b[1]-a[1]],
            Element::Arc {c,r,from,sweep} => {
                let (sn,cs) = (from+s*sweep).dsin_cos();
                [c[0]+r*cs,c[1]+r*sn,-r*sn*sweep,r*cs*sweep]
            }
        }
    }
    /// Whether it lies level, a flat end (a cap): no slab, a face of the region at its height.
    fn level(&self) -> bool { matches!(self.element,Element::Line {a,b} if a[1] == b[1]) }
}

/// **What the road reads**: the plane kept, its origin on the tool's axis; the path and roll; the
/// tool's meridian; the path's feet sampled for the cover; the tolerance.
pub struct Stand<'a> {
    path: &'a Trajectory,
    roll: [f64;2],
    plane: Plane,
    slabs: Vec<Slab>,
    tool: &'a Brep,
    /// The path's ends, `c(a)` and `c(b)`, and their unit velocities.
    ends: [P;2],
    heads: [P;2],
    /// The feet `(t, c(t))` at evenly spaced rolls, and the most the path moves between two.
    feet: Vec<(f64,P)>,
    reach: f64,
    /// The roll's grain at the tolerance, as a share of the roll (`closed::grain`): a trace's step
    /// shorter than it moves nothing the tolerance sees.
    grain: f64,
    tol: f64,
    scale: f64,
}

/// How many rolls the path's feet are sampled at for the cover: its certified minimum refines
/// only the stretches its bound cannot clear.
const FEET: usize = 1024;

impl<'a> Stand<'a> {
    /// The road's reading of `tool` under `path` over `roll`; none where the tool is no convex
    /// solid of revolution with flat ends or the motion keeps no plane square to its axis.
    pub fn read(tool: &'a Brep,path: &'a Trajectory,roll: [f64;2],tol: f64) -> Result<Option<Stand<'a>>,Unbuilt> {
        let roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
        if !(roll[0] < roll[1]) { return Ok(None) }
        let Ok(rev) = Revolved::read(tool) else { return Ok(None) };
        let g = rev.frame;
        let n = rev.elements.len();
        // flat ends, from the axis: the region of a disc of no radius is the path, no region
        let flat = |e: &Element| matches!(*e,Element::Line {a,b} if a[1] == b[1]);
        if n < 3 || !flat(&rev.elements[0]) || !flat(&rev.elements[n-1]) { return Ok(None) }
        let plane = Plane {o:g.o,u:g.x,v:g.y,n:g.z};
        // every pose keeps the plane, the same face up (`region::keeps`, at the region's own grid's
        // first readings)
        let size = tool.size();
        for k in 0..=16 {
            let t = roll[0]+(roll[1]-roll[0])*k as f64/16.;
            let Ok(m) = path.at(t) else { return Ok(None) };
            if !super::region::keeps(&m,g.o,g.z,size) { return Ok(None) }
        }
        let slabs = rev.elements.iter().zip(&rev.faces).map(|(&element,&face)| Slab {element,face}).collect();
        let mut stand = Stand {path,roll,plane,slabs,tool,ends:[[0.;2];2],heads:[[0.;2];2],feet:Vec::new(),reach:0.,grain:0.,tol,scale:size};
        let feet: Vec<(f64,P)> = (0..=FEET).map(|i| { let t = roll[0]+(roll[1]-roll[0])*i as f64/FEET as f64; (t,stand.foot(t).0) }).collect();
        if feet.iter().any(|f| !f.1[0].is_finite() || !f.1[1].is_finite()) { return Err(Unbuilt::Failed("the path cannot be read over its roll".into())) }
        stand.reach = feet.windows(2).map(|w| p_len(p_sub(w[1].1,w[0].1))).fold(0.,f64::max);
        // (the bound between two samples: a stretch moves at most its speed times its length, read
        // here as the greatest step sampled grown by the path's own enclosure where it gives one)
        if let Some(s) = match path { Trajectory::General(p) => p.stretch(roll[0],roll[1]),_ => None } { stand.reach = stand.reach.max(s.speed*(roll[1]-roll[0])/FEET as f64) }
        stand.feet = feet;
        stand.grain = super::closed::grain(tool,path,roll,tol)?/(roll[1]-roll[0]);
        for e in 0..2 {
            let (c,v) = stand.foot(roll[e]);
            stand.ends[e] = c;
            let l = p_len(v);
            if !(l > 0.) { return Err(Unbuilt::Failed(format!("the path stands still at its {}",["start","end"][e]))) }
            stand.heads[e] = p_scale(v,1./l);
        }
        Ok(Some(stand))
    }

    /// The axis's foot in the plane at `t` and its velocity (NaN where the path cannot be read).
    fn foot(&self,t: f64) -> (P,P) {
        let Ok(m) = self.path.at(t) else { return ([f64::NAN;2],[f64::NAN;2]) };
        let o = self.plane.o;
        let w = sub(m.point(o),o);
        let v = m.velocity(o);
        ([dot(w,self.plane.u),dot(w,self.plane.v)],[dot(v,self.plane.u),dot(v,self.plane.v)])
    }

    /// The path's left unit normal at `t`.
    fn normal(&self,t: f64) -> P { let v = self.foot(t).1; p_scale(left(v),1./p_len(v)) }

    /// The way a sheet's point moves as its radius grows: the side's normal, or the end's radius.
    fn dir(&self,sheet: Sheet,p: f64) -> P {
        match sheet {
            Sheet::Side(s) => p_scale(self.normal(p),s as f64),
            Sheet::End(_) => { let (sn,cs) = p.dsin_cos(); [cs,sn] }
        }
    }

    /// The sheet's point at its parameter `p` and radius `rho`, in the plane.
    fn at(&self,sheet: Sheet,p: f64,rho: f64) -> P {
        match sheet {
            Sheet::Side(_) => p_add(self.foot(p).0,p_scale(self.dir(sheet,p),rho)),
            Sheet::End(e) => p_add(self.ends[e as usize],p_scale(self.dir(sheet,p),rho)),
        }
    }

    /// The sheet's point's rate along its parameter: the path's velocity and the normal's turning
    /// carried out to `rho` (read by central differences at the cube root of the rounding of the
    /// roll, where their rounding and their truncation balance), or the disc's tangent.
    fn rate(&self,sheet: Sheet,p: f64,rho: f64) -> P {
        match sheet {
            Sheet::Side(s) => {
                let d = THIRD_READ*(1.+(self.roll[1]-self.roll[0]).abs().max(p.abs()));
                let turn = p_scale(p_sub(self.normal(p+d),self.normal(p-d)),0.5/d);
                p_add(self.foot(p).1,p_scale(turn,s as f64*rho))
            }
            Sheet::End(_) => { let (sn,cs) = p.dsin_cos(); [-rho*sn,rho*cs] }
        }
    }

    /// The point in space of slab `k`'s sheet at `(p, σ)`.
    fn point(&self,k: usize,sheet: Sheet,p: f64,s: f64) -> V {
        let [rho,h,..] = self.slabs[k].at(s);
        self.plane.world(self.at(sheet,p,rho),h)
    }

    /// **The path's nearest point to `q`**: its distance and roll, certified — every stretch
    /// between two sampled feet whose bound (the nearer foot less half the stretch's reach) does
    /// not clear the best found is read by a golden section.
    fn nearest(&self,q: P) -> (f64,f64) {
        let d: Vec<f64> = self.feet.iter().map(|f| p_len(p_sub(f.1,q))).collect();
        let (mut best,mut at) = (f64::INFINITY,self.roll[0]);
        for (i,&x) in d.iter().enumerate() { if x < best { best = x; at = self.feet[i].0 } }
        let mut order: Vec<usize> = (0..FEET).collect();
        order.sort_by(|&i,&j| d[i].min(d[i+1]).total_cmp(&d[j].min(d[j+1])));
        for i in order {
            if d[i].min(d[i+1])-0.5*self.reach >= best { break }
            let f = |t: f64| p_len(p_sub(self.foot(t).0,q));
            let (mut lo,mut hi) = (self.feet[i].0,self.feet[i+1].0);
            let g = 0.5*(5f64.sqrt()-1.);
            let (mut x1,mut x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
            let (mut f1,mut f2) = (f(x1),f(x2));
            while hi-lo > 4.*f64::EPSILON*(1.+lo.abs()) {
                if f1 <= f2 { hi = x2; x2 = x1; f2 = f1; x1 = hi-g*(hi-lo); f1 = f(x1) }
                else { lo = x1; x1 = x2; f1 = f2; x2 = lo+g*(hi-lo); f2 = f(x2) }
            }
            let t = 0.5*(lo+hi);
            let x = f(t);
            if x < best { best = x; at = t }
        }
        (best,at)
    }
}

/// Two sheets in order, the first the lesser.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
struct Pair { a: Sheet,b: Sheet }

/// A corner of the region at one `σ`: two sheets' points at one place.
#[derive(Clone,Copy,Debug)]
struct Corner { pair: Pair,x: X }

/// A visible stretch of one sheet at one `σ`, from its first parameter to its last as the region
/// walks it.
#[derive(Clone,Copy,Debug)]
struct Run { sheet: Sheet,p: [f64;2] }

/// One loop of the region at one `σ`: its runs in order.
#[derive(Clone,Debug)]
struct Ring { runs: Vec<Run> }

/// Solves the `n × n` system `a x = b` by Gaussian elimination with partial pivoting.
fn solve<const N: usize>(mut a: [[f64;N];N],mut b: [f64;N]) -> Option<[f64;N]> {
    for c in 0..N {
        let p = (c..N).max_by(|&i,&j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if !(a[p][c].abs() > 0.) { return None }
        a.swap(c,p); b.swap(c,p);
        for r in c+1..N {
            let f = a[r][c]/a[c][c];
            for k in c..N { a[r][k] -= f*a[c][k] }
            b[r] -= f*b[c];
        }
    }
    let mut x = [0.;N];
    for c in (0..N).rev() {
        let s: f64 = (c+1..N).map(|k| a[c][k]*x[k]).sum();
        x[c] = (b[c]-s)/a[c][c];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// How many Newton steps a point is settled in, at most.
const NEWTON: usize = 40;

impl Stand<'_> {
    /// The parameters of the sheets an end's disc may be read on: an end's domain is the half of
    /// its disc the sweep leaves behind it (`[lo, hi]`, the start's behind it, the end's ahead),
    /// between its two joins; a side's the roll.
    fn domain(&self,sheet: Sheet) -> [f64;2] {
        match sheet {
            Sheet::Side(_) => self.roll,
            Sheet::End(e) => {
                let h = self.heads[e as usize];
                let w = h[1].datan2(h[0]);
                if e == 0 { [w+0.5*PI,w+1.5*PI] } else { [w-0.5*PI,w+0.5*PI] }
            }
        }
    }

    /// An end's angle `p` turned by whole turns into its domain (or nearest it).
    fn wrapped(&self,sheet: Sheet,p: f64) -> f64 {
        let Sheet::End(_) = sheet else { return p };
        let [lo,hi] = self.domain(sheet);
        let mid = 0.5*(lo+hi);
        p-TAU*((p-mid)/TAU).round()
    }

    /// The join where a side meets an end's disc: the side's roll there, and the end's angle.
    fn join(&self,side: i8,e: u8) -> (f64,f64) {
        let [lo,hi] = self.domain(Sheet::End(e));
        // (the start's domain runs from its left join round behind it to its right; the end's
        // from its right join round ahead to its left)
        let phi = match (e,side) { (0,1) | (1,-1) => lo,_ => hi };
        (self.roll[e as usize],phi)
    }

    /// The pair's two points' difference at `x` on slab `k`, and its Jacobian in `(p₁, p₂, σ)`.
    fn residual(&self,k: usize,pair: Pair,x: X) -> (P,[P;3]) {
        let [rho,_,drho,_] = self.slabs[k].at(x[2]);
        let f = p_sub(self.at(pair.a,x[0],rho),self.at(pair.b,x[1],rho));
        let ja = self.rate(pair.a,x[0],rho);
        let jb = p_scale(self.rate(pair.b,x[1],rho),-1.);
        let js = p_scale(p_sub(self.dir(pair.a,x[0]),self.dir(pair.b,x[1])),drho);
        (f,[ja,jb,js])
    }

    /// The pair's residual's Jacobian in its own chart, and the map from a step there to a step in
    /// `(p₁, p₂, σ)`: for two instants of one sheet the chart `(m, h, σ)` of their middle and half
    /// their parting (`p₁ = m − h`, `p₂ = m + h`), whose residual is divided by `2h` — every place
    /// of the sheet is its own double point at `h = 0`, a plane of roots Newton's method falls onto
    /// beside a birth, which the divided form leaves out; for two sheets `(p₁, p₂, σ)` itself.
    /// (Its rows are the divided residual's times `2h`, so the residual stays `F`: the parting's
    /// column `∂F/∂h − F/h`.)
    fn chart(&self,pair: Pair,x: X,f: P,j: [P;3]) -> ([P;3],fn(X) -> X) {
        if pair.a != pair.b { return (j,|d| d) }
        let h = 0.5*(x[1]-x[0]);
        ([p_add(j[0],j[1]),p_sub(p_sub(j[1],j[0]),p_scale(f,1./h)),j[2]],|d| [d[0]-d[1],d[0]+d[1],d[2]])
    }

    /// The pair's point settled from `x` onto the double curve on the plane `row · y = at`.
    fn settle(&self,k: usize,pair: Pair,mut x: X,row: X,at: f64) -> Option<X> {
        let size = 1.+self.scale;
        let mut last = f64::INFINITY;
        for _ in 0..NEWTON {
            let (f,j) = self.residual(k,pair,x);
            let r = p_len(f);
            let (jc,back) = self.chart(pair,x,f,j);
            // (the constraint read in the chart: `row · back(d)`)
            let rc = [dot(row,back([1.,0.,0.])),dot(row,back([0.,1.,0.])),dot(row,back([0.,0.,1.]))];
            let a = [[jc[0][0],jc[1][0],jc[2][0]],[jc[0][1],jc[1][1],jc[2][1]],rc];
            let d = solve(a,[-f[0],-f[1],at-dot(row,x)])?;
            let dx = back(d);
            x = [x[0]+dx[0],x[1]+dx[1],x[2]+dx[2]];
            let step = dx.iter().fold(0f64,|m,d| m.max(d.abs()));
            if r <= 4.*f64::EPSILON*size && step <= 4.*f64::EPSILON*(1.+x.iter().fold(0f64,|m,v| m.max(v.abs()))) { return Some(x) }
            // (settled where the residual stops falling at the rounding)
            if r >= last && r <= 1e3*f64::EPSILON*size { return Some(x) }
            last = r;
        }
        let (f,_) = self.residual(k,pair,x);
        (p_len(f) <= HALF_READ*self.tol.max(f64::EPSILON*size)).then_some(x)
    }

    /// The double curve's way at `x`: the null vector of its Jacobian in its chart, unit.
    fn way(&self,k: usize,pair: Pair,x: X) -> Option<X> {
        let (f,j) = self.residual(k,pair,x);
        let (j,back) = self.chart(pair,x,f,j);
        let r0 = [j[0][0],j[1][0],j[2][0]];
        let r1 = [j[0][1],j[1][1],j[2][1]];
        crate::util::space::normalised(back(crate::util::space::cross(r0,r1)))
    }
}

/// How a traced double curve ends.
#[derive(Clone,Copy,Debug,PartialEq)]
enum End {
    /// On the slab's rim at `σ = 0` or `1`.
    Rim,
    /// On a join, a side's parameter at its roll's end or an end's at its join.
    Join,
    /// Where a third instant reaches it: the triple point by its index.
    Triple(usize),
    /// Where its two instants merge on a side: the birth by its index.
    Birth(usize),
}

/// **A double curve of a slab**: its pair, its points in order, how each end ends, and its curve in
/// space once fitted.
#[derive(Clone,Debug)]
struct Trace { pair: Pair,points: Vec<X>,ends: [End;2] }

/// A point three instants reach at once: each sheet's point and the slab's `σ`.
#[derive(Clone,Debug)]
struct Triple { feet: Vec<(Sheet,f64)>,s: f64 }

/// A swallowtail's birth on a side: the side, the roll and `σ` where its two instants merge, and
/// which way along the slab its double curve opens (`1` up, `-1` down).
#[derive(Clone,Debug)]
struct Birth { side: i8,t: f64,s: f64,opens: f64,
    /// The curve's stretch into it in its normal form: the half parting `h` it is read from, and
    /// `m` and `σ` as quadratics in `h²` (each the coefficients of `1, h², h⁴`).
    reach: f64,m: [f64;3],sigma: [f64;3],
    /// How many traced points stand past the one it was reached from (the last the birth).
    count: usize }

impl Birth {
    /// `(m, σ)` at the half parting `h`.
    fn at(&self,h: f64) -> (f64,f64) {
        let q = h*h;
        let v = |c: [f64;3]| c[0]+q*(c[1]+q*c[2]);
        (v(self.m),v(self.sigma))
    }
}

/// What a slab's double curves are traced into.
#[derive(Default,Debug)]
struct Curves { traces: Vec<Trace>,triples: Vec<Triple>,births: Vec<Birth> }

/// The largest step a trace takes in `(p₁, p₂, σ)`, and its first.
const MOST_STEP: f64 = 1./64.;
/// The turn between two steps' ways past which a step is halved (radians).
const MOST_TURN: f64 = 0.1;

impl Stand<'_> {
    /// Whether the point `x` of the pair lies where a third instant covers it more than the
    /// tolerance deep, and the roll of the nearest.
    fn covered(&self,k: usize,pair: Pair,x: X) -> (bool,f64,f64) {
        let [rho,..] = self.slabs[k].at(x[2]);
        let q = self.at(pair.a,x[0],rho);
        let (d,t) = self.nearest(q);
        (d < rho-self.tol,t,d-rho)
    }

    /// Whether `x` has left its pair's domains: `σ` past the slab, a parameter past its sheet's.
    fn outside(&self,pair: Pair,x: X) -> Option<(usize,f64)> {
        if x[2] < 0. { return Some((2,0.)) }
        if x[2] > 1. { return Some((2,1.)) }
        for (i,sheet) in [pair.a,pair.b].into_iter().enumerate() {
            let [lo,hi] = self.domain(sheet);
            if x[i] < lo { return Some((i,lo)) }
            if x[i] > hi { return Some((i,hi)) }
        }
        None
    }

    /// **The triple point** between `x0` (uncovered) and `x1` (covered by the instant at `t3`):
    /// the pair's point and the third foot settled together by Newton's method — `|q − c(t₃)| = ρ`
    /// and `(q − c(t₃)) · c'(t₃) = 0`, or at an end of the roll the first alone.
    fn triple(&self,k: usize,pair: Pair,x0: X,x1: X) -> Option<(X,Sheet,f64)> {
        // the bracket halved to where the cover's sign changes
        let (mut lo,mut hi) = (x0,x1);
        let w = sub(x1,x0);
        for _ in 0..60 {
            let m = [0.5*(lo[0]+hi[0]),0.5*(lo[1]+hi[1]),0.5*(lo[2]+hi[2])];
            let m = self.settle(k,pair,m,w,dot(w,m))?;
            if self.covered(k,pair,m).2 < 0. { hi = m } else { lo = m }
            if dot(sub(hi,lo),sub(hi,lo)) <= (f64::EPSILON*4.).powi(2) { break }
        }
        // (the third foot read where the cover is plain, at the covered end of the step: at the
        // bracket's end the three feet tie to the rounding)
        let (_,t3,_) = self.covered(k,pair,x1);
        let [a,b] = self.roll;
        let at_end = |t: f64| (t-a).abs() <= HALF_READ*(b-a) || (b-t).abs() <= HALF_READ*(b-a);
        let g = |y: [f64;4]| -> [f64;4] {
            let [rho,..] = self.slabs[k].at(y[2]);
            let (f,_) = self.residual(k,pair,[y[0],y[1],y[2]]);
            let q = self.at(pair.a,y[0],rho);
            let (c3,v3) = self.foot(y[3]);
            let r = p_sub(q,c3);
            [f[0],f[1],p_len(r)-rho,if at_end(t3) { y[3]-t3 } else { p_dot(r,p_scale(v3,1./p_len(v3))) }]
        };
        let mut y = [lo[0],lo[1],lo[2],t3];
        let h = THIRD_READ;
        for _ in 0..NEWTON {
            let r = g(y);
            let mut jac = [[0.;4];4];
            for c in 0..4 {
                let (mut yp,mut ym) = (y,y);
                yp[c] += h; ym[c] -= h;
                let (gp,gm) = (g(yp),g(ym));
                for row in 0..4 { jac[row][c] = (gp[row]-gm[row])/(2.*h) }
            }
            let dy = solve(jac,[-r[0],-r[1],-r[2],-r[3]])?;
            for i in 0..4 { y[i] += dy[i] }
            if dy.iter().fold(0f64,|m,d| m.max(d.abs())) <= 4.*f64::EPSILON*(1.+self.scale) { break }
        }
        let r = g(y);
        // (settled to the rounding of the plane's lengths)
        if !(r.iter().fold(0f64,|m,v| m.max(v.abs())) <= 1e3*f64::EPSILON*(1.+self.scale)) { return None }
        let x = self.settle(k,pair,[y[0],y[1],y[2]],[0.,0.,1.],y[2])?;
        // which sheet the third instant's point lies on: a side by its normal's sense, or an end
        let [rho,..] = self.slabs[k].at(x[2]);
        let q = self.at(pair.a,x[0],rho);
        let sheet = if at_end(t3) {
            let e = if (t3-a).abs() <= (b-t3).abs() { 0 } else { 1 };
            let r = p_sub(q,self.ends[e]);
            let side = p_dot(r,self.normal(self.roll[e]));
            // (an end's point square to the path there is the side's join; elsewhere the disc's)
            if (p_dot(r,self.heads[e]).abs()) <= HALF_READ*rho { Sheet::Side(side.signum() as i8) } else { Sheet::End(e as u8) }
        } else { Sheet::Side(p_dot(p_sub(q,self.foot(y[3]).0),self.normal(y[3])).signum() as i8) };
        let p3 = match sheet { Sheet::Side(_) => y[3],Sheet::End(e) => { let r = p_sub(q,self.ends[e as usize]); self.wrapped(sheet,r[1].datan2(r[0])) } };
        Some((x,sheet,p3))
    }
}

impl Stand<'_> {
    /// **The region a disc of radius `rho` sweeps** (`region::SweptRegion`), read within a box
    /// holding every pose of it.
    fn region(&self,rho: f64) -> Result<SweptRegion,Unbuilt> {
        use crate::plane2::Edge as E;
        let section = super::subtraction::section_of(vec![vec![E::Arc {center:[0.,0.],radius:rho,start:0.,sweep:TAU,ends:[[rho,0.],[rho,0.]]}]]);
        let setting = super::region::Setting {section,path:self.path.clone(),plane:self.plane,roll:self.roll,scale:self.scale};
        let whole = setting.alike(self.roll[0],self.roll[1]);
        let bounds = super::planar_solid::holding(&setting,self.roll);
        SweptRegion::read(setting,bounds,whole).map_err(|r| Unbuilt::Failed(format!("the region a disc of radius {rho} sweeps: {}",
            r.words("the motion"))))
    }

    /// The region's loops as runs of the sheets, each run's ends its corners'.
    fn rings(&self,region: &SweptRegion,rho: f64) -> Result<Vec<Ring>,Unbuilt> {
        let mut out = Vec::new();
        for (l,lp) in region.loops.iter().enumerate() {
            let n = lp.pieces.len();
            let mut runs = Vec::with_capacity(n);
            for (k,piece) in lp.pieces.iter().enumerate() {
                let Piece::Run {candidate,from,to} = *piece else {
                    return Err(Unbuilt::Failed(format!("the region a disc of radius {rho} sweeps reaches past the box held about it")))
                };
                let (a,b) = (lp.corners[k],lp.corners[(k+1)%n]);
                runs.push(match region.held(candidate) {
                    Some(at) => {
                        let e = if (at-self.roll[0]).abs() <= (self.roll[1]-at).abs() { 0 } else { 1 };
                        let sheet = Sheet::End(e as u8);
                        let angle = |q: P| { let r = p_sub(q,self.ends[e]); self.wrapped(sheet,r[1].datan2(r[0])) };
                        Run {sheet,p:[angle(a),angle(b)]}
                    }
                    None => {
                        let mid = region.point(l,k,0.5).ok_or_else(|| Unbuilt::Failed("a region's run cannot be read".into()))?;
                        let t = 0.5*(from+to);
                        let off = |s: i8| p_len(p_sub(self.at(Sheet::Side(s),t,rho),mid));
                        Run {sheet:Sheet::Side(if off(1) <= off(-1) { 1 } else { -1 }),p:[from,to]}
                    }
                });
            }
            // (a run going on where the last ended — the section's own seam, or an end's angle turning
            // past its edge's seam — is one with it: no corner between them)
            let on = |a: &Run,b: &Run| a.sheet == b.sheet && (a.p[1]-b.p[0]).abs() <= HALF_READ*(1.+a.p[1].abs());
            let mut merged: Vec<Run> = Vec::with_capacity(runs.len());
            for r in runs {
                match merged.last_mut() { Some(last) if on(last,&r) => last.p[1] = r.p[1],_ => merged.push(r) }
            }
            while merged.len() > 1 && on(&merged[merged.len()-1],&merged[0]) {
                let last = merged.pop().expect("runs");
                merged[0].p[0] = last.p[0];
            }
            out.push(Ring {runs:merged});
        }
        Ok(out)
    }

    /// Whether two runs' meeting is a join: a side's run ending at its roll's end onto that end's
    /// disc, the one surface running on into the other.
    fn is_join(&self,(a,pa): (Sheet,f64),(b,pb): (Sheet,f64)) -> bool {
        let one = |x: Sheet,px: f64,y: Sheet| matches!((x,y),(Sheet::Side(_),Sheet::End(e)) if px == self.roll[e as usize]);
        one(a,pa,b) || one(b,pb,a)
    }

    /// The corners of the rings at `σ` of slab `k`, joins left out, each settled there.
    fn corners(&self,k: usize,s: f64,rings: &[Ring]) -> Result<Vec<Corner>,Unbuilt> {
        let mut out = Vec::new();
        for ring in rings {
            let n = ring.runs.len();
            for i in 0..n {
                let (r0,r1) = (ring.runs[(i+n-1)%n],ring.runs[i]);
                let (a,b) = ((r0.sheet,r0.p[1]),(r1.sheet,r1.p[0]));
                if self.is_join(a,b) { continue }
                let ((a,pa),(b,pb)) = if a <= b { (a,b) } else { (b,a) };
                let pair = Pair {a,b};
                let x = self.settle(k,pair,[pa,pb,s],[0.,0.,1.],s)
                    .ok_or_else(|| Unbuilt::Failed(format!("a corner of the region at σ {s} of slab {k} cannot be settled")))?;
                out.push(Corner {pair,x});
            }
        }
        Ok(out)
    }
}

/// How many stations a curved slab's corners are read at, past its first.
const STATIONS: usize = 16;

impl Stand<'_> {
    /// **A slab's double curves**: traced from every corner the region shows at its stations that
    /// no curve traced already passes.
    fn curves(&self,k: usize) -> Result<Curves,Unbuilt> {
        let mut c = Curves::default();
        let stations = if matches!(self.slabs[k].element,Element::Line {a,b} if a[0] == b[0]) { 1 } else { STATIONS };
        for j in 0..=stations {
            let s = j as f64/stations as f64;
            let [rho,..] = self.slabs[k].at(s);
            let rings = self.rings(&self.region(rho)?,rho)?;
            for corner in self.corners(k,s,&rings)? {
                let near = |t: &Trace| t.pair == corner.pair && t.points.iter().any(|y|
                    (0..3).all(|i| (y[i]-corner.x[i]).abs() <= MOST_STEP));
                if c.traces.iter().any(near) { continue }
                self.trace(k,&mut c,corner)?;
            }
        }
        if debug() {
            eprintln!("upright: slab {k}: {} curves, {} triple points, {} births",c.traces.len(),c.triples.len(),c.births.len());
            for t in &c.traces { eprintln!("  {:?}: {} points, {:?} .. {:?}, ends {:?}",t.pair,t.points.len(),t.points[0],t.points[t.points.len()-1],t.ends) }
            for t in &c.triples { eprintln!("  triple {:?}",t) }
            for b in &c.births { eprintln!("  birth {:?}",b) }
        }
        Ok(c)
    }

    /// The double curve through `corner`, traced both ways to its ends.
    fn trace(&self,k: usize,c: &mut Curves,corner: Corner) -> Result<(),Unbuilt> {
        let pair = corner.pair;
        let w = self.way(k,pair,corner.x).ok_or_else(|| Unbuilt::Failed(format!("a double curve's way at {:?} cannot be read",corner.x)))?;
        let (mut back,e0) = self.walk(k,c,pair,corner.x,crate::util::space::scale(w,-1.))?;
        let (ahead,e1) = self.walk(k,c,pair,corner.x,w)?;
        back.reverse();
        back.push(corner.x);
        back.extend(ahead);
        // (a corner on a rim is its curve's end there: the walk off the rim settled back onto it)
        back.dedup_by(|y,x| (0..3).all(|i| (y[i]-x[i]).abs() <= 4.*f64::EPSILON*(1.+x[i].abs())));
        c.traces.push(Trace {pair,points:back,ends:[e0,e1]});
        Ok(())
    }

    /// One way of a double curve from `x` along `w`: its points past `x` and how it ends.
    fn walk(&self,k: usize,c: &mut Curves,pair: Pair,x0: X,w0: X) -> Result<(Vec<X>,End),Unbuilt> {
        let refused = |why: &str,x: X| Unbuilt::Failed(format!("a double curve of slab {k} ({:?}) {why} at {:?}",pair,x));
        let (mut x,mut w) = (x0,w0);
        let mut h = MOST_STEP/4.;
        let mut out: Vec<X> = Vec::new();
        let same_side = pair.a == pair.b && matches!(pair.a,Sheet::Side(_));
        for _ in 0..100_000 {
            let pred = [x[0]+h*w[0],x[1]+h*w[1],x[2]+h*w[2]];
            let Some(y) = self.settle(k,pair,pred,w,dot(w,pred)) else {
                h *= 0.5;
                if h < self.grain { return Err(refused("cannot be continued",x)) }
                continue
            };
            let Some(mut w2) = self.way(k,pair,y) else { return Err(refused("loses its way",y)) };
            if dot(w2,w) < 0. { w2 = crate::util::space::scale(w2,-1.) }
            // (a turn within the grain is no turn the tolerance sees)
            if dot(w2,w) < MOST_TURN.dcos() && h > self.grain { h *= 0.5; continue }
            // past a domain's edge: settled on it
            if let Some((i,bound)) = self.outside(pair,y) {
                let mut row = [0.;3];
                row[i] = 1.;
                let z = self.settle(k,pair,x,row,bound).ok_or_else(|| refused("cannot be settled on its domain's edge",y))?;
                out.push(z);
                return Ok((out,if i == 2 { End::Rim } else { End::Join }))
            }
            // two instants of one side merging: a swallowtail's birth
            if same_side && (y[1]-y[0]).abs() < (x[1]-x[0]).abs() && 0.5*(y[1]-y[0]).abs() <= MOST_STEP {
                let (pts,b) = self.birth(k,pair,y).ok_or_else(|| refused("does not settle where its instants merge",y))?;
                out.push(y);
                out.extend(pts);
                c.births.push(b);
                return Ok((out,End::Birth(c.births.len()-1)))
            }
            // reached by a third instant: the triple point
            if self.covered(k,pair,y).0 {
                let (z,sheet,p3) = self.triple(k,pair,x,y).ok_or_else(|| refused("does not settle where a third instant reaches it",y))?;
                let i = self.triple_of(c,pair,z,sheet,p3);
                let z = self.snapped(c,i,pair,z);
                out.push(z);
                return Ok((out,End::Triple(i)))
            }
            out.push(y);
            if out.len() > 8 && (0..3).all(|i| (y[i]-x0[i]).abs() <= h) { return Err(refused("closes on itself",y)) }
            x = y;
            w = w2;
            h = (h*1.5).min(MOST_STEP);
        }
        Err(refused("runs on past every step",x))
    }

    /// The triple point `z` of `pair` with a third instant on `sheet` at `p3`: one already found
    /// whose feet hold the pair's at the rounding, or a new one.
    fn triple_of(&self,c: &mut Curves,pair: Pair,z: X,sheet: Sheet,p3: f64) -> usize {
        let close = |a: f64,b: f64| (a-b).abs() <= HALF_READ*(1.+a.abs());
        let holds = |t: &Triple,s: Sheet,p: f64| t.feet.iter().any(|&(fs,fp)| fs == s && close(fp,p));
        if let Some(i) = c.triples.iter().position(|t| close(t.s,z[2]) && holds(t,pair.a,z[0]) && holds(t,pair.b,z[1])) { return i }
        c.triples.push(Triple {feet:vec![(pair.a,z[0]),(pair.b,z[1]),(sheet,p3)],s:z[2]});
        c.triples.len()-1
    }

    /// A pair's point at triple point `i`, its parameters the triple's own feet.
    fn snapped(&self,c: &Curves,i: usize,pair: Pair,z: X) -> X {
        let t = &c.triples[i];
        let foot = |s: Sheet,p: f64,skip: Option<usize>| t.feet.iter().enumerate()
            .filter(|(j,&(fs,_))| fs == s && Some(*j) != skip)
            .min_by(|a,b| (a.1.1-p).abs().total_cmp(&(b.1.1-p).abs())).map(|(j,f)| (j,f.1));
        let Some((ja,pa)) = foot(pair.a,z[0],None) else { return z };
        let Some((_,pb)) = foot(pair.b,z[1],Some(ja)) else { return z };
        [pa,pb,t.s]
    }

    /// **A swallowtail's birth** where the same side's instants `y` of a double curve merge: the
    /// curve's divided form `(P(m + h) − P(m − h)) / 2h = 0` settled for `(m, σ)` at halvings of
    /// the parting `h`, and its limit at `h = 0` (Richardson's, in `h²`) — the birth; the points
    /// along the way and the birth itself, last.
    fn birth(&self,k: usize,pair: Pair,y: X) -> Option<(Vec<X>,Birth)> {
        let Sheet::Side(side) = pair.a else { return None };
        let divided = |m: f64,s: f64,h: f64| -> P {
            let [rho,..] = self.slabs[k].at(s);
            p_scale(p_sub(self.at(pair.a,m+h,rho),self.at(pair.a,m-h,rho)),0.5/h)
        };
        let mut m = 0.5*(y[0]+y[1]);
        let mut s = y[2];
        let h0 = 0.5*(y[1]-y[0]);
        let mut h = h0;
        let mut seen: Vec<(f64,f64,f64)> = vec![(h0,m,s)];
        let d = THIRD_READ;
        // (the divided form's part along the normal is `O(h²)`, read with the rounding's `ε|P|/h`:
        // halved only while it reads to half its digits, `h³` past half the rounding of the roll's)
        let span = self.roll[1]-self.roll[0];
        let least = HALF_READ.cbrt()*span;
        while seen.len() < 3 || 0.5*h >= least {
            h *= 0.5;
            for _ in 0..NEWTON {
                let f = divided(m,s,h);
                let (fm,fs) = (p_scale(p_sub(divided(m+d,s,h),divided(m-d,s,h)),0.5/d),p_scale(p_sub(divided(m,s+d,h),divided(m,s-d,h)),0.5/d));
                let dx = solve([[fm[0],fs[0]],[fm[1],fs[1]]],[-f[0],-f[1]])?;
                m += dx[0]; s += dx[1];
                if dx[0].abs().max(dx[1].abs()) <= 4.*f64::EPSILON*(1.+m.abs()) { break }
            }
            if !(m.is_finite() && s.is_finite()) { return None }
            seen.push((h,m,s));
        }
        // `m` and `σ` even in `h`: each a quadratic in `h²` by least squares through the readings
        let fit = |v: &dyn Fn(&(f64,f64,f64)) -> f64| -> Option<[f64;3]> {
            let mut a = [[0.;3];3];
            let mut r = [0.;3];
            for e in &seen {
                let q = e.0*e.0;
                let row = [1.,q,q*q];
                for i in 0..3 { for j in 0..3 { a[i][j] += row[i]*row[j] } r[i] += row[i]*v(e) }
            }
            solve(a,r)
        };
        let b = Birth {side,t:0.,s:0.,opens:0.,reach:h0,m:fit(&|e| e.1)?,sigma:fit(&|e| e.2)?,count:seen.len()};
        let (t,s0) = b.at(0.);
        let opens = (b.at(h0).1-s0).signum();
        let b = Birth {t,s:s0,opens,..b};
        // held where the form's points are the double points they stand for
        for j in 1..=8 {
            let h = h0*j as f64/8.;
            let (m,sg) = b.at(h);
            let [rho,..] = self.slabs[k].at(sg);
            if !(p_len(p_sub(self.at(pair.a,m-h,rho),self.at(pair.a,m+h,rho))) <= FIT*self.tol) { return None }
        }
        let mut pts: Vec<X> = seen[1..].iter().map(|&(h,..)| { let (m,sg) = b.at(h); [m-h,m+h,sg] }).collect();
        pts.push([t,t,s0]);
        Some((pts,b))
    }
}

/// What an edge of a sheet's graph is: a double curve's preimage on it (the curve, and which of
/// its two sheets'), a birth's cut, a stretch of a rim (`σ`) or of a join (the parameter).
#[derive(Clone,Copy,Debug,PartialEq)]
enum Kind { Trace(usize,usize),Cut(usize),Rim(f64),Join(f64) }

/// An edge of a sheet's graph: what it is and its points in `(p, σ)`.
#[derive(Clone,Debug)]
struct Arc2 { kind: Kind,points: Vec<P> }

/// A face of a sheet's graph: its uses of the graph's edges in order round it (each edge, and
/// whether along its points), the area it bounds and a point inside it.
#[derive(Clone,Debug)]
struct Part { uses: Vec<(usize,bool)>,inside: P }

impl Stand<'_> {
    /// **The parts of a sheet** of slab `k`: its domain parted by the curves on it, its rims and
    /// joins, and its births' cuts, walked as a planar graph — each face kept counter-clockwise
    /// with the point inside it, and whether the cover keeps it. Refused where a curve closes
    /// inside the domain, or where a curve's two sides are both kept or both dropped.
    fn parts(&self,k: usize,c: &Curves,sheet: Sheet) -> Result<(Vec<Arc2>,Vec<(Part,bool)>),Unbuilt> {
        let refused = |why: String| Unbuilt::Failed(format!("slab {k}'s sheet {sheet:?}: {why}"));
        let [p0,p1] = self.domain(sheet);
        let mut arcs: Vec<Arc2> = Vec::new();
        for (i,t) in c.traces.iter().enumerate() {
            for (j,s) in [t.pair.a,t.pair.b].into_iter().enumerate() {
                if s == sheet { arcs.push(Arc2 {kind:Kind::Trace(i,j),points:t.points.iter().map(|x| [x[j],x[2]]).collect()}) }
            }
        }
        for (i,b) in c.births.iter().enumerate() {
            if Sheet::Side(b.side) == sheet {
                let to = if b.opens > 0. { 0. } else { 1. };
                arcs.push(Arc2 {kind:Kind::Cut(i),points:vec![[b.t,b.s],[b.t,to]]});
            }
        }
        // the domain's sides, parted where a curve ends on them
        let ends: Vec<P> = arcs.iter().flat_map(|a| [a.points[0],a.points[a.points.len()-1]]).collect();
        let corners = [[p0,0.],[p1,0.],[p1,1.],[p0,1.]];
        for side in 0..4 {
            let (a,b) = (corners[side],corners[(side+1)%4]);
            let along = if side%2 == 0 { 0 } else { 1 };
            let fixed = 1-along;
            let mut on: Vec<P> = ends.iter().copied().filter(|q| q[fixed] == a[fixed] && q[along] > a[along].min(b[along]) && q[along] < a[along].max(b[along])).collect();
            on.extend([a,b]);
            on.sort_by(|x,y| x[along].total_cmp(&y[along]));
            on.dedup_by(|x,y| x == y);
            if a[along] > b[along] { on.reverse() }
            for w in on.windows(2) {
                let kind = if fixed == 1 { Kind::Rim(a[1]) } else { Kind::Join(a[0]) };
                arcs.push(Arc2 {kind,points:vec![w[0],w[1]]});
            }
        }
        // the graph: nodes by their exact place, each edge's way out of either end
        let key = |q: P| (q[0].to_bits(),q[1].to_bits());
        let mut nodes: std::collections::BTreeMap<(u64,u64),Vec<(f64,usize,bool)>> = std::collections::BTreeMap::new();
        let width = p1-p0;
        for (i,a) in arcs.iter().enumerate() {
            let n = a.points.len();
            for (fwd,from,next) in [(true,a.points[0],a.points[1]),(false,a.points[n-1],a.points[n-2])] {
                let d = [(next[0]-from[0])/width,next[1]-from[1]];
                nodes.entry(key(from)).or_default().push((d[1].datan2(d[0]),i,fwd));
            }
        }
        for (_,out) in nodes.iter_mut() { out.sort_by(|a,b| a.0.total_cmp(&b.0)) }
        if let Some((n,_)) = nodes.iter().find(|(_,o)| o.len() < 2) {
            return Err(refused(format!("a curve ends at ({}, {}) meeting nothing",f64::from_bits(n.0),f64::from_bits(n.1))))
        }
        let end_of = |i: usize,fwd: bool| { let p = &arcs[i].points; if fwd { p[p.len()-1] } else { p[0] } };
        let mut seen = std::collections::BTreeSet::new();
        let (mut kept,mut outside) = (Vec::new(),0);
        for i in 0..arcs.len() {
            for fwd in [true,false] {
                if seen.contains(&(i,fwd)) { continue }
                let mut uses = Vec::new();
                let (mut e,mut f) = (i,fwd);
                loop {
                    if !seen.insert((e,f)) { break }
                    uses.push((e,f));
                    let out = &nodes[&key(end_of(e,f))];
                    let at = out.iter().position(|&(_,j,g)| j == e && g != f).ok_or_else(|| refused("a graph's edge lost at its end".into()))?;
                    let (_,j,g) = out[(at+out.len()-1)%out.len()];
                    e = j; f = g;
                }
                let poly: Vec<P> = uses.iter().flat_map(|&(e,f)| {
                    let p = &arcs[e].points;
                    let v: Vec<P> = if f { p[..p.len()-1].to_vec() } else { p[1..].iter().rev().copied().collect() };
                    v
                }).map(|q| [(q[0]-p0)/width,q[1]]).collect();
                let area: f64 = (0..poly.len()).map(|j| { let (a,b) = (poly[j],poly[(j+1)%poly.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()*0.5;
                if !(area > 0.) { outside += 1; continue }
                let inside = interior(&poly).ok_or_else(|| refused("a part has no inside".into()))?;
                kept.push(Part {uses,inside:[p0+inside[0]*width,inside[1]]});
            }
        }
        if outside != 1 { return Err(refused(format!("its curves part it into {outside} pieces round the outside: a curve closes inside it"))) }
        // each part kept where the cover leaves its inside on the boundary
        let mut out = Vec::new();
        for part in kept {
            let [rho,..] = self.slabs[k].at(part.inside[1]);
            let q = self.at(sheet,part.inside[0],rho);
            let keep = self.nearest(q).0 >= rho-self.tol;
            out.push((part,keep));
        }
        // a curve has the boundary on one side, a cut on both
        let mut sides: Vec<Vec<bool>> = vec![Vec::new();arcs.len()];
        for (part,keep) in &out { for &(e,_) in &part.uses { sides[e].push(*keep) } }
        for (i,a) in arcs.iter().enumerate() {
            let s = &sides[i];
            let ok = match a.kind {
                Kind::Trace(..) => s.len() == 2 && s[0] != s[1],
                Kind::Cut(_) => s.len() == 2 && s[0] && s[1],
                _ => s.len() == 1,
            };
            if !ok { return Err(refused(format!("its {:?} from {:?} to {:?} parts it wrongly: kept {:?}",a.kind,a.points[0],a.points[a.points.len()-1],s))) }
        }
        if debug() { eprintln!("upright: slab {k} {sheet:?}: {} parts, {} kept",out.len(),out.iter().filter(|p| p.1).count()) }
        Ok((arcs,out))
    }
}

/// A point inside the polygon `poly`: the middle of the widest stretch inside it along one of a few
/// lines across its box, the middle one first.
fn interior(poly: &[P]) -> Option<P> {
    let (lo,hi) = poly.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),q| (l.min(q[1]),h.max(q[1])));
    for f in [0.5,0.25,0.75,0.125,0.375,0.625,0.875] {
        let y = lo+(hi-lo)*f;
        let mut xs: Vec<f64> = (0..poly.len()).filter_map(|j| {
            let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
            ((a[1] <= y) != (b[1] <= y)).then(|| a[0]+(y-a[1])/(b[1]-a[1])*(b[0]-a[0]))
        }).collect();
        xs.sort_by(f64::total_cmp);
        if let Some(w) = xs.chunks(2).filter(|w| w.len() == 2).max_by(|a,b| (a[1]-a[0]).total_cmp(&(b[1]-b[0]))) {
            if w[1] > w[0] { return Some([0.5*(w[0]+w[1]),y]) }
        }
    }
    None
}

/// The degree of every curve and net the road fits.
const DEGREE: usize = 5;
/// The most points a fitted curve or a net's side is read at.
const MOST_FIT: usize = 4096;

/// **A curve through `f` over `[0, 1]`**, interpolated at evenly spaced parameters doubled until
/// it holds `f` within `fit` halfway between them and at their quarters.
fn fitted_curve(f: &dyn Fn(f64) -> V,fit: f64) -> Result<BSpline,Unbuilt> {
    let mut n = 8;
    loop {
        let ts: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let pts: Vec<V> = ts.iter().map(|&t| f(t)).collect();
        let b = crate::nurbs::interpolate(&pts,&ts,DEGREE).ok_or_else(|| Unbuilt::Failed("a curve could not be interpolated".into()))?;
        let worst = (0..n).flat_map(|i| [0.25,0.5,0.75].map(|q| (i as f64+q)/n as f64))
            .map(|t| crate::util::space::distance(b.point(t),f(t))).fold(0.,f64::max);
        if worst <= fit { return Ok(b) }
        if debug() && n >= 512 {
            let w = (0..n).flat_map(|i| [0.25,0.5,0.75].map(|q| (i as f64+q)/n as f64)).map(|t| (crate::util::space::distance(b.point(t),f(t)),t)).fold((0.,0.),|a,b| if b.0 > a.0 { b } else { a });
            eprintln!("fit at {n}: worst {:e} at {} ({:?})",w.0,w.1,f(w.1));
        }
        if n >= MOST_FIT { return Err(Unbuilt::Failed(format!("a curve fits within {fit:e} only to {worst:e}"))) }
        n *= 2;
    }
}

/// **A net through `f` over `[u0, u1] × [v0, v1]`**, rows along `u` interpolated at each `v` and
/// their poles' columns along `v`, each way doubled until the net holds `f` within `fit` at the
/// middles of its cells and of their sides.
fn fitted_net(f: &dyn Fn(f64,f64) -> V,[u0,u1]: [f64;2],[v0,v1]: [f64;2],fit: f64) -> Result<Net,Unbuilt> {
    let failed = || Unbuilt::Failed("a sheet could not be interpolated".into());
    let (mut nu,mut nv) = (8,8);
    loop {
        let us: Vec<f64> = (0..=nu).map(|i| u0+(u1-u0)*i as f64/nu as f64).collect();
        let vs: Vec<f64> = (0..=nv).map(|j| v0+(v1-v0)*j as f64/nv as f64).collect();
        let rows: Vec<BSpline> = vs.iter().map(|&v| crate::nurbs::interpolate(&us.iter().map(|&u| f(u,v)).collect::<Vec<_>>(),&us,DEGREE))
            .collect::<Option<_>>().ok_or_else(failed)?;
        let cols: Vec<BSpline> = (0..rows[0].poles.len()).map(|i| crate::nurbs::interpolate(&rows.iter().map(|r| r.poles[i]).collect::<Vec<_>>(),&vs,DEGREE))
            .collect::<Option<_>>().ok_or_else(failed)?;
        let net = Net {du:rows[0].degree,dv:cols[0].degree,uknots:rows[0].knots.clone(),vknots:cols[0].knots.clone(),
            poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
        let off = |u: f64,v: f64| crate::util::space::distance(net.point(u,v),f(u,v));
        let mid = |a: &[f64],i: usize| 0.5*(a[i]+a[i+1]);
        let err_u = (0..nu).flat_map(|i| vs.iter().map(move |&v| (i,v))).map(|(i,v)| off(mid(&us,i),v)).fold(0.,f64::max);
        let err_v = (0..nv).flat_map(|j| us.iter().map(move |&u| (j,u))).map(|(j,u)| off(u,mid(&vs,j))).fold(0.,f64::max);
        let err_c = (0..nu).flat_map(|i| (0..nv).map(move |j| (i,j))).map(|(i,j)| off(mid(&us,i),mid(&vs,j))).fold(0.,f64::max);
        // (each way grown by its own misses; a cell's middle missed alone grows the worse way)
        let (mut grow_u,mut grow_v) = (err_u > fit,err_v > fit);
        if !grow_u && !grow_v && err_c > fit { if err_u >= err_v { grow_u = true } else { grow_v = true } }
        if !grow_u && !grow_v {
            if debug() { eprintln!("upright: a net of {} by {} over {:?} × {:?}",nu,nv,[u0,u1],[v0,v1]) }
            net.check().map_err(Unbuilt::Failed)?;
            return Ok(net)
        }
        if (grow_u && nu >= MOST_FIT) || (grow_v && nv >= MOST_FIT) {
            return Err(Unbuilt::Failed(format!("a sheet fits within {fit:e} only to {:e}",err_u.max(err_v).max(err_c))))
        }
        if grow_u { nu *= 2 }
        if grow_v { nv *= 2 }
    }
}

/// How finely the road fits its curves and sheets, as a part of the tolerance.
const FIT: f64 = 0.1;

impl Stand<'_> {
    /// The posed tool's face of slab `k` at the roll's end `e`.
    /// (Its frame turned about its axis so its seam stands opposite the middle of the end's half,
    /// as the pieces road lays an arc's: a face across a surface's seam is read on either side of
    /// it, and the mesher found the half's boundary passing through its own points.)
    fn end_surface(&self,k: usize,e: u8) -> Result<Surface,Unbuilt> {
        let pose = self.path.pose(self.roll[e as usize])?;
        let s = self.tool.faces[self.slabs[k].face].surface.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
        let [lo,hi] = self.domain(Sheet::End(e));
        let (sn,cs) = (0.5*(lo+hi)+PI).dsin_cos();
        let away = crate::util::space::add(crate::util::space::scale(self.plane.u,cs),crate::util::space::scale(self.plane.v,sn));
        let turned = |f: Frame| {
            let x = crate::util::space::normalised(sub(away,crate::util::space::scale(f.z,dot(away,f.z)))).unwrap_or(f.x);
            Frame {o:f.o,x,y:crate::util::space::cross(f.z,x),z:f.z}
        };
        Ok(match s {
            Surface::Torus(f,big,small) => Surface::Torus(turned(f),big,small),
            Surface::Cylinder(f,r) => Surface::Cylinder(turned(f),r),
            Surface::Cone(f,r,a) => Surface::Cone(turned(f),r,a),
            other => other,
        })
    }

    /// **A double curve's curves in space**: the curve cut where the coordinate of `(p₁, p₂, σ)` it
    /// runs most along changes (kept while the curve leans at least 30° off its plane: past it a
    /// coordinate the curve turns back over, the apex of an island, comes too near), each piece read with that coordinate its parameter — each point settled on the plane
    /// where the coordinate has its value, the implicit function's own smooth chart (a point
    /// settled across a guide through the traced points reads the curve only as smoothly as the
    /// guide is) — and fitted over `[0, 1]`; each with the index of the traced point it ends at.
    fn trace_curves(&self,k: usize,c: &Curves,t: &Trace) -> Result<Vec<(Arc<Curve>,[Arc<Curve>;2],usize)>,Unbuilt> {
        let all = &t.points;
        // (a stretch into a birth is read in its normal form, apart: `birth_piece`)
        let into = |e: usize| match t.ends[e] { End::Birth(b) => Some(&c.births[b]),_ => None };
        let (head,tail) = (into(0).map_or(0,|b| b.count),into(1).map_or(0,|b| b.count));
        let mut out = Vec::new();
        if let Some(b) = into(0) { out.push(self.birth_piece(k,b,false,head)?) }
        let pts = &all[head..all.len()-tail];
        let shift = head;
        let n = pts.len();
        let way = |j: usize| { let (a,b) = (pts[j.saturating_sub(1)],pts[(j+1).min(n-1)]); sub(b,a) };
        let mut cuts: Vec<(usize,usize)> = Vec::new();
        let most = |w: X| (0..3).max_by(|&i,&j| w[i].abs().total_cmp(&w[j].abs())).expect("three");
        let mut along = most(way(0));
        let mut from = 0;
        for j in 1..n {
            let w = way(j);
            let m = most(w);
            if w[along].abs() < 0.5*crate::util::space::norm(w) && m != along { cuts.push((from,along)); from = j-1; along = m }
        }
        cuts.push((from,along));
        let mut out = Vec::new();
        for (c,&(i0,i)) in cuts.iter().enumerate() {
            let i1 = cuts.get(c+1).map_or(n-1,|&(j,_)| j);
            let (a,b) = (pts[i0][i],pts[i1][i]);
            let settled = |s: f64| -> X {
                if s <= 0. { pts[i0] } else if s >= 1. { pts[i1] } else {
                    let v = a+(b-a)*s;
                    // (the traced stretch holding the value, read between its points)
                    let j = (i0..i1).find(|&j| (pts[j][i]-v)*(pts[j+1][i]-v) <= 0.).unwrap_or(i0);
                    let f = (v-pts[j][i])/(pts[j+1][i]-pts[j][i]);
                    let pred = std::array::from_fn(|q| pts[j][q]+f*(pts[j+1][q]-pts[j][q]));
                    let mut row = [0.;3];
                    row[i] = 1.;
                    self.settle(k,t.pair,pred,row,v).unwrap_or([f64::NAN;3])
                }
            };
            let curve = fitted_curve(&|s| { let x = settled(s); self.point(k,t.pair.a,x[0],x[2]) },FIT*self.tol)?;
            // each sheet's own place along it, on the same parameter (its pcurve)
            let place = |j: usize| fitted_curve(&|s| { let x = settled(s); [x[j],x[2],0.] },FIT*self.tol/(1.+self.scale));
            let pc = [Arc::new(Curve::BSpline(Arc::new(place(0)?))),Arc::new(Curve::BSpline(Arc::new(place(1)?)))];
            out.push((Arc::new(Curve::BSpline(Arc::new(curve))),pc,i1+shift));
        }
        if let Some(b) = into(1) { out.push(self.birth_piece(k,b,true,all.len()-1)?) }
        Ok(out)
    }

    /// A double curve's stretch into a birth as a curve in space and its two pcurves, over `[0, 1]`
    /// from the traced point it was reached from to the birth (`into`) or back (its normal form,
    /// `Birth::at`, read at the half parting), with the index of the traced point it ends at.
    fn birth_piece(&self,k: usize,b: &Birth,into: bool,end: usize) -> Result<(Arc<Curve>,[Arc<Curve>;2],usize),Unbuilt> {
        let h0 = b.reach;
        let x = |s: f64| -> X {
            let h = if into { h0*(1.-s) } else { h0*s };
            if h == 0. { return [b.t,b.t,b.s] }
            let (m,sg) = b.at(h);
            [m-h,m+h,sg]
        };
        let side = Sheet::Side(b.side);
        let curve = fitted_curve(&|s| { let y = x(s); self.point(k,side,y[0],y[2]) },FIT*self.tol)?;
        let place = |j: usize| fitted_curve(&|s| { let y = x(s); [y[j],y[2],0.] },FIT*self.tol/(1.+self.scale));
        Ok((Arc::new(Curve::BSpline(Arc::new(curve))),[Arc::new(Curve::BSpline(Arc::new(place(0)?))),Arc::new(Curve::BSpline(Arc::new(place(1)?)))],end))
    }

    /// **The faces of slab `k`**: every kept part of each of its sheets, a side's on a net fitted
    /// over its part's box, an end's on the tool's face posed there.
    fn slab_faces(&self,k: usize) -> Result<Vec<Brep>,Unbuilt> {
        let c = self.curves(k)?;
        let traced: Vec<Vec<(Arc<Curve>,[Arc<Curve>;2],usize)>> = c.traces.iter().map(|t| self.trace_curves(k,&c,t)).collect::<Result<_,_>>()?;
        let mut sheets = vec![Sheet::Side(1),Sheet::Side(-1)];
        if !self.whole()? { sheets.extend([Sheet::End(0),Sheet::End(1)]) }
        let mut out = Vec::new();
        for sheet in sheets {
            let (arcs,parts) = self.parts(k,&c,sheet)?;
            for (part,keep) in parts {
                if keep { out.push(self.face(k,&c,sheet,&arcs,&part,&traced)?) }
            }
        }
        Ok(out)
    }

    /// Whether the roll is a whole period, so no end pose bounds the sweep.
    fn whole(&self) -> Result<bool,Unbuilt> {
        let [rho,..] = self.slabs[1].at(0.);
        let section = super::subtraction::section_of(vec![vec![crate::plane2::Edge::Arc {center:[0.,0.],radius:rho,start:0.,sweep:TAU,ends:[[rho,0.],[rho,0.]]}]]);
        let setting = super::region::Setting {section,path:self.path.clone(),plane:self.plane,roll:self.roll,scale:self.scale};
        Ok(setting.alike(self.roll[0],self.roll[1]))
    }

    /// One kept part of a sheet as a face of its own: its surface, its loop's edges and vertices.
    fn face(&self,k: usize,c: &Curves,sheet: Sheet,arcs: &[Arc2],part: &Part,traced: &[Vec<(Arc<Curve>,[Arc<Curve>;2],usize)>]) -> Result<Brep,Unbuilt> {
        let point = |q: P| self.point(k,sheet,q[0],q[1]);
        let fit = FIT*self.tol;
        // the surface, and where a place of the part lies on it
        let (surface,uv): (Surface,Box<dyn Fn(P) -> [f64;2]>) = match sheet {
            Sheet::Side(_) => {
                let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
                for &(e,_) in &part.uses { for q in &arcs[e].points { for i in 0..2 { lo[i] = lo[i].min(q[i]); hi[i] = hi[i].max(q[i]) } } }
                // (grown by a trace's step within the sheet: a curve's turn, an island's apex, lies
                // between its traced points)
                let [p0,p1] = self.domain(sheet);
                let grow = MOST_STEP*(p1-p0);
                let (u,v) = ([(lo[0]-grow).max(p0),(hi[0]+grow).min(p1)],[(lo[1]-MOST_STEP).max(0.),(hi[1]+MOST_STEP).min(1.)]);
                let net = fitted_net(&|u,v| self.point(k,sheet,u,v),u,v,fit)?;
                (Surface::BSpline(Frame::WORLD,Arc::new(net)),Box::new(|q: P| q))
            }
            Sheet::End(e) => {
                let s = self.end_surface(k,e)?;
                let s2 = s.clone();
                (s,Box::new(move |q: P| s2.inverse(point(q))))
            }
        };
        // each use of the graph's edges as the curves it is laid in, in the loop's order: a
        // double curve's pieces, any other edge one curve
        let mut segments: Vec<(Arc<Curve>,Option<Arc<Curve>>,P,P,bool)> = Vec::new();
        let line = |f: &dyn Fn(f64) -> V| -> Result<Arc<Curve>,Unbuilt> { Ok(Arc::new(Curve::BSpline(Arc::new(fitted_curve(f,fit)?)))) };
        for &(e,fwd) in &part.uses {
            let a = &arcs[e];
            let (q0,q1) = (a.points[0],a.points[a.points.len()-1]);
            let mut own: Vec<(Arc<Curve>,Option<Arc<Curve>>,P,P)> = match a.kind {
                Kind::Trace(i,j) => {
                    let mut from = 0;
                    traced[i].iter().map(|(curve,pc,to)| { let seg = (curve.clone(),Some(pc[j].clone()),a.points[from],a.points[*to]); from = *to; seg }).collect()
                }
                Kind::Cut(i) => { let bth = &c.births[i]; let to = q1[1]; vec![(line(&|s| self.point(k,sheet,bth.t,bth.s+(to-bth.s)*s))?,None,q0,q1)] }
                Kind::Rim(sg) => vec![(line(&|s| self.point(k,sheet,q0[0]+(q1[0]-q0[0])*s,sg))?,None,q0,q1)],
                Kind::Join(pj) => vec![(line(&|s| self.point(k,sheet,pj,q0[1]+(q1[1]-q0[1])*s))?,None,q0,q1)],
            };
            if !fwd { own.reverse() }
            segments.extend(own.into_iter().map(|(c,pc,x,y)| (c,pc,x,y,fwd)));
        }
        let mut b = Brep::default();
        let n = segments.len();
        // the loop's places, in order, each unwrapped against the last on an end's periodic surface
        let mut places: Vec<[f64;2]> = Vec::with_capacity(n+1);
        for (j,(_,_,x,y,fwd)) in segments.iter().enumerate() {
            let from = if *fwd { *x } else { *y };
            if j == 0 { places.push(uv(from)) }
            let to = if *fwd { *y } else { *x };
            let mut w = uv(to);
            // (each parameter the surface repeats in, through the edge's middle: a rim of an end
            // spans half a turn)
            let mut mid = uv([0.5*(x[0]+y[0]),0.5*(x[1]+y[1])]);
            for (i,period) in surface.periods().into_iter().enumerate() {
                let Some(period) = period else { continue };
                mid[i] = unwrap(mid[i],places[places.len()-1][i],period);
                w[i] = unwrap(w[i],mid[i],period);
            }
            places.push(w);
            b.vertex(point(from));
        }
        let mut uses = Vec::with_capacity(n);
        for (j,(curve,own,_,_,fwd)) in segments.into_iter().enumerate() {
            let (v0,v1) = (j as u32,((j+1)%n) as u32);
            let (u0,u1) = (places[j],places[j+1]);
            let (ends,pc) = if fwd { ([v0,v1],Pcurve::Inverse {a:u0,b:u1}) } else { ([v1,v0],Pcurve::Inverse {a:u1,b:u0}) };
            // (on a side's net its place is known: a double curve's own, any other edge straight
            // in `(p, σ)` on the parameter it was fitted on)
            let pc = match (sheet,own) {
                (Sheet::Side(_),Some(own)) => Pcurve::Curve(own),
                (Sheet::Side(_),None) => match pc { Pcurve::Inverse {a,b} => Pcurve::Line {a,b},other => other },
                (Sheet::End(_),_) => pc,
            };
            b.edges.push(Edge {curve:EdgeCurve::Curve((*curve).clone()),t:[0.,1.],v:ends,tol:0.});
            uses.push(Coedge {edge:j as u32,reversed:!fwd,pcurve:pc});
        }
        // a loop counter-clockwise in the surface's own parameters
        // (the places' polygon's area says which way round the loop turns in them)
        let area: f64 = (0..n).map(|j| { let (p,q) = (places[j],places[j+1]); p[0]*q[1]-p[1]*q[0] }).sum();
        if area < 0. { uses = crate::topo::reverse_loop(uses) }
        b.faces.push(Face {surface,reversed:false,loops:vec![uses],name:format!("{sheet:?}")});
        Ok(b)
    }
}

/// `x` moved by whole periods to stand within half a period of `near`.
fn unwrap(x: f64,near: f64,period: f64) -> f64 { x-period*((x-near)/period).round() }

impl Stand<'_> {
    /// **A flat end's face**: the region its disc sweeps, at its height — its loops' runs read on
    /// the slab beside it (`k`) at its rim (`s`), each corner settled there, a loop of one run cut
    /// at its middle (an edge has two ends).
    fn cap(&self,k: usize,s: f64) -> Result<Brep,Unbuilt> {
        let [rho,h,..] = self.slabs[k].at(s);
        let region = self.region(rho)?;
        let rings = self.rings(&region,rho)?;
        let fit = FIT*self.tol;
        let mut b = Brep::default();
        let mut loops = Vec::new();
        for (l,holes) in region.nesting(8) {
            for r in std::iter::once(l).chain(holes) {
                let ring = &rings[r];
                let n = ring.runs.len();
                // each run's ends settled at its corners
                let mut runs: Vec<Run> = ring.runs.clone();
                for i in 0..n {
                    let j = (i+n-1)%n;
                    let (a,b2) = ((runs[j].sheet,runs[j].p[1]),(runs[i].sheet,runs[i].p[0]));
                    if self.is_join(a,b2) {
                        let (side,e) = match (a.0,b2.0) { (Sheet::Side(sd),Sheet::End(e)) | (Sheet::End(e),Sheet::Side(sd)) => (sd,e),_ => unreachable!("a join") };
                        let (t,phi) = self.join(side,e);
                        for (x,idx,end) in [(a.0,j,1),(b2.0,i,0)] { runs[idx].p[end] = if let Sheet::Side(_) = x { t } else { phi } }
                        continue
                    }
                    let swap = a > b2;
                    let ((sa,pa),(sb,pb)) = if swap { (b2,a) } else { (a,b2) };
                    let x = self.settle(k,Pair {a:sa,b:sb},[pa,pb,s],[0.,0.,1.],s)
                        .ok_or_else(|| Unbuilt::Failed(format!("a corner of the cap at height {h} cannot be settled")))?;
                    let (xa,xb) = if swap { (x[1],x[0]) } else { (x[0],x[1]) };
                    runs[j].p[1] = xa;
                    runs[i].p[0] = xb;
                }
                let pieces: Vec<(Sheet,[f64;2])> = if n == 1 {
                    let [p0,p1] = runs[0].p;
                    let p1 = if let Sheet::End(_) = runs[0].sheet { p1 } else if p1 == p0 { p0+(self.roll[1]-self.roll[0]) } else { p1 };
                    let m = 0.5*(p0+p1);
                    vec![(runs[0].sheet,[p0,m]),(runs[0].sheet,[m,p1])]
                } else { runs.iter().map(|r| (r.sheet,r.p)).collect() };
                let mut uses = Vec::new();
                let first = b.vertices.len() as u32;
                let m = pieces.len();
                for (i,&(sheet,[p0,p1])) in pieces.iter().enumerate() {
                    let v0 = if i == 0 { b.vertex(self.point(k,sheet,p0,s)) } else { (b.vertices.len()-1) as u32 };
                    let v1 = if i+1 == m { first } else { b.vertex(self.point(k,sheet,p1,s)) };
                    let curve = fitted_curve(&|f| self.point(k,sheet,p0+(p1-p0)*f,s),fit)?;
                    let (a,z) = (self.plane.page(curve.point(0.)),self.plane.page(curve.point(1.)));
                    b.edges.push(Edge {curve:EdgeCurve::Curve(Curve::BSpline(Arc::new(curve))),t:[0.,1.],v:[v0,v1],tol:0.});
                    uses.push(Coedge {edge:(b.edges.len()-1) as u32,reversed:false,pcurve:Pcurve::Inverse {a,b:z}});
                }
                loops.push(uses);
            }
        }
        let o = self.plane.world([0.,0.],h);
        let frame = Frame {o,x:self.plane.u,y:self.plane.v,z:self.plane.n};
        b.faces.push(Face {surface:Surface::Plane(frame),reversed:false,loops,name:"cap".into()});
        Ok(b)
    }

    /// **The closed solid**: each slab's faces and each flat end's, sewn.
    fn solid(&self) -> Result<Brep,Unbuilt> {
        let n = self.slabs.len();
        let mut pieces = Vec::new();
        for k in 0..n {
            if self.slabs[k].level() {
                // a flat end, read on the slab beside it
                let (j,s) = if k == 0 { (1,0.) } else if k+1 == n { (n-2,1.) } else {
                    return Err(Unbuilt::Failed("a tool of revolution with a flat step inside its ends".into()))
                };
                pieces.push(self.cap(j,s)?);
            } else {
                pieces.extend(self.slab_faces(k)?);
            }
        }
        if debug() {
            eprintln!("upright: {} faces to sew",pieces.len());
            for (fi,p) in pieces.iter().enumerate() {
                let f = &p.faces[0];
                for (ei,e) in p.edges.iter().enumerate() {
                    let mut worst: (f64,V) = (0.,[0.;3]);
                    for j in 0..=32 {
                        let x = e.point(j as f64/32.,&p.vertices);
                        let d = crate::util::space::distance(f.surface.point(f.surface.inverse(x)),x);
                        if d > worst.0 { worst = (d,x) }
                    }
                    if worst.0 > self.tol { eprintln!("  face {fi} ({}) edge {ei}: {:e} off at {:?}",f.name,worst.0,worst.1) }
                }
            }
        }
        crate::sew::sewn(&pieces,self.tol).map_err(|e| Unbuilt::Failed(format!("the sweep's faces sewn: {e}")))
    }
}

/// **The closed solid `tool` sweeps under `path` over `roll`, where the motion keeps a plane square
/// to the axis of the convex tool of revolution with flat ends** (the module's method), built to
/// `tol` and judged by membership read on the tool meshed within `bar`; none where the road does
/// not read the tool or the motion.
pub fn swept(tool: &Brep,path: &Trajectory,roll: [f64;2],bar: f64,tol: f64) -> Result<Option<Brep>,Unbuilt> {
    let Some(stand) = Stand::read(tool,path,roll,tol)? else { return Ok(None) };
    let out = stand.solid()?;
    let roll = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    // **judged by its definition before it is returned** (#777 V2), over a box holding it
    let (lo,hi) = out.bounds();
    let (domain,..) = super::closed::cube_holding(&[lo,hi],tol)?;
    let solid = super::member::Solid::new(tool,bar,1.)?;
    super::closed::judge(&out,&super::member::Swept::new(&solid,path,roll),&domain,None,&[],tol)?;
    Ok(Some(out))
}
