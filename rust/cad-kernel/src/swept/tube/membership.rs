//! **The tube road's membership over its charts, and its charts cut into patches** (#880 T1f;
//! `docs/tube-trims-by-membership.md` §2–3). A chart's point is *kept* where it bounds the sweep —
//! nothing else covers it, or another instant comes no nearer than a far cover above nought —
//! *covered* where another instant holds it below nought, *held* where the instants beside its own
//! hold it, and *unread* where the chart does not bound the sweep at all (past a fold, in a
//! swallowtail point's disc, inside a hole's rim another chart reads on). The trims are where it
//! passes from kept to covered: the depth's zero set (`Depth`), read here as the one field the
//! topology is read from.
//!
//! Each chart is cut into **fixed patches** in its own parameters — a sheet into quarters of `θ` by
//! stretches of the roll each turning the motion at most a quarter turn, an end pose's torus into
//! quarters each way, the collar and a band one patch each — whose edges are parameter lines
//! **shared** by the patches beside them: what crosses an edge is read once, on the edge
//! (`crossings`), and is the same for both.

use super::{Chart,Depth,Tube};
use crate::swept::depth::{Cover,Reading};
use crate::util::space::distance;
use std::f64::consts::FRAC_PI_2;

type V = [f64;3];

/// **How a chart's point stands under the sweep.**
#[derive(Clone,Debug)]
pub enum Membership {
    /// The chart bounds the sweep here: nothing else covers it (none, `Reading::Alone`), or the
    /// nearest other instant's far cover stands above nought.
    Kept(Option<Cover>),
    /// Another instant holds it below nought: that instant's far cover.
    Covered(Cover),
    /// The instants beside its own hold it (`Reading::Near`): inside, no other instant needed.
    Held,
    /// The chart does not bound the sweep here: past a fold, in a swallowtail point's disc, or
    /// inside a hole's rim (the pinch's, a band's seam), which another chart reads on.
    Unread,
}

impl Membership {
    /// Whether it is inside the sweep (covered or held); none where unread.
    pub fn inside(&self) -> Option<bool> {
        match self { Membership::Kept(_) => Some(false),Membership::Covered(_) | Membership::Held => Some(true),Membership::Unread => None }
    }

    /// The far cover's depth where there is one, `+∞` where nothing else covers the point: the
    /// field whose zero set the trims are. None where held or unread.
    pub fn depth(&self) -> Option<f64> {
        match self {
            Membership::Kept(Some(c)) | Membership::Covered(c) => Some(c.depth),
            Membership::Kept(None) => Some(f64::INFINITY),
            Membership::Held | Membership::Unread => None,
        }
    }
}

/// **The membership of the chart's point at `uv`** (`Depth::reading`; inside a hole's rim unread).
pub fn membership(field: &Depth,uv: [f64;2]) -> Membership {
    let rims = field.tube.hole(field.chart).map_or(0,|r| r.len());
    if (0..rims).any(|k| !(field.tube.hole_side(field.chart,k,uv) > 0.)) { return Membership::Unread }
    match field.reading(uv) {
        None => Membership::Unread,
        Some((_,Reading::Near)) => Membership::Held,
        Some((_,Reading::Alone)) => Membership::Kept(None),
        Some((_,Reading::Far(c))) => if c.depth < 0. { Membership::Covered(c) } else { Membership::Kept(Some(c)) },
    }
}

/// **A chart cut into patches**: the parameter lines `us` and `vs` (each from the chart's rectangle's
/// start to its end), the patch `(i, j)` the box between `us[i]`, `us[i + 1]` and `vs[j]`, `vs[j + 1]`;
/// a parameter that runs a whole turn has its last line its first.
#[derive(Clone,Debug)]
pub struct Patches { pub chart: Chart,pub us: Vec<f64>,pub vs: Vec<f64>,pub periodic: [bool;2] }

/// **A patch's edge**: the stretch `span` of the line where the parameter not `along` is `at` — the
/// line `v = at` over `u ∈ span` (`along` 0) or `u = at` over `v ∈ span` (`along` 1).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Line { pub along: usize,pub at: f64,pub span: [f64;2] }

impl Line {
    /// The line's place at `s` of its running parameter.
    pub fn place(&self,s: f64) -> [f64;2] { if self.along == 0 { [s,self.at] } else { [self.at,s] } }
}

impl Patches {
    /// How many patches each way.
    pub fn count(&self) -> [usize;2] { [self.us.len()-1,self.vs.len()-1] }

    /// The box of patch `(i, j)`.
    pub fn rect(&self,i: usize,j: usize) -> crate::zero_set::Rect { [[self.us[i],self.us[i+1]],[self.vs[j],self.vs[j+1]]] }

    /// **Every patch edge, once**: the lines `v = vs[j]` cut at each `us`, then the lines `u = us[i]`
    /// cut at each `vs` — a parameter running a whole turn has its last line its first, read once.
    pub fn edges(&self) -> Vec<Line> {
        let [nu,nv] = self.count();
        let mut out = Vec::with_capacity(2*nu*nv+nu+nv);
        for j in 0..(if self.periodic[1] { nv } else { nv+1 }) {
            for i in 0..nu { out.push(Line {along:0,at:self.vs[j],span:[self.us[i],self.us[i+1]]}) }
        }
        for i in 0..(if self.periodic[0] { nu } else { nu+1 }) {
            for j in 0..nv { out.push(Line {along:1,at:self.us[i],span:[self.vs[j],self.vs[j+1]]}) }
        }
        out
    }

    /// **Patch `(i, j)`'s four edges**, as `edges` numbers them: its side at `vs[j]`, at `us[i + 1]`,
    /// at `vs[j + 1]` and at `us[i]` — round it, so a side two patches share is one edge.
    pub fn sides(&self,i: usize,j: usize) -> [usize;4] {
        let [nu,nv] = self.count();
        let rows = if self.periodic[1] { nv } else { nv+1 };
        let across = |j: usize| (if self.periodic[1] { j%nv } else { j })*nu+i;
        let up = |i: usize| rows*nu+(if self.periodic[0] { i%nu } else { i })*nv+j;
        [across(j),up(i+1),across(j+1),up(i)]
    }
}

/// **A chart's patches** (the module's rule): a sheet's quarters of `θ` by stretches of the roll each
/// turning the motion at most a quarter turn (`MOST_TURN`), an end pose's quarters each way, the
/// collar and a band one patch.
pub fn patches(tube: &Tube,chart: Chart) -> Patches {
    let [[u0,u1],[v0,v1]] = tube.rect(chart);
    let periodic = tube.periodic(chart);
    let parts = |a: f64,b: f64,n: usize| (0..=n).map(|k| a+(b-a)*k as f64/n as f64).collect::<Vec<f64>>();
    match chart {
        Chart::Sheet(_) => Patches {chart,us:parts(u0,u1,4),vs:turned(tube,[v0,v1]),periodic},
        Chart::End(_) => Patches {chart,us:parts(u0,u1,4),vs:parts(v0,v1,4),periodic},
        Chart::Collar | Chart::Band(..) => Patches {chart,us:vec![u0,u1],vs:vec![v0,v1],periodic},
    }
}

/// The most a patch's stretch of the roll turns the motion (`patches`).
const MOST_TURN: f64 = FRAC_PI_2;

/// How many steps the roll's turn is read in (`turned`).
const TURN_STEPS: usize = 512;

/// **The roll `[a, b]` cut where the motion has turned `MOST_TURN` since the last cut**: the angle of
/// the rotation taking one pose to the next, summed over `TURN_STEPS` steps; then cut evenly, so
/// each stretch turns at most that.
fn turned(tube: &Tube,[a,b]: [f64;2]) -> Vec<f64> {
    let rotation = |t: f64| tube.path.pose(t).ok();
    let angle = |p: &crate::trajectory::Pose,q: &crate::trajectory::Pose| -> f64 {
        let e = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
        let trace: f64 = (0..3).map(|k| q.vector(p.inverse().vector(e[k]))[k]).sum();
        ((trace-1.)/2.).clamp(-1.,1.).acos()
    };
    let mut total = 0.;
    let mut last = rotation(a);
    for k in 1..=TURN_STEPS {
        let next = rotation(a+(b-a)*k as f64/TURN_STEPS as f64);
        if let (Some(p),Some(q)) = (&last,&next) { total += angle(p,q) }
        last = next;
    }
    let n = ((total/MOST_TURN).ceil() as usize).max(1);
    (0..=n).map(|k| a+(b-a)*k as f64/n as f64).collect()
}

/// **A crossing of a patch's edge by a trim**: where on the line, its place in the chart, the far
/// cover on its covered side (the instant that covers it, and so the face across), and whether the
/// line passes from kept into covered there as its parameter grows.
#[derive(Clone,Debug)]
pub struct Crossing { pub s: f64,pub uv: [f64;2],pub cover: Cover,pub into: bool }

/// How finely a line is first read: a quarter of the feature in space (`crossings`).
const LINE_READ: f64 = 0.25;

/// **Where the trims cross a patch's edge** (`Line`): the line read at a quarter of the feature in
/// space (its length read on `LINE_STEPS` chords); each stretch between two readings whose
/// memberships' depths differ in sign halved to the tolerance in space, each end kept on its side;
/// a stretch of one sign halved where the depth, 1-Lipschitz in space, could reach nought between
/// its ends (`|d₀| + |d₁|` short of the chord), to the tolerance; and a stretch with one end held or
/// unread (a fold's, a disc's or a hole's) halved toward it, to the tolerance, the read part read as
/// any other — a stretch with both so is not read. Where nothing else covers either
/// end (`Alone`) the depth is taken clear of nought over the quarter feature — the evidence the
/// search reads a cell by (`Depth::bound`), here four times finer.
pub fn crossings(field: &Depth,line: &Line,feature: f64) -> Vec<Crossing> {
    let tol = field.within;
    let at = |s: f64| field.tube.at(field.chart,line.place(s)).map(|p| p.x);
    let [s0,s1] = line.span;
    let mut length = 0.;
    let mut last: Option<V> = None;
    for k in 0..=LINE_STEPS {
        if let Some(x) = at(s0+(s1-s0)*k as f64/LINE_STEPS as f64) { if let Some(y) = last { length += distance(x,y) } last = Some(x) }
    }
    let n = ((length/(LINE_READ*feature)).ceil() as usize).max(8);
    let read = |s: f64| -> (f64,Option<f64>,Option<V>,Membership) {
        let m = membership(field,line.place(s));
        (s,m.depth(),at(s),m)
    };
    let samples: Vec<(f64,Option<f64>,Option<V>,Membership)> = (0..=n).map(|k| read(s0+(s1-s0)*k as f64/n as f64)).collect();
    let mut out = Vec::new();
    // (each stretch: halved while it may hold a crossing, a sign change settled)
    let mut stack: Vec<[(f64,Option<f64>,Option<V>,Membership);2]> = samples.windows(2).rev().map(|w| [w[0].clone(),w[1].clone()]).collect();
    while let Some([a,b]) = stack.pop() {
        let (Some(xa),Some(xb)) = (a.2,b.2) else { continue };
        let chord = distance(xa,xb);
        // (an end held or unread — a fold's, a disc's, a hole's — and the other read: halved toward
        // it, to the tolerance, so a crossing a hair short of a fold is read — a sliver beside the
        // crossed reverse member's fold, 0.011 across, lay between a quarter feature's readings)
        let (Some(da),Some(db)) = (a.1,b.1) else {
            if (a.1.is_some() || b.1.is_some()) && chord > tol {
                let m = read(0.5*(a.0+b.0));
                stack.push([m.clone(),b]);
                stack.push([a,m]);
            }
            continue
        };
        if (da < 0.) != (db < 0.) {
            // a sign change: halved to the tolerance, each end on its side
            let (mut p,mut q) = (a.clone(),b.clone());
            let mut settled = true;
            while p.2.zip(q.2).is_some_and(|(x,y)| distance(x,y) > tol) && (q.0-p.0).abs() > f64::EPSILON*(s1-s0).abs() {
                let m = read(0.5*(p.0+q.0));
                match m.1 {
                    Some(d) if (d < 0.) == (da < 0.) => p = m,
                    Some(_) => q = m,
                    None => { settled = false; break }
                }
            }
            if !settled { continue }
            let (inside,outside) = if da < 0. { (&p,&q) } else { (&q,&p) };
            let Membership::Covered(cover) = inside.3.clone() else { continue };
            // (the place on the kept side, a hair from the zero)
            out.push(Crossing {s:outside.0,uv:line.place(outside.0),cover,into:da >= 0.});
            continue
        }
        // one sign: clear where the depth cannot reach nought between the ends
        let reach = if da.is_finite() && db.is_finite() { da.abs()+db.abs() } else if da.is_finite() { da.abs() } else if db.is_finite() { db.abs() } else { f64::INFINITY };
        if reach > chord || chord <= tol { continue }
        let m = read(0.5*(a.0+b.0));
        stack.push([m.clone(),b]);
        stack.push([a,m]);
    }
    out.sort_by(|x,y| x.s.total_cmp(&y.s));
    out
}

/// Chords a line's length is read on (`crossings`).
const LINE_STEPS: usize = 64;

/// The chart's patches' every edge with its crossings, each edge read once (`crossings`), on every
/// core.
pub fn edge_crossings(field: &Depth,patches: &Patches,feature: f64) -> Vec<(Line,Vec<Crossing>)> {
    let lines = patches.edges();
    let read = crate::util::par::map(&lines,|l| crossings(field,l,feature));
    lines.into_iter().zip(read).collect()
}
