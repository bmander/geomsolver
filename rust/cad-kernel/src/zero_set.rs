//! **The zero set of a scalar function over a rectangle of a surface's parameters, traced** (#942):
//! the curves where `f(u, v) = 0`, each a polyline in `(u, v)` whose chords keep within the caller's
//! tolerance of the true zero set *in space*, each point carrying what `f` was read at (its
//! minimiser, where `f` is a least value `min over s of g(u, v, s)`: the partner face and point),
//! cut where that minimiser switches between branches (a corner, solved exactly as the
//! simultaneous root `g(u, v, s₁) = g(u, v, s₂) = 0`) and where the curve meets the domain's
//! boundary, each piece's ends classified (`End`). It knows no sweep and no surface kind: the
//! caller's `Field` answers `f`, its branches and the surface's metric.
//!
//! **Seeds, and what is shown complete.** A zero curve either reaches the domain's boundary or
//! closes inside it. Those reaching it are seeded where `f` changes sign round the rectangle's rim
//! and along the given inner edges (fold curves, holes), and where `f` touches nought there without
//! changing sign (a curve tangent to the boundary: the swallowtail's end), each found to the
//! rounding; the caller's seeds (events it knows) are traced next. The rest — closed curves inside
//! — are searched for by halving the rectangle (`search`):
//! * where the caller encloses `f` over a cell (`Field::bound`), a cell whose enclosure misses
//!   nought holds no zero (shown); one whose gradient's enclosure misses nought in a parameter is
//!   monotone in it, so holds no closed zero curve (a closed curve has a point where its tangent
//!   runs along each parameter, where the other partial vanishes), and every zero curve meeting it
//!   crosses or touches its edges, where each zero is isolated by the bound along the edge (a
//!   piece of an edge either misses nought, or is monotone along the edge and holds a root exactly
//!   when its ends' signs differ, or is under the tolerance long and its least `|f|` is read);
//! * a cell no bound settles is halved to the tolerance and answered beside a piece already traced,
//!   or by a point pulled onto the zero set from its middle (traced in turn), or else returned in
//!   `Search::unresolved` — never dropped;
//! * where the bound gives `f` but not its gradient (a Lipschitz bound), the cells it does not
//!   clear are answered beside a traced piece once no wider than the caller's feature size;
//! * where there is no bound at all, a cell no wider than the feature is *sampled*: pulled onto the
//!   zero set from its middle, and `f`'s critical point near it found by Newton (a closed curve
//!   bounds a disc on which a smooth `f` has an extremum) and rays cast from it to where `f`
//!   changes sign. That is evidence, not proof (`Search::sampled` counts such cells).
//!
//! So with a bound of `f` and its gradient on every cell (a branch's gradient, or the hull of the
//! branches' gradients where the least may switch between them), every component of the zero set
//! is traced, but for one lying within the tolerance of a traced piece (not told apart), one in a
//! cell returned unresolved, and a touch of an *inner* edge between that edge's samples (inner
//! edges are sampled at the feature size; the rim is isolated by the bound).
//!
//! **The march.** From a seed the curve is followed both ways by arclength in space (the caller's
//! surface gives the metric): a step along the tangent, then Newton on the branch's `g = 0` along
//! its gradient in the metric (the corrector), held to half the step so it never lands on another
//! curve. A step is kept only where its chord's quarter points are within the tolerance of the
//! zero set and no other branch undercuts them; else it is halved, and a step under the rounding is
//! refused by name — a short curve is never returned silently. A piece ends where it arrives at a
//! boundary seed (a tangential end is reached so, not solved for), at its own start (closed), where
//! it leaves the domain (the simultaneous root of `g` and the boundary's side, by Newton), or where
//! another branch undercuts it (a corner: `End::Switch`, the next piece leaving along the other
//! branch). Each number is the caller's tolerance or feature size, the rounding's, or structural.
//!
//! Generalised rather than written beside: `meeting::parametric` and `ssi::trace` march where two
//! surfaces meet, a curve in space with a predictor along the normals' cross product; this marches
//! one function's zero in one surface's parameters with branches, and `ssi::unseen`'s cell classes
//! (cleared, beside a known curve, a point pulled onto it) are the search's.
use crate::geom::{Uv,V};
use crate::util::fmath::HALF_READ;
use crate::util::interval::Iv;
use crate::util::space::{distance,segment_distance,Grid};

/// A box of parameters, `[[u0, u1], [v0, v1]]`.
pub type Rect = [[f64;2];2];

/// `f` read at a point: its value, its gradient in the parameters where the caller has it (by the
/// envelope theorem a least value's gradient is `g`'s partial gradient at the minimiser), and what
/// it was read at (the minimiser: the branch, the partner face and point).
#[derive(Clone,Debug)]
pub struct Value<A> { pub f: f64,pub grad: Option<[f64;2]>,pub at: A }

/// Enclosures over a cell of `f` and of its two partial derivatives (`Iv::ANY` where not known).
/// Over a part of the cell outside `f`'s domain (beyond an inner edge) the bound may answer as if
/// `f` had no zero there.
#[derive(Clone,Copy,Debug)]
pub struct Bound { pub value: Iv,pub grad: [Iv;2] }

impl Bound {
    /// The bound of a function Lipschitz in space with constant `l` (a depth, a distance: 1), read
    /// `f` at the cell's middle and every point of the cell within `reach` of it in space.
    pub fn lipschitz(f: f64,l: f64,reach: f64) -> Bound {
        Bound {value:Iv::point(f)+Iv::new(-l*reach,l*reach),grad:[Iv::ANY,Iv::ANY]}
    }
}

/// **What the tracer asks of `f`.**
pub trait Field {
    /// What a value was read at: the minimiser naming the branch (the partner face and point).
    type At: Clone;
    /// `f` at `uv`, the least over the set, with its minimiser; none where `f` is not defined.
    fn least(&self,uv: Uv) -> Option<Value<Self::At>>;
    /// The branch of `from` continued to `uv`: `g(uv, s)` at the local minimiser `s` nearest
    /// `from`'s (smooth along the branch where the least may switch); none where it ends.
    fn along(&self,uv: Uv,from: &Self::At) -> Option<Value<Self::At>>;
    /// The surface's point at `uv` and its two partial derivatives: the metric lengths are read in.
    fn surface(&self,uv: Uv) -> (V,V,V);
    /// Enclosures of `f` and its gradient over `cell`, where the caller has them.
    fn bound(&self,_cell: Rect) -> Option<Bound> { None }
}

/// An inner edge of the domain, a curve in the parameters (a fold curve, a hole's rim): its point
/// at `t ∈ [0, 1]`, and a side function, positive in the domain and nought on the curve.
pub struct Edge<'a> { pub at: &'a dyn Fn(f64) -> Uv,pub side: &'a dyn Fn(Uv) -> f64 }

/// **The question**: the rectangle, its inner edges, the caller's seeds, the tolerance in space
/// every returned chord keeps to, and the feature size in space — the longest step and the spacing
/// at which what no bound answers is sampled.
pub struct Question<'a> { pub rect: Rect,pub edges: &'a [Edge<'a>],pub seeds: &'a [Uv],pub tol: f64,pub feature: f64 }

/// How a piece ends: on the rectangle's rim, on an inner edge (by index), at a corner where the
/// minimiser switches branches (the next piece starts there), or closed on itself.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum End { Rim,Edge(usize),Switch,Closed }

/// A traced piece: its points in the parameters, what each was read at, and its two ends. A
/// closed piece's last point is its first.
#[derive(Clone,Debug)]
pub struct Piece<A> { pub uv: Vec<Uv>,pub at: Vec<A>,pub ends: [End;2] }

impl<A> Piece<A> {
    pub fn closed(&self) -> bool { self.ends[0] == End::Closed }
}

/// What the search for closed curves showed, cell by cell: cleared by the bound, monotone (their
/// edges seeded), answered beside a traced piece, sampled without a bound, and the cells it could
/// not resolve.
#[derive(Clone,Debug,Default)]
pub struct Search { pub free: usize,pub monotone: usize,pub beside: usize,pub sampled: usize,pub unresolved: Vec<Rect> }

/// The zero set traced, and what its search showed.
#[derive(Clone,Debug)]
pub struct Traced<A> { pub pieces: Vec<Piece<A>>,pub search: Search }

/// Newton's steps a corrector takes: one started within a step's sagitta converges quadratically
/// in three or four, and one that has not in sixteen is not converging (structural).
const CORRECTIONS: usize = 16;
/// A cap on the march's steps and the search's cells that a pathology ends at, far past any curve
/// the tolerance resolves (structural, as `ssi::unseen`'s).
const CAP: usize = 4_000_000;

/// `f` read with its gradient.
#[derive(Clone)]
struct Read<A> { f: f64,grad: [f64;2],at: A }

/// **The one place a gradient is taken by differences**, where the caller gave none: central
/// differences of `read` a forward difference's step apart (half the digits of the rectangle's
/// widths, `HALF_READ`), one-sided where a side leaves `read`'s domain.
fn differenced(read: &dyn Fn(Uv) -> Option<f64>,uv: Uv,step: [f64;2]) -> Option<[f64;2]> {
    let f0 = read(uv)?;
    let d = |k: usize| -> Option<f64> {
        let (mut a,mut b) = (uv,uv);
        a[k] += step[k];
        b[k] -= step[k];
        match (read(a),read(b)) {
            (Some(x),Some(y)) => Some((x-y)/(2.*step[k])),
            (Some(x),None) => Some((x-f0)/step[k]),
            (None,Some(y)) => Some((f0-y)/step[k]),
            (None,None) => None,
        }
    };
    Some([d(0)?,d(1)?])
}

fn inverse(g: [[f64;2];2]) -> Option<[[f64;2];2]> {
    let det = g[0][0]*g[1][1]-g[0][1]*g[1][0];
    (det.abs() > 0. && det.is_finite()).then(|| [[g[1][1]/det,-g[0][1]/det],[-g[1][0]/det,g[0][0]/det]])
}
fn apply(m: [[f64;2];2],x: [f64;2]) -> [f64;2] { [m[0][0]*x[0]+m[0][1]*x[1],m[1][0]*x[0]+m[1][1]*x[1]] }
fn dot2(a: [f64;2],b: [f64;2]) -> f64 { a[0]*b[0]+a[1]*b[1] }
fn lerp2(a: Uv,b: Uv,t: f64) -> Uv { [a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t] }
/// A 2 × 2 system `[a; b] x = r` solved, none where it is singular.
fn solve2(a: [f64;2],b: [f64;2],r: [f64;2]) -> Option<[f64;2]> {
    let det = a[0]*b[1]-a[1]*b[0];
    (det.abs() > 0. && det.is_finite()).then(|| [(r[0]*b[1]-r[1]*a[1])/det,(a[0]*r[1]-b[0]*r[0])/det])
}

/// A boundary of the domain: one of the rectangle's four sides (`u0`, `u1`, `v0`, `v1`) or an
/// inner edge.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Side { Rim(usize),Edge(usize) }

impl Side {
    fn end(self) -> End { match self { Side::Rim(_) => End::Rim,Side::Edge(i) => End::Edge(i) } }
}

/// A run of the march: its points, what each was read at, how it started (none: an interior seed)
/// and how it ended.
struct Run<A> { uv: Vec<Uv>,at: Vec<A>,start: Option<End>,end: End }

/// A seed on the domain's boundary, the end it is, and whether a piece has used it.
struct Seed { uv: Uv,end: End,used: bool }

struct Tracer<'a,F: Field> {
    field: &'a F,
    q: &'a Question<'a>,
    /// the step of `differenced`
    step: [f64;2],
    seeds: Vec<Seed>,
    pieces: Vec<Piece<F::At>>,
    /// the traced pieces' chords in space, filed by their ends
    chords: Vec<(V,V)>,
    grid: Grid,
    steps: usize,
}

/// **The zero set of `field` over the question's domain, traced** (see the module): every piece,
/// and what the search for closed curves showed. Refused, by name, where a march's step falls
/// under the rounding or a march or search runs past the cap.
pub fn trace<F: Field>(field: &F,q: &Question) -> Result<Traced<F::At>,String> {
    if !(q.tol > 0. && q.feature >= q.tol) {
        return Err(format!("a zero set traced to a tolerance of {:e} with a feature size of {:e}: the feature must be \
            no less than the tolerance",q.tol,q.feature))
    }
    let r = q.rect;
    let mut t = Tracer {field,q,step:[HALF_READ*(r[0][1]-r[0][0]),HALF_READ*(r[1][1]-r[1][0])],seeds:Vec::new(),
        pieces:Vec::new(),chords:Vec::new(),grid:Grid::new(q.feature),steps:0};
    // the rim's sign changes and touches, then the inner edges'
    let corners = [[r[0][0],r[1][0]],[r[0][1],r[1][0]],[r[0][1],r[1][1]],[r[0][0],r[1][1]]];
    for k in 0..4 {
        for uv in t.segment_seeds(corners[k],corners[(k+1)%4]) { t.add_seed(uv,End::Rim); }
    }
    for (i,e) in q.edges.iter().enumerate() {
        for uv in t.edge_seeds(e) { t.add_seed(uv,End::Edge(i)); }
    }
    for k in 0..t.seeds.len() {
        if t.seeds[k].used { continue }
        let (uv,end) = (t.seeds[k].uv,t.seeds[k].end);
        t.seeds[k].used = true;
        t.from_seed(uv,Some(end))?;
    }
    for &s in q.seeds {
        if let Some(uv) = t.pulled(s,q.feature) { t.from_seed(uv,None)?; }
    }
    let search = t.search()?;
    Ok(Traced {pieces:t.pieces,search})
}

impl<'a,F: Field> Tracer<'a,F> {
    fn point(&self,uv: Uv) -> V { self.field.surface(uv).0 }

    /// The first fundamental form at `uv`.
    fn metric(&self,uv: Uv) -> [[f64;2];2] {
        let (_,su,sv) = self.field.surface(uv);
        let d = crate::util::space::dot;
        [[d(su,su),d(su,sv)],[d(su,sv),d(sv,sv)]]
    }

    /// A parameter vector's length in space.
    fn length(&self,uv: Uv,d: [f64;2]) -> f64 { dot2(d,apply(self.metric(uv),d)).max(0.).sqrt() }

    /// How fast `grad` changes its function per length in space: `√(∇ᵀ G⁻¹ ∇)`.
    fn slope(&self,uv: Uv,grad: [f64;2]) -> Option<f64> {
        let s = dot2(grad,apply(inverse(self.metric(uv))?,grad));
        (s > 0.).then(|| s.sqrt())
    }

    /// How far in space a read value stands from its zero, to first order.
    fn off(&self,uv: Uv,r: &Read<F::At>) -> f64 { match self.slope(uv,r.grad) { Some(s) => r.f.abs()/s,None => f64::INFINITY } }

    fn along(&self,uv: Uv,at: &F::At) -> Option<Read<F::At>> {
        let v = self.field.along(uv,at)?;
        let grad = match v.grad { Some(g) => g,None => differenced(&|x| self.field.along(x,at).map(|w| w.f),uv,self.step)? };
        Some(Read {f:v.f,grad,at:v.at})
    }

    fn least(&self,uv: Uv) -> Option<Read<F::At>> {
        let v = self.field.least(uv)?;
        let grad = match v.grad { Some(g) => g,None => differenced(&|x| self.field.least(x).map(|w| w.f),uv,self.step)? };
        Some(Read {f:v.f,grad,at:v.at})
    }

    /// The zero curve's tangent at `uv` (unit in space), from the branch's gradient.
    fn tangent(&self,uv: Uv,grad: [f64;2]) -> Option<[f64;2]> {
        let t = [-grad[1],grad[0]];
        let l = self.length(uv,t);
        (l > 0. && l.is_finite()).then(|| [t[0]/l,t[1]/l])
    }

    /// The side of the domain `uv` lies beyond, if any.
    fn outside(&self,uv: Uv) -> Option<Side> {
        let r = self.q.rect;
        if uv[0] < r[0][0] { return Some(Side::Rim(0)) }
        if uv[0] > r[0][1] { return Some(Side::Rim(1)) }
        if uv[1] < r[1][0] { return Some(Side::Rim(2)) }
        if uv[1] > r[1][1] { return Some(Side::Rim(3)) }
        self.q.edges.iter().position(|e| (e.side)(uv) < 0.).map(Side::Edge)
    }

    /// A side's function, positive inside, with its gradient.
    fn side(&self,s: Side,uv: Uv) -> Option<(f64,[f64;2])> {
        let r = self.q.rect;
        Some(match s {
            Side::Rim(0) => (uv[0]-r[0][0],[1.,0.]),
            Side::Rim(1) => (r[0][1]-uv[0],[-1.,0.]),
            Side::Rim(2) => (uv[1]-r[1][0],[0.,1.]),
            Side::Rim(_) => (r[1][1]-uv[1],[0.,-1.]),
            Side::Edge(i) => {
                let side = &self.q.edges[i].side;
                (side(uv),differenced(&|x| Some(side(x)),uv,self.step)?)
            }
        })
    }

    /// **The corrector**: Newton on the branch's `g = 0` along its gradient in the metric from
    /// `uv`, until a step is under the rounding's part of the tolerance; none where it moves
    /// farther than `limit` in space (it would land on another curve), leaves `g`'s domain, or does
    /// not come within the tolerance.
    fn correct(&self,mut uv: Uv,at: &F::At,limit: f64) -> Option<(Uv,Read<F::At>)> {
        let mut moved = 0.;
        let mut at = at.clone();
        for _ in 0..CORRECTIONS {
            let r = self.along(uv,&at)?;
            let n = apply(inverse(self.metric(uv))?,r.grad);
            let s = dot2(r.grad,n);
            if !(s > 0.) { return None }
            let length = r.f.abs()/s.sqrt();
            if length <= self.q.tol*HALF_READ { return Some((uv,r)) }
            uv = [uv[0]-r.f*n[0]/s,uv[1]-r.f*n[1]/s];
            moved += length;
            if moved > limit { return None }
            at = r.at;
        }
        let r = self.along(uv,&at)?;
        (self.off(uv,&r) <= self.q.tol).then_some((uv,r))
    }

    /// Whether the chord from `p` to `q` keeps within the tolerance of the branch's zero at its
    /// quarter points, with no other branch undercutting them by more than the tolerance.
    fn chord(&self,p: Uv,q: Uv,at: &F::At) -> bool {
        let tol = self.q.tol;
        [0.25,0.5,0.75].iter().all(|&s| {
            let x = lerp2(p,q,s);
            if self.outside(x).is_some() { return false }
            let Some(r) = self.along(x,at) else { return false };
            let Some(l) = self.least(x) else { return false };
            self.off(x,&r) <= tol && !self.undercut(x,&l)
        })
    }

    /// Whether the least at `uv` stands below nought by more than the tolerance (in space): another
    /// branch covers the point.
    fn undercut(&self,uv: Uv,l: &Read<F::At>) -> bool { l.f < 0. && self.off(uv,l) > self.q.tol }

    /// Whether a point in space lies within `r` of a traced piece's chord.
    fn near_traced(&self,x: V,r: f64) -> bool {
        let reach = r+self.q.feature;
        let mut hit = false;
        self.grid.in_box(x.map(|c| c-reach),x.map(|c| c+reach),|i| {
            let (a,b) = self.chords[i as usize];
            if !hit && segment_distance(x,a,b) <= r { hit = true }
        });
        hit
    }

    fn add_seed(&mut self,uv: Uv,end: End) {
        let x = self.point(uv);
        // a point found twice (a rectangle's corner, on two sides) is one seed
        if self.seeds.iter().any(|s| distance(self.point(s.uv),x) <= 2.*self.q.tol) { return }
        self.seeds.push(Seed {uv,end,used:false});
    }

    /// `uv` pulled onto the zero set along the least's branch, kept where it moved no farther than
    /// `limit` in space, stays in the domain and is undercut by no other branch.
    fn pulled(&self,uv: Uv,limit: f64) -> Option<Uv> {
        let l = self.least(uv)?;
        let (q,_) = self.correct(uv,&l.at,limit)?;
        if self.outside(q).is_some() { return None }
        let lq = self.least(q)?;
        (!self.undercut(q,&lq)).then_some(q)
    }

    /// **The zeros along the segment from `p` to `q`** (a side of the rectangle or of a cell),
    /// isolated by the bound along it where there is one, else sampled at the feature size.
    fn segment_seeds(&self,p: Uv,q: Uv) -> Vec<Uv> {
        let mut out = Vec::new();
        let at = |t: f64| lerp2(p,q,t);
        let read = |t: f64| self.field.least(at(t)).map(|v| v.f);
        let mut stack = vec![(0.,1.)];
        while let Some((a,b)) = stack.pop() {
            let (x,y) = (at(a),at(b));
            let long = distance(self.point(x),self.point(y));
            let cell = [[x[0].min(y[0]),x[0].max(y[0])],[x[1].min(y[1]),x[1].max(y[1])]];
            let Some(bd) = self.field.bound(cell) else {
                self.sampled_roots(&|t| at(t),a,b,&mut out);
                continue
            };
            if !bd.value.has_zero() { continue }
            let rate = bd.grad[0].times(q[0]-p[0])+bd.grad[1].times(q[1]-p[1]);
            if !rate.has_zero() {
                // monotone along it: a root exactly where its ends' signs differ
                let (fa,fb) = (read(a),read(b));
                match (fa,fb) {
                    (Some(fa),Some(fb)) => {
                        if fa == 0. { out.push(x) } else if fb == 0. { out.push(y) }
                        else if (fa < 0.) != (fb < 0.) {
                            out.push(at(crate::util::roots::bracketed_root(read,a,fa,b,fb,f64::EPSILON)));
                        }
                    }
                    // an end beyond `f`'s domain: halved toward it, down to the tolerance
                    _ => if long > self.q.tol { let m = 0.5*(a+b); stack.push((m,b)); stack.push((a,m)); },
                }
                continue
            }
            if long <= self.q.tol {
                if let Some(t) = self.touch(&|t| at(t),a,b) { out.push(at(t)) }
                continue
            }
            let m = 0.5*(a+b);
            stack.push((m,b));
            stack.push((a,m));
        }
        out
    }

    /// A zero of `f` on `[a, b]` along the curve `at`: a sign change's root, else where `|f|` is
    /// least, kept where it is within the tolerance of the zero set.
    fn touch(&self,at: &dyn Fn(f64) -> Uv,a: f64,b: f64) -> Option<f64> {
        let read = |t: f64| self.field.least(at(t)).map(|v| v.f);
        if let (Some(fa),Some(fb)) = (read(a),read(b)) {
            if fa == 0. { return Some(a) }
            if fb == 0. { return Some(b) }
            if (fa < 0.) != (fb < 0.) { return Some(crate::util::roots::bracketed_root(read,a,fa,b,fb,f64::EPSILON)) }
        }
        // the least of `|f|`, to the rounding of the parameter (64 golden or parabolic steps: each
        // keeps at least 0.38 of the bracket, so 64 reach a part in 10¹³, structural)
        let abs = |t: f64| read(t).map_or(f64::INFINITY,f64::abs);
        let least = crate::util::roots::brent(&abs,a,b,f64::EPSILON*(a.abs()+b.abs()),64,|_,_| false);
        let uv = at(least.at);
        let r = self.least(uv)?;
        (self.off(uv,&r) <= self.q.tol && self.outside(uv).map_or(true,|s| matches!(s,Side::Edge(_)))).then_some(least.at)
    }

    /// The zeros along the curve `at` over `[a, b]` with no bound: sampled no farther apart in
    /// space than the feature size (halving where a chord is longer), each sign change's root and
    /// each touch (a sample whose `|f|` is less than both its neighbours') read to the rounding.
    fn sampled_roots(&self,at: &dyn Fn(f64) -> Uv,a: f64,b: f64,out: &mut Vec<Uv>) {
        let mut ts = vec![a,b];
        let mut k = 0;
        while k+1 < ts.len() {
            let (x,y) = (ts[k],ts[k+1]);
            if distance(self.point(at(x)),self.point(at(y))) > self.q.feature && y-x > f64::EPSILON*(x.abs()+y.abs()) {
                ts.insert(k+1,0.5*(x+y));
            } else { k += 1 }
        }
        let fs: Vec<Option<f64>> = ts.iter().map(|&t| self.field.least(at(t)).map(|v| v.f)).collect();
        let mut found: Vec<f64> = Vec::new();
        for i in 0..ts.len() {
            let Some(fi) = fs[i] else { continue };
            if fi == 0. { found.push(ts[i]); continue }
            if i+1 < ts.len() {
                if let Some(fj) = fs[i+1] {
                    if fj != 0. && (fi < 0.) != (fj < 0.) {
                        found.push(crate::util::roots::bracketed_root(|t| self.field.least(at(t)).map(|v| v.f),ts[i],fi,ts[i+1],fj,f64::EPSILON));
                    }
                }
            }
            // a touch: `|f|` least here among its neighbours, which share its sign
            let lo = if i > 0 { fs[i-1] } else { None };
            let hi = fs.get(i+1).copied().flatten();
            let side_ok = |o: Option<f64>| o.map_or(true,|o| (o < 0.) == (fi < 0.) && o.abs() >= fi.abs());
            if side_ok(lo) && side_ok(hi) {
                let (l,h) = (ts[i.saturating_sub(1)],ts[(i+1).min(ts.len()-1)]);
                if let Some(t) = self.touch(at,l,h) { found.push(t) }
            }
        }
        out.extend(found.into_iter().map(at));
    }

    /// The zeros along an inner edge, sampled (`sampled_roots`).
    fn edge_seeds(&self,e: &Edge) -> Vec<Uv> {
        let mut out = Vec::new();
        self.sampled_roots(e.at,0.,1.,&mut out);
        out
    }

    /// **From one seed, the pieces through it**, both ways, recorded. A boundary seed's pieces each
    /// start there; an interior seed's two ways are joined, or close on it.
    fn from_seed(&mut self,uv: Uv,boundary: Option<End>) -> Result<(),String> {
        if self.near_traced(self.point(uv),2.*self.q.tol) { return Ok(()) }
        let Some(l) = self.least(uv) else { return Ok(()) };
        let own = boundary.is_none();
        let (mut ahead,closed) = self.march(uv,l.at.clone(),1.,own,boundary)?;
        let mut pieces = Vec::new();
        if closed {
            if ahead.len() == 1 {
                let r = ahead.pop().unwrap();
                pieces.push(Piece {uv:r.uv,at:r.at,ends:[End::Closed,End::Closed]});
            } else {
                // corners on the loop: the last run and the first are one piece through the seed
                let first = ahead.remove(0);
                let mut last = ahead.pop().unwrap();
                last.uv.extend_from_slice(&first.uv[1..]);
                last.at.extend(first.at.into_iter().skip(1));
                for r in ahead { pieces.push(Piece {uv:r.uv,at:r.at,ends:[r.start.unwrap_or(End::Switch),r.end]}); }
                pieces.push(Piece {uv:last.uv,at:last.at,ends:[last.start.unwrap_or(End::Switch),first.end]});
            }
        } else {
            let (mut back,_) = self.march(uv,l.at,-1.,own,boundary)?;
            if own && !back.is_empty() && !ahead.is_empty() {
                // the two ways' first runs are one piece through the seed
                let b = back.remove(0);
                let a = ahead.remove(0);
                let mut uvs: Vec<Uv> = b.uv.into_iter().rev().collect();
                let mut ats: Vec<F::At> = b.at.into_iter().rev().collect();
                uvs.extend_from_slice(&a.uv[1..]);
                ats.extend(a.at.into_iter().skip(1));
                pieces.push(Piece {uv:uvs,at:ats,ends:[b.end,a.end]});
            }
            for r in back { pieces.push(Piece {uv:r.uv.into_iter().rev().collect(),at:r.at.into_iter().rev().collect(),ends:[r.end,r.start.unwrap_or(End::Switch)]}); }
            for r in ahead { pieces.push(Piece {uv:r.uv,at:r.at,ends:[r.start.unwrap_or(End::Switch),r.end]}); }
        }
        for p in pieces {
            // a way that leaves the domain at once from a boundary seed is no piece
            let long: f64 = p.uv.windows(2).map(|w| distance(self.point(w[0]),self.point(w[1]))).sum();
            let bounded = |e: End| matches!(e,End::Rim | End::Edge(_));
            if long <= self.q.tol && bounded(p.ends[0]) && bounded(p.ends[1]) { continue }
            self.record(p);
        }
        Ok(())
    }

    fn record(&mut self,p: Piece<F::At>) {
        let xs: Vec<V> = p.uv.iter().map(|&uv| self.point(uv)).collect();
        for w in xs.windows(2) {
            let i = self.chords.len() as u32;
            self.chords.push((w[0],w[1]));
            self.grid.insert(w[0],i);
            self.grid.insert(w[1],i);
        }
        if xs.len() == 1 { let i = self.chords.len() as u32; self.chords.push((xs[0],xs[0])); self.grid.insert(xs[0],i); }
        self.pieces.push(p);
    }

    /// The step halved, refused once it falls under the rounding of the parameters at `p`.
    fn halve(&self,h: &mut f64,p: Uv,t: [f64;2]) -> Result<(),String> {
        *h /= 2.;
        let r = self.q.rect;
        let room = f64::EPSILON*(p[0].abs()+p[1].abs()+(r[0][1]-r[0][0])+(r[1][1]-r[1][0]));
        if *h*(t[0].abs()+t[1].abs()) <= room {
            return Err(format!("the zero set's march stalled at ({:.9}, {:.9}): its step fell under the rounding there \
                (a singular point of the zero set the tolerance does not resolve), not traced",p[0],p[1]))
        }
        Ok(())
    }

    /// **One way of the march** from `start` on the branch `at`, the tangent's sense `sign`: its
    /// runs (cut at corners), and whether it closed on `start` (`own`: an interior seed).
    #[allow(clippy::type_complexity)]
    fn march(&mut self,start: Uv,at: F::At,sign: f64,own: bool,first: Option<End>) -> Result<(Vec<Run<F::At>>,bool),String> {
        let (tol,feature) = (self.q.tol,self.q.feature);
        let mut runs = Vec::new();
        let Some(r0) = self.along(start,&at) else { return Ok((runs,false)) };
        // a seed where the gradient vanishes (a cusp's point, a touch of the boundary to second
        // order) is no start: it stays a target the marches from elsewhere arrive at
        let Some(t0) = self.tangent(start,r0.grad) else { return Ok((runs,false)) };
        let mut t = [t0[0]*sign,t0[1]*sign];
        let mut cur = r0.at.clone();
        let mut run = Run {uv:vec![start],at:vec![r0.at],start:first,end:End::Closed};
        let mut p = start;
        let mut h = feature;
        let mut moved = 0.;
        loop {
            self.steps += 1;
            if self.steps > CAP {
                return Err(format!("the zero set's march took {CAP} steps without ending, at ({:.9}, {:.9}), not traced",p[0],p[1]))
            }
            let xp = self.point(p);
            // a boundary seed, or the start, within the step ahead: arrived (a tangential end is
            // reached so, never solved for)
            let mut best: Option<(f64,usize)> = None;
            for (k,s) in self.seeds.iter().enumerate() {
                let d = distance(xp,self.point(s.uv));
                if d > 0. && d <= h && best.is_none_or(|b| d < b.0) { best = Some((d,k)) }
            }
            let mut arrived: Option<(Uv,End,Option<usize>)> = None;
            if let Some((_,k)) = best {
                let x = self.seeds[k].uv;
                if dot2(apply(self.metric(p),t),[x[0]-p[0],x[1]-p[1]]) > 0. && self.chord(p,x,&cur) {
                    arrived = Some((x,self.seeds[k].end,Some(k)));
                }
            }
            if arrived.is_none() && own && moved > 0. {
                let d = distance(xp,self.point(start));
                if d <= h && dot2(apply(self.metric(p),t),[start[0]-p[0],start[1]-p[1]]) > 0. && self.chord(p,start,&cur) {
                    arrived = Some((start,End::Closed,None));
                }
            }
            if let Some((x,end,k)) = arrived {
                if let Some(k) = k { self.seeds[k].used = true }
                let a = self.along(x,&cur).map_or(cur.clone(),|r| r.at);
                run.uv.push(x);
                run.at.push(a);
                run.end = end;
                runs.push(run);
                return Ok((runs,end == End::Closed))
            }
            let q0 = [p[0]+h*t[0],p[1]+h*t[1]];
            let corrected = if self.outside(q0).is_some() { None } else { self.correct(q0,&cur,h/2.) };
            let beyond = match corrected { None => self.outside(q0),Some((q,_)) => self.outside(q) };
            if let Some(side) = beyond {
                let to = corrected.as_ref().map_or(q0,|c| c.0);
                if let Some((e,a)) = self.exit(p,to,&cur,side,h) {
                    run.uv.push(e);
                    run.at.push(a);
                    run.end = side.end();
                    runs.push(run);
                    return Ok((runs,false))
                }
                // within the tolerance of the side already: the piece ends where it stands
                if let Some((s,g)) = self.side(side,p) {
                    if self.slope(p,g).is_some_and(|k| s/k <= tol) {
                        run.end = side.end();
                        runs.push(run);
                        return Ok((runs,false))
                    }
                }
                self.halve(&mut h,p,t)?;
                continue
            }
            let Some((q,rq)) = corrected else { self.halve(&mut h,p,t)?; continue };
            let Some(lq) = self.least(q) else { self.halve(&mut h,p,t)?; continue };
            if self.undercut(q,&lq) {
                // another branch covers `q`: the corner between, where both vanish
                match self.corner(p,q,&cur,&lq.at,h) {
                    Some((c,mine,other)) => {
                        run.uv.push(c);
                        run.at.push(mine.at.clone());
                        run.end = End::Switch;
                        runs.push(run);
                        let Some(tc) = self.tangent(c,other.grad) else {
                            return Err(format!("the zero set's corner at ({:.9}, {:.9}) leaves along no tangent, not traced",c[0],c[1]))
                        };
                        // along the other branch into where the first stands above nought
                        let s = if dot2(mine.grad,tc) >= 0. { 1. } else { -1. };
                        t = [tc[0]*s,tc[1]*s];
                        cur = other.at.clone();
                        run = Run {uv:vec![c],at:vec![other.at],start:Some(End::Switch),end:End::Closed};
                        moved += distance(xp,self.point(c));
                        p = c;
                        h = feature;
                    }
                    None => self.halve(&mut h,p,t)?,
                }
                continue
            }
            if !self.chord(p,q,&cur) { self.halve(&mut h,p,t)?; continue }
            let Some(tq) = self.tangent(q,rq.grad) else { self.halve(&mut h,p,t)?; continue };
            let s = if dot2(apply(self.metric(q),tq),[q[0]-p[0],q[1]-p[1]]) >= 0. { 1. } else { -1. };
            t = [tq[0]*s,tq[1]*s];
            moved += distance(xp,self.point(q));
            cur = rq.at.clone();
            run.uv.push(q);
            run.at.push(rq.at);
            p = q;
            h = (2.*h).min(feature);
        }
    }

    /// **Where the march leaves the domain** across `side`, between `p` (inside) and `q`
    /// (beyond): the simultaneous root of the branch's `g` and the side's function, by Newton from
    /// where the chord crosses the side; a boundary seed of that side within the tolerance stands
    /// for it. None where Newton does not settle within the step.
    fn exit(&mut self,p: Uv,q: Uv,at: &F::At,side: Side,h: f64) -> Option<(Uv,F::At)> {
        let tol = self.q.tol;
        let s = |x: Uv| self.side(side,x).map(|v| v.0);
        let (sp,sq) = (s(p)?,s(q)?);
        let mut c = if sp > 0. && sq < 0. {
            lerp2(p,q,crate::util::roots::bracketed_root(|x| s(lerp2(p,q,x)),0.,sp,1.,sq,f64::EPSILON))
        } else { p };
        let mut a = at.clone();
        let mut done = false;
        for _ in 0..CORRECTIONS {
            let r = self.along(c,&a)?;
            let (sv,sg) = self.side(side,c)?;
            let d = solve2(r.grad,sg,[-r.f,-sv])?;
            c = [c[0]+d[0],c[1]+d[1]];
            a = r.at;
            if self.length(c,d) <= tol*HALF_READ { done = true; break }
        }
        let r = self.along(c,&a)?;
        let (sv,sg) = self.side(side,c)?;
        if !(done || (self.off(c,&r) <= tol && self.slope(c,sg).is_some_and(|k| sv.abs()/k <= tol))) { return None }
        if distance(self.point(p),self.point(c)) > h+tol { return None }
        // the end is on the side, or within the rounding of it; inside every other
        if !self.chord(p,lerp2(p,c,1.-HALF_READ),at) { return None }
        let x = self.point(c);
        let end = side.end();
        if let Some(k) = self.seeds.iter().position(|s| s.end == end && distance(self.point(s.uv),x) <= 2.*tol) {
            self.seeds[k].used = true;
            let u = self.seeds[k].uv;
            return Some((u,self.along(u,&a).map_or(r.at,|w| w.at)))
        }
        Some((c,r.at))
    }

    /// **The corner** between the branch `mine` (zero along the march) and `other`, which
    /// undercuts `q`: Newton on both from where `other` changes sign along the chord, kept where
    /// it is within the step of `p` and no third branch undercuts it. Returns the corner and both
    /// branches read there.
    #[allow(clippy::type_complexity)]
    fn corner(&self,p: Uv,q: Uv,mine: &F::At,other: &F::At,h: f64) -> Option<(Uv,Read<F::At>,Read<F::At>)> {
        let tol = self.q.tol;
        let g = |x: Uv| self.field.along(x,other).map(|v| v.f);
        let (gp,gq) = (g(p)?,g(q)?);
        let mut c = if (gp < 0.) != (gq < 0.) {
            lerp2(p,q,crate::util::roots::bracketed_root(|x| g(lerp2(p,q,x)),0.,gp,1.,gq,f64::EPSILON))
        } else { lerp2(p,q,0.5) };
        let (mut a,mut b) = (mine.clone(),other.clone());
        for _ in 0..CORRECTIONS {
            let ra = self.along(c,&a)?;
            let rb = self.along(c,&b)?;
            let d = solve2(ra.grad,rb.grad,[-ra.f,-rb.f])?;
            c = [c[0]+d[0],c[1]+d[1]];
            a = ra.at;
            b = rb.at;
            if self.length(c,d) <= tol*HALF_READ { break }
        }
        let ra = self.along(c,&a)?;
        let rb = self.along(c,&b)?;
        if self.off(c,&ra) > tol || self.off(c,&rb) > tol { return None }
        if self.outside(c).is_some() || distance(self.point(p),self.point(c)) > h+tol { return None }
        let l = self.least(c)?;
        if self.undercut(c,&l) { return None }
        Some((c,ra,rb))
    }

    /// A cell's reach in space from its middle, sampled at its corners and its sides' middles.
    fn reach(&self,c: Rect) -> f64 {
        let m = self.point([(c[0][0]+c[0][1])/2.,(c[1][0]+c[1][1])/2.]);
        let mut r: f64 = 0.;
        for i in 0..3 { for j in 0..3 {
            let uv = [c[0][0]+(c[0][1]-c[0][0])*i as f64/2.,c[1][0]+(c[1][1]-c[1][0])*j as f64/2.];
            r = r.max(distance(m,self.point(uv)));
        } }
        r
    }

    /// **The search for closed curves** (see the module): the rectangle halved, each cell cleared
    /// by the bound, shown monotone (its edges seeded), answered beside a traced piece, or pulled
    /// onto the zero set from its middle; seeds found are traced at once.
    fn search(&mut self) -> Result<Search,String> {
        let (tol,feature) = (self.q.tol,self.q.feature);
        let mut out = Search::default();
        let mut cells = vec![self.q.rect];
        let mut asked = 0;
        while let Some(c) = cells.pop() {
            asked += 1;
            if asked > CAP {
                return Err(format!("the search for the zero set's closed curves asked {CAP} cells without settling, not traced"))
            }
            let mid = [(c[0][0]+c[0][1])/2.,(c[1][0]+c[1][1])/2.];
            let reach = self.reach(c);
            let bound = self.field.bound(c);
            if let Some(b) = bound {
                if !b.value.has_zero() { out.free += 1; continue }
                if !b.grad[0].has_zero() || !b.grad[1].has_zero() {
                    out.monotone += 1;
                    let corners = [[c[0][0],c[1][0]],[c[0][1],c[1][0]],[c[0][1],c[1][1]],[c[0][0],c[1][1]]];
                    for k in 0..4 {
                        for uv in self.segment_seeds(corners[k],corners[(k+1)%4]) {
                            if self.outside(uv).is_none() { self.from_seed(uv,None)?; }
                        }
                    }
                    continue
                }
            }
            // what the bound does not settle is answered beside a traced piece: at the tolerance
            // where it bounds the gradient too, at the feature size where it bounds `f` alone or
            // nothing
            let graded = bound.is_some_and(|b| b.grad[0].0.is_some() || b.grad[1].0.is_some());
            let leaf = if graded { tol } else { feature };
            if 2.*reach <= leaf {
                if self.near_traced(self.point(mid),reach+tol) { out.beside += 1; continue }
                if let Some(s) = self.pulled(mid,reach) {
                    self.from_seed(s,None)?;
                    if self.near_traced(self.point(mid),reach+tol) { out.beside += 1; continue }
                }
                if bound.is_none() {
                    out.sampled += 1;
                    for s in self.rays(mid,c) { self.from_seed(s,None)?; }
                    continue
                }
                if 2.*reach <= tol { out.unresolved.push(c); continue }
            }
            // halved across its longer way in space
            let (pu,pv) = (self.point([c[0][0],mid[1]]),self.point([c[0][1],mid[1]]));
            let (qu,qv) = (self.point([mid[0],c[1][0]]),self.point([mid[0],c[1][1]]));
            if distance(pu,pv) >= distance(qu,qv) {
                cells.push([[mid[0],c[0][1]],c[1]]);
                cells.push([[c[0][0],mid[0]],c[1]]);
            } else {
                cells.push([c[0],[mid[1],c[1][1]]]);
                cells.push([c[0],[c[1][0],mid[1]]]);
            }
        }
        Ok(out)
    }

    /// **Seeds from `f`'s critical point in a cell, with no bound** (sampled evidence): Newton on
    /// `∇f = 0` from its middle (the Hessian by differences of the gradient), kept where it settles
    /// in the cell with `f` off nought; from it, rays along the parameters' four ways out to where
    /// `f` first changes sign, read at lengths doubling from the tolerance (a closed curve round an
    /// extremum of a smooth `f` is crossed by every ray from it).
    fn rays(&self,mid: Uv,c: Rect) -> Vec<Uv> {
        let tol = self.q.tol;
        let grad = |x: Uv| self.least(x).map(|r| r.grad);
        let mut x = mid;
        let mut settled = false;
        for _ in 0..CORRECTIONS {
            let Some(g) = grad(x) else { return Vec::new() };
            let Some(gu) = differenced(&|y| grad(y).map(|g| g[0]),x,self.step) else { return Vec::new() };
            let Some(gv) = differenced(&|y| grad(y).map(|g| g[1]),x,self.step) else { return Vec::new() };
            let Some(d) = solve2(gu,gv,[-g[0],-g[1]]) else { return Vec::new() };
            x = [x[0]+d[0],x[1]+d[1]];
            if x[0] < c[0][0] || x[0] > c[0][1] || x[1] < c[1][0] || x[1] > c[1][1] { return Vec::new() }
            if self.length(x,d) <= tol*HALF_READ { settled = true; break }
        }
        let Some(f0) = self.least(x).map(|r| r.f) else { return Vec::new() };
        if !settled || f0 == 0. { return Vec::new() }
        let r = self.q.rect;
        let span = self.reach(r)*2.;
        let mut out = Vec::new();
        for w in [[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]] {
            let l = self.length(x,w);
            if !(l > 0.) { continue }
            let at = |s: f64| [x[0]+w[0]*s/l,x[1]+w[1]*s/l];
            let (mut a,mut fa) = (0.,f0);
            let mut s = tol;
            while s <= 2.*span {
                let y = at(s);
                if self.outside(y).is_some() { break }
                let Some(fs) = self.least(y).map(|r| r.f) else { break };
                if (fs < 0.) != (fa < 0.) || fs == 0. {
                    let root = if fs == 0. { s } else {
                        crate::util::roots::bracketed_root(|t| self.least(at(t)).map(|r| r.f),a,fa,s,fs,f64::EPSILON*s) };
                    if let Some(p) = self.pulled(at(root),tol) { out.push(p) }
                    break
                }
                (a,fa) = (s,fs);
                s *= 2.;
            }
        }
        out
    }
}
