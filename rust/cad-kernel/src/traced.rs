//! **A traced meeting read as a smooth curve** (#1034): the meeting of two surfaces with no closed
//! form solves an ODE — its unit tangent is `T = ∇a × ∇b / |∇a × ∇b|`, a closed form in the two
//! surfaces — and a trace's points are that curve's nodes. What it reads once, at its points
//! (`Read`):
//!
//! * **The rates at each point** (`jet`): `T`, and the curvature vector `K = C''(s)` by arc length,
//!   closed too — differentiating `∇a(C) · C' = 0` gives `n_a · K = κ_a`, the normal curvature of
//!   `a` along `T` (`Surface::bend`), likewise `b`, and `T · K = 0`: three rows, one solve.
//! * **Between two points, a quintic of the arc length** (`Smooth`): the Hermite interpolant of
//!   both points with their `T` and `K`, its length read off itself (the fixed point of
//!   `L = ∫ |P'|`), so it follows the curve to the sixth power of the step — the parabola it
//!   replaces, through the chords alone, to the third.
//! * **A parameter running evenly along the curve** (`read`): its `m` stretches over `[0, m]` as
//!   before, but each point where the length run to it puts it, so the speed is the same
//!   everywhere, through every point.
//!
//! A point of the curve is that quintic's point pulled onto both surfaces (`Traced::project`,
//! as before); its rates are read there, not differenced: `C' = T v`, `v = T · g'` the speed of
//! the guess `g` along the curve (the projection's derivative at the curve is `T Tᵀ`), and
//! `C'' = K v² + T (v K · g' + T · g'')`.
//!
//! **Where it is not read so.** A stretch whose ends do not both stand clear of a touch (the two
//! surfaces nearer tangent than `CLEAR`), or that turns too far for its chord to stand for it
//! (`ALONG_COS`, its ends' curvature over its length), keeps the parabola through the chords
//! (`Traced::between`) and its pace — touches stay their own case (#716): a trace ends at one,
//! and where `∇a × ∇b` vanishes the field has no direction to read.
use crate::enclose::{I3,Patch};
use crate::geom::{Surface,Traced,V};
use crate::util::interval::Iv;
use crate::util::quadrature::GAUSS8;
use crate::util::space::{cross,dot,norm,normalised,scale,sub};

/// The sine of the angle between the two surfaces below which their meeting's direction is not
/// read from them: a touch's, where `T` and `K` (which goes as one over its square) are lost.
pub(crate) const CLEAR: f64 = 1e-3;
/// How far a stretch read by its quintic may turn: each end's tangent within this cosine of its
/// chord (25°), and its curvature times its length at most one.
const ALONG_COS: f64 = 0.9;

/// **The meeting's unit tangent**, from the two surfaces' gradients `g` at a point of it, turned
/// along `along`: `None` where the two stand within `CLEAR` of tangent there, or a gradient
/// vanishes.
pub(crate) fn tangent(g: (V,V),along: V) -> Option<V> { normals(g,along).map(|(t,..)| t) }

/// `T` and the two unit normals, from the gradients `g`.
fn normals((ga,gb): (V,V),along: V) -> Option<(V,V,V)> {
    let (na,nb) = (normalised(ga)?,normalised(gb)?);
    let w = cross(na,nb);
    let s = norm(w);
    if !(s >= CLEAR) { return None }
    let t = scale(w,if dot(w,along) < 0. { -1./s } else { 1./s });
    Some((t,na,nb))
}

/// **The meeting's unit tangent and curvature vector at `p`** (the module's), `g` the two
/// surfaces' gradients there: `K` solves `n_a · K = κ_a`, `n_b · K = κ_b` with the least norm,
/// which lies in the plane of the two normals, so `T · K = 0` too.
pub(crate) fn jet(a: &Surface,b: &Surface,p: V,g: (V,V),along: V) -> Option<(V,V)> {
    let (t,na,nb) = normals(g,along)?;
    let (ka,kb) = (a.bend(p,t,na)?,b.bend(p,t,nb)?);
    let k = crate::util::roots::least_norm_step(&[na,nb],&[ka,kb])?;
    k.iter().all(|x| x.is_finite()).then_some((t,k))
}

/// The quintic Hermite basis on `[0, 1]` in powers of `x`: value, rate and second rate at 0, then
/// at 1 (second rate, rate, value).
const BASIS: [[f64;6];6] = [
    [1.,0.,0.,-10.,15.,-6.],
    [0.,1.,0.,-6.,8.,-3.],
    [0.,0.,0.5,-1.5,1.5,-0.5],
    [0.,0.,0.,0.5,-1.,0.5],
    [0.,0.,0.,-4.,7.,-3.],
    [0.,0.,0.,10.,-15.,6.],
];

/// A quintic's value and two rates at `x`, its coefficients `c` in powers of `x`.
fn quintic(c: &[f64;6],x: f64) -> (f64,f64,f64) {
    let v = ((((c[5]*x+c[4])*x+c[3])*x+c[2])*x+c[1])*x+c[0];
    let d = (((5.*c[5]*x+4.*c[4])*x+3.*c[3])*x+2.*c[2])*x+c[1];
    let dd = ((20.*c[5]*x+12.*c[4])*x+6.*c[3])*x+2.*c[2];
    (v,d,dd)
}

/// A vector quintic's value and two rates at `x`.
fn quintic3(c: &[V;6],x: f64) -> (V,V,V) {
    let r: [(f64,f64,f64);3] = std::array::from_fn(|k| quintic(&c.map(|v| v[k]),x));
    (r.map(|q| q.0),r.map(|q| q.1),r.map(|q| q.2))
}

/// The quintic in `x ∈ [0, 1]` holding `ends` (value, first and second rate in `x`) at 0 and 1.
fn hermite(ends: [V;6]) -> [V;6] {
    std::array::from_fn(|j| std::array::from_fn(|k| (0..6).map(|i| ends[i][k]*BASIS[i][j]).sum()))
}

/// A stretch read by its quintic: its length along the curve, and the quintic in `x = σ / len`,
/// `σ` the length along it — the fraction of the stretch, the curve's parameter running evenly
/// along its length (`read`).
#[derive(Clone,Debug)]
struct Smooth { len: f64,curve: [V;6] }

impl Smooth {
    /// The quintic between `p0` and `p1` with their tangents and curvature vectors, its length the
    /// fixed point of `L = ∫₀¹ |P_x| dx` (`P_x = L dP/dσ`, of unit length where `L` is right);
    /// `None` where it does not run along its chord (its speed off one by half anywhere).
    fn new((p0,t0,k0): (V,V,V),(p1,t1,k1): (V,V,V)) -> Option<Smooth> {
        let curve = |l: f64| hermite([p0,scale(t0,l),scale(k0,l*l),scale(k1,l*l),scale(t1,l),p1]);
        let length = |c: &[V;6]| GAUSS8.iter().map(|&(x,w)| 0.5*w*norm(quintic3(c,0.5*(1.+x)).1)).sum::<f64>();
        let mut len = norm(sub(p1,p0));
        let mut c = curve(len);
        for _ in 0..12 {
            let next = length(&c);
            if !(next > 0.) { return None }
            let settled = (next-len).abs() <= 4.*f64::EPSILON*len;
            len = next;
            c = curve(len);
            if settled { break }
        }
        let fair = GAUSS8.iter().all(|&(x,_)| { let s = norm(quintic3(&c,0.5*(1.+x)).1)/len; (0.5..=1.5).contains(&s) });
        fair.then_some(Smooth {len,curve:c})
    }
    /// The guess at the fraction `f` of the stretch and its two rates in `f`.
    fn at(&self,f: f64) -> (V,V,V) { quintic3(&self.curve,f) }
    /// The guess's rate in `f` over the fractions `[f₀, f₁]`, enclosed: the quintic's slope read
    /// in intervals (`isolate::Poly`).
    fn rate_over(&self,[f0,f1]: [f64;2]) -> I3 {
        use crate::util::isolate::Function;
        std::array::from_fn(|k| crate::util::isolate::Poly(&self.curve.map(|v| v[k])).slope(f0,f1))
    }
    /// The guess's greatest rate in `f` over the stretch, sampled.
    fn fastest(&self) -> f64 { (0..=32).map(|j| norm(quintic3(&self.curve,j as f64/32.).1)).fold(0.,f64::max) }
}

/// **What a trace reads once at its points** (the module's): its points pulled onto both surfaces
/// (`nodes`), each stretch's quintic where it has one, the curve's parameter at each point (`at`),
/// whether its rate may step at each (`kinks`), its greatest speed (no less than its longest chord
/// over its extent), and each whole stretch's enclosure and their hull, each read once when first
/// asked (`enclose`).
#[derive(Debug,Default)]
pub(crate) struct Read {
    smooth: Vec<Option<Smooth>>,pub(crate) speed: f64,pub(crate) nodes: Vec<V>,
    pub(crate) at: Vec<f64>,pub(crate) kinks: Vec<bool>,
    pub(crate) wholes: std::sync::OnceLock<Vec<Patch<1>>>,pub(crate) whole: std::sync::OnceLock<Patch<1>>,
}

/// Stretch `i`'s extent in the parameter, `at` the parameter at each point (a stretch of none, two
/// points at one place, read as a unit of it).
fn span(at: &[f64],i: usize) -> f64 { let d = at[i+1]-at[i]; if d > 0. { d } else { 1. } }

/// **A trace's `Read`**: its points pulled onto both surfaces, their jets read, each stretch's
/// quintic built where both its ends have one and it runs along its chord, and the parameter laid:
/// the length run along the curve to each point, by its stretches' lengths (a quintic's own, a
/// chord where none), scaled so the trace's `m` stretches run over `[0, m]` — **the curve's
/// parameter runs evenly along its length**, its speed the same everywhere (#1034). Its points then
/// stand at no whole parameter in general, and a stretch's fraction `f` is its parameter's over
/// it. (Its parameter counted its points before: where they stood unevenly, its speed had to
/// change within a stretch however smoothly it was laid, and a stretch two-thirds its neighbour's
/// length swayed it by a quarter, which a ruled wall's net could not follow to the file's 1e-6.) A
/// trace carried from another (`Traced::carried`, moved, reflected or mapped by a similarity, which
/// scales every stretch alike) keeps that one's parameters at its points, so whatever was placed on
/// it is where it was (under any other map a trace's image is no trace, `Curve::mapped`). A point is
/// a kink where a stretch either side has no smooth reading, or where its speed steps (carried
/// parameters that do not fit, which none of those do but by rounding).
pub(crate) fn read(tr: &Traced) -> Read {
    let n = tr.pts.len();
    if n < 2 { return Read::default() }
    let m = n-1;
    let mut jets: Vec<Option<(V,V,V)>> = Vec::with_capacity(n);
    let mut points: Vec<V> = Vec::with_capacity(n);
    for i in 0..=m {
        if tr.closed && i == m { let first = jets[0]; jets.push(first); points.push(points[0]); continue }
        let (p,g) = tr.projected(tr.pts[i]);
        points.push(p);
        let prev = if i > 0 { tr.pts[i-1] } else if tr.closed && m > 1 { tr.pts[m-1] } else { tr.pts[i] };
        let next = if i < m { tr.pts[i+1] } else { tr.pts[i] };
        jets.push(jet(&tr.a,&tr.b,p,g,sub(next,prev)).map(|(t,k)| (p,t,k)));
    }
    let smooth: Vec<Option<Smooth>> = (0..m).map(|i| {
        let ((p0,t0,k0),(p1,t1,k1)) = (jets[i]?,jets[i+1]?);
        let c = sub(p1,p0);
        let l = norm(c);
        let along = |t: V| dot(t,c) >= ALONG_COS*l;
        if !(l > 0.) || !along(t0) || !along(t1) || norm(k0).max(norm(k1))*l > 1. { return None }
        Smooth::new((p0,t0,k0),(p1,t1,k1))
    }).collect();
    // each stretch's length: its quintic's, or its chord's
    let lens: Vec<f64> = (0..m).map(|i| match &smooth[i] { Some(s) => s.len,None => norm(sub(tr.pts[i+1],tr.pts[i])) }).collect();
    let at: Vec<f64> = match tr.carried_at() {
        Some(at) => at.to_vec(),
        _ => {
            let mut run = vec![0.;n];
            for i in 0..m { run[i+1] = run[i]+lens[i] }
            let total = run[m];
            if total > 0. { run.iter().enumerate().map(|(i,&s)| if i == m { m as f64 } else { m as f64*s/total }).collect() }
            else { (0..=m).map(|i| i as f64).collect() }
        }
    };
    // the speed over each stretch read smoothly, its length over its extent
    let pace = |i: usize| smooth[i].as_ref().map(|s| s.len/span(&at,i));
    let kinks: Vec<bool> = (0..=m).map(|i| {
        let (left,right) = (if i > 0 { Some(i-1) } else if tr.closed { Some(m-1) } else { None },
            if i < m { Some(i) } else if tr.closed { Some(0) } else { None });
        match (left.map(pace),right.map(pace)) {
            (Some(Some(a)),Some(Some(b))) => (a-b).abs() > 1e-9*a.max(b),
            (None,Some(Some(_))) | (Some(Some(_)),None) => false,
            _ => true,
        }
    }).collect();
    let speed = (0..m).map(|i| {
        let chord = norm(sub(tr.pts[i+1],tr.pts[i]));
        (match &smooth[i] { Some(s) => s.fastest().max(chord),None => 1.25*lens[i] })/span(&at,i)
    }).fold(0.,f64::max);
    Read {smooth,speed,nodes:points,at,kinks,..Read::default()}
}

impl Read {
    /// The stretch `i`'s extent in the curve's parameter (a stretch of none read as a unit).
    pub(crate) fn span(&self,i: usize) -> f64 { span(&self.at,i) }
    /// The guess at the fraction `f` of stretch `i` and its two rates in the curve's parameter: the
    /// stretch's quintic where it has one, the parabola through the chords where not.
    pub(crate) fn guess(&self,tr: &Traced,i: usize,f: f64) -> (V,V,V) {
        let (p,d,dd) = match self.smooth.get(i) { Some(Some(s)) => s.at(f),_ => tr.between(i,f) };
        let w = 1./self.span(i);
        (p,scale(d,w),scale(dd,w*w))
    }
    /// The guess's rate in the curve's parameter over the fractions `f` of stretch `i`, enclosed:
    /// the stretch's quintic in intervals, or the parabola's rate, linear in `f` (its value at the
    /// two ends of `f` hulled), or the chord.
    pub(crate) fn rate_over(&self,tr: &Traced,i: usize,f: [f64;2]) -> I3 {
        let r: I3 = match self.smooth.get(i) {
            Some(Some(s)) => s.rate_over(f),
            _ => {
                let (a,b) = (tr.between(i,f[0]).1,tr.between(i,f[1]).1);
                std::array::from_fn(|k| Iv::new(a[k],b[k]))
            }
        };
        let w = Iv::point(1.)/Iv::point(self.span(i));
        r.map(|x| x*w)
    }
}
