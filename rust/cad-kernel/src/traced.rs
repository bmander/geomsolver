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
//! * **A pace with no jump** (`Smooth::pace`): the curve's parameter still counts its points, and
//!   `σ(f)`, the length along a stretch at the fraction `f` of it, is a quintic rising from 0 to
//!   the stretch's length, its rate at each point the harmonic mean of the two stretches' lengths
//!   there and its second rate nought: the same either side, so the speed and its rate run on
//!   through every point.
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
use crate::util::space::{add,cross,dot,norm,normalised,scale,sub};

/// The sine of the angle between the two surfaces below which their meeting's direction is not
/// read from them: a touch's, where `T` and `K` (which goes as one over its square) are lost.
pub(crate) const CLEAR: f64 = 1e-3;
/// How far a stretch read by its quintic may turn: each end's tangent within this cosine of its
/// chord (25°), and its curvature times its length at most one.
const ALONG_COS: f64 = 0.9;

/// **The meeting's unit tangent at `p`**, turned along `along`: `None` where the two surfaces
/// stand within `CLEAR` of tangent there, or a gradient vanishes.
pub(crate) fn tangent(a: &Surface,b: &Surface,p: V,along: V) -> Option<V> { normals(a,b,p,along).map(|(t,..)| t) }

/// `T` and the two unit normals at `p`.
fn normals(a: &Surface,b: &Surface,p: V,along: V) -> Option<(V,V,V)> {
    let (na,nb) = (normalised(a.side(p).1)?,normalised(b.side(p).1)?);
    let w = cross(na,nb);
    let s = norm(w);
    if !(s >= CLEAR) { return None }
    let t = scale(w,if dot(w,along) < 0. { -1./s } else { 1./s });
    Some((t,na,nb))
}

/// **The meeting's unit tangent and curvature vector at `p`** (the module's): `K` solves
/// `n_a · K = κ_a`, `n_b · K = κ_b`, `T · K = 0` by Cramer's rule.
pub(crate) fn jet(a: &Surface,b: &Surface,p: V,along: V) -> Option<(V,V)> {
    let (t,na,nb) = normals(a,b,p,along)?;
    let (ka,kb) = (a.bend(p,t,na)?,b.bend(p,t,nb)?);
    let det = dot(na,cross(nb,t));
    let k = scale(add(scale(cross(nb,t),ka),scale(cross(t,na),kb)),1./det);
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

/// A quintic's value and rate over the interval `x`, by Horner's rule in intervals.
fn quintic_over(c: &[f64;6],x: Iv) -> (Iv,Iv) {
    let k = |a: f64| Iv::point(a);
    let v = ((((k(c[5])*x+k(c[4]))*x+k(c[3]))*x+k(c[2]))*x+k(c[1]))*x+k(c[0]);
    let d = (((k(5.*c[5])*x+k(4.*c[4]))*x+k(3.*c[3]))*x+k(2.*c[2]))*x+k(c[1]);
    (v,d)
}

/// The quintic in `x ∈ [0, 1]` holding `ends` (value, first and second rate in `x`) at 0 and 1.
fn hermite(ends: [V;6]) -> [V;6] {
    std::array::from_fn(|j| std::array::from_fn(|k| (0..6).map(|i| ends[i][k]*BASIS[i][j]).sum()))
}

/// The scalar quintic rising from 0 to `len` over `[0, 1]`, its rates `v0` and `v1` at its ends
/// and its second rates nought.
fn rising(len: f64,v0: f64,v1: f64) -> [f64;6] {
    std::array::from_fn(|j| v0*BASIS[1][j]+v1*BASIS[4][j]+len*BASIS[5][j])
}

/// A stretch read by its quintic: its length along the curve, the quintic in `x = σ / len`, and
/// the pace, `σ` as a quintic of the fraction of the stretch.
#[derive(Clone,Debug)]
pub(crate) struct Smooth { pub(crate) len: f64,curve: [V;6],pace: [f64;6] }

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
        fair.then_some(Smooth {len,curve:c,pace:[0.,len,0.,0.,0.,0.]})
    }
    /// The guess at the fraction `f` of the stretch and its two rates in `f`.
    fn at(&self,f: f64) -> (V,V,V) {
        let (s,ds,dds) = quintic(&self.pace,f);
        let l = self.len;
        let (p,px,pxx) = quintic3(&self.curve,s/l);
        (p,scale(px,ds/l),add(scale(pxx,ds*ds/(l*l)),scale(px,dds/l)))
    }
    /// The guess's rate in the fraction `f` over the fractions `f`, enclosed: the quintics read in
    /// intervals.
    fn rate_over(&self,f: Iv) -> I3 {
        let (s,ds) = quintic_over(&self.pace,f);
        let l = Iv::point(self.len);
        let x = s/l;
        std::array::from_fn(|k| quintic_over(&self.curve.map(|v| v[k]),x).1*ds/l)
    }
    /// The pace's greatest rate over the stretch, sampled.
    fn fastest(&self) -> f64 { (0..=32).map(|j| quintic(&self.pace,j as f64/32.).1).fold(0.,f64::max) }
}

/// **What a trace reads once at its points** (the module's): each stretch's quintic where it has
/// one, and the curve's greatest speed.
/// Its points pulled onto both surfaces (`nodes`), and each whole stretch's enclosure, read once
/// when first asked (`enclose`).
#[derive(Debug,Default)]
pub(crate) struct Read {
    pub(crate) smooth: Vec<Option<Smooth>>,pub(crate) speed: f64,pub(crate) nodes: Vec<V>,
    pub(crate) wholes: std::sync::OnceLock<Vec<Patch<1>>>,
}

/// A trace's `Read`: its points pulled onto both surfaces, their jets read, each stretch's quintic
/// built where both its ends have one and it runs along its chord, and the paces set.
pub(crate) fn read(tr: &Traced) -> Read {
    let n = tr.pts.len();
    if n < 2 { return Read::default() }
    let m = n-1;
    let mut nodes: Vec<Option<(V,V,V)>> = Vec::with_capacity(n);
    let mut at: Vec<V> = Vec::with_capacity(n);
    for i in 0..=m {
        if tr.closed && i == m { let first = nodes[0]; nodes.push(first); at.push(at[0]); continue }
        let p = tr.project(tr.pts[i]);
        at.push(p);
        let prev = if i > 0 { tr.pts[i-1] } else if tr.closed && m > 1 { tr.pts[m-1] } else { tr.pts[i] };
        let next = if i < m { tr.pts[i+1] } else { tr.pts[i] };
        nodes.push(jet(&tr.a,&tr.b,p,sub(next,prev)).map(|(t,k)| (p,t,k)));
    }
    let mut smooth: Vec<Option<Smooth>> = (0..m).map(|i| {
        let ((p0,t0,k0),(p1,t1,k1)) = (nodes[i]?,nodes[i+1]?);
        let c = sub(p1,p0);
        let l = norm(c);
        let along = |t: V| dot(t,c) >= ALONG_COS*l;
        if !(l > 0.) || !along(t0) || !along(t1) || norm(k0).max(norm(k1))*l > 1. { return None }
        Smooth::new((p0,t0,k0),(p1,t1,k1))
    }).collect();
    // the paces: at each point the harmonic mean of the lengths of the read stretches either side
    let len = |i: Option<usize>| i.and_then(|i| smooth[i].as_ref().map(|s| s.len));
    let pace: Vec<f64> = (0..=m).map(|i| {
        let left = if i > 0 { Some(i-1) } else if tr.closed { Some(m-1) } else { None };
        let right = if i < m { Some(i) } else if tr.closed { Some(0) } else { None };
        match (len(left),len(right)) {
            (Some(a),Some(b)) => 2.*a*b/(a+b),
            (Some(a),None) | (None,Some(a)) => a,
            (None,None) => 0.,
        }
    }).collect();
    for (i,s) in smooth.iter_mut().enumerate() {
        let Some(s) = s else { continue };
        let p = rising(s.len,pace[i],pace[i+1]);
        // (a pace that would stand still within the stretch is read evenly over it)
        if (0..=32).all(|j| quintic(&p,j as f64/32.).1 > 0.05*s.len) { s.pace = p }
    }
    let speed = (0..m).map(|i| match &smooth[i] {
        Some(s) => s.fastest(),
        None => norm(sub(tr.pts[i+1],tr.pts[i])),
    }).fold(0.,f64::max);
    Read {smooth,speed,nodes:at,wholes:std::sync::OnceLock::new()}
}

impl Read {
    /// The guess at the fraction `f` of stretch `i` and its two rates in the curve's parameter: the
    /// stretch's quintic where it has one, the parabola through the chords where not.
    pub(crate) fn guess(&self,tr: &Traced,i: usize,f: f64) -> (V,V,V) {
        match self.smooth.get(i) { Some(Some(s)) => s.at(f),_ => tr.between(i,f) }
    }
    /// The guess's rate in the curve's parameter over the fractions `f` of stretch `i`, enclosed:
    /// the stretch's quintics in intervals, or the parabola's rate, linear in `f` (its value at the
    /// two ends of `f` hulled), or the chord.
    pub(crate) fn rate_over(&self,tr: &Traced,i: usize,f: [f64;2]) -> I3 {
        match self.smooth.get(i) {
            Some(Some(s)) => s.rate_over(Iv::new(f[0],f[1])),
            _ => {
                let (a,b) = (tr.between(i,f[0]).1,tr.between(i,f[1]).1);
                std::array::from_fn(|k| Iv::new(a[k],b[k]))
            }
        }
    }
}
