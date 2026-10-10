//! B-spline curves and tensor surfaces: a degree, a clamped knot vector and poles, evaluated with
//! their first two derivatives by the Cox–de Boor recurrence (Piegl and Tiller's A2.2 and A2.3).
//! A rational one carries a positive weight per pole and is the quotient `Σ N w P / Σ N w`, its
//! derivatives by the quotient rule (A4.2, A4.4): what a conic is exactly, and what other kernels
//! write circles, fillets and blends as. Without weights nothing here divides, so a polynomial
//! spline reads the same bits it always did. Positive weights keep a span within its poles' hull,
//! so every bound read off the poles holds for both.
use super::geom::{Frame,V};

/// The greatest degree evaluated (on the stack): OCCT's own bound.
pub const MAX_DEGREE: usize = 25;
const W: usize = MAX_DEGREE+1;
/// The degrees evaluated with a small table, the cubics and their like (most of what is read).
const SMALL: usize = 10;

/// A curve: its fields are its own, so that its weights go wherever its poles go. It is made weighed
/// or not by name (`polynomial`, `rational`), moved by `mapped` and compared by `same_as`, and no
/// code outside rebuilds one from its parts — which is how a rational curve once came back as the
/// polynomial through its poles.
#[derive(Clone,Debug,PartialEq)]
pub struct BSpline { degree: usize,knots: Vec<f64>,poles: Vec<V>,weights: Option<Vec<f64>> }

/// Whether `weights` may weigh `count` poles: one each, every one finite and positive (a weight at
/// or below zero lets the curve leave its poles' hull, or pass through infinity).
fn weighs(weights: &[f64],count: usize) -> bool { weights.len() == count && weights.iter().all(|&w| w.is_finite() && w > 0.) }

/// `C`, `C'`, `C''` from the weighted sums `A⁽ᵏ⁾ = Σ N⁽ᵏ⁾ w P` and `W⁽ᵏ⁾ = Σ N⁽ᵏ⁾ w` (Piegl and
/// Tiller A4.2).
fn quotient(a: [V;3],w: [f64;3]) -> (V,V,V) {
    let c0: V = std::array::from_fn(|c| a[0][c]/w[0]);
    let c1: V = std::array::from_fn(|c| (a[1][c]-w[1]*c0[c])/w[0]);
    let c2: V = std::array::from_fn(|c| (a[2][c]-2.*w[1]*c1[c]-w[2]*c0[c])/w[0]);
    (c0,c1,c2)
}

impl BSpline {
    /// A polynomial curve from its parts, refused unless its knots are non-decreasing and one more
    /// than its poles and degree.
    pub fn polynomial(degree: usize,knots: Vec<f64>,poles: Vec<V>) -> Result<BSpline,String> {
        if degree == 0 || degree > MAX_DEGREE || poles.len() <= degree || knots.len() != poles.len()+degree+1
            || knots.windows(2).any(|w| !(w[1] >= w[0])) || !(knots[degree] < knots[poles.len()]) {
            return Err(format!("a B-spline needs a degree from 1 to {MAX_DEGREE}, more poles than its degree and non-decreasing knots, one more than both"))
        }
        Ok(BSpline {degree,knots,poles,weights:None})
    }
    /// A rational curve from its parts, refused as `polynomial` refuses and unless each pole has a
    /// finite positive weight.
    pub fn rational(degree: usize,knots: Vec<f64>,poles: Vec<V>,weights: Vec<f64>) -> Result<BSpline,String> {
        let s = BSpline::polynomial(degree,knots,poles)?;
        if !weighs(&weights,s.poles.len()) { return Err("a rational B-spline needs a finite positive weight for each pole".into()) }
        Ok(BSpline {weights:Some(weights),..s})
    }
    pub fn degree(&self) -> usize { self.degree }
    pub fn knots(&self) -> &[f64] { &self.knots }
    pub fn poles(&self) -> &[V] { &self.poles }
    /// Each pole's weight, or none for a polynomial curve.
    pub fn weights(&self) -> Option<&[f64]> { self.weights.as_deref() }
    /// The same curve with every pole moved by `f`, its knots and weights kept: exactly the curve
    /// moved by `f` where `f` is affine (a rigid motion, a step along a direction, a projection
    /// along one onto a plane), which is all it may be — a weighed sum is moved by an affine map
    /// as its poles are, and by no other.
    pub fn mapped(&self,f: impl Fn(V) -> V) -> BSpline {
        BSpline {poles:self.poles.iter().map(|&p| f(p)).collect(),..self.clone()}
    }
    /// Whether `other` is this curve once `seen` takes a motion out of both: the same degree, knots
    /// and weights, and poles within `tol` of one another.
    pub fn same_as(&self,other: &BSpline,tol: f64,seen: impl Fn(V) -> V) -> bool {
        // every field named, so that one added must be compared here
        let BSpline {degree,knots,poles,weights} = self;
        *degree == other.degree && *knots == other.knots && *weights == other.weights && poles.len() == other.poles.len()
            && poles.iter().zip(&other.poles).all(|(&p,&q)| crate::space::norm(crate::space::sub(seen(p),seen(q))) <= tol)
    }
    pub fn is_rational(&self) -> bool { self.weights.is_some() }
    /// Where the curve is defined.
    pub fn domain(&self) -> [f64;2] { [self.knots[self.degree],self.knots[self.poles.len()]] }
    /// The distinct knots strictly inside `[a, b]`, where the curve's third derivative may jump.
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        let mut out: Vec<f64> = self.knots.iter().copied().filter(|&k| k > a && k < b).collect();
        out.dedup();
        out
    }
    fn span(&self,t: f64) -> usize { span(self.degree,&self.knots,self.poles.len(),t) }
    /// `C`, `C'`, `C''` at `t` (clamped into the domain).
    pub fn d2(&self,t: f64) -> (V,V,V) {
        let p = self.degree;
        let [a,b] = self.domain();
        let (s,ders) = basis_ders(p,&self.knots,self.poles.len(),t.clamp(a,b),2);
        if let Some(w) = &self.weights {
            let (mut out,mut ws) = ([[0.;3];3],[0.;3]);
            for (k,row) in ders.iter().enumerate().take(2.min(p)+1) {
                for (j,&n) in row.iter().enumerate().take(p+1) {
                    let (q,nw) = (self.poles[s-p+j],n*w[s-p+j]);
                    for c in 0..3 { out[k][c] += nw*q[c]; }
                    ws[k] += nw;
                }
            }
            return quotient(out,ws)
        }
        let mut out = [[0.;3];3];
        for (k,row) in ders.iter().enumerate().take(2.min(p)+1) {
            for (j,&n) in row.iter().enumerate().take(p+1) {
                let q = self.poles[s-p+j];
                for c in 0..3 { out[k][c] += n*q[c]; }
            }
        }
        (out[0],out[1],out[2])
    }
    pub fn point(&self,t: f64) -> V { self.d2(t).0 }
    /// The length of the control polygon, a bound on the curve's (a rational one's too: its de
    /// Casteljau steps cut corners).
    pub fn hull_length(&self) -> f64 { self.poles.windows(2).map(|w| crate::space::distance(w[0],w[1])).sum() }
    /// The poles whose basis reaches `[a, b]` (clamped into the domain): their hull holds the curve
    /// there.
    pub fn poles_over(&self,[a,b]: [f64;2]) -> &[V] {
        &self.poles[self.span(a.min(b))-self.degree..=self.span(a.max(b))]
    }
}

/// The knot span holding `t`, for `count` poles of `degree`.
fn span(degree: usize,knots: &[f64],count: usize,t: f64) -> usize {
    let (p,n) = (degree,count);
    if t >= knots[n] { return n-1 }
    if t <= knots[p] { return p }
    let (mut lo,mut hi) = (p,n);
    while hi-lo > 1 { let mid = (lo+hi)/2; if t < knots[mid] { hi = mid } else { lo = mid } }
    lo
}

/// The span holding `t`, and there the `degree + 1` basis functions and their derivatives up to
/// order `n` (at most 2) — Piegl and Tiller's A2.3, on the stack: a degree under `SMALL` in a table
/// that size, a higher one (OCCT approximates up to 25) in one of `W`.
fn basis_ders(p: usize,u: &[f64],count: usize,t: f64,n: usize) -> (usize,[[f64;W];3]) {
    if p < SMALL { basis_ders_in::<SMALL>(p,u,count,t,n) } else { basis_ders_in::<W>(p,u,count,t,n) }
}

fn basis_ders_in<const M: usize>(p: usize,u: &[f64],count: usize,t: f64,n: usize) -> (usize,[[f64;W];3]) {
    let s = span(p,u,count,t);
    let mut ndu = [[0.;M];M];
    let (mut left,mut right) = ([0.;M],[0.;M]);
    ndu[0][0] = 1.;
    for j in 1..=p {
        left[j] = t-u[s+1-j];
        right[j] = u[s+j]-t;
        let mut saved = 0.;
        for r in 0..j {
            ndu[j][r] = right[r+1]+left[j-r];
            let temp = ndu[r][j-1]/ndu[j][r];
            ndu[r][j] = saved+right[r+1]*temp;
            saved = left[j-r]*temp;
        }
        ndu[j][j] = saved;
    }
    let orders = n.min(p).min(2);
    let mut ders = [[0.;W];3];
    for j in 0..=p { ders[0][j] = ndu[j][p]; }
    let mut a2 = [[0.;M];2];
    for r in 0..=p {
        let (mut s1,mut s2) = (0usize,1usize);
        a2[0][0] = 1.;
        for k in 1..=orders {
            let mut d = 0.;
            let rk = r as isize-k as isize;
            let pk = p-k;
            if r >= k {
                a2[s2][0] = a2[s1][0]/ndu[pk+1][rk as usize];
                d = a2[s2][0]*ndu[rk as usize][pk];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if (r as isize-1) <= pk as isize { k-1 } else { p-r };
            for j in j1..=j2 {
                let idx = (rk+j as isize) as usize;
                a2[s2][j] = (a2[s1][j]-a2[s1][j-1])/ndu[pk+1][idx];
                d += a2[s2][j]*ndu[idx][pk];
            }
            if r <= pk {
                a2[s2][k] = -a2[s1][k-1]/ndu[pk+1][r];
                d += a2[s2][k]*ndu[r][pk];
            }
            ders[k][r] = d;
            std::mem::swap(&mut s1,&mut s2);
        }
    }
    let mut factor = p as f64;
    for k in 1..=orders {
        for j in 0..=p { ders[k][j] *= factor; }
        factor *= (p-k) as f64;
    }
    (s,ders)
}

/// Averaged knots for interpolation at `t` (Piegl and Tiller 9.8): clamped, each interior knot the
/// mean of the `degree` parameters it spans.
fn averaged(t: &[f64],degree: usize) -> Vec<f64> {
    let m = t.len();
    let mut u = vec![t[0];degree+1];
    for j in 1..m-degree { u.push(t[j..j+degree].iter().sum::<f64>()/degree as f64); }
    u.extend(std::iter::repeat_n(t[m-1],degree+1));
    u
}

/// The B-spline of `degree` (lowered to fit the count) through `pts` at the increasing parameters
/// `t` (global interpolation, one collocation solve per coordinate).
pub fn interpolate(pts: &[V],t: &[f64],degree: usize) -> Option<BSpline> {
    let m = pts.len();
    let degree = degree.min(m.checked_sub(1)?);
    if degree == 0 || t.len() != m || t.windows(2).any(|w| !(w[1] > w[0])) { return None }
    let knots = averaged(t,degree);
    let s = BSpline {degree,knots,poles:vec![[0.;3];m],weights:None};
    if m > BANDED { if let Some(poles) = banded(&s,pts,t) { return Some(BSpline {poles,..s}) } }
    // the collocation matrix: each parameter's basis functions, in the columns of its span
    let mut n = vec![0.;m*m];
    for (k,&tk) in t.iter().enumerate() {
        let span = s.span(tk.clamp(t[0],t[m-1]));
        let basis = s.basis(tk,span);
        for (j,&b) in basis.iter().enumerate().take(degree+1) { n[k*m+span-degree+j] = b; }
    }
    let mut piv = Vec::with_capacity(m);
    if !crate::linalg::lu_factor(m,&mut n,&mut piv) { return None }
    let mut poles = vec![[0.;3];m];
    for c in 0..3 {
        let mut x: Vec<f64> = pts.iter().map(|p| p[c]).collect();
        crate::linalg::lu_apply(m,&n,&piv,&mut x);
        for k in 0..m { poles[k][c] = x[k]; }
    }
    poles.iter().flatten().all(|v| v.is_finite()).then(|| BSpline {poles,..s})
}

/// The arc of radius `r` about `f.o` in its `xy` plane from angle `a0` to `a1` (counter-clockwise
/// about `f.z`, `a0 < a1`, at most a whole turn), exactly: quadratic and rational, one span per quarter turn or
/// less, each span's middle pole where its end tangents meet, weighted the cosine of half its turn
/// (Piegl and Tiller A7.1). Its parameter runs over `[a0, a1]`, meeting the angle at every knot.
pub fn arc(f: &Frame,r: f64,[a0,a1]: [f64;2]) -> BSpline {
    use crate::fmath::Det;
    let turn = (a1-a0).clamp(0.,std::f64::consts::TAU);
    let n = ((turn/std::f64::consts::FRAC_PI_2-1e-12).ceil() as usize).max(1);
    let d = turn/n as f64;
    let w = (d/2.).dcos();
    let at = |a: f64,rr: f64| -> V { let (s,c) = a.dsin_cos(); std::array::from_fn(|k| f.o[k]+rr*(c*f.x[k]+s*f.y[k])) };
    let (mut poles,mut weights,mut knots) = (vec![at(a0,r)],vec![1.],vec![a0;3]);
    for k in 0..n {
        let end = if k+1 == n { a0+turn } else { a0+(k+1) as f64*d };
        poles.push(at(a0+(k as f64+0.5)*d,r/w));
        poles.push(at(end,r));
        weights.extend([w,1.]);
        knots.extend(std::iter::repeat_n(end,if k+1 == n { 3 } else { 2 }));
    }
    BSpline {degree:2,knots,poles,weights:Some(weights)}
}

/// Above this many points an interpolation is solved in its band: the dense factorisation below is
/// cubic in them, and a trace across a whole blank refined to thousands of points took minutes a
/// round. Below it the dense solve stands, so every curve and sheet fitted before is fitted alike.
const BANDED: usize = 200;

/// The poles interpolating `pts` at `t` in the knots of `s`, by Gaussian elimination in the
/// collocation matrix's band (a row's basis functions span `degree + 1` columns about its own,
/// with averaged knots) without pivoting — stable, the matrix being totally positive (de Boor) —
/// or nothing where a row falls outside the band or a pivot vanishes.
fn banded(s: &BSpline,pts: &[V],t: &[f64]) -> Option<Vec<V>> {
    let (m,p) = (pts.len(),s.degree);
    let w = 2*p+1;
    // a[k][j-k+p]: row k's entry in column j
    let mut a = vec![0.;m*w];
    for (k,&tk) in t.iter().enumerate() {
        let span = s.span(tk.clamp(t[0],t[m-1]));
        let basis = s.basis(tk,span);
        for (j,&b) in basis.iter().enumerate().take(p+1) {
            let col = span-p+j;
            if col+p < k || col > k+p { return None }
            a[k*w+col+p-k] = b;
        }
    }
    let mut x: Vec<V> = pts.to_vec();
    for i in 0..m {
        let pivot = a[i*w+p];
        if !(pivot.abs() > 1e-300) { return None }
        for r in i+1..m.min(i+p+1) {
            let f = a[r*w+i+p-r]/pivot;
            if f == 0. { continue }
            for c in i..m.min(i+p+1) { a[r*w+c+p-r] -= f*a[i*w+c+p-i]; }
            for d in 0..3 { x[r][d] -= f*x[i][d]; }
        }
    }
    for i in (0..m).rev() {
        for c in i+1..m.min(i+p+1) { let v = a[i*w+c+p-i]; for d in 0..3 { x[i][d] -= v*x[c][d]; } }
        for d in 0..3 { x[i][d] /= a[i*w+p]; }
    }
    x.iter().flatten().all(|v| v.is_finite()).then_some(x)
}

impl BSpline {
    /// The `degree + 1` basis functions not zero at `t`, in span `span` (Piegl and Tiller A2.2).
    fn basis(&self,t: f64,span: usize) -> [f64;W] {
        let p = self.degree;
        let u = &self.knots;
        let (mut n,mut left,mut right) = ([0.;W],[0.;W],[0.;W]);
        n[0] = 1.;
        for j in 1..=p {
            left[j] = t-u[span+1-j];
            right[j] = u[span+j]-t;
            let mut saved = 0.;
            for r in 0..j {
                let temp = n[r]/(right[r+1]+left[j-r]);
                n[r] = saved+right[r+1]*temp;
                saved = left[j-r]*temp;
            }
            n[j] = saved;
        }
        n
    }
}

pub fn distinct(knots: &[f64]) -> Vec<(f64,usize)> {
    let mut out: Vec<(f64,usize)> = Vec::new();
    for &k in knots { match out.last_mut() { Some((x,n)) if *x == k => *n += 1,_ => out.push((k,1)) } }
    out
}

/// Knot `t` put into a B-spline of degree `p` once (Boehm): the same curve, one more pole. Each pole
/// is a row of points (a net's column along the other parameter), combined alike; a rational one's
/// in homogeneous coordinates `(w x, w y, w z, w)`.
fn insert<const N: usize>(knots: &mut Vec<f64>,poles: &mut Vec<Vec<[f64;N]>>,p: usize,t: f64) {
    let n = poles.len();
    // the last span starting at or before t
    let k = knots[..n].iter().rposition(|&x| x <= t).unwrap_or(p).max(p);
    let mut out: Vec<Vec<[f64;N]>> = Vec::with_capacity(n+1);
    for i in 0..=n {
        if i+p <= k { out.push(poles[i].clone()) }
        else if i > k { out.push(poles[i-1].clone()) }
        else {
            let a = (t-knots[i])/(knots[i+p]-knots[i]);
            out.push(poles[i-1].iter().zip(&poles[i]).map(|(x,y)| std::array::from_fn(|c| (1.-a)*x[c]+a*y[c])).collect());
        }
    }
    knots.insert(k+1,t);
    *poles = out;
}

/// The stretch `[a, b]` of a B-spline of degree `p` along its first index, exactly: each end put in
/// until the curve passes through a pole there, and the poles and knots between kept, clamped.
fn segment<const N: usize>(knots: &[f64],poles: &[Vec<[f64;N]>],p: usize,a: f64,b: f64) -> (Vec<f64>,Vec<Vec<[f64;N]>>) {
    let (mut k,mut q) = (knots.to_vec(),poles.to_vec());
    for t in [a,b] {
        while k.iter().filter(|&&x| x == t).count() < p { insert(&mut k,&mut q,p,t); }
    }
    // the first pole: the last copy of `a` less the degree
    let first = k.iter().rposition(|&x| x == a).unwrap()-p;
    let knots: Vec<f64> = std::iter::repeat_n(a,p+1).chain(k.iter().copied().filter(|&x| x > a && x < b)).chain(std::iter::repeat_n(b,p+1)).collect();
    let count = knots.len()-p-1;
    (knots,q[first..first+count].to_vec())
}

/// A tensor-product B-spline surface: `poles[i][j]` the pole at `u` index `i` and `v` index `j`, and
/// for a rational one `weights[i][j]` its weight. Its fields are its own, as a curve's are: made
/// weighed or not by name, moved by `mapped`, its boundary curves handed out weighed (`row`,
/// `column`).
#[derive(Clone,Debug,PartialEq)]
pub struct Net { du: usize,dv: usize,uknots: Vec<f64>,vknots: Vec<f64>,poles: Vec<Vec<V>>,weights: Option<Vec<Vec<f64>>> }

impl Net {
    /// A polynomial surface from its parts, refused unless its degrees are from 1 to `MAX_DEGREE`,
    /// its net is a grid with more poles each way than its degree, and its knots are non-decreasing
    /// and one more than both.
    pub fn polynomial(du: usize,dv: usize,uknots: Vec<f64>,vknots: Vec<f64>,poles: Vec<Vec<V>>) -> Result<Net,String> {
        let n = Net {du,dv,uknots,vknots,poles,weights:None};
        n.check()?;
        Ok(n)
    }
    /// A rational surface from its parts, refused as `polynomial` refuses and unless its weights are
    /// a grid alike, each finite and positive.
    pub fn rational(du: usize,dv: usize,uknots: Vec<f64>,vknots: Vec<f64>,poles: Vec<Vec<V>>,weights: Vec<Vec<f64>>) -> Result<Net,String> {
        let n = Net {du,dv,uknots,vknots,poles,weights:Some(weights)};
        n.check()?;
        Ok(n)
    }
    fn check(&self) -> Result<(),String> {
        let (nu,nv) = (self.poles.len(),self.poles.first().map_or(0,Vec::len));
        let knots = |k: &[f64],d: usize,n: usize| k.len() == n+d+1 && k.windows(2).all(|w| w[1] >= w[0]) && k[d] < k[n];
        if self.du == 0 || self.dv == 0 || self.du > MAX_DEGREE || self.dv > MAX_DEGREE || nu <= self.du || nv <= self.dv
            || self.poles.iter().any(|r| r.len() != nv) || !knots(&self.uknots,self.du,nu) || !knots(&self.vknots,self.dv,nv) {
            return Err("a B-spline surface whose net and knots do not agree".into())
        }
        if let Some(w) = &self.weights {
            if w.len() != nu || w.iter().any(|r| !weighs(r,nv)) { return Err("a rational B-spline surface needs a finite positive weight for each pole".into()) }
        }
        Ok(())
    }
    pub fn du(&self) -> usize { self.du }
    pub fn dv(&self) -> usize { self.dv }
    pub fn uknots(&self) -> &[f64] { &self.uknots }
    pub fn vknots(&self) -> &[f64] { &self.vknots }
    pub fn poles(&self) -> &[Vec<V>] { &self.poles }
    /// Each pole's weight, or none for a polynomial surface.
    pub fn weights(&self) -> Option<&[Vec<f64>]> { self.weights.as_deref() }
    /// The same surface with every pole moved by `f`, knots and weights kept (`BSpline::mapped`:
    /// exact for an affine `f`, and meant for no other).
    pub fn mapped(&self,f: impl Fn(V) -> V) -> Net {
        Net {poles:self.poles.iter().map(|row| row.iter().map(|&p| f(p)).collect()).collect(),..self.clone()}
    }
    /// The curve along `v` of `u` index `i` — at the domain's ends, the surface's edge there —
    /// weighed as the net weighs it.
    pub fn row(&self,i: usize) -> BSpline {
        BSpline {degree:self.dv,knots:self.vknots.clone(),poles:self.poles[i].clone(),weights:self.weights.as_ref().map(|w| w[i].clone())}
    }
    /// The curve along `u` of `v` index `j`, weighed as the net weighs it.
    pub fn column(&self,j: usize) -> BSpline {
        BSpline {degree:self.du,knots:self.uknots.clone(),poles:self.poles.iter().map(|r| r[j]).collect(),
            weights:self.weights.as_ref().map(|w| w.iter().map(|r| r[j]).collect())}
    }
    pub fn is_rational(&self) -> bool { self.weights.is_some() }
    /// Where the surface is defined, in `u` and in `v`.
    pub fn domain(&self) -> [[f64;2];2] {
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        [[self.uknots[self.du],self.uknots[nu]],[self.vknots[self.dv],self.vknots[nv]]]
    }
    /// `S`, `S_u`, `S_v` at `(u, v)`, clamped into the domain.
    pub fn d1(&self,u: f64,v: f64) -> (V,V,V) {
        let [[u0,u1],[v0,v1]] = self.domain();
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        let (su,bu) = basis_ders(self.du,&self.uknots,nu,u.clamp(u0,u1),1);
        let (sv,bv) = basis_ders(self.dv,&self.vknots,nv,v.clamp(v0,v1),1);
        if let Some(w) = &self.weights {
            let (mut a,mut ws) = ([[0.;3];3],[0.;3]);
            for i in 0..=self.du {
                let (row,wrow) = (&self.poles[su-self.du+i],&w[su-self.du+i]);
                for j in 0..=self.dv {
                    let (q,wq) = (row[sv-self.dv+j],wrow[sv-self.dv+j]);
                    let n = [bu[0][i]*bv[0][j]*wq,bu[1][i]*bv[0][j]*wq,bu[0][i]*bv[1][j]*wq];
                    for k in 0..3 { for c in 0..3 { a[k][c] += n[k]*q[c]; } ws[k] += n[k]; }
                }
            }
            let s: V = std::array::from_fn(|c| a[0][c]/ws[0]);
            return (s,std::array::from_fn(|c| (a[1][c]-ws[1]*s[c])/ws[0]),std::array::from_fn(|c| (a[2][c]-ws[2]*s[c])/ws[0]))
        }
        let mut out = [[0.;3];3];
        for i in 0..=self.du {
            let row = &self.poles[su-self.du+i];
            for j in 0..=self.dv {
                let q = row[sv-self.dv+j];
                let (n,nu_,nv_) = (bu[0][i]*bv[0][j],bu[1][i]*bv[0][j],bu[0][i]*bv[1][j]);
                for c in 0..3 { out[0][c] += n*q[c]; out[1][c] += nu_*q[c]; out[2][c] += nv_*q[c]; }
            }
        }
        (out[0],out[1],out[2])
    }
    pub fn point(&self,u: f64,v: f64) -> V { self.d1(u,v).0 }
    /// `S`, `S_u`, `S_v`, `S_uu`, `S_uv`, `S_vv` at `(u, v)`, clamped into the domain.
    pub fn d2(&self,u: f64,v: f64) -> [V;6] {
        let [[u0,u1],[v0,v1]] = self.domain();
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        let (su,bu) = basis_ders(self.du,&self.uknots,nu,u.clamp(u0,u1),2);
        let (sv,bv) = basis_ders(self.dv,&self.vknots,nv,v.clamp(v0,v1),2);
        // (u order, v order) of each output
        const ORDERS: [(usize,usize);6] = [(0,0),(1,0),(0,1),(2,0),(1,1),(0,2)];
        if let Some(w) = &self.weights {
            let (mut a,mut ws) = ([[0.;3];6],[0.;6]);
            for i in 0..=self.du {
                let (row,wrow) = (&self.poles[su-self.du+i],&w[su-self.du+i]);
                for j in 0..=self.dv {
                    let (q,wq) = (row[sv-self.dv+j],wrow[sv-self.dv+j]);
                    for (k,&(x,y)) in ORDERS.iter().enumerate() {
                        let n = bu[x][i]*bv[y][j]*wq;
                        for c in 0..3 { a[k][c] += n*q[c]; }
                        ws[k] += n;
                    }
                }
            }
            // the quotient rule, order by order (Piegl and Tiller A4.4)
            let at = |f: &dyn Fn(usize) -> f64| -> V { std::array::from_fn(f) };
            let s = at(&|c| a[0][c]/ws[0]);
            let su_ = at(&|c| (a[1][c]-ws[1]*s[c])/ws[0]);
            let sv_ = at(&|c| (a[2][c]-ws[2]*s[c])/ws[0]);
            let suu = at(&|c| (a[3][c]-2.*ws[1]*su_[c]-ws[3]*s[c])/ws[0]);
            let suv = at(&|c| (a[4][c]-ws[1]*sv_[c]-ws[2]*su_[c]-ws[4]*s[c])/ws[0]);
            let svv = at(&|c| (a[5][c]-2.*ws[2]*sv_[c]-ws[5]*s[c])/ws[0]);
            return [s,su_,sv_,suu,suv,svv]
        }
        let mut out = [[0.;3];6];
        for i in 0..=self.du {
            let row = &self.poles[su-self.du+i];
            for j in 0..=self.dv {
                let q = row[sv-self.dv+j];
                for (k,&(a,b)) in ORDERS.iter().enumerate() {
                    let n = bu[a][i]*bv[b][j];
                    for c in 0..3 { out[k][c] += n*q[c]; }
                }
            }
        }
        out
    }
    /// The surface over `[u0, u1] × [v0, v1]` (within its domain) alone, exactly: the same points at
    /// the same parameters, only the net that reaches there.
    pub fn segment(&self,[u0,u1]: [f64;2],[v0,v1]: [f64;2]) -> Net {
        let [[a0,a1],[b0,b1]] = self.domain();
        let (u0,u1,v0,v1) = (u0.max(a0),u1.min(a1),v0.max(b0),v1.min(b1));
        if let Some(w) = &self.weights {
            // in homogeneous coordinates, and back
            let h: Vec<Vec<[f64;4]>> = self.poles.iter().zip(w).map(|(r,wr)| r.iter().zip(wr).map(|(p,&w)| [w*p[0],w*p[1],w*p[2],w]).collect()).collect();
            let (uknots,rows) = segment(&self.uknots,&h,self.du,u0,u1);
            let cols: Vec<Vec<[f64;4]>> = (0..rows[0].len()).map(|j| rows.iter().map(|r| r[j]).collect()).collect();
            let (vknots,cols) = segment(&self.vknots,&cols,self.dv,v0,v1);
            let h: Vec<Vec<[f64;4]>> = (0..cols[0].len()).map(|i| cols.iter().map(|c| c[i]).collect()).collect();
            let poles = h.iter().map(|r| r.iter().map(|q| [q[0]/q[3],q[1]/q[3],q[2]/q[3]]).collect()).collect();
            let weights = h.iter().map(|r| r.iter().map(|q| q[3]).collect()).collect();
            return Net {du:self.du,dv:self.dv,uknots,vknots,poles,weights:Some(weights)}
        }
        let (uknots,rows) = segment(&self.uknots,&self.poles,self.du,u0,u1);
        // along v: each row's points as poles of their own
        let cols: Vec<Vec<V>> = (0..rows[0].len()).map(|j| rows.iter().map(|r| r[j]).collect()).collect();
        let (vknots,cols) = segment(&self.vknots,&cols,self.dv,v0,v1);
        let poles = (0..cols[0].len()).map(|i| cols.iter().map(|c| c[i]).collect()).collect();
        Net {du:self.du,dv:self.dv,uknots,vknots,poles,weights:None}
    }
    /// The poles whose basis reaches `[u0, u1] × [v0, v1]` (clamped into the domain): their hull
    /// holds the surface there.
    pub fn poles_over(&self,[u0,u1]: [f64;2],[v0,v1]: [f64;2]) -> impl Iterator<Item=V> + '_ {
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        let (i0,i1) = (span(self.du,&self.uknots,nu,u0.min(u1))-self.du,span(self.du,&self.uknots,nu,u0.max(u1)));
        let (j0,j1) = (span(self.dv,&self.vknots,nv,v0.min(v1))-self.dv,span(self.dv,&self.vknots,nv,v0.max(v1)));
        self.poles[i0..=i1].iter().flat_map(move |row| row[j0..=j1].iter().copied())
    }
    /// The distinct knots strictly inside `[a, b]` of parameter `k` (0 for u, 1 for v).
    pub fn breaks(&self,k: usize,[a,b]: [f64;2]) -> Vec<f64> {
        let (lo,hi) = (a.min(b),a.max(b));
        let mut out: Vec<f64> = (if k == 0 { &self.uknots } else { &self.vknots }).iter().copied().filter(|&x| x > lo && x < hi).collect();
        out.dedup();
        out
    }
}

/// The surface of degree `du` × `dv` through `f` on a uniform `nu` × `nv` grid of `[0, 1]²`,
/// the samples doubled in whichever direction a sample halfway between two grid points strays
/// from it by more than `tol`, at most 1024 × 512 — the fit and the error it measured.
pub fn fit_net(f: &dyn Fn(f64,f64) -> V,du: usize,dv: usize,mut nu: usize,mut nv: usize,tol: f64) -> Option<(Net,f64)> {
    loop {
        let tu: Vec<f64> = (0..=nu).map(|i| i as f64/nu as f64).collect();
        let tv: Vec<f64> = (0..=nv).map(|j| j as f64/nv as f64).collect();
        // each row along u, then each column of their poles along v
        let rows: Vec<BSpline> = tv.iter().map(|&v| interpolate(&tu.iter().map(|&u| f(u,v)).collect::<Vec<_>>(),&tu,du))
            .collect::<Option<_>>()?;
        let cols: Vec<BSpline> = (0..=nu).map(|i| interpolate(&rows.iter().map(|r| r.poles[i]).collect::<Vec<_>>(),&tv,dv))
            .collect::<Option<_>>()?;
        let net = Net {du:rows[0].degree,dv:cols[0].degree,uknots:rows[0].knots.clone(),vknots:cols[0].knots.clone(),
            poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
        let off = |u: f64,v: f64| crate::space::distance(net.point(u,v),f(u,v));
        let (mut eu,mut ev) = (0f64,0f64);
        for i in 0..nu { for &v in &tv { eu = eu.max(off((tu[i]+tu[i+1])/2.,v)); } }
        for j in 0..nv { for &u in &tu { ev = ev.max(off(u,(tv[j]+tv[j+1])/2.)); } }
        let err = eu.max(ev);
        let (grow_u,grow_v) = (eu > tol && nu < 1024,ev > tol && nv < 512);
        if !grow_u && !grow_v { return Some((net,err)) }
        if grow_u { nu *= 2; }
        if grow_v { nv *= 2; }
    }
}

/// The curve of degree `d` through `f` on a uniform grid of `[0, 1]`, refined as `fit_net` is.
pub fn fit_curve(f: &dyn Fn(f64) -> V,d: usize,mut n: usize,tol: f64) -> Option<(BSpline,f64)> {
    loop {
        let t: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let s = interpolate(&t.iter().map(|&t| f(t)).collect::<Vec<_>>(),&t,d)?;
        let err = (0..n).map(|i| { let m = (t[i]+t[i+1])/2.; crate::space::distance(s.point(m),f(m)) }).fold(0.,f64::max);
        if err <= tol || n >= 4096 { return Some((s,err)) }
        n *= 2;
    }
}

/// How a grid's parameters are spread: evenly, by chord length, or by its square root.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Parametrization { Even,ChordLength,Centripetal }

/// The cubic surface (lower where a direction has fewer than four points) through a row-major
/// grid of `rows × columns` points, rows along `u`: each direction's parameters from 0 to 1, spread
/// by `kind` along each column (or row) and averaged over them, so every row is interpolated at
/// one set of parameters; the rows interpolated along `v`, then their poles along `u`.
pub fn interpolate_net(points: &[V],rows: usize,columns: usize,kind: Parametrization) -> Option<Net> {
    if rows < 2 || columns < 2 || points.len() != rows*columns { return None }
    let at = |i: usize,j: usize| points[i*columns+j];
    let spread = |n: usize,lines: usize,gap: &dyn Fn(usize,usize) -> f64| -> Option<Vec<f64>> {
        let mut t = vec![0.;n];
        for line in 0..lines {
            let steps: Vec<f64> = (1..n).map(|k| match kind {
                Parametrization::Even => 1.,
                Parametrization::ChordLength => gap(line,k),
                Parametrization::Centripetal => gap(line,k).sqrt(),
            }).collect();
            let total: f64 = steps.iter().sum();
            if !(total > 0.) { return None }
            let mut s = 0.;
            for k in 1..n { s += steps[k-1]; t[k] += s/total/lines as f64; }
        }
        t[n-1] = 1.;
        t.windows(2).all(|w| w[1] > w[0]).then_some(t)
    };
    let tu = spread(rows,columns,&|j,i| crate::space::distance(at(i-1,j),at(i,j)))?;
    let tv = spread(columns,rows,&|i,j| crate::space::distance(at(i,j-1),at(i,j)))?;
    let rows_fit: Vec<BSpline> = (0..rows).map(|i| interpolate(&(0..columns).map(|j| at(i,j)).collect::<Vec<_>>(),&tv,3)).collect::<Option<_>>()?;
    let nv = rows_fit[0].poles.len();
    let cols_fit: Vec<BSpline> = (0..nv).map(|j| interpolate(&rows_fit.iter().map(|r| r.poles[j]).collect::<Vec<_>>(),&tu,3)).collect::<Option<_>>()?;
    let poles: Vec<Vec<V>> = (0..cols_fit[0].poles.len()).map(|i| cols_fit.iter().map(|c| c.poles[i]).collect()).collect();
    Some(Net {du:cols_fit[0].degree,dv:rows_fit[0].degree,uknots:cols_fit[0].knots.clone(),vknots:rows_fit[0].knots.clone(),poles,weights:None})
}
