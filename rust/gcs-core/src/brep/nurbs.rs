//! B-spline curves, non-rational: a degree, a clamped knot vector and poles, evaluated with their
//! first two derivatives by the Cox–de Boor recurrence (Piegl and Tiller's A2.2 and A2.3).
use super::geom::V;

/// The greatest degree evaluated (on the stack).
pub const MAX_DEGREE: usize = 9;
const W: usize = MAX_DEGREE+1;

#[derive(Clone,Debug,PartialEq)]
pub struct BSpline { pub degree: usize,pub knots: Vec<f64>,pub poles: Vec<V> }

impl BSpline {
    /// A curve from its parts, refused unless its knots are non-decreasing and one more than its
    /// poles and degree.
    pub fn new(degree: usize,knots: Vec<f64>,poles: Vec<V>) -> Result<BSpline,String> {
        if degree == 0 || degree > MAX_DEGREE || poles.len() <= degree || knots.len() != poles.len()+degree+1
            || knots.windows(2).any(|w| !(w[1] >= w[0])) || !(knots[degree] < knots[poles.len()]) {
            return Err(format!("a B-spline needs a degree from 1 to {MAX_DEGREE}, more poles than its degree and non-decreasing knots, one more than both"))
        }
        Ok(BSpline {degree,knots,poles})
    }
    /// Where the curve is defined.
    pub fn domain(&self) -> [f64;2] { [self.knots[self.degree],self.knots[self.poles.len()]] }
    /// The distinct knots strictly inside `[a, b]`, where the curve's third derivative may jump.
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        let mut out: Vec<f64> = self.knots.iter().copied().filter(|&k| k > a && k < b).collect();
        out.dedup();
        out
    }
    fn span(&self,t: f64) -> usize {
        let (p,n) = (self.degree,self.poles.len());
        if t >= self.knots[n] { return n-1 }
        if t <= self.knots[p] { return p }
        let (mut lo,mut hi) = (p,n);
        while hi-lo > 1 { let mid = (lo+hi)/2; if t < self.knots[mid] { hi = mid } else { lo = mid } }
        lo
    }
    /// `C`, `C'`, `C''` at `t` (clamped into the domain).
    pub fn d2(&self,t: f64) -> (V,V,V) {
        let p = self.degree;
        let [a,b] = self.domain();
        let t = t.clamp(a,b);
        let s = self.span(t);
        let u = &self.knots;
        // basis functions and their derivatives (Piegl and Tiller A2.3), orders 0..=2
        let mut ndu = [[0.;W];W];
        let (mut left,mut right) = ([0.;W],[0.;W]);
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
        let orders = 2.min(p);
        let mut ders = [[0.;W];3];
        for j in 0..=p { ders[0][j] = ndu[j][p]; }
        let mut a2 = [[0.;W];2];
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
        let mut out = [[0.;3];3];
        for (k,row) in ders.iter().enumerate().take(orders+1) {
            for (j,&n) in row.iter().enumerate().take(p+1) {
                let q = self.poles[s-p+j];
                for c in 0..3 { out[k][c] += n*q[c]; }
            }
        }
        (out[0],out[1],out[2])
    }
    pub fn point(&self,t: f64) -> V { self.d2(t).0 }
    /// The length of the control polygon, a bound on the curve's.
    pub fn hull_length(&self) -> f64 { self.poles.windows(2).map(|w| crate::space::distance(w[0],w[1])).sum() }
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
    let s = BSpline {degree,knots,poles:vec![[0.;3];m]};
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

/// A tensor-product B-spline surface: `poles[i][j]` the pole at `u` index `i` and `v` index `j`.
#[derive(Clone,Debug,PartialEq)]
pub struct Net { pub du: usize,pub dv: usize,pub uknots: Vec<f64>,pub vknots: Vec<f64>,pub poles: Vec<Vec<V>> }

impl Net {
    pub fn point(&self,u: f64,v: f64) -> V {
        let rows: Vec<V> = self.poles.iter().map(|col| BSpline {degree:self.dv,knots:self.vknots.clone(),poles:col.clone()}.point(v)).collect();
        BSpline {degree:self.du,knots:self.uknots.clone(),poles:rows}.point(u)
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
            poles:cols.into_iter().map(|c| c.poles).collect()};
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
