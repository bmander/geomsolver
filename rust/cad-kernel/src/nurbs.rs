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
/// The degrees under this evaluated with a table that size (a cubic's or less in one of four).
const SMALL: usize = 10;

#[derive(Clone,Debug,PartialEq)]
pub struct BSpline { pub degree: usize,pub knots: Vec<f64>,pub poles: Vec<V>,pub weights: Option<Vec<f64>> }

/// **A curve's heights over a plane as a scalar B-spline** (`BSpline::heights`; #780 R2, where
/// `query` meets a B-spline curve with a plane): its coefficients `c`, its value and rate enclosed
/// over a stretch by de Boor's algorithm in intervals (`enclose::scalar`, #814).
#[derive(Clone,Debug)]
pub struct Heights<'a> { degree: usize,knots: &'a [f64],c: Vec<f64> }

impl Heights<'_> {
    /// Its coefficients, what its value and rate are enclosed from (`enclose::scalar`).
    pub fn coefficients(&self) -> &[f64] { &self.c }
    /// The height and its rate at `t`, clamped into the domain.
    pub fn at(&self,t: f64) -> (f64,f64) {
        let (p,n) = (self.degree,self.c.len());
        let t = t.clamp(self.knots[p],self.knots[n]);
        with_basis(p,self.knots,n,t,1,|s,ders| {
            let (mut v,mut r) = (0.,0.);
            for j in 0..=p { v += ders[0][j]*self.c[s-p+j]; r += ders[1][j]*self.c[s-p+j] }
            (v,r)
        })
    }
}

/// Whether `weights` may weigh `count` poles: one each, every one finite and positive (a weight at
/// or below zero lets the curve leave its poles' hull, or pass through infinity).
fn weighs(weights: &[f64],count: usize) -> bool { weights.len() == count && weights.iter().all(|&w| w.is_finite() && w > 0.) }

/// `C`, `C'`, `C''` from the weighted sums `A⁽ᵏ⁾ = Σ N⁽ᵏ⁾ w P` and `W⁽ᵏ⁾ = Σ N⁽ᵏ⁾ w` (Piegl and
/// Tiller A4.2).
fn quotient(a: [V;3],w: [f64;3]) -> [V;3] {
    let c0: V = std::array::from_fn(|c| a[0][c]/w[0]);
    let c1: V = std::array::from_fn(|c| (a[1][c]-w[1]*c0[c])/w[0]);
    let c2: V = std::array::from_fn(|c| (a[2][c]-2.*w[1]*c1[c]-w[2]*c0[c])/w[0]);
    [c0,c1,c2]
}

impl BSpline {
    /// A curve from its parts, refused unless its knots are non-decreasing and one more than its
    /// poles and degree.
    pub fn new(degree: usize,knots: Vec<f64>,poles: Vec<V>) -> Result<BSpline,String> {
        if degree == 0 || degree > MAX_DEGREE || poles.len() <= degree || knots.len() != poles.len()+degree+1
            || knots.windows(2).any(|w| !(w[1] >= w[0])) || !(knots[degree] < knots[poles.len()]) {
            return Err(format!("a B-spline needs a degree from 1 to {MAX_DEGREE}, more poles than its degree and non-decreasing knots, one more than both"))
        }
        Ok(BSpline {degree,knots,poles,weights:None})
    }
    /// A rational curve from its parts, refused as `new` refuses and unless each pole has a finite
    /// positive weight.
    pub fn rational(degree: usize,knots: Vec<f64>,poles: Vec<V>,weights: Vec<f64>) -> Result<BSpline,String> {
        let s = BSpline::new(degree,knots,poles)?;
        if !weighs(&weights,s.poles.len()) { return Err("a rational B-spline needs a finite positive weight for each pole".into()) }
        Ok(BSpline {weights:Some(weights),..s})
    }
    pub fn is_rational(&self) -> bool { self.weights.is_some() }
    /// Bounds on `|C′|` and `|C″|` over the whole domain (`derivative_bounds`); none for a rational
    /// curve.
    pub fn derivative_bounds(&self) -> Option<[f64;2]> {
        (!self.is_rational()).then(|| derivative_bounds(self.degree,&self.knots,&self.poles))
    }
    /// **The curve's derivative as a B-spline** (its hodograph): a degree lower, on the knots less
    /// their ends, its poles the poles' differences. None for a rational or a constant curve.
    pub fn hodograph(&self) -> Option<BSpline> {
        let p = self.degree;
        if self.is_rational() || p == 0 { return None }
        let poles = (0..self.poles.len()-1).map(|i| {
            let dt = self.knots[i+p+1]-self.knots[i+1];
            if dt > 0. { crate::util::space::scale(crate::util::space::sub(self.poles[i+1],self.poles[i]),p as f64/dt) } else { [0.;3] }
        }).collect();
        BSpline::new(p-1,self.knots[1..self.knots.len()-1].to_vec(),poles).ok()
    }
    /// Where the curve is defined.
    pub fn domain(&self) -> [f64;2] { [self.knots[self.degree],self.knots[self.poles.len()]] }
    /// The same curve with the knot `t` put in once (`insert`, Boehm): one more pole, so a change of
    /// the poles at either end reaches no farther in than the span `t` bounds. Unchanged where `t`
    /// is not strictly inside the domain.
    pub fn with_knot(&self,t: f64) -> BSpline {
        let [lo,hi] = self.domain();
        if !(t > lo && t < hi) { return self.clone() }
        let (mut knots,mut h) = (self.knots.clone(),self.lifted());
        insert(&mut knots,&mut h,self.degree,t);
        self.dropped(knots,h)
    }
    /// The curve over `[a, b]` (within its domain) alone, exactly (`segment`): the same points at
    /// the same parameters, clamped there; a rational one's in homogeneous coordinates.
    pub fn segment(&self,[a,b]: [f64;2]) -> BSpline {
        let [lo,hi] = self.domain();
        let (knots,h) = segment(&self.knots,&self.lifted(),self.degree,a.max(lo),b.min(hi));
        self.dropped(knots,h)
    }
    /// Its poles as rows of one homogeneous point, `(w x, w y, w z, w)` (`w` 1 unless rational):
    /// what `insert` and `segment` combine.
    fn lifted(&self) -> Vec<Vec<[f64;4]>> {
        let w = |i: usize| self.weights.as_ref().map_or(1.,|w| w[i]);
        self.poles.iter().enumerate().map(|(i,q)| { let w = w(i); vec![[w*q[0],w*q[1],w*q[2],w]] }).collect()
    }
    /// A curve of its degree and kind from `knots` and `lifted` rows.
    fn dropped(&self,knots: Vec<f64>,h: Vec<Vec<[f64;4]>>) -> BSpline {
        let poles = h.iter().map(|r| [r[0][0]/r[0][3],r[0][1]/r[0][3],r[0][2]/r[0][3]]).collect();
        let weights = self.weights.is_some().then(|| h.iter().map(|r| r[0][3]).collect());
        BSpline {degree:self.degree,knots,poles,weights}
    }
    /// The distinct knots strictly inside `[a, b]`, where the curve's third derivative may jump.
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        let mut out: Vec<f64> = self.knots.iter().copied().filter(|&k| k > a && k < b).collect();
        out.dedup();
        out
    }
    fn span(&self,t: f64) -> usize { span(self.degree,&self.knots,self.poles.len(),t) }
    /// `C`, `C'`, `C''` at `t` (clamped into the domain).
    pub fn d2(&self,t: f64) -> (V,V,V) { let [x,d,dd] = self.ders(t,2); (x,d,dd) }
    /// `C`, `C'` at `t` (clamped into the domain): `d2`'s, its second derivative not read.
    pub fn d1(&self,t: f64) -> (V,V) { let [x,d,_] = self.ders(t,1); (x,d) }
    /// `C` and its derivatives up to order `n` (at most 2; zero past it) at `t`, clamped.
    fn ders(&self,t: f64,n: usize) -> [V;3] {
        let p = self.degree;
        let [a,b] = self.domain();
        with_basis(p,&self.knots,self.poles.len(),t.clamp(a,b),n,|s,ders| {
            if let Some(w) = &self.weights {
                let (mut out,mut ws) = ([[0.;3];3],[0.;3]);
                for (k,row) in ders.iter().enumerate().take(n.min(p)+1) {
                    for (j,&b) in row.iter().enumerate().take(p+1) {
                        let (q,bw) = (self.poles[s-p+j],b*w[s-p+j]);
                        for c in 0..3 { out[k][c] += bw*q[c]; }
                        ws[k] += bw;
                    }
                }
                return quotient(out,ws)
            }
            let mut out = [[0.;3];3];
            for (k,row) in ders.iter().enumerate().take(n.min(p)+1) {
                for (j,&b) in row.iter().enumerate().take(p+1) {
                    let q = self.poles[s-p+j];
                    for c in 0..3 { out[k][c] += b*q[c]; }
                }
            }
            out
        })
    }
    pub fn point(&self,t: f64) -> V { self.ders(t,0)[0] }
    /// The curve's point at `t` in homogeneous coordinates (clamped): `(Σ N w P, Σ N w)`, every
    /// weight one for a curve not rational. Polynomial in `t` over each knot span, of the curve's
    /// degree, where the point itself is a quotient (`query`'s meridians read it so).
    pub fn homogeneous(&self,t: f64) -> (V,f64) {
        let p = self.degree;
        let [a,b] = self.domain();
        with_basis(p,&self.knots,self.poles.len(),t.clamp(a,b),0,|s,ders| {
            let (mut out,mut w) = ([0.;3],0.);
            for (j,&n) in ders[0].iter().enumerate().take(p+1) {
                let k = s-p+j;
                let wk = self.weights.as_ref().map_or(1.,|w| w[k]);
                for c in 0..3 { out[c] += n*wk*self.poles[k][c] }
                w += n*wk;
            }
            (out,w)
        })
    }
    /// **The heights of the curve's points over the plane through `o` square to `n`**, weighed
    /// (`Heights`): `Σ N_i w_i n · (P_i − o)`, a scalar B-spline nought exactly where the curve's
    /// point is on the plane (its weight is positive), its value and rate enclosed in intervals.
    pub fn heights(&self,n: V,o: V) -> Heights<'_> {
        let h = crate::util::space::dot(n,o);
        let c: Vec<f64> = self.poles.iter().enumerate()
            .map(|(i,q)| self.weights.as_ref().map_or(1.,|w| w[i])*(crate::util::space::dot(n,*q)-h)).collect();
        Heights {degree:self.degree,knots:&self.knots,c}
    }
    /// The length of the control polygon, a bound on the curve's (a rational one's too: its de
    /// Casteljau steps cut corners).
    pub fn hull_length(&self) -> f64 { self.poles.windows(2).map(|w| crate::util::space::distance(w[0],w[1])).sum() }
    /// The poles whose basis reaches `[a, b]` (clamped into the domain): their hull holds the curve
    /// there.
    pub fn poles_over(&self,[a,b]: [f64;2]) -> &[V] {
        &self.poles[self.span(a.min(b))-self.degree..=self.span(a.max(b))]
    }
    /// The parameter of the curve's point nearest `p`: the nearest of samples across every knot
    /// span, then Newton on (C − p)·C'. A span lies in its poles' hull, so the distance to their
    /// box bounds its samples' from below: spans are read nearest box first, and none past the
    /// nearest sample found, which finds the sample the whole walk would (the earlier of two as
    /// near). A span's samples are read off its Bézier form, by de Casteljau's steps.
    pub fn nearest(&self,p: V) -> f64 {
        if self.degree < 4 { self.nearest_in::<4>(p) } else { self.nearest_in::<W>(p) }
    }
    /// **The foot of `p` on `b`, its spans read off their index** (`SpanIndex`): the same foot
    /// `nearest` finds, each span's samples evaluated once for the curve, not again at every point;
    /// where it has many spans, found through their tree rather than boxing every span at every
    /// point.
    pub fn nearest_indexed(b: &std::sync::Arc<BSpline>,p: V) -> f64 {
        if b.degree >= 4 || b.weights.is_some() { return b.nearest(p) }
        let index = span_index(b);
        let gap = |x: crate::util::bvh::Bounds<3>| gap_to(x,p)*(1.-1e-12);
        let read = |i: usize,best: &mut (f64,f64)| b.read_samples(index.spans[i],&index.samples[i],p,best);
        let Some(tree) = &index.tree else {
            return b.walked(p,index.boxes.iter().enumerate().map(|(i,&x)| (gap(x),i)).collect(),&read)
        };
        let found = std::cell::Cell::new((f64::INFINITY,b.domain()[0]));
        tree.query_nearest(gap,|| found.get().0,|i| { let mut x = found.get(); read(i,&mut x); found.set(x) });
        b.settled(p,found.get().1)
    }
    /// **How far `p` is from the nearest of `b`'s spans' boxes**, where its spans are indexed
    /// (`nearest_indexed`): no point of the curve is nearer — a bound far tighter than its whole
    /// box for a long curve, which a point beside its middle is within.
    pub fn gap_indexed(b: &std::sync::Arc<BSpline>,p: V) -> Option<f64> {
        if b.degree >= 4 || b.weights.is_some() { return None }
        let index = span_index(b);
        let tree = index.tree.as_ref()?;
        let best = std::cell::Cell::new(f64::INFINITY);
        tree.query_nearest(|x| gap_to(x,p),|| best.get(),|s| best.set(best.get().min(gap_to(index.boxes[s],p))));
        Some(best.get())
    }
    /// Span `s`'s `SAMPLES` samples, evenly across it: its Bézier form's points, by de Casteljau's
    /// steps.
    fn samples<const M: usize>(&self,s: usize) -> [V;SAMPLES] {
        let bezier = self.bezier::<M>(s);
        std::array::from_fn(|j| casteljau::<M>(&bezier[..=self.degree],j as f64/(SAMPLES-1) as f64))
    }
    /// Span `s`'s samples `at` read against `best`, the nearer kept (the earlier on a tie).
    fn read_samples(&self,s: usize,at: &[V;SAMPLES],p: V,best: &mut (f64,f64)) {
        let (w0,w1) = (self.knots[s],self.knots[s+1]);
        for (j,&x) in at.iter().enumerate() {
            let f = j as f64/(SAMPLES-1) as f64;
            let t = w0+(w1-w0)*f;
            let d = crate::util::space::distance(x,p);
            if d < best.0 || (d == best.0 && t < best.1) { *best = (d,t); }
        }
    }
    /// `nearest`, a span's Bézier poles in a table of `M` (more than the degree).
    fn nearest_in<const M: usize>(&self,p: V) -> f64 {
        let spans = (self.degree..self.poles.len()).filter(|&s| self.knots[s] < self.knots[s+1])
            .map(|s| (gap_to(self.span_box(s),p)*(1.-1e-12),s)).collect();
        self.walked(p,spans,&|s,best| self.read_samples(s,&self.samples::<M>(s),p,best))
    }
    /// **`nearest`'s walk**: `spans`, each with the least distance from `p` its box allows, read by
    /// `read` nearest box first (the earlier on a tie) and none past the nearest sample found,
    /// which alone decide it; that sample refined.
    fn walked(&self,p: V,mut spans: Vec<(f64,usize)>,read: &dyn Fn(usize,&mut (f64,f64))) -> f64 {
        spans.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        let mut best = (f64::INFINITY,self.domain()[0]);
        for (bound,s) in spans {
            if bound > best.0 { break }
            read(s,&mut best);
        }
        self.settled(p,best.1)
    }
    /// Span `s`'s poles' box: the span lies within it.
    fn span_box(&self,s: usize) -> crate::util::bvh::Bounds<3> {
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        for q in &self.poles[s-self.degree..=s] { for k in 0..3 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } }
        crate::util::bvh::Bounds {lo,hi}
    }
    /// The foot refined from the nearest sample `t`, a closed spline's read from both ends.
    fn settled(&self,p: V,t: f64) -> f64 {
        let [a,z] = self.domain();
        let t = self.refined(p,t);
        // (a closed spline passing its seam smoothly, its foot held at one end by the search's clamp,
        // read again from the other end, the nearer kept: a point of a closed spline's extrusion just
        // short of its seam, seeded at the seam's first end, read a foot 0.07 off, #880 Q1b S6)
        if t == a || t == z {
            use crate::util::space::{distance,norm};
            let (da,dz) = (self.ders(a,1),self.ders(z,1));
            let closed = distance(da[0],dz[0]) <= 1e-12*(1.+norm(da[0])) && distance(da[1],dz[1]) <= 1e-9*norm(da[1]);
            if closed {
                let other = self.refined(p,if t == a { z } else { a });
                if distance(self.point(other),p) < distance(self.point(t),p) { return other }
            }
        }
        t
    }
    /// The foot of `p` on the curve found by Newton on (C − p)·C' from `t` alone (a local search:
    /// the nearest where `t` is nearest it). **Where Newton's curvature `|C'|² + (C − p)·C''` is no
    /// longer positive, the step is Gauss–Newton's (`|C'|²` alone), which still descends** (#1013):
    /// a curve that barely moves with its parameter and bends hard there — a sheet's iso line beside
    /// the fold of its net, `x = s²` — has it negative a step from its end, where the search was left
    /// standing, the foot of a point inside the curve read at the end, 3e-3 off.
    pub fn refined(&self,p: V,t: f64) -> f64 {
        let [a,z] = self.domain();
        let mut t = t.clamp(a,z);
        for _ in 0..20 {
            let (x,d,dd) = self.d2(t);
            let e = crate::util::space::sub(x,p);
            let g = crate::util::space::dot(e,d);
            let speed = crate::util::space::dot(d,d);
            let h = speed+crate::util::space::dot(e,dd);
            let h = if h > 0. { h } else { speed };
            if !(h > 0.) { break }
            let next = (t-g/h).clamp(a,z);
            if (next-t).abs() <= 1e-15*(z-a) { t = next; break }
            t = next;
        }
        t
    }
    /// Span `s`'s Bézier poles (`degree + 1` of them, in homogeneous coordinates: each weighed by
    /// its weight, the weight last), each the curve's blossom at the span's ends taken so many
    /// times each (de Boor's steps with an argument a step).
    fn bezier<const M: usize>(&self,s: usize) -> [[f64;4];M] {
        let deg = self.degree;
        let (lo,hi) = (self.knots[s],self.knots[s+1]);
        let lift = |j: usize| -> [f64;4] {
            let (q,w) = (self.poles[j],self.weights.as_ref().map_or(1.,|w| w[j]));
            [q[0]*w,q[1]*w,q[2]*w,w]
        };
        let mut out = [[0.;4];M];
        for (i,o) in out.iter_mut().enumerate().take(deg+1) {
            let mut d = [[0.;4];M];
            for (j,e) in d.iter_mut().enumerate().take(deg+1) { *e = lift(s-deg+j); }
            for r in 1..=deg {
                let x = if r <= deg-i { lo } else { hi };
                for j in (r..=deg).rev() {
                    let (k0,k1) = (self.knots[s+j-deg],self.knots[s+1+j-r]);
                    let alpha = (x-k0)/(k1-k0);
                    for c in 0..4 { d[j][c] = (1.-alpha)*d[j-1][c]+alpha*d[j][c]; }
                }
            }
            *o = d[deg];
        }
        out
    }
}

/// The knot span holding `t`, for `count` poles of `degree`.
pub(crate) fn span(degree: usize,knots: &[f64],count: usize,t: f64) -> usize {
    let (p,n) = (degree,count);
    if t >= knots[n] { return n-1 }
    if t <= knots[p] { return p }
    let (mut lo,mut hi) = (p,n);
    while hi-lo > 1 { let mid = (lo+hi)/2; if t < knots[mid] { hi = mid } else { lo = mid } }
    lo
}

/// `f` of the span holding `t`, and there the `degree + 1` basis functions and their derivatives
/// up to order `n` (at most 2) — Piegl and Tiller's A2.3, on the stack: a cubic's or less in a
/// table of four, a degree under `SMALL` in a table that size, a higher one (OCCT approximates up
/// to 25) in one of `W`; a clamped cubic of four poles in its Bernstein basis, with no table.
#[inline]
pub(crate) fn with_basis<R>(p: usize,u: &[f64],count: usize,t: f64,n: usize,f: impl FnOnce(usize,&[&[f64];3]) -> R) -> R {
    // A clamped cubic with four poles is a Bezier segment. Its basis and first two
    // derivatives are Bernstein polynomials; no recurrence table is needed. Check the
    // clamping too: four poles alone do not imply a Bernstein basis on an unclamped vector.
    if p == 3 && count == 4 && u[0] == u[3] && u[4] == u[7] {
        let h = u[4]-u[3];
        let (a,b) = ((t-u[3])/h,(u[4]-t)/h);
        let (aa,bb) = (a*a,b*b);
        let mut d = [[b*bb,3.*a*bb,3.*aa*b,a*aa],[0.;4],[0.;4]];
        if n > 0 {
            let r = 3./h;
            d[1] = [-r*bb,r*b*(b-2.*a),r*a*(2.*b-a),r*aa];
        }
        if n > 1 {
            let r = 6./h/h;
            d[2] = [r*b,r*(a-2.*b),r*(b-2.*a),r*a];
        }
        return f(3,&[&d[0],&d[1],&d[2]])
    }
    fn tabled<const M: usize,R>(p: usize,u: &[f64],count: usize,t: f64,n: usize,f: impl FnOnce(usize,&[&[f64];3]) -> R) -> R {
        let (s,d) = basis_ders_in::<M>(p,u,count,t,n);
        f(s,&[&d[0],&d[1],&d[2]])
    }
    if p < 4 { tabled::<4,R>(p,u,count,t,n,f) } else if p < SMALL { tabled::<SMALL,R>(p,u,count,t,n,f) } else { tabled::<W,R>(p,u,count,t,n,f) }
}

/// **The `j`-th Bézier control point of a polynomial piece restricted to `[t0, t1]`**: the piece
/// over knot span `k` of the spline of degree `deg` on `knots` whose `deg + 1` poles reaching it
/// are `poles`, its blossom at `t0` taken `deg − j` times and `t1` `j` times, by de Boor's steps
/// with those arguments — `[t0, t1]` the span itself (its Bézier points), any stretch within it,
/// or a point.
pub(crate) fn blossom(knots: &[f64],poles: &[V],k: usize,deg: usize,t: [f64;2],j: usize) -> V {
    fn tabled<const M: usize>(knots: &[f64],poles: &[V],k: usize,deg: usize,[t0,t1]: [f64;2],j: usize) -> V {
        let mut q = [[0.;3];M];
        q[..=deg].copy_from_slice(&poles[..=deg]);
        for r in 1..=deg {
            let arg = if r <= deg-j { t0 } else { t1 };
            for m in (r..=deg).rev() {
                let i = k-deg+m;
                let den = knots[i+deg+1-r]-knots[i];
                let alpha = if den > 0. { (arg-knots[i])/den } else { 0. };
                q[m] = std::array::from_fn(|c| (1.-alpha)*q[m-1][c]+alpha*q[m][c]);
            }
        }
        q[deg]
    }
    if deg < 4 { tabled::<4>(knots,poles,k,deg,t,j) } else if deg < SMALL { tabled::<SMALL>(knots,poles,k,deg,t,j) } else { tabled::<W>(knots,poles,k,deg,t,j) }
}

/// **`blossom`'s weights**: the `deg + 1` numbers its point is of the poles reaching span `k` —
/// the blossom is linear in them, so these are de Boor's same steps over the poles `e_0 … e_deg` —
/// for a piece's every Bézier point read off many rows of poles over the one span.
pub(crate) fn blossom_weights(knots: &[f64],k: usize,deg: usize,[t0,t1]: [f64;2],j: usize) -> Vec<f64> {
    let mut q: Vec<Vec<f64>> = (0..=deg).map(|m| (0..=deg).map(|c| if c == m { 1. } else { 0. }).collect()).collect();
    for r in 1..=deg {
        let arg = if r <= deg-j { t0 } else { t1 };
        for m in (r..=deg).rev() {
            let i = k-deg+m;
            let den = knots[i+deg+1-r]-knots[i];
            let alpha = if den > 0. { (arg-knots[i])/den } else { 0. };
            q[m] = (0..=deg).map(|c| (1.-alpha)*q[m-1][c]+alpha*q[m][c]).collect();
        }
    }
    q.swap_remove(deg)
}

/// A Bézier curve's point at `f`, its poles in homogeneous coordinates: de Casteljau's steps.
fn casteljau<const M: usize>(poles: &[[f64;4]],f: f64) -> V {
    let mut d = [[0.;4];M];
    d[..poles.len()].copy_from_slice(poles);
    for r in 1..poles.len() {
        for j in 0..poles.len()-r {
            for c in 0..4 { d[j][c] = (1.-f)*d[j][c]+f*d[j+1][c]; }
        }
    }
    [d[0][0]/d[0][3],d[0][1]/d[0][3],d[0][2]/d[0][3]]
}

fn basis_ders_in<const M: usize>(p: usize,u: &[f64],count: usize,t: f64,n: usize) -> (usize,[[f64;M];3]) {
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
    let mut ders = [[0.;M];3];
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
/// **Bounds on `|C′|` and `|C″|` of the polynomial B-spline of `degree` on `knots` and `poles`, over
/// its whole domain**: each derivative lies in its hodograph's hull, so within its longest pole.
pub fn derivative_bounds(degree: usize,knots: &[f64],poles: &[V]) -> [f64;2] {
    let mut poles = poles.to_vec();
    let mut out = [0.;2];
    for order in 0..2 {
        let d = degree.saturating_sub(order);
        poles = (0..poles.len().saturating_sub(1)).map(|i| {
            let dt = knots[i+degree+1]-knots[i+order+1];
            if dt > 0. { crate::util::space::scale(crate::util::space::sub(poles[i+1],poles[i]),d as f64/dt) } else { [0.;3] }
        }).collect();
        out[order] = poles.iter().map(|&q| crate::util::space::norm(q)).fold(0.,f64::max);
    }
    out
}

pub fn interpolate(pts: &[V],t: &[f64],degree: usize) -> Option<BSpline> {
    let m = pts.len();
    let degree = degree.min(m.checked_sub(1)?);
    if degree == 0 || t.len() != m || t.windows(2).any(|w| !(w[1] > w[0])) { return None }
    collocated(BSpline {degree,knots:averaged(t,degree),poles:vec![[0.;3];m],weights:None},pts,t)
}

/// The B-spline of `degree` on the clamped `knots` through `f` at their Greville abscissae,
/// each pole's mean of the `degree` knots past its first: a collocation always well posed
/// (Schoenberg–Whitney), so every curve of that degree on those knots, or on fewer of them, is
/// its own interpolant exactly. None where the knots are not clamped (each end `degree + 1` times
/// over, no inner knot more than `degree` times) or not in order.
pub fn interpolate_on(f: &dyn Fn(f64) -> V,degree: usize,knots: Vec<f64>) -> Option<BSpline> {
    let d = distinct(&knots);
    let clamped = d.len() >= 2 && d[0].1 == degree+1 && d[d.len()-1].1 == degree+1
        && d[1..d.len()-1].iter().all(|k| k.1 <= degree);
    if degree == 0 || degree > MAX_DEGREE || !clamped || knots.windows(2).any(|w| !(w[1] >= w[0])) {
        return None
    }
    let m = knots.len()-degree-1;
    let t: Vec<f64> = (0..m).map(|i| greville(&knots,degree,i)).collect();
    let pts: Vec<V> = t.iter().map(|&t| f(t)).collect();
    collocated(BSpline {degree,knots,poles:vec![[0.;3];m],weights:None},&pts,&t)
}

/// The Greville abscissa of pole `i` of a B-spline of `degree` on `knots`: the mean of the
/// `degree` knots past its first, where a spline whose poles are a linear function's values there
/// is that function.
pub(crate) fn greville(knots: &[f64],degree: usize,i: usize) -> f64 { knots[i+1..=i+degree].iter().sum::<f64>()/degree as f64 }

/// The poles of `s` (its degree and knots) through `pts` at the increasing parameters `t`: one
/// collocation solve per coordinate, in its band where it is long.
fn collocated(s: BSpline,pts: &[V],t: &[f64]) -> Option<BSpline> {
    let (m,degree) = (pts.len(),s.degree);
    if m > BANDED { if let Some(poles) = banded(&s,pts,t) { return Some(BSpline {poles,..s}) } }
    // the collocation matrix: each parameter's basis functions, in the columns of its span
    let mut n = vec![0.;m*m];
    for (k,&tk) in t.iter().enumerate() {
        let span = s.span(tk.clamp(t[0],t[m-1]));
        let basis = s.basis(tk,span);
        for (j,&b) in basis.iter().enumerate().take(degree+1) { n[k*m+span-degree+j] = b; }
    }
    let mut piv = Vec::with_capacity(m);
    if !crate::util::linalg::lu_factor(m,&mut n,&mut piv) { return None }
    let mut poles = vec![[0.;3];m];
    for c in 0..3 {
        let mut x: Vec<f64> = pts.iter().map(|p| p[c]).collect();
        crate::util::linalg::lu_apply(m,&n,&piv,&mut x);
        for k in 0..m { poles[k][c] = x[k]; }
    }
    poles.iter().flatten().all(|v| v.is_finite()).then(|| BSpline {poles,..s})
}

/// The arc of radius `r` about `f.o` in its `xy` plane from angle `a0` to `a1` (counter-clockwise
/// about `f.z`, `a0 < a1`, at most a whole turn), exactly: quadratic and rational, one span per quarter turn or
/// less, each span's middle pole where its end tangents meet, weighted the cosine of half its turn
/// (Piegl and Tiller A7.1). Its parameter runs over `[a0, a1]`, meeting the angle at every knot.
pub fn arc(f: &Frame,r: f64,[a0,a1]: [f64;2]) -> BSpline {
    use crate::util::fmath::Det;
    let (turn,n,d) = arc_spans([a0,a1]);
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

/// The turn `arc` makes over `[a0, a1]` (at most a whole one), its spans (a quarter turn each or
/// fewer) and the turn of each.
fn arc_spans([a0,a1]: [f64;2]) -> (f64,usize,f64) {
    let turn = (a1-a0).clamp(0.,std::f64::consts::TAU);
    let n = ((turn/std::f64::consts::FRAC_PI_2-1e-12).ceil() as usize).max(1);
    (turn,n,turn/n as f64)
}

/// Where the angle `a` falls on `arc`'s parameter over `[a0, a1]`, and the parameter's rate per
/// unit of angle there, in closed form: within a span of turn `d` from the knot at `α`, a rational
/// quadratic weighted `cos(d/2)` reaches the angle `α + θ` at the fraction `τ` of its span where
/// `τ / (1 − τ) = sin(θ/2) / sin((d − θ)/2)`, whose rate is `sin(d/2) / (2 (sin(θ/2) + sin((d − θ)/2))²)`.
/// `a` is held to `[a0, a1]`.
pub fn arc_parameter([a0,a1]: [f64;2],a: f64) -> (f64,f64) {
    use crate::util::fmath::Det;
    let (turn,n,d) = arc_spans([a0,a1]);
    let along = (a-a0).clamp(0.,turn);
    let k = ((along/d).floor() as usize).min(n-1);
    let theta = (along-k as f64*d).clamp(0.,d);
    let (p,q) = ((theta/2.).dsin(),((d-theta)/2.).dsin());
    (a0+d*(k as f64+p/(p+q)),d*(d/2.).dsin()/(2.*(p+q)*(p+q)))
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

/// A clamped B-spline of degree `p` along its first index carried on past the end of its domain
/// (`high`, else its start) by `by`, exactly: its end span's polynomial continued, a Bézier piece
/// joined at the old end with that knot `p` times, so the curve over the old domain is unchanged.
fn extend<const N: usize>(knots: &[f64],poles: &[Vec<[f64;N]>],p: usize,high: bool,by: f64) -> (Vec<f64>,Vec<Vec<[f64;N]>>) {
    if !high {
        // the start is the end of the curve read backwards
        let back: Vec<f64> = knots.iter().rev().map(|k| -k).collect();
        let rows: Vec<Vec<[f64;N]>> = poles.iter().rev().cloned().collect();
        let (k,q) = extend(&back,&rows,p,true,by);
        return (k.iter().rev().map(|k| -k).collect(),q.into_iter().rev().collect())
    }
    let n = poles.len();
    let z = knots[n];
    let a = knots[..n].iter().copied().filter(|&k| k < z).fold(f64::NEG_INFINITY,f64::max);
    // the end span alone, a Bézier piece over [a, z]; de Casteljau at s > 1 gives the same polynomial's
    // piece over [a, z + by], and at the old end splitting that leaves the piece over [z, z + by]
    let (_,bezier) = segment(knots,poles,p,a,z);
    let lerp = |x: &Vec<[f64;N]>,y: &Vec<[f64;N]>,t: f64| -> Vec<[f64;N]> {
        x.iter().zip(y).map(|(x,y)| std::array::from_fn(|c| (1.-t)*x[c]+t*y[c])).collect()
    };
    let casteljau = |rows: &[Vec<[f64;N]>],t: f64| -> (Vec<Vec<[f64;N]>>,Vec<Vec<[f64;N]>>) {
        let (mut left,mut right,mut level) = (Vec::new(),Vec::new(),rows.to_vec());
        while !level.is_empty() {
            left.push(level[0].clone());
            right.push(level[level.len()-1].clone());
            level = level.windows(2).map(|w| lerp(&w[0],&w[1],t)).collect();
        }
        right.reverse();
        (left,right)
    };
    let (longer,_) = casteljau(&bezier,(z+by-a)/(z-a));
    let (_,beyond) = casteljau(&longer,(z-a)/(z+by-a));
    let mut k = knots[..knots.len()-1].to_vec();
    k.extend(std::iter::repeat_n(z+by,p+1));
    let mut q = poles.to_vec();
    q.extend(beyond.into_iter().skip(1));
    (k,q)
}

/// A tensor-product B-spline surface: `poles[i][j]` the pole at `u` index `i` and `v` index `j`, and
/// for a rational one `weights[i][j]` its weight.
#[derive(Clone,Debug,PartialEq)]
pub struct Net { pub du: usize,pub dv: usize,pub uknots: Vec<f64>,pub vknots: Vec<f64>,pub poles: Vec<Vec<V>>,pub weights: Option<Vec<Vec<f64>>> }

/// One way of a net's basis at a parameter (`Net::basis`): the span holding it, and the basis
/// functions (`n[0]`) and their rates (`n[1]`) there, the first `degree + 1` of each.
#[derive(Clone,Copy,Debug)]
pub(crate) struct Basis { span: usize,n: [[f64;SMALL];2] }

impl Net {
    /// Refused unless its degrees are from 1 to `MAX_DEGREE`, its net is a grid with more poles each
    /// way than its degree, its knots are non-decreasing and one more than both, and a rational
    /// one's weights are a grid alike, each finite and positive.
    pub fn check(&self) -> Result<(),String> {
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
        with_basis(self.du,&self.uknots,nu,u.clamp(u0,u1),1,|su,bu|
        with_basis(self.dv,&self.vknots,nv,v.clamp(v0,v1),1,|sv,bv| self.d1_of(su,[bu[0],bu[1]],sv,[bv[0],bv[1]])))
    }
    /// **One way's basis at a parameter, kept to be read again** (`d1_by`): the span holding it and
    /// its basis functions and their rates there, the parameter clamped into the domain as `d1`
    /// clamps it — so `d1_by` of two of them is `d1`, to the bit. None for a degree past `SMALL`.
    /// A cell's rule reads each node across at every node along (`props`'s `Cells`), and the
    /// basis was most of a read.
    pub(crate) fn basis(&self,way: usize,t: f64) -> Option<Basis> {
        let (p,knots,count) = if way == 0 { (self.du,&self.uknots,self.poles.len()) } else { (self.dv,&self.vknots,self.poles[0].len()) };
        if p >= SMALL { return None }
        let t = t.clamp(knots[p],knots[count]);
        Some(with_basis(p,knots,count,t,1,|s,b| {
            let mut n = [[0.;SMALL];2];
            n[0][..=p].copy_from_slice(&b[0][..=p]);
            n[1][..=p].copy_from_slice(&b[1][..=p]);
            Basis {span:s,n}
        }))
    }
    /// `d1` read off the two ways' bases (`basis`).
    pub(crate) fn d1_by(&self,u: &Basis,v: &Basis) -> (V,V,V) {
        self.d1_of(u.span,[&u.n[0],&u.n[1]],v.span,[&v.n[0],&v.n[1]])
    }
    /// `S_u` and `S_v` read off the two ways' bases of a polynomial net, `d1_by`'s to the bit, the
    /// point unread.
    pub(crate) fn rates_by(&self,u: &Basis,v: &Basis) -> (V,V) {
        debug_assert!(self.weights.is_none(),"a polynomial net's rates");
        let (su,sv,bu,bv) = (u.span,v.span,&u.n,&v.n);
        let mut out = [[0.;3];2];
        for i in 0..=self.du {
            let row = &self.poles[su-self.du+i];
            for j in 0..=self.dv {
                let q = row[sv-self.dv+j];
                let (nu_,nv_) = (bu[1][i]*bv[0][j],bu[0][i]*bv[1][j]);
                for c in 0..3 { out[0][c] += nu_*q[c]; out[1][c] += nv_*q[c]; }
            }
        }
        (out[0],out[1])
    }
    /// `d1` from the spans holding `(u, v)` and the bases there, each way's functions and rates.
    #[inline(always)]
    fn d1_of(&self,su: usize,bu: [&[f64];2],sv: usize,bv: [&[f64];2]) -> (V,V,V) {
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
            return (s,std::array::from_fn(|c| (a[1][c]-ws[1]*s[c])/ws[0]),
                std::array::from_fn(|c| (a[2][c]-ws[2]*s[c])/ws[0]))
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
    /// The curve along `u` through the `j`th column of poles: the iso line at the `v` that column
    /// stands for where it is an end of a clamped net.
    pub fn column(&self,j: usize) -> BSpline {
        BSpline {degree:self.du,knots:self.uknots.clone(),poles:self.poles.iter().map(|row| row[j]).collect(),
            weights:self.weights.as_ref().map(|w| w.iter().map(|row| row[j]).collect())}
    }
    /// The curve along `v` through the `i`th row of poles: the iso line at the `u` that row stands
    /// for where it is an end of a clamped net.
    pub fn row(&self,i: usize) -> BSpline {
        BSpline {degree:self.dv,knots:self.vknots.clone(),poles:self.poles[i].clone(),weights:self.weights.as_ref().map(|w| w[i].clone())}
    }
    /// `S`, `S_u`, `S_v`, `S_uu`, `S_uv`, `S_vv` at `(u, v)`, clamped into the domain.
    pub fn d2(&self,u: f64,v: f64) -> [V;6] {
        let [[u0,u1],[v0,v1]] = self.domain();
        self.d2_continued(u.clamp(u0,u1),v.clamp(v0,v1))
    }
    /// `d2` read past the domain on the sheet's **continuation**: each end span's polynomial (its
    /// rational function, weighted) carried on beyond the knot it ends at, smooth across the
    /// domain's edge, where `d2` holds the point at the edge.
    pub fn d2_continued(&self,u: f64,v: f64) -> [V;6] {
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        with_basis(self.du,&self.uknots,nu,u,2,|su,bu|
        with_basis(self.dv,&self.vknots,nv,v,2,|sv,bv| {
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
        }))
    }
    /// The surface over `[u0, u1] × [v0, v1]` (within its domain) alone, exactly: the same points at
    /// the same parameters, only the net that reaches there (`BSpline::segment` is a curve's).
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
    /// The surface carried on past its domain by its own polynomials (`extend`): `by` along `u` (`k`
    /// 0) or `v` (`k` 1), past the end (`high`) or before the start. The same surface over the old
    /// domain. None for a rational net.
    pub fn extended(&self,k: usize,high: bool,by: f64) -> Option<Net> {
        if self.weights.is_some() { return None }
        let mut net = self.clone();
        if k == 0 {
            (net.uknots,net.poles) = extend(&self.uknots,&self.poles,self.du,high,by);
        } else {
            let cols: Vec<Vec<V>> = (0..self.poles[0].len()).map(|j| self.poles.iter().map(|r| r[j]).collect()).collect();
            let (knots,cols) = extend(&self.vknots,&cols,self.dv,high,by);
            net.vknots = knots;
            net.poles = (0..cols[0].len()).map(|i| cols.iter().map(|c| c[i]).collect()).collect();
        }
        Some(net)
    }
    /// The poles whose basis reaches `[u0, u1] × [v0, v1]` (clamped into the domain): their hull
    /// holds the surface there.
    pub fn poles_over(&self,[u0,u1]: [f64;2],[v0,v1]: [f64;2]) -> impl Iterator<Item=V> + '_ {
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        let (i0,i1) = (span(self.du,&self.uknots,nu,u0.min(u1))-self.du,span(self.du,&self.uknots,nu,u0.max(u1)));
        let (j0,j1) = (span(self.dv,&self.vknots,nv,v0.min(v1))-self.dv,span(self.dv,&self.vknots,nv,v0.max(v1)));
        self.poles[i0..=i1].iter().flat_map(move |row| row[j0..=j1].iter().copied())
    }
    /// **The knots inside the domain at which the sheet is no more than continuous**, along `u`
    /// (`k` 0) or `v` (1): each one `degree` times or more, where the sheet's derivative across them
    /// may jump (a net fitted patch by patch, its knots each the degree's multiplicity), distinct
    /// and rising.
    pub fn kinks(&self,k: usize) -> Vec<f64> {
        let (p,t) = if k == 0 { (self.du,&self.uknots) } else { (self.dv,&self.vknots) };
        let [a,b] = self.domain()[k];
        let mut out: Vec<f64> = t.windows(p).filter(|w| w[0] == w[p-1] && w[0] > a && w[0] < b).map(|w| w[0]).collect();
        out.dedup();
        out
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
pub fn fit_net(f: &dyn Fn(f64,f64) -> V,du: usize,dv: usize,nu: usize,nv: usize,tol: f64) -> Option<(Net,f64)> {
    fit_net_each(f,du,dv,nu,nv,tol).map(|(net,eu,ev)| (net,eu.max(ev)))
}

/// `fit_net`, with how far the net misses between its columns (along `u`) and between its rows
/// (along `v`) apart, so a caller refining it again grows only the way that misses.
pub fn fit_net_each(f: &dyn Fn(f64,f64) -> V,du: usize,dv: usize,mut nu: usize,mut nv: usize,tol: f64) -> Option<(Net,f64,f64)> {
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
        let off = |u: f64,v: f64| crate::util::space::distance(net.point(u,v),f(u,v));
        let (mut eu,mut ev) = (0f64,0f64);
        for i in 0..nu { for &v in &tv { eu = eu.max(off((tu[i]+tu[i+1])/2.,v)); } }
        for j in 0..nv { for &u in &tu { ev = ev.max(off(u,(tv[j]+tv[j+1])/2.)); } }
        let (grow_u,grow_v) = (eu > tol && nu < 1024,ev > tol && nv < 512);
        if !grow_u && !grow_v { return Some((net,eu,ev)) }
        if grow_u { nu *= 2; }
        if grow_v { nv *= 2; }
    }
}

/// **`fit_net` across breaks**: the net of degrees `du`, `dv` through `f` over `[0, 1]²` whose
/// knots along `u` hold each of `breaks` (in `(0, 1)`, increasing) `du` times over, so it may turn
/// its rate there as `f` does — a surface ruled between two traced rails, whose parameter's pace
/// jumps at each of their points, which no net smooth across them follows within a fit's
/// tolerance. Each stretch between breaks gets `m` even spans along `u` (`m` from 2), each row
/// interpolated at its knots' Greville abscissae (`interpolate_on`), the rows' poles along `v` as
/// `fit_net`'s; `m` and `nv` doubled while the net misses `f` by more than `tol` midway between
/// knots, up to 64 a stretch and 512 rows. With how far it misses.
pub fn fit_net_broken(f: &dyn Fn(f64,f64) -> V,du: usize,dv: usize,breaks: &[f64],mut nv: usize,tol: f64) -> Option<(Net,f64)> {
    let ends: Vec<f64> = std::iter::once(0.).chain(breaks.iter().copied().filter(|&b| b > 0. && b < 1.)).chain([1.]).collect();
    let mut m = 2;
    loop {
        let mut knots = vec![0.;du+1];
        for (k,w) in ends.windows(2).enumerate() {
            knots.extend((1..m).map(|i| w[0]+(w[1]-w[0])*i as f64/m as f64));
            knots.extend(std::iter::repeat(w[1]).take(if k+2 == ends.len() { du+1 } else { du }));
        }
        let tv: Vec<f64> = (0..=nv).map(|j| j as f64/nv as f64).collect();
        let rows: Vec<BSpline> = tv.iter().map(|&v| interpolate_on(&|u| f(u,v),du,knots.clone())).collect::<Option<_>>()?;
        let count = rows[0].poles.len();
        let cols: Vec<BSpline> = (0..count).map(|i| interpolate(&rows.iter().map(|r| r.poles[i]).collect::<Vec<_>>(),&tv,dv))
            .collect::<Option<_>>()?;
        let net = Net {du,dv:cols[0].degree,uknots:knots.clone(),vknots:cols[0].knots.clone(),
            poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
        let off = |u: f64,v: f64| crate::util::space::distance(net.point(u,v),f(u,v));
        let us: Vec<f64> = distinct(&knots).into_iter().map(|k| k.0).collect();
        let (mut eu,mut ev) = (0f64,0f64);
        for w in us.windows(2) { for &v in &tv { eu = eu.max(off((w[0]+w[1])/2.,v)); } }
        for j in 0..nv { for &u in &us { ev = ev.max(off(u,(tv[j]+tv[j+1])/2.)); } }
        let (grow_u,grow_v) = (eu > tol && m < 64,ev > tol && nv < 512);
        if !grow_u && !grow_v { return Some((net,eu.max(ev))) }
        if grow_u { m *= 2; }
        if grow_v { nv *= 2; }
    }
}

/// The curve of degree `d` through `f` on a uniform grid of `[0, 1]`, refined as `fit_net` is.
pub fn fit_curve(f: &dyn Fn(f64) -> V,d: usize,mut n: usize,tol: f64) -> Option<(BSpline,f64)> {
    loop {
        let t: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let s = interpolate(&t.iter().map(|&t| f(t)).collect::<Vec<_>>(),&t,d)?;
        let err = (0..n).map(|i| { let m = (t[i]+t[i+1])/2.; crate::util::space::distance(s.point(m),f(m)) }).fold(0.,f64::max);
        if err <= tol || n >= 4096 { return Some((s,err)) }
        n *= 2;
    }
}

impl BSpline {
    /// **The same curve run the other way**: the point at `t` is the old one at `a + b − t`, `a`
    /// and `b` its outer knots (a clamped curve's domain), its knots reflected, the outer ones
    /// exactly, and its poles (and weights) in reverse.
    pub fn reversed(&self) -> BSpline {
        let (a,b) = (self.knots[0],self.knots[self.knots.len()-1]);
        let knots = self.knots.iter().rev().map(|&k| if k == a { b } else if k == b { a } else { a+b-k }).collect();
        let poles = self.poles.iter().rev().copied().collect();
        let weights = self.weights.as_ref().map(|w| w.iter().rev().copied().collect());
        BSpline {degree:self.degree,knots,poles,weights}
    }
    /// **The same curve over `[a, b]`**: its knots carried there by the affine map of its domain
    /// onto it, the ends exactly `a` and `b`, the poles untouched.
    pub fn over(&self,[a,b]: [f64;2]) -> BSpline {
        let [lo,hi] = self.domain();
        let knots = self.knots.iter().map(|&k| {
            if k <= lo { a } else if k >= hi { b } else { a+(b-a)*((k-lo)/(hi-lo)) }
        }).collect();
        BSpline {knots,..self.clone()}
    }
    /// **The same curve as a B-spline of degree `by` higher** (degree elevation), exactly: each
    /// distinct knot once more, so it is as continuous at each as it was, a rational curve's in
    /// homogeneous coordinates. Split into its Bézier pieces (`insert`), each piece raised by the
    /// Bernstein identity, and each inner knot taken out again (`remove`) as often as it went in,
    /// which the raised curve's continuity there makes exact. Refused past `MAX_DEGREE`.
    pub fn elevated(&self,by: usize) -> Result<BSpline,String> {
        if by == 0 { return Ok(self.clone()) }
        if self.degree+by > MAX_DEGREE { return Err(format!("a B-spline raised past degree {MAX_DEGREE}")) }
        let c = self.segment(self.domain());
        let (knots,h) = elevate(&c.knots,&c.lifted(),c.degree,by);
        Ok(BSpline {degree:c.degree+by,..c.dropped(knots,h)})
    }
}

/// `n` choose `k`, as a float.
fn choose(n: usize,k: usize) -> f64 { (0..k).fold(1.,|c,i| c*(n-i) as f64/(i+1) as f64) }

/// A clamped B-spline of degree `p` along its first index raised `by` degrees, exactly (see
/// `BSpline::elevated`): its knots each once more for each degree, the poles of the same curve.
fn elevate<const N: usize>(knots: &[f64],poles: &[Vec<[f64;N]>],p: usize,by: usize) -> (Vec<f64>,Vec<Vec<[f64;N]>>) {
    let d = distinct(&knots[p..knots.len()-p]);
    let inner = &d[1..d.len()-1];
    // the Bézier pieces: every inner knot `p` times
    let (mut k,mut q) = (knots.to_vec(),poles.to_vec());
    for &(t,m) in inner { for _ in m..p { insert(&mut k,&mut q,p,t); } }
    // each piece raised (Q_i = Σ_j C(p, j) C(by, i − j) / C(p + by, i) P_j), joined at its first pole
    let e = p+by;
    let mut raised: Vec<Vec<[f64;N]>> = Vec::new();
    for s in 0..=inner.len() {
        let piece = &q[s*p..=s*p+p];
        for i in (if s == 0 { 0 } else { 1 })..=e {
            let mut row = vec![[0.;N];piece[0].len()];
            for j in i.saturating_sub(by)..=p.min(i) {
                let c = choose(p,j)*choose(by,i-j)/choose(e,i);
                for (out,x) in row.iter_mut().zip(&piece[j]) { for c2 in 0..N { out[c2] += c*x[c2]; } }
            }
            raised.push(row);
        }
    }
    let mut k: Vec<f64> = distinct(&k).into_iter().flat_map(|(t,m)| std::iter::repeat_n(t,m+by)).collect();
    // each inner knot back to its multiplicity, one more than it was
    for &(t,m) in inner { for _ in m..p { remove(&mut k,&mut raised,e,t); } }
    (k,raised)
}

/// Knot `t` taken out of a B-spline of degree `p` once, where the curve is smooth enough there
/// that it is the same curve without it (Piegl and Tiller A5.8, `insert` undone): the poles it
/// moved solved for from either end, the left half from the left and the right from the right,
/// so neither divides by a small weight of the insertion.
fn remove<const N: usize>(knots: &mut Vec<f64>,poles: &mut Vec<Vec<[f64;N]>>,p: usize,t: f64) {
    let r = knots.iter().rposition(|&x| x == t).unwrap();
    let s = knots.iter().filter(|&&x| x == t).count();
    let (first,last) = (r-p,r-s);
    // P_i = α_i Q_i + (1 − α_i) Q_{i−1} for i in first..=last, Q_{first−1} and Q_last known
    let alpha = |i: usize| (t-knots[i])/(knots[i+p+1]-knots[i]);
    let mix = |x: &Vec<[f64;N]>,a: f64,y: &Vec<[f64;N]>,b: f64| -> Vec<[f64;N]> {
        x.iter().zip(y).map(|(x,y)| std::array::from_fn(|c| a*x[c]+b*y[c])).collect()
    };
    let count = last-first;
    let left = count.div_ceil(2);
    let mut q: Vec<Vec<[f64;N]>> = vec![Vec::new();count];
    for i in first..first+left {
        let a = alpha(i);
        let before = if i == first { &poles[first-1] } else { &q[i-1-first] };
        q[i-first] = mix(&poles[i],1./a,before,-(1.-a)/a);
    }
    for i in (first+left+1..=last).rev() {
        let a = alpha(i);
        let after = if i == last { &poles[last+1] } else { &q[i-first] };
        q[i-1-first] = mix(&poles[i],1./(1.-a),after,-a/(1.-a));
    }
    poles.splice(first..=last,q);
    knots.remove(r);
}

/// **The bilinearly blended Coons patch of four curves closing a loop**, as a net whose boundary
/// rows are the curves exactly. The curves run end to end round the loop: the first along `u` at
/// `v = 0`, the second along `v` at `u = 1`, the third back along `u` at `v = 1` and the fourth
/// back along `v` at `u = 0`, each one's parameter carried onto `[0, 1]` (`BSpline::over`). The
/// patch is the sum of the ruled surfaces between opposite curves less the bilinear patch of the
/// corners, `S(u, v) = (1 − v) c₀(u) + v c₂(u) + (1 − u) d₀(v) + u d₁(v) − B(u, v)`, each blend
/// linear.
///
/// Opposite curves are made one space (the lower raised to the other's degree, `elevated`, and
/// each given the other's knots), so each ruled surface is a net on the patch's knots, its linear
/// direction's poles at the other's Greville abscissae, where a spline of any degree is linear;
/// the patch's net is the three nets' sum, and its boundary rows the curves' own poles. Refused
/// where a curve is rational (a sum of rational curves is no NURBS on their weights) or the loop
/// does not close within `geom::tolerance` of its poles' box.
pub fn coons(edges: [&BSpline;4]) -> Result<Net,String> {
    if edges.iter().any(|c| c.is_rational()) { return Err("no Coons patch of rational curves".into()) }
    // each clamped, so its end poles are its ends
    let edges = edges.map(|c| c.segment(c.domain()));
    let last = |c: &BSpline| c.poles[c.poles.len()-1];
    let tol = super::geom::tolerance(super::geom::diagonal(super::geom::hull(edges.iter().flat_map(|c| c.poles.iter().copied()))));
    if let Some(k) = (0..4).position(|k| crate::util::space::distance(last(&edges[k]),edges[(k+1)%4].poles[0]) > tol) {
        return Err(format!("no Coons patch of curves whose loop is open: curve {k} ends where curve {} does not start",(k+1)%4))
    }
    let unit = [0.,1.];
    let (c0,c2) = compatible(&edges[0].over(unit),&edges[2].reversed().over(unit))?;
    let (d0,d1) = compatible(&edges[3].reversed().over(unit),&edges[1].over(unit))?;
    let abscissae = |c: &BSpline| -> Vec<f64> { (0..c.poles.len()).map(|i| greville(&c.knots,c.degree,i)).collect() };
    let (xi,eta) = (abscissae(&c0),abscissae(&d0));
    let (p00,p10,p01,p11) = (c0.poles[0],last(&c0),c2.poles[0],last(&c2));
    let (nu,nv) = (xi.len(),eta.len());
    let poles: Vec<Vec<V>> = (0..nu).map(|i| (0..nv).map(|j| {
        if j == 0 { return c0.poles[i] }
        if j == nv-1 { return c2.poles[i] }
        if i == 0 { return d0.poles[j] }
        if i == nu-1 { return d1.poles[j] }
        let (s,t) = (xi[i],eta[j]);
        std::array::from_fn(|k| (1.-t)*c0.poles[i][k]+t*c2.poles[i][k]+(1.-s)*d0.poles[j][k]+s*d1.poles[j][k]
            -((1.-s)*(1.-t)*p00[k]+s*(1.-t)*p10[k]+(1.-s)*t*p01[k]+s*t*p11[k]))
    }).collect()).collect();
    let net = Net {du:c0.degree,dv:d0.degree,uknots:c0.knots,vknots:d0.knots,poles,weights:None};
    net.check()?;
    Ok(net)
}

/// Two curves over one domain made one space: the lower raised to the other's degree, and each
/// given every inner knot of the other as many times as the other has it.
fn compatible(a: &BSpline,b: &BSpline) -> Result<(BSpline,BSpline),String> {
    let p = a.degree.max(b.degree);
    let (a,b) = (a.elevated(p-a.degree)?,b.elevated(p-b.degree)?);
    // each lifted once, given every inner knot of the other it lacks, and dropped once
    let given = |c: &BSpline,other: &BSpline| {
        let (mut k,mut h) = (c.knots.clone(),c.lifted());
        let d = distinct(&other.knots);
        for &(t,m) in &d[1..d.len()-1] {
            for _ in c.knots.iter().filter(|&&x| x == t).count()..m { insert(&mut k,&mut h,p,t); }
        }
        c.dropped(k,h)
    };
    Ok((given(&a,&b),given(&b,&a)))
}

/// How a grid's parameters are spread: evenly, by chord length, or by its square root.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Parametrization { Even,ChordLength,Centripetal }

/// The cubic surface (lower where a direction has fewer than four points) through a row-major
/// grid of `rows × columns` points, rows along `u`: each direction's parameters from 0 to 1, spread
/// by `kind` along each column (or row) and averaged over them, so every row is interpolated at
/// one set of parameters; the rows interpolated along `v`, then their poles along `u`.
pub fn interpolate_net(points: &[V],rows: usize,columns: usize,kind: Parametrization) -> Option<Net> {
    let (tu,tv) = net_parameters(points,rows,columns,kind)?;
    let at = |i: usize,j: usize| points[i*columns+j];
    let rows_fit: Vec<BSpline> = (0..rows).map(|i| interpolate(&(0..columns).map(|j| at(i,j)).collect::<Vec<_>>(),&tv,3)).collect::<Option<_>>()?;
    let nv = rows_fit[0].poles.len();
    let cols_fit: Vec<BSpline> = (0..nv).map(|j| interpolate(&rows_fit.iter().map(|r| r.poles[j]).collect::<Vec<_>>(),&tu,3)).collect::<Option<_>>()?;
    let poles: Vec<Vec<V>> = (0..cols_fit[0].poles.len()).map(|i| cols_fit.iter().map(|c| c.poles[i]).collect()).collect();
    Some(Net {du:cols_fit[0].degree,dv:rows_fit[0].degree,uknots:cols_fit[0].knots.clone(),vknots:rows_fit[0].knots.clone(),poles,weights:None})
}

/// `interpolate_net`'s surface creased along the rows `creases` (each strictly between the first
/// and the last): the rows interpolated along `v` as there, and each stretch of their poles between
/// two creases (or an end) along `u` at its own rows' parameters, cubic, the stretches joined at
/// their shared row by a knot `degree` times over. The surface passes through every row at its
/// `net_parameters` `u` as `interpolate_net`'s does, and is only continuous across a crease, where
/// each side is the smooth fit of its own stretch. None where a stretch has fewer than four rows.
pub fn interpolate_creased(points: &[V],rows: usize,columns: usize,kind: Parametrization,creases: &[usize]) -> Option<Net> {
    if creases.is_empty() { return interpolate_net(points,rows,columns,kind) }
    let (tu,tv) = net_parameters(points,rows,columns,kind)?;
    let at = |i: usize,j: usize| points[i*columns+j];
    let rows_fit: Vec<BSpline> = (0..rows).map(|i| interpolate(&(0..columns).map(|j| at(i,j)).collect::<Vec<_>>(),&tv,3)).collect::<Option<_>>()?;
    let nv = rows_fit[0].poles.len();
    let mut ends: Vec<usize> = std::iter::once(0).chain(creases.iter().copied()).chain(std::iter::once(rows-1)).collect();
    ends.dedup();
    if ends.windows(2).any(|w| w[1] < w[0]+3) { return None }
    // each stretch's columns of poles, cubic over its own rows' parameters
    let (mut uknots,mut poles): (Vec<f64>,Vec<Vec<V>>) = (Vec::new(),Vec::new());
    for (k,w) in ends.windows(2).enumerate() {
        let t = &tu[w[0]..=w[1]];
        let cols: Vec<BSpline> = (0..nv).map(|j| interpolate(&rows_fit[w[0]..=w[1]].iter().map(|r| r.poles[j]).collect::<Vec<_>>(),t,3))
            .collect::<Option<_>>()?;
        let count = cols[0].poles.len();
        // each stretch's knots but the last of its last end's four and all of its first end's after
        // the first stretch, so a joint stands three times over; its first pole is the last one's
        // before it
        let knots = &cols[0].knots;
        let (from,to) = (if k == 0 { 0 } else { 4 },knots.len()-if k+2 == ends.len() { 0 } else { 1 });
        uknots.extend_from_slice(&knots[from..to]);
        poles.extend((if k == 0 { 0 } else { 1 }..count).map(|i| cols.iter().map(|c| c.poles[i]).collect()));
    }
    let net = Net {du:3,dv:rows_fit[0].degree,uknots,vknots:rows_fit[0].knots.clone(),poles,weights:None};
    net.check().ok()?;
    Some(net)
}

/// The parameters `interpolate_net` interpolates a grid at: each row's `u` and each column's `v`,
/// so a row of the grid is the net's iso-curve at its `u` (#212).
pub fn net_parameters(points: &[V],rows: usize,columns: usize,kind: Parametrization) -> Option<(Vec<f64>,Vec<f64>)> {
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
    let tu = spread(rows,columns,&|j,i| crate::util::space::distance(at(i-1,j),at(i,j)))?;
    let tv = spread(columns,rows,&|i,j| crate::util::space::distance(at(i,j-1),at(i,j)))?;
    Some((tu,tv))
}

/// The fewest poles a curve has for its spans to be found through a tree (`SpanIndex::tree`):
/// fewer are walked nearest box first, as `nearest` walks them.
const INDEXED_SPANS: usize = 32;

/// How many samples `nearest` reads of each span: its ends and fifteen between.
const SAMPLES: usize = 17;

/// How far `p` is from the box `x` (nought within it).
fn gap_to(x: crate::util::bvh::Bounds<3>,p: V) -> f64 {
    crate::util::space::norm(std::array::from_fn(|k| (x.lo[k]-p[k]).max(p[k]-x.hi[k]).max(0.)))
}

/// **A curve's spans, each boxed by its poles**: each item a span's first knot's index, each span's
/// samples (`BSpline::samples`), read once for the curve, and where it has `INDEXED_SPANS` poles or
/// more, the boxes in a bounding-volume hierarchy.
struct SpanIndex { spans: Vec<usize>,boxes: Vec<crate::util::bvh::Bounds<3>>,samples: Vec<[V;SAMPLES]>,tree: Option<crate::util::bvh::Bvh<3>> }

thread_local! {
    /// The indexes each thread asked for last (`Kept`).
    static HELD_INDEXES: Held<SpanIndex> = const { std::cell::RefCell::new(([const { None };HELD],0)) };
}

/// The indexes of curves' spans (`Kept`).
static INDEXES: Kept<SpanIndex> = Kept::new();

/// The index of `b`'s spans, built once for the curve and kept (`Kept`).
fn span_index(b: &std::sync::Arc<BSpline>) -> std::sync::Arc<SpanIndex> {
    INDEXES.get(&HELD_INDEXES,b,|b| {
        let spans: Vec<usize> = (b.degree..b.poles.len()).filter(|&s| b.knots[s] < b.knots[s+1]).collect();
        let boxes: Vec<crate::util::bvh::Bounds<3>> = spans.iter().map(|&s| b.span_box(s)).collect();
        let samples = spans.iter().map(|&s| b.samples::<4>(s)).collect();
        let tree = (b.poles.len() >= INDEXED_SPANS).then(|| crate::util::bvh::Bvh::new(boxes.iter().copied()));
        SpanIndex {tree,boxes,samples,spans}
    })
}

/// How many curves a thread holds what it was last asked for of (`Held`).
pub(crate) const HELD: usize = 16;

/// What a thread holds of what it was last asked for of curves (`Kept`): each with its curve.
pub(crate) type Held<T> = std::cell::RefCell<([Option<(std::sync::Arc<BSpline>,std::sync::Arc<T>)>;HELD],usize)>;

/// **What is read once for a curve and kept for it** (a span index, `span_index`; its spans' whole
/// Bézier forms, `enclose`): keyed by the curve, held so its address is no other's, in a store
/// every thread shares — emptied past `KEPT` curves — and the last few each thread asked for held
/// on the thread too, read without taking the store's lock, which every thread asks at every
/// reading.
pub(crate) struct Kept<T> { store: std::sync::RwLock<std::collections::BTreeMap<usize,(std::sync::Arc<BSpline>,std::sync::Arc<T>)>> }

/// How many curves' readings a store keeps at once (`Kept`): past it the store is emptied.
const KEPT: usize = 1024;

impl<T> Kept<T> {
    pub(crate) const fn new() -> Self { Kept {store:std::sync::RwLock::new(std::collections::BTreeMap::new())} }
    /// What is kept for `b`, `make` of it where nothing is: from what this thread holds (`held`),
    /// or from the store, or made and put in both.
    pub(crate) fn get(&self,held: &'static std::thread::LocalKey<Held<T>>,b: &std::sync::Arc<BSpline>,make: impl FnOnce(&BSpline) -> T)
        -> std::sync::Arc<T> {
        let found = held.with(|c| c.borrow().0.iter().flatten().find(|(c,_)| std::sync::Arc::ptr_eq(c,b)).map(|e| e.1.clone()));
        if let Some(x) = found { return x }
        let key = std::sync::Arc::as_ptr(b) as usize;
        let stored = self.store.read().unwrap().get(&key).map(|e| e.1.clone());
        let x = stored.unwrap_or_else(|| {
            let x = std::sync::Arc::new(make(b));
            let mut store = self.store.write().unwrap();
            if store.len() >= KEPT { store.clear() }
            store.insert(key,(b.clone(),x.clone()));
            x
        });
        held.with(|c| {
            let (slots,next) = &mut *c.borrow_mut();
            slots[*next] = Some((b.clone(),x.clone()));
            *next = (*next+1)%HELD;
        });
        x
    }
}
