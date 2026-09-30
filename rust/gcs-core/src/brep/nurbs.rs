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
