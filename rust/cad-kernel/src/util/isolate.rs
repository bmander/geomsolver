//! **Certified root isolation** (#777 M1): the zeros of a function over a stretch, or of a system
//! over a box, found from bounds on its derivatives there — never from samples, so no root is
//! missed for falling between two of them, however close two roots stand (a thin sheet's two
//! crossings a hair apart).
//!
//! **One unknown** (`roots`): over a stretch about its middle `m`, half-width `h`, a function whose
//! first derivative is bounded by `D₁` there cannot reach nought where `|G(m)| > D₁ h`, so the
//! stretch is dropped; where its second derivative is bounded by `D₂` and `|G'(m)| > D₂ h`, `G` is
//! monotone across it and holds one root at most, bracketed by a sign change at its ends and
//! settled by bisection; otherwise the stretch is halved. A stretch that cannot be parted before it
//! is `width` wide is a double root, a touch, where `near` accepts its middle (the function nought
//! there to the caller's tolerance). Written by hand three times on #776 — a line against a screwed
//! rim's sheet, a helix against a ball, a circle's trigonometric polynomial — and now this one
//! routine.
//!
//! **Several unknowns** (`solve`): the same over a box of `N` unknowns, by the theorem of
//! Kantorovich. At the box's middle `m` (half-width `r` in the largest-coordinate norm), with each
//! equation's second derivatives bounded over the box (`Hᵢ`, the sum of its Hessian's entries'
//! magnitudes), equation `i` stays off nought across the box where `|Fᵢ(m)| > |∇Fᵢ(m)|₁ r + ½ Hᵢ r²`
//! and the box is dropped; where the Jacobian `J` at `m` is invertible, with `β = ‖J⁻¹‖`, `η =
//! ‖J⁻¹ F(m)‖` and `K = maxᵢ Hᵢ` (the Jacobian's Lipschitz constant over the box), `h = β K η ≤ ½`
//! puts exactly one root within `ρ₋ = (1 − √(1 − 2h)) / (β K)` of `m` and no other within `ρ₊ = (1 +
//! √(1 − 2h)) / (β K)`: where `ρ₋ ≤ r ≤ ρ₊` the box holds that one root and no other, settled by
//! Newton's method from `m`. Otherwise the box is halved across its widest side. A box that cannot
//! be parted before it is `width` wide holds a singular root — two surfaces touching — where `near`
//! accepts its middle.
//!
//! Each stretch or box read is counted against the build's budget (`util::meter`, the ray stage):
//! past it, or past `MOST` boxes, the search gives up (`None`) and says so, never a guess.

type Root = super::roots::Root;

/// The most stretches or boxes one search reads.
pub const MOST: usize = 1 << 16;

/// **The roots of `g` over `[t0, t1]`**, each isolated with a bound: `bound(a, b)` bounds `|g'|`
/// and `|g''|` over `[a, b]`; `dg` is `g'`. A stretch narrower than `width` that cannot be parted
/// holds a touch where `near(t)` says `g` is nought there. Each root once, in order; an end of the
/// stretch where `g` is nought a root there. `None` where the search read more than `MOST`
/// stretches or the build stopped.
pub fn roots(g: &dyn Fn(f64) -> f64,dg: &dyn Fn(f64) -> f64,bound: &dyn Fn(f64,f64) -> [f64;2],[t0,t1]: [f64;2],width: f64,
    near: &dyn Fn(f64) -> bool) -> Option<Vec<Root>> {
    let mut out: Vec<Root> = Vec::new();
    if !(t1 >= t0) { return Some(out) }
    let width = width.max(1e-15*(t0.abs()+t1.abs()).max(1e-300));
    let mut stack = vec![(t0,t1,0usize)];
    let mut read = 0;
    while let Some((a,b,depth)) = stack.pop() {
        read += 1;
        if read > MOST || !super::meter::tick(super::meter::Stage::Ray) { return None }
        let (m,h) = (0.5*(a+b),0.5*(b-a));
        let [d1,d2] = bound(a,b);
        let gm = g(m);
        // nought nowhere across the stretch: but where it comes within the tolerance of nought and
        // turns back there (`near` at the turn, `g'` changing sign), a touch to the tolerance — a
        // line passing a hair from tangent, which the caller reads as touching, as a sampled
        // search did
        if gm.abs() > d1*h {
            let (da,db) = (dg(a),dg(b));
            if (da < 0.) != (db < 0.) && (gm.abs() <= d1*h*4. || near(m)) {
                let at = super::roots::bracketed_root(|t| Some(dg(t)),a,da,b,db,width*1e-3);
                if near(at) { out.push(Root {at,touch:true}) }
            }
            continue
        }
        // monotone across it: one root at most, where its ends differ in sign
        if dg(m).abs() > d2*h {
            let (ga,gb) = (g(a),g(b));
            if ga == 0. { out.push(Root {at:a,touch:false}) }
            if ga*gb < 0. {
                let (mut l,mut r,mut gl) = (a,b,ga);
                for _ in 0..200 {
                    if r-l <= width*1e-3 { break }
                    let s = 0.5*(l+r);
                    let gs = g(s);
                    if gs == 0. { l = s; r = s; break }
                    if (gs < 0.) == (gl < 0.) { l = s; gl = gs } else { r = s }
                }
                out.push(Root {at:0.5*(l+r),touch:false});
            }
            continue
        }
        // too narrow to part: a double root where the function is nought there
        if b-a <= width || depth >= 96 {
            if near(m) { out.push(Root {at:m,touch:true}) }
            continue
        }
        stack.push((m,b,depth+1));
        stack.push((a,m,depth+1));
    }
    // an end where the function is nought is a root there
    if g(t1) == 0. { out.push(Root {at:t1,touch:false}) }
    out.sort_by(|x,y| x.at.total_cmp(&y.at));
    // one root once: the narrow stretches a touch leaves beside one another are one touch, placed
    // where `g'` changes sign among them (a double root is a simple root of the derivative, placed
    // to its rounding, never a least `|g|`), and a crossing within a width of one is that crossing
    let mut kept: Vec<Root> = Vec::new();
    let mut k = 0;
    while k < out.len() {
        if !out[k].touch {
            if !kept.last().is_some_and(|l| (out[k].at-l.at).abs() <= 2.*width) { kept.push(out[k]) }
            else if let Some(l) = kept.last_mut() { *l = out[k] }
            k += 1;
            continue
        }
        let mut j = k;
        while j+1 < out.len() && out[j+1].touch && out[j+1].at-out[j].at <= 8.*width { j += 1 }
        let (a,b) = (out[k].at-width,out[j].at+width);
        let (da,db) = (dg(a),dg(b));
        let at = if (da < 0.) != (db < 0.) {
            super::roots::bracketed_root(|t| Some(dg(t)),a,da,b,db,width*1e-3)
        } else { 0.5*(out[k].at+out[j].at) };
        if !kept.last().is_some_and(|l| (at-l.at).abs() <= 2.*width) { kept.push(Root {at,touch:true}) }
        k = j+1;
    }
    Some(kept)
}

/// A root of a system: where it is, and whether it is singular (a touch) rather than certified
/// simple.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Point<const N: usize> { pub at: [f64;N],pub touch: bool }

/// **The roots of the system `f` over the box `domain`** (`N` unknowns, `N` equations), each
/// isolated by Kantorovich's theorem: `f(x)` gives the values and the Jacobian (rows the
/// equations), `second(box)` bounds each equation's second derivatives over a box (the sum of its
/// Hessian's entries' magnitudes). A box narrower than `width` (each side) that cannot be parted
/// holds a singular root where `near(x)` accepts its middle. Each root once. `None` where the search
/// read more than `MOST` boxes or the build stopped.
pub fn solve<const N: usize>(f: &dyn Fn([f64;N]) -> ([f64;N],[[f64;N];N]),second: &dyn Fn(&[[f64;2];N]) -> [f64;N],
    domain: [[f64;2];N],width: f64,near: &dyn Fn([f64;N]) -> bool) -> Option<Vec<Point<N>>> {
    let mut out: Vec<Point<N>> = Vec::new();
    if domain.iter().any(|s| !(s[1] >= s[0])) { return Some(out) }
    let mut stack = vec![(domain,0usize)];
    let mut read = 0;
    while let Some((bx,depth)) = stack.pop() {
        read += 1;
        if read > MOST || !super::meter::tick(super::meter::Stage::Ray) { return None }
        let m: [f64;N] = std::array::from_fn(|k| 0.5*(bx[k][0]+bx[k][1]));
        let r = (0..N).map(|k| 0.5*(bx[k][1]-bx[k][0])).fold(0.,f64::max);
        let (fm,jm) = f(m);
        let hs = second(&bx);
        // an equation off nought across the box (each unknown within its own half-width)
        let off = (0..N).any(|i| {
            let reach: f64 = (0..N).map(|k| jm[i][k].abs()*0.5*(bx[k][1]-bx[k][0])).sum();
            fm[i].abs() > reach+0.5*hs[i]*r*r
        });
        if off { continue }
        let inv = inverse(jm);
        // and preconditioned (Krawczyk's): `G = J(m)⁻¹ F` has `∇G = I` at the middle, so each of its
        // components moves across the box by its own half-width at first order, and by the second
        // derivatives' bound carried through `J(m)⁻¹` beyond — a box whose Newton step reaches past
        // that on any unknown holds no root. Far stronger than `F`'s own components where they
        // couple (a ray against a long sheet), it drops what they leave standing
        if let Some(c) = &inv {
            let g: [f64;N] = std::array::from_fn(|i| (0..N).map(|k| c[i][k]*fm[k]).sum());
            let far = (0..N).any(|i| {
                let bend: f64 = (0..N).map(|k| c[i][k].abs()*hs[k]).sum();
                g[i].abs() > 0.5*(bx[i][1]-bx[i][0])+0.5*bend*r*r
            });
            if far { continue }
        }
        // one root, and no other, in the box: Kantorovich at its middle
        if let Some(inv) = inv {
            let beta = norm_inf(&inv);
            let step: [f64;N] = std::array::from_fn(|i| (0..N).map(|k| inv[i][k]*fm[k]).sum());
            let eta = step.iter().fold(0f64,|a,&s| a.max(s.abs()));
            let big = hs.iter().fold(0f64,|a,&h| a.max(h));
            let bk = beta*big;
            let h = bk*eta;
            let (lo,hi) = if bk > 0. {
                let s = (1.-2.*h).max(0.).sqrt();
                ((1.-s)/bk,(1.+s)/bk)
            } else { (eta,f64::INFINITY) };
            if h <= 0.5 && lo <= r && r <= hi {
                // Newton's method from the middle converges to it, within `lo` of the middle
                let mut x = m;
                for _ in 0..60 {
                    let (fx,jx) = f(x);
                    let Some(ix) = inverse(jx) else { break };
                    let dx: [f64;N] = std::array::from_fn(|i| (0..N).map(|k| ix[i][k]*fx[k]).sum());
                    let next: [f64;N] = std::array::from_fn(|i| x[i]-dx[i]);
                    let moved = (0..N).fold(0f64,|a,i| a.max((next[i]-x[i]).abs()));
                    // held within the root's ball, where it is
                    if (0..N).any(|i| (next[i]-m[i]).abs() > lo*(1.+1e-9)+1e-300) { break }
                    x = next;
                    if moved <= 1e-15*(1.+x.iter().fold(0f64,|a,&v| a.max(v.abs()))) { break }
                }
                out.push(Point {at:x,touch:false});
                continue
            }
        }
        if (0..N).all(|k| bx[k][1]-bx[k][0] <= width) || depth >= 200 {
            if near(m) { out.push(Point {at:m,touch:true}) }
            continue
        }
        // halved across its widest side
        let k = (0..N).max_by(|&i,&j| (bx[i][1]-bx[i][0]).total_cmp(&(bx[j][1]-bx[j][0]))).unwrap_or(0);
        let (mut a,mut b) = (bx,bx);
        a[k][1] = m[k];
        b[k][0] = m[k];
        stack.push((b,depth+1));
        stack.push((a,depth+1));
    }
    // a root on the face two boxes share is found in both: once, and singular only where both are
    let mut kept: Vec<Point<N>> = Vec::new();
    for p in out {
        match kept.iter_mut().find(|q| (0..N).all(|k| (q.at[k]-p.at[k]).abs() <= 2.*width)) {
            Some(q) => { if q.touch && !p.touch { *q = p } }
            None => kept.push(p),
        }
    }
    Some(kept)
}

/// The largest row sum of `a`'s magnitudes: its norm on the largest-coordinate norm.
fn norm_inf<const N: usize>(a: &[[f64;N];N]) -> f64 { a.iter().map(|r| r.iter().map(|x| x.abs()).sum::<f64>()).fold(0.,f64::max) }

/// `a`'s inverse by elimination with partial pivoting, `None` where it is singular to rounding.
fn inverse<const N: usize>(a: [[f64;N];N]) -> Option<[[f64;N];N]> {
    let mut m = a;
    let mut inv: [[f64;N];N] = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1. } else { 0. }));
    let scale = norm_inf(&a);
    if !(scale > 0.) || !scale.is_finite() { return None }
    for c in 0..N {
        let p = (c..N).max_by(|&i,&j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if !(m[p][c].abs() > 1e-14*scale) { return None }
        m.swap(c,p);
        inv.swap(c,p);
        let d = m[c][c];
        for j in 0..N { m[c][j] /= d; inv[c][j] /= d; }
        for i in 0..N {
            if i == c { continue }
            let f = m[i][c];
            if f == 0. { continue }
            for j in 0..N { m[i][j] -= f*m[c][j]; inv[i][j] -= f*inv[c][j]; }
        }
    }
    Some(inv)
}

/// **A polynomial's value and its derivatives' bounds** (coefficients lowest first): what `roots`
/// asks of a polynomial along a curve (a torus's quartic, a surface of revolution's meridian against
/// a line). `[|p'|, |p''|]` bounded over `[a, b]` by evaluating each in interval arithmetic.
pub fn poly_bounds(c: &[f64],a: f64,b: f64) -> [f64;2] {
    let d1: Vec<f64> = c.iter().enumerate().skip(1).map(|(k,&x)| k as f64*x).collect();
    let d2: Vec<f64> = d1.iter().enumerate().skip(1).map(|(k,&x)| k as f64*x).collect();
    [range_abs(&d1,a,b),range_abs(&d2,a,b)]
}

/// The largest magnitude the polynomial `c` reaches over `[a, b]`, bounded above (Horner's rule on
/// intervals, outward a hair for rounding).
fn range_abs(c: &[f64],a: f64,b: f64) -> f64 {
    let (mut lo,mut hi) = (0f64,0f64);
    for &x in c.iter().rev() {
        let ps = [lo*a,lo*b,hi*a,hi*b];
        let (pl,ph) = (ps.iter().fold(f64::INFINITY,|m,&v| m.min(v)),ps.iter().fold(f64::NEG_INFINITY,|m,&v| m.max(v)));
        lo = pl+x;
        hi = ph+x;
    }
    lo.abs().max(hi.abs())*(1.+1e-12)+1e-300
}

/// The polynomial `c` (coefficients lowest first) at `t`.
pub fn poly(c: &[f64],t: f64) -> f64 { c.iter().rev().fold(0.,|acc,&x| acc*t+x) }

/// The polynomial `c`'s derivative at `t`.
pub fn poly_d(c: &[f64],t: f64) -> f64 { c.iter().enumerate().skip(1).rev().fold(0.,|acc,(k,&x)| acc*t+k as f64*x) }

/// The product of two polynomials (coefficients lowest first).
pub fn poly_mul(a: &[f64],b: &[f64]) -> Vec<f64> {
    let mut out = vec![0.;(a.len()+b.len()).saturating_sub(1)];
    for (i,&x) in a.iter().enumerate() { for (j,&y) in b.iter().enumerate() { out[i+j] += x*y } }
    out
}

/// The sum of polynomials, each times its weight.
pub fn poly_sum(terms: &[(f64,&[f64])]) -> Vec<f64> {
    let n = terms.iter().map(|t| t.1.len()).max().unwrap_or(0);
    let mut out = vec![0.;n];
    for &(w,p) in terms { for (k,&x) in p.iter().enumerate() { out[k] += w*x } }
    out
}
