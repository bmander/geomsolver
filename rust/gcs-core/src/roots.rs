//! Roots and minima of functions of one variable, a bisection over any bracket, and the
//! minimum-norm Newton step onto up to three zero sets in space: the searches the field's readings,
//! its creases and the refinement's crossings share.

/// A root of `f` in the bracket `[a, b]` (`fa`, `fb` of strictly opposite
/// signs), to within `width`: regula falsi with the Illinois fix, which keeps
/// a bracket like bisection but converges superlinearly, a handful of steps
/// where bisection takes one per bit. A step that leaves the domain (`f`
/// gives none) ends the search at the bracket's middle, as bisection did.
pub fn bracketed_root(mut f: impl FnMut(f64) -> Option<f64>,mut a: f64,mut fa: f64,mut b: f64,mut fb: f64,width: f64) -> f64 {
    let mut side = 0;
    for _ in 0..64 {
        if (b-a).abs() <= width { break; }
        let mut c = (a*fb-b*fa)/(fb-fa);
        // a secant step outside the bracket (a flat or huge difference) bisects
        if !(c > a.min(b) && c < a.max(b)) { c = 0.5*(a+b); }
        let Some(fc) = f(c) else { break };
        if fc == 0. { return c; }
        if (fc < 0.) == (fb < 0.) {
            b = c; fb = fc;
            if side == -1 { fa *= 0.5; }
            side = -1;
        } else {
            a = c; fa = fc;
            if side == 1 { fb *= 0.5; }
            side = 1;
        }
    }
    0.5*(a+b)
}

/// The least of `f` on [lo, hi] by Brent's method — parabolic steps through the three best points,
/// golden section where a parabola is not to be trusted — until the bracket is within `xtol` or
/// `stop` accepts a value and how far below it the minimum may lie: `(value, argument)`. The
/// minimum of a smooth function is found in a few steps where golden section alone takes one per
/// 0.62 of the bracket.
///
/// How far below the best value the minimum may lie is read off the parabola through the three
/// best points: its curvature times the bracket's width squared, over two — infinite until three
/// points make a parabola that holds water. A reading, like golden section's, not a bound.
pub fn brent(f: &impl Fn(f64) -> f64,lo: f64,hi: f64,xtol: f64,steps: usize,stop: impl Fn(f64,f64) -> bool) -> (f64,f64) {
    const GOLD: f64 = 0.381_966_011_250_105_1;
    let (mut a,mut b) = (lo,hi);
    let mut x = a+GOLD*(b-a);
    let (mut w,mut v) = (x,x);
    let mut fx = f(x);
    let (mut fw,mut fv) = (fx,fx);
    let (mut d,mut e) = (0f64,0f64);
    let slack = |x: f64,fx: f64,w: f64,fw: f64,v: f64,fv: f64,width: f64| -> f64 {
        if x == w || x == v || w == v { return f64::INFINITY; }
        let curvature = 2.*((fw-fx)/(w-x)-(fv-fx)/(v-x))/(w-v);
        if curvature > 0. && curvature.is_finite() { 0.5*curvature*width*width } else { f64::INFINITY }
    };
    for _ in 0..steps {
        if stop(fx,slack(x,fx,w,fw,v,fv,b-a)) { break; }
        let xm = 0.5*(a+b);
        let (tol1,tol2) = (xtol,2.*xtol);
        if (x-xm).abs() <= tol2-0.5*(b-a) { break; }
        let mut golden = true;
        if e.abs() > tol1 {
            let r = (x-w)*(fx-fv);
            let mut q = (x-v)*(fx-fw);
            let mut p = (x-v)*q-(x-w)*r;
            q = 2.*(q-r);
            if q > 0. { p = -p; }
            q = q.abs();
            let previous = e;
            e = d;
            if p.abs() < (0.5*q*previous).abs() && p > q*(a-x) && p < q*(b-x) {
                d = p/q;
                let u = x+d;
                if u-a < tol2 || b-u < tol2 { d = tol1.copysign(xm-x); }
                golden = false;
            }
        }
        if golden { e = if x >= xm { a-x } else { b-x }; d = GOLD*e; }
        let u = if d.abs() >= tol1 { x+d } else { x+tol1.copysign(d) };
        let fu = f(u);
        if fu <= fx {
            if u >= x { a = x } else { b = x }
            (v,fv,w,fw,x,fx) = (w,fw,x,fx,u,fu);
        } else {
            if u < x { a = u } else { b = u }
            if fu <= fw || w == x { (v,fv,w,fw) = (w,fw,u,fu); }
            else if fu <= fv || v == x || v == w { (v,fv) = (u,fu); }
        }
    }
    (fx,x)
}

/// Bisect the bracket `[lo, hi]` while `wide` says it is: `keep` is asked of each middle, and
/// `true` moves `lo` there, `false` moves `hi`. The bracket as it ends. `mid` is the caller's,
/// so a bracket of points is halved where the caller halves it.
pub fn bisect<T: Copy>(mut lo: T,mut hi: T,mid: impl Fn(T,T) -> T,wide: impl Fn(T,T) -> bool,
    mut keep: impl FnMut(T) -> bool) -> (T,T) {
    while wide(lo,hi) {
        let m = mid(lo,hi);
        if keep(m) { lo = m } else { hi = m }
    }
    (lo,hi)
}

/// The least move that puts a point on two or three linearised zero sets at once: the values
/// `values` with gradients `rows`, the step `Jᵀ(JJᵀ)⁻¹F` to subtract (for three, `J⁻¹F`, by the
/// adjugate). `None` where the gradients are dependent: two within `1e-8` of parallel, as the
/// determinant of their Gram matrix says against the product of their lengths squared, three
/// within `1e-6` of coplanar against the product of their lengths.
pub fn least_norm_step(rows: &[[f64;3]],values: &[f64]) -> Option<[f64;3]> {
    use crate::space::{dot,cross,norm};
    match (rows,values) {
        (&[ga,gb],&[va,vb]) => {
            let (aa,ab,bb) = (dot(ga,ga),dot(ga,gb),dot(gb,gb));
            let det = aa*bb-ab*ab;
            if !(det > 1e-8*aa*bb) { return None; }
            let (l0,l1) = ((bb*va-ab*vb)/det,(aa*vb-ab*va)/det);
            Some(std::array::from_fn(|k| l0*ga[k]+l1*gb[k]))
        }
        (&[a,b,c],&[..]) => {
            let det = dot(a,cross(b,c));
            let scale = norm(a)*norm(b)*norm(c);
            if !(det.abs() > 1e-6*scale) { return None; }
            // Cramer's rule through the adjugate: J⁻¹ = [b×c, c×a, a×b]ᵀ / det.
            let columns = [cross(b,c),cross(c,a),cross(a,b)];
            Some(std::array::from_fn(|i| (0..3).map(|k| columns[k][i]*values[k]).sum::<f64>()/det))
        }
        _ => None,
    }
}

/// Newton onto several zero sets, each step `least_norm_step`'s, none longer than `limit`: `read`
/// gives the values and gradients at a point (or `None`, which ends the search), `settled` says
/// when the values are near enough zero. The point, or `None` where the gradients go dependent,
/// a step is not finite, or forty steps do not settle.
pub fn newton_onto<const N: usize>(mut p: [f64;3],limit: f64,mut read: impl FnMut([f64;3]) -> Option<[([f64;3],f64);N]>,
    settled: impl Fn(&[([f64;3],f64);N]) -> bool) -> Option<[f64;3]> {
    for _ in 0..40 {
        let at = read(p)?;
        if settled(&at) { return Some(p); }
        let mut step = least_norm_step(&at.map(|r| r.0),&at.map(|r| r.1))?;
        let length = crate::space::norm(step);
        if !length.is_finite() { return None; }
        if length > limit { step = step.map(|x| x*limit/length); }
        p = crate::space::sub(p,step);
    }
    None
}
