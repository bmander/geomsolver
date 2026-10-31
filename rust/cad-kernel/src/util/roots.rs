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

/// Where a function of one variable is least (`brent`), and its value there: named, never a pair,
/// since both are numbers and a pair read the wrong way round reads the value as the argument.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Least { pub at: f64,pub value: f64 }

/// The least of `f` on [lo, hi] by Brent's method — parabolic steps through the three best points,
/// golden section where a parabola is not to be trusted — until the bracket is within `xtol` or
/// `stop` accepts a value and how far below it the minimum may lie. The minimum of a smooth
/// function is found in a few steps where golden section alone takes one per 0.62 of the bracket.
///
/// How far below the best value the minimum may lie is read off the parabola through the three
/// best points: its curvature times the bracket's width squared, over two — infinite until three
/// points make a parabola that holds water. A reading, like golden section's, not a bound.
pub fn brent(f: &impl Fn(f64) -> f64,lo: f64,hi: f64,xtol: f64,steps: usize,stop: impl Fn(f64,f64) -> bool) -> Least {
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
    Least {at:x,value:fx}
}

/// A zero of a function sampled along an interval (`sampled_roots`): where it crosses, or where it
/// only touches — comes within the tolerance of zero and turns back.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Root { pub at: f64,pub touch: bool }

/// Where `f`, sampled at the ascending `ts` as `fs` and with derivative `df`, comes to zero within
/// `tol`, in order: a sign change between two samples bracketed and refined, a crossing; a sample
/// within `tol` a touch where its neighbours are both clear on one side, else a crossing, refined
/// between them where they are clear on either side and at the sample where they are not; and
/// a turn of `f` between samples clear on one side — beside a sampled minimum of its size, ties
/// included, `df` changing sign — refined as a root of `df`, a touch where `f` there is within
/// `tol`, two crossings either side where it is past zero. `width` is the bracket a refinement
/// stops at.
///
/// **A touch is placed as a root of `df`, never as a least `|f|`**: it is a double root of `f`,
/// flat as the square of the distance from it, so a least `|f|` is placed only to the square root
/// of `f`'s rounding (1e-7 on a ball of radius 7, wider than the tolerance its vertex is pooled
/// to), and a sample within `tol` of it may stand farther still; a simple root of `df` is placed
/// to `df`'s rounding. A touch found at a sample is moved to the turn beside it where `f` there is
/// within `tol` too. `df` is read only at a sample within `tol` or beside a sampled minimum of
/// `|f|`, so a curve clear of the surface costs nothing more than its samples.
pub fn sampled_roots(f: impl Fn(f64) -> f64,df: impl Fn(f64) -> f64,ts: &[f64],fs: &[f64],tol: f64,width: f64) -> Vec<Root> {
    let n = ts.len();
    let near = |v: f64| v.abs() <= tol;
    // where `df` changes sign between samples `i` and `i + 1`, refined
    let turn = |i: usize| -> Option<f64> {
        let (a,b) = (df(ts[i]),df(ts[i+1]));
        ((a < 0.) != (b < 0.)).then(|| bracketed_root(|t| Some(df(t)),ts[i],a,ts[i+1],b,width))
    };
    // a crossing between `t0` and `t1`, where `f` is `f0` and `f1` of opposite signs
    let crossing = |t0: f64,f0: f64,t1: f64,f1: f64| Root {at:bracketed_root(|t| Some(f(t)),t0,f0,t1,f1,width),touch:false};
    // whether samples `j` and `k`, both clear of zero, are on one side of it (None where either is not)
    let side = |j: usize,k: usize| (!near(fs[j]) && !near(fs[k])).then(|| (fs[j] < 0.) == (fs[k] < 0.));
    // a sampled minimum of `|f|`, ties included
    let least = |k: usize| k > 0 && k+1 < n && fs[k].abs() <= fs[k-1].abs() && fs[k].abs() <= fs[k+1].abs();
    let mut out: Vec<Root> = Vec::new();
    for i in 0..n {
        if near(fs[i]) {
            let around = if i > 0 && i+1 < n { side(i-1,i+1) } else { None };
            out.push(match around {
                // a touch: at the turn beside the sample nearest it, where `f` is within `tol` there
                Some(true) => Root {at:[turn(i-1),turn(i)].into_iter().flatten().filter(|&x| near(f(x)))
                    .min_by(|x,y| (x-ts[i]).abs().total_cmp(&(y-ts[i]).abs())).unwrap_or(ts[i]),touch:true},
                // a crossing, refined between the sample's neighbours where they are clear either side
                Some(false) => crossing(ts[i-1],fs[i-1],ts[i+1],fs[i+1]),
                None => Root {at:ts[i],touch:false},
            });
            continue
        }
        if i+1 == n || near(fs[i+1]) { continue }
        let (a,b) = (fs[i],fs[i+1]);
        if (a < 0.) != (b < 0.) { out.push(crossing(ts[i],a,ts[i+1],b)); continue }
        // a turn toward zero is beside a sampled minimum of `|f|`
        if !(least(i) || least(i+1)) { continue }
        let Some(x) = turn(i) else { continue };
        let v = f(x);
        if near(v) { out.push(Root {at:x,touch:true}) }
        else if (v < 0.) != (a < 0.) {
            out.push(crossing(ts[i],a,x,v));
            out.push(crossing(x,v,ts[i+1],b));
        }
    }
    out
}

/// The real roots of `a t² + b t + c`, ascending, without cancellation (the larger root by the
/// quadratic formula, the smaller by Vieta's product); a linear one where `a` is nothing against
/// `b` and `c` (a parabola's along its axis), none where it is all nothing.
pub fn quadratic(a: f64,b: f64,c: f64) -> Vec<f64> {
    if a.abs() <= 1e-14*(b.abs()+c.abs()) { return if b == 0. { Vec::new() } else { vec![-c/b] } }
    let disc = b*b-4.*a*c;
    if disc < 0. { return Vec::new() }
    let k = -0.5*(b+disc.sqrt().copysign(b));
    if k == 0. { return vec![0.] }
    let (x,y) = (k/a,c/k);
    if x <= y { vec![x,y] } else { vec![y,x] }
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
    use crate::util::space::{dot,cross,norm};
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
        let length = crate::util::space::norm(step);
        if !length.is_finite() { return None; }
        if length > limit { step = step.map(|x| x*limit/length); }
        p = crate::util::space::sub(p,step);
    }
    None
}
