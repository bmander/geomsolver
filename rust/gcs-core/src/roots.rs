//! Roots of functions of one variable.

/// A root of `f` in the bracket `[a, b]` (`fa`, `fb` of strictly opposite
/// signs), to within `width`: regula falsi with the Illinois fix, which keeps
/// a bracket like bisection but converges superlinearly, a handful of steps
/// where bisection takes one per bit. A step that leaves the domain (`f`
/// gives none) ends the search at the bracket's middle, as bisection did.
pub(crate) fn bracketed_root(mut f: impl FnMut(f64) -> Option<f64>,mut a: f64,mut fa: f64,mut b: f64,mut fb: f64,width: f64) -> f64 {
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
