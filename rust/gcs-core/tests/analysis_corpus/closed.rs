//! **The corpus's answers** (#510 P8): each closed form a case is held to, and each held in turn
//! to the equations that make it the answer — its equilibrium, its boundary conditions, its
//! limits — so a wrong reference cannot pass a right analysis.  Any consistent units.

use std::f64::consts::PI;

/// A cantilever's tip deflection under a tip load, Euler–Bernoulli's `P L³ / (3 E I)`.
pub fn cantilever_tip(p: f64, l: f64, e: f64, i: f64) -> f64 {
    p * l.powi(3) / (3.0 * e * i)
}

/// What shear adds to a cantilever's tip deflection, Timoshenko's `P L / (κ G A)`, with Cowper's
/// shear coefficient of a rectangle, `κ = 10 (1 + ν) / (12 + 11 ν)`.
pub fn cantilever_shear(p: f64, l: f64, e: f64, nu: f64, area: f64) -> f64 {
    let kappa = 10.0 * (1.0 + nu) / (12.0 + 11.0 * nu);
    p * l / (kappa * e / (2.0 * (1.0 + nu)) * area)
}

/// `β₁ L`, the least root of `1 + cos x cosh x = 0`: a built-in beam's first mode.
pub fn cantilever_root() -> f64 {
    let mut x: f64 = 1.875;
    for _ in 0..50 {
        let f = 1.0 + x.cos() * x.cosh();
        let df = x.cos() * x.sinh() - x.sin() * x.cosh();
        x -= f / df;
    }
    x
}

/// A cantilever's first natural frequency, cycles a unit time.
pub fn cantilever_mode(e: f64, i: f64, rho: f64, area: f64, l: f64) -> f64 {
    cantilever_root().powi(2) / (2.0 * PI * l * l) * (e * i / (rho * area)).sqrt()
}

/// Euler's load of a column built in at its foot and free at its head, `π² E I / (4 L²)`.
pub fn euler_cantilever(e: f64, i: f64, l: f64) -> f64 {
    PI * PI * e * i / (4.0 * l * l)
}

/// **Lamé's thick cylinder**, bore `a`, outside `c`, internal pressure `p`, ends open.
pub struct Lame {
    pub a: f64,
    pub c: f64,
    pub p: f64,
}

impl Lame {
    fn big_a(&self) -> f64 {
        self.p * self.a * self.a / (self.c * self.c - self.a * self.a)
    }
    pub fn radial(&self, r: f64) -> f64 {
        self.big_a() * (1.0 - self.c * self.c / (r * r))
    }
    pub fn hoop(&self, r: f64) -> f64 {
        self.big_a() * (1.0 + self.c * self.c / (r * r))
    }
    /// How far a radius moves out, the axial stress nothing.
    pub fn moves(&self, r: f64, e: f64, nu: f64) -> f64 {
        r / e * (self.hoop(r) - nu * self.radial(r))
    }
    /// The von Mises stress at a radius, the closed form `A √(1 + 3 c⁴/r⁴)`.
    pub fn mises(&self, r: f64) -> f64 {
        self.big_a() * (1.0 + 3.0 * (self.c / r).powi(4)).sqrt()
    }
}

/// **Kirsch's field** about a hole of radius `a` in an infinite plate pulled at `s` along x:
/// `(σrr, σθθ, σrθ)` at polar `(r, θ)`.
pub fn kirsch(s: f64, a: f64, r: f64, th: f64) -> [f64; 3] {
    let (q2, q4) = ((a / r).powi(2), (a / r).powi(4));
    let (c, n) = ((2.0 * th).cos(), (2.0 * th).sin());
    [
        s / 2.0 * (1.0 - q2) + s / 2.0 * (1.0 - 4.0 * q2 + 3.0 * q4) * c,
        s / 2.0 * (1.0 + q2) - s / 2.0 * (1.0 + 3.0 * q4) * c,
        -s / 2.0 * (1.0 + 2.0 * q2 - 3.0 * q4) * n,
    ]
}

/// Heywood's gross stress concentration of a hole of diameter `d` in a strip `w` wide.
pub fn heywood(d: f64, w: f64) -> f64 {
    let x = d / w;
    (2.0 + (1.0 - x).powi(3)) / (1.0 - x)
}

/// An isotropic solid's two wave speeds, `(c_L, c_T)`.
fn speeds(e: f64, nu: f64, rho: f64) -> (f64, f64) {
    let g = e / (2.0 * (1.0 + nu));
    let lambda = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    (((lambda + 2.0 * g) / rho).sqrt(), (g / rho).sqrt())
}

/// The flexural (antisymmetric) Rayleigh–Lamb equation of a plate `h` thick at wavenumber `k` and
/// angular frequency `w`, below the shear speed: `(k² + Q²)² tanh(P h/2) − 4 k² P Q tanh(Q h/2)`,
/// `P² = k² − w²/c_L²`, `Q² = k² − w²/c_T²`.  Zero at a flexural wave.
pub fn lamb(w: f64, k: f64, h: f64, e: f64, nu: f64, rho: f64) -> f64 {
    let (cl, ct) = speeds(e, nu, rho);
    let p = (k * k - w * w / (cl * cl)).sqrt();
    let q = (k * k - w * w / (ct * ct)).sqrt();
    (k * k + q * q).powi(2) * (p * h / 2.0).tanh() - 4.0 * k * k * p * q * (q * h / 2.0).tanh()
}

/// Kirchhoff's flexural frequency at wavenumber `k`, angular: `√(D / ρh) k²`.
pub fn kirchhoff(k: f64, h: f64, e: f64, nu: f64, rho: f64) -> f64 {
    let d = e * h.powi(3) / (12.0 * (1.0 - nu * nu));
    (d / (rho * h)).sqrt() * k * k
}

/// The least flexural root of `lamb` at `k`, angular: bracketed between half Kirchhoff's (the
/// equation's double root at zero lies below) and the lesser of Kirchhoff's and the shear wave's.
pub fn flexural(k: f64, h: f64, e: f64, nu: f64, rho: f64) -> f64 {
    let (_, ct) = speeds(e, nu, rho);
    let thin = kirchhoff(k, h, e, nu, rho);
    let (lo, hi) = (0.5 * thin.min(ct * k), (1.0 + 1e-9) * thin.min(ct * k * (1.0 - 1e-12)));
    let f = |w: f64| lamb(w, k, h, e, nu, rho);
    // the first change of sign, then bisection to the last bit
    let n = 2000;
    let mut a = lo;
    for j in 1..=n {
        let b = lo + (hi - lo) * j as f64 / n as f64;
        if f(a) * f(b) <= 0.0 {
            let (mut a, mut b) = (a, b);
            for _ in 0..200 {
                let m = 0.5 * (a + b);
                if f(a) * f(m) <= 0.0 { b = m } else { a = m }
            }
            return 0.5 * (a + b);
        }
        a = b;
    }
    panic!("no flexural root at k = {k}")
}

/// A simply supported (diaphragm) plate's modes, `side` square and `h` thick, as `(f, m, n)`,
/// cycles a unit time, ascending, the first `count`.
pub fn plate_modes(side: f64, h: f64, e: f64, nu: f64, rho: f64, count: usize)
    -> Vec<(f64, u32, u32)> {
    let mut all = Vec::new();
    for m in 1..=6u32 {
        for n in 1..=6u32 {
            let k = PI * ((m * m + n * n) as f64).sqrt() / side;
            all.push((flexural(k, h, e, nu, rho) / (2.0 * PI), m, n));
        }
    }
    all.sort_by(|x, y| x.0.total_cmp(&y.0).then((x.1, x.2).cmp(&(y.1, y.2))));
    all.truncate(count);
    all
}

/// NAFEMS LE1's target, σyy at D (NAFEMS, *The Standard NAFEMS Benchmarks*, TNSB Rev. 3), in
/// pascals.
pub const LE1_SYY_D: f64 = 92.7e6;

/// NAFEMS FV32's six frequencies, hertz (the same publication).
pub const FV32: [f64; 6] = [44.623, 130.03, 162.70, 246.05, 379.90, 391.44];

/// The Rayleigh wave's speed over the shear wave's: the root in (0, 1) of
/// `(2 − ξ²)² = 4 √(1 − ξ² c_T²/c_L²) √(1 − ξ²)`.
fn rayleigh(nu: f64) -> f64 {
    let (cl, ct) = speeds(1.0, nu, 1.0);
    let kk = (ct / cl).powi(2);
    let f = |x: f64| (2.0 - x * x).powi(2) - 4.0 * (1.0 - x * x * kk).sqrt() * (1.0 - x * x).sqrt();
    let (mut a, mut b) = (0.5, 1.0 - 1e-12);
    assert!(f(a) * f(b) < 0.0);
    for _ in 0..200 {
        let m = 0.5 * (a + b);
        if f(a) * f(m) <= 0.0 { b = m } else { a = m }
    }
    0.5 * (a + b)
}

/// A central difference of `f` at `x`.
fn d(f: impl Fn(f64) -> f64, x: f64, step: f64) -> f64 {
    (f(x + step) - f(x - step)) / (2.0 * step)
}

#[test]
fn a_cantilevers_answers_are_its_beams() {
    // the root of the frequency equation, against the tabulated 1.875104069
    let b = cantilever_root();
    assert!((1.0 + b.cos() * b.cosh()).abs() < 1e-13 && (b - 1.875104069).abs() < 1e-9, "{b}");
    // EI w'''' = 0 under a tip load: w = P x² (3L − x) / (6EI) is built in at the root, its
    // moment EI w'' = P (L − x) and shear the load, and its tip the closed form
    let (p, l, e, i) = (1000.0, 1.0, 200e9, 2.6e-7);
    let w = |x: f64| p * x * x * (3.0 * l - x) / (6.0 * e * i);
    assert!(w(0.0) == 0.0 && d(w, 0.0, 1e-6).abs() < 1e-12);
    let curve = |x: f64| d(|y| d(w, y, 1e-4), x, 1e-4);
    for x in [0.1, 0.5, 0.9] {
        assert!((e * i * curve(x) - p * (l - x)).abs() < 1e-6 * p, "moment at {x}");
    }
    assert!((w(l) - cantilever_tip(p, l, e, i)).abs() < 1e-15);
    // a column built in at its foot buckles as w = δ (1 − cos kx), k² = P/EI, which meets its
    // free head only where cos kL = 0: kL = π/2
    let pe = euler_cantilever(e, i, l);
    assert!(((pe / (e * i)).sqrt() * l - PI / 2.0).abs() < 1e-14);
}

#[test]
fn lames_stresses_are_in_equilibrium_and_meet_the_pressure() {
    let t = Lame { a: 50.0, c: 100.0, p: 50.0 };
    let (e, nu) = (200e3, 0.3);
    // the bore carries the pressure, the outside nothing
    assert!((t.radial(t.a) + t.p).abs() < 1e-12 && t.radial(t.c).abs() < 1e-12);
    for r in [55.0, 70.0, 95.0] {
        // dσr/dr + (σr − σθ)/r = 0
        let eq = d(|x| t.radial(x), r, 1e-4) + (t.radial(r) - t.hoop(r)) / r;
        assert!(eq.abs() < 1e-8, "equilibrium at {r}: {eq}");
        // the strains are one displacement's: εθ = u/r holds by construction, so εr = du/dr is
        // the compatibility, in plane stress
        let er = d(|x| t.moves(x, e, nu), r, 1e-4);
        assert!((er - (t.radial(r) - nu * t.hoop(r)) / e).abs() < 1e-12, "compatibility at {r}");
        // the closed von Mises is the invariant's
        let (s1, s2) = (t.radial(r), t.hoop(r));
        assert!((t.mises(r) - (s1 * s1 - s1 * s2 + s2 * s2).sqrt()).abs() < 1e-10);
        assert!(t.mises(r) < t.mises(t.a), "greatest at the bore");
    }
}

#[test]
fn kirschs_field_is_in_equilibrium_free_at_the_hole_and_three_times_the_pull() {
    let (s, a) = (50.0, 10.0);
    let k = |r: f64, th: f64| kirsch(s, a, r, th);
    for (r, th) in [(12.0, 0.3), (20.0, 1.1), (35.0, 2.5)] {
        // ∂σrr/∂r + (1/r) ∂σrθ/∂θ + (σrr − σθθ)/r = 0 and ∂σrθ/∂r + (1/r) ∂σθθ/∂θ + 2σrθ/r = 0
        let [rr, tt, rt] = k(r, th);
        let one = d(|x| k(x, th)[0], r, 1e-5) + d(|x| k(r, x)[2], th, 1e-6) / r + (rr - tt) / r;
        let two = d(|x| k(x, th)[2], r, 1e-5) + d(|x| k(r, x)[1], th, 1e-6) / r + 2.0 * rt / r;
        assert!(one.abs() < 1e-6 && two.abs() < 1e-6, "equilibrium at ({r}, {th}): {one}, {two}");
    }
    for th in [0.0, 0.4, 1.2, PI / 2.0] {
        let [rr, _, rt] = k(a, th);
        assert!(rr.abs() < 1e-12 && rt.abs() < 1e-12, "the hole is free at {th}");
        // far off, the pull: σxx = s, and nothing else
        let [rr, tt, rt] = k(1e7 * a, th);
        let (c, n) = (th.cos(), th.sin());
        let sxx = rr * c * c + tt * n * n - 2.0 * rt * n * c;
        let syy = rr * n * n + tt * c * c + 2.0 * rt * n * c;
        assert!((sxx - s).abs() < 1e-9 * s && syy.abs() < 1e-9 * s);
    }
    // three times the pull at the hole's edge square to it, and the least, minus the pull,
    // along it
    assert!((k(a, PI / 2.0)[1] - 3.0 * s).abs() < 1e-12 && (k(a, 0.0)[1] + s).abs() < 1e-12);
    // Heywood's finite width tends to Kirsch's
    assert!((heywood(1e-6, 1.0) - 3.0).abs() < 1e-5 && heywood(0.05, 1.0) > 3.0);
}

#[test]
fn the_flexural_root_is_kirchhoffs_thin_and_rayleighs_thick() {
    let (e, nu, rho) = (200e9, 0.3, 7850.0);
    // a root, to the bit the equation allows
    let (k, h) = (PI * 2f64.sqrt(), 0.05);
    let w = flexural(k, h, e, nu, rho);
    let scale = k.powi(4) * (k * h / 2.0).tanh();
    assert!(lamb(w, k, h, e, nu, rho).abs() < 1e-9 * scale);
    // thin, Kirchhoff's, short of it by the square of the thinness (thinner than this the
    // equation's two terms cancel past what a double holds: it is (kh)⁵ of either)
    let short = |h: f64| 1.0 - flexural(k, h, e, nu, rho) / kirchhoff(k, h, e, nu, rho);
    let (s1, s2) = (short(0.02), short(0.01));
    assert!(s2 > 0.0 && s2 < 1e-3 && (s1 / s2 - 4.0).abs() < 2e-2, "{s1}, {s2}");
    // thick, a Rayleigh wave along either face
    let (_, ct) = speeds(e, nu, rho);
    let thick = flexural(k, 400.0 / k, e, nu, rho) / (k * ct);
    assert!((thick - rayleigh(nu)).abs() < 1e-9, "{thick} against {}", rayleigh(nu));
    assert!((rayleigh(0.3) - 0.92741).abs() < 1e-5);
    // the square plate's first six: (1, 1), a pair, (2, 2), a pair
    let modes = plate_modes(1.0, 0.05, e, nu, rho, 6);
    let mn: Vec<_> = modes.iter().map(|&(_, m, n)| (m, n)).collect();
    assert_eq!(mn, [(1, 1), (1, 2), (2, 1), (2, 2), (1, 3), (3, 1)]);
    assert!(modes[1].0 == modes[2].0 && modes[4].0 == modes[5].0, "{modes:?}");
}
