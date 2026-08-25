//! A compiled expression agrees with the language it was written in, and with its own calculus.
//!
//! Two properties, and they are deliberately checked against different things:
//!
//! * every **value** against `expr::eval`, so a tape cannot drift from the language a person
//!   writes — including its units, where the trigonometry is in degrees;
//! * every **derivative** against a finite difference of the tape itself, so the calculus is
//!   checked without a second hand-derived formula to keep in step.
//!
//! Between them a units mistake fails the first and a chain-rule mistake fails the second, which
//! is the whole reason they are not the same test.

use gcs_core::expr::{self, Aff};
use gcs_core::tape::{Scratch, Tape, MAX_VARS};
use std::collections::BTreeMap;

fn vars(names: &[&str]) -> Vec<String> {
    names.iter().map(|s| s.to_string()).collect()
}

fn tape(src: &str, names: &[&str]) -> Tape {
    let p = expr::parse(src).unwrap_or_else(|e| panic!("{src}: {e}"));
    Tape::compile(&p.body, &vars(names)).unwrap_or_else(|e| panic!("{src}: {e}"))
}

fn at(t: &Tape, x: &[f64]) -> (f64, [f64; MAX_VARS]) {
    let mut s = Scratch::new();
    let v = t.eval(x, &mut s);
    (v.v, v.d)
}

/// The same expression through `expr::eval`, which is what a dimension in a document goes
/// through — so the two cannot mean different things by `sin`.
fn by_expr(src: &str, names: &[&str], x: &[f64]) -> f64 {
    let p = expr::parse(src).unwrap();
    let env: BTreeMap<String, Aff> =
        names.iter().zip(x).map(|(n, &v)| (n.to_string(), Aff::num(v))).collect();
    expr::eval(&p.body, &env).unwrap().number().unwrap()
}

const CASES: &[(&str, &[&str])] = &[
    ("1 + 2 * 3", &[]),
    ("a + b", &["a", "b"]),
    ("a - b * c", &["a", "b", "c"]),
    ("a / b", &["a", "b"]),
    ("a ^ 2", &["a"]),
    ("a ^ b", &["a", "b"]),
    ("-a + 3", &["a"]),
    ("sin(a)", &["a"]),
    ("cos(a)", &["a"]),
    ("tan(a)", &["a"]),
    ("atan(a)", &["a"]),
    ("asin(a)", &["a"]),
    ("acos(a)", &["a"]),
    ("sqrt(a)", &["a"]),
    ("exp(a)", &["a"]),
    ("ln(a)", &["a"]),
    ("log(a)", &["a"]),
    ("hypot(a, b)", &["a", "b"]),
    ("atan2(a, b)", &["a", "b"]),
    ("abs(a)", &["a"]),
    ("pi * a", &["a"]),
    ("a * sin(b) + b * cos(a)", &["a", "b"]),
    ("sqrt(a ^ 2 + b ^ 2 - 2 * a * b * cos(c))", &["a", "b", "c"]),
    // the involute of a circle, which is what all of this is for
    ("cx + r * (cos(u) + u * pi / 180 * sin(u))", &["cx", "r", "u"]),
    ("cy + r * (sin(u) - u * pi / 180 * cos(u))", &["cy", "r", "u"]),
];

/// Sample points chosen to stay inside every domain in `CASES`: positive, away from zero, and
/// with the angle-like ones well inside asin/acos's range.
fn sample(n: usize, k: usize) -> Vec<f64> {
    (0..n).map(|i| 0.3 + 0.17 * ((i + 2 * k) % 5) as f64).collect()
}

#[test]
fn a_tape_evaluates_to_what_the_language_says() {
    for (src, names) in CASES {
        let t = tape(src, names);
        for k in 0..5 {
            let x = sample(names.len(), k);
            let (got, _) = at(&t, &x);
            let want = by_expr(src, names, &x);
            assert!(
                (got - want).abs() <= 1e-12 * want.abs().max(1.0),
                "{src} at {x:?}: tape {got}, expr {want}",
            );
        }
    }
}

#[test]
fn a_tape_differentiates_itself_correctly() {
    let mut s = Scratch::new();
    for (src, names) in CASES {
        let t = tape(src, names);
        for k in 0..5 {
            let x = sample(names.len(), k);
            let (_, d) = at(&t, &x);
            for i in 0..names.len() {
                // a central difference, stepped relative to the coordinate
                let h = 1e-6 * x[i].abs().max(1.0);
                let mut lo = x.clone();
                let mut hi = x.clone();
                lo[i] -= h;
                hi[i] += h;
                let fd = (t.eval(&hi, &mut s).v - t.eval(&lo, &mut s).v) / (2.0 * h);
                let tol = 1e-5 * fd.abs().max(1.0);
                assert!(
                    (d[i] - fd).abs() <= tol,
                    "d({src})/d{} at {x:?}: tape {}, finite difference {fd}",
                    names[i],
                    d[i],
                );
            }
        }
    }
}

/// The gradient in a variable the expression does not read is zero, not noise.
#[test]
fn an_unread_variable_has_no_gradient() {
    let t = tape("a * 2", &["a", "b"]);
    let (v, d) = at(&t, &[3.0, 99.0]);
    assert_eq!(v, 6.0);
    assert_eq!(d[0], 2.0);
    assert_eq!(d[1], 0.0);
}

/// `pi` is a constant of the language, not a variable to be differentiated.
#[test]
fn a_constant_of_the_language_is_a_constant() {
    let t = tape("pi", &[]);
    assert!((at(&t, &[]).0 - std::f64::consts::PI).abs() < 1e-15);
}

/// A name nothing declares is an error where a curve is written, and *not* a free variable.
///
/// A dimension may read a name nothing defines — that is the solver's unknown to answer.  A
/// curve may not: it is written over geometry that exists, so a name with no coordinate behind
/// it is a misspelling, and saying so is better than quietly adding a degree of freedom to
/// every point on the curve.
#[test]
fn a_name_the_curve_cannot_read_is_an_error() {
    let p = expr::parse("a + nope").unwrap();
    let e = Tape::compile(&p.body, &vars(&["a"])).unwrap_err();
    assert!(e.contains("nope"), "{e}");
}

/// The involute, differentiated: `C'(u)` is `Rb u` in the radial direction at the tangent point,
/// which is the geometric fact the whole curve rests on — the string is perpendicular to the
/// radius and grows exactly as fast as the arc it unwinds.
#[test]
fn the_involutes_derivative_is_the_unwinding_string() {
    let names = ["cx", "cy", "r", "u"];
    let x = tape("cx + r * (cos(u) + u * pi / 180 * sin(u))", &names);
    let y = tape("cy + r * (sin(u) - u * pi / 180 * cos(u))", &names);
    for &deg in &[10.0f64, 45.0, 90.0, 137.0] {
        let v = [3.0, -2.0, 5.0, deg];
        let (_, dx) = at(&x, &v);
        let (_, dy) = at(&y, &v);
        let (rb, urad) = (v[2], deg.to_radians());
        // dC/du, per *degree* of u, is Rb u (cos u, sin u) * (pi/180)
        let k = std::f64::consts::PI / 180.0;
        let want = (rb * urad * deg.to_radians().cos() * k, rb * urad * deg.to_radians().sin() * k);
        assert!(
            (dx[3] - want.0).abs() < 1e-9 && (dy[3] - want.1).abs() < 1e-9,
            "at {deg} degrees: got ({}, {}), want {want:?}",
            dx[3],
            dy[3],
        );
        // and the speed is Rb * u, the length of string unwound
        let speed = (dx[3] / k).hypot(dy[3] / k);
        assert!((speed - rb * urad).abs() < 1e-9, "|C'| = {speed}, want {}", rb * urad);
        // moving the centre moves the curve one for one, and nothing else
        assert!((dx[0] - 1.0).abs() < 1e-15 && dy[0].abs() < 1e-15);
        assert!(dx[1].abs() < 1e-15 && (dy[1] - 1.0).abs() < 1e-15);
    }
}

/// A tape that would run away is refused rather than run.
#[test]
fn a_tape_is_bounded() {
    let deep = "1".to_string() + &" + 1".repeat(3000);
    let p = expr::parse(&deep);
    // `expr` has its own depth cap, so either it refuses or the tape does; what must not happen
    // is that a document makes the core allocate without limit
    if let Ok(p) = p {
        let t = Tape::compile(&p.body, &[]);
        if let Ok(t) = t {
            assert!(t.ops.len() <= gcs_core::tape::MAX_OPS);
        }
    }
    let wide: Vec<String> = (0..64).map(|i| format!("v{i}")).collect();
    assert!(Tape::compile(&expr::parse("v0").unwrap().body, &wide).is_err(), "too many variables");
}
