//! Numbers and statements that used to be accepted and should not have been (issue #43), and
//! two mistakes that used to be reported four ways.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

/// #43.12 — a point-to-point distance and a radius are magnitudes: their kernels square the
/// sign away, so a negative literal quietly meant the positive and the drawing showed a circle
/// the document did not describe.  The signed dimensions keep their sign.
#[test]
fn a_negative_magnitude_is_refused_and_a_signed_dimension_is_not() {
    let (_, d) = read("point o hint(x: 0, y: 0)\ncircle c(center: o) hint(r: 20)\nradius(-20) c\nground o\n");
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("radius is a magnitude")), "{d:?}");
    let (_, d) = read("point a hint(x: 0, y: 0)\npoint b hint(x: 40, y: 0)\na distance(-40) b\n");
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("distance is a magnitude")), "{d:?}");
    // the run is signed from the first point to the second, and −40 is a statement
    let (e, d) = read("point a hint(x: 0, y: 0)\npoint b hint(x: -40, y: 0)\na distance(-40, along: x) b\nground a\n");
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(1).0 + 40.0).abs() < 1e-9);
    // and the app's own write path says the same
    let (e, _) = read("point o hint(x: 0, y: 0)\ncircle c(center: o) hint(r: 20)\nradius(20) c\n");
    let mut sk = e.sketch;
    let id = sk.user_constraints()[0].id;
    assert!(gcs_core::expr::set_dimension(&mut sk, id, "r", "-5").is_err());
    assert!(gcs_core::expr::set_dimension(&mut sk, id, "r", "5").is_ok());
}

/// #43.13 — a second `param w` is the E001 a second `point w` is, and the first stands.
#[test]
fn a_param_declared_twice_is_an_error() {
    let (e, d) = read("param w = 60\nparam w = 80\npoint a hint(x: 0, y: 0)\npoint b hint(x: 40, y: 0)\na distance(w) b\nground a\n");
    assert!(d.iter().any(|m| m.starts_with("E001") && m.contains("`w` is declared twice")), "{d:?}");
    assert!(!e.ok());
    // one per body: a component may have its own
    let (e, d) = read("param w = 60\ncomponent C() { param w = 5\n point p hint(x: w, y: 0) }\nc: C()\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
}

/// #43.19 — a bad key in a `hint(…)` is the mistake, not the declaration: one error, and the
/// entity is still declared for everything that names it.
#[test]
fn a_bad_hint_key_is_one_error_and_keeps_the_declaration() {
    let (_, d) = read("point a hint(x: 0, y: 0, z: 5)\npoint b hint(x: 40, y: 0)\nline ab(a, b)\nhorizontal ab\na distance(40) b\nground a\n");
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("no scalar `z`"), "{d:?}");
}

/// #43.20 — a value a known property cannot read is a mistake and is said to be one; an
/// unknown property still has no rule, as in CSS.
#[test]
fn an_unreadable_style_value_is_reported() {
    let (_, d) = read("style .weird { color: ; width: nope; dash: }\npoint a hint(x: 0, y: 0)\nground a\n");
    assert!(d.iter().any(|m| m.contains("`color:` is given no value")), "{d:?}");
    assert!(d.iter().any(|m| m.contains("`width` cannot read `nope`")), "{d:?}");
    assert!(!d.iter().any(|m| m.contains("dash")), "`dash:` states solid: {d:?}");
    let (e, d) = read("style .odd { glow: 3 }\npoint a hint(x: 0, y: 0)\nground a\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
}

/// #43.21 — `distance` between two circles reads two radii and neither centre (the annular gap
/// between concentric circles), so over two circles centred apart it is refused with that
/// reading, instead of conflicting with the radii it silently duplicated.
#[test]
fn distance_between_circles_centred_apart_is_refused() {
    let (_, d) = read(
        "point o1 hint(x: 0, y: 0)\npoint o2 hint(x: 70, y: 0)\ncircle c1(center: o1) hint(r: 15)\ncircle c2(center: o2) hint(r: 20)\nc1 distance(10) c2\n",
    );
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("concentric")), "{d:?}");
    let (e, d) = read(
        "point o hint(x: 0, y: 0)\ncircle c1(center: o) hint(r: 15)\ncircle c2(center: o) hint(r: 20)\nradius(15) c1\nc1 distance(5) c2\nground o\n",
    );
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.params[sk.circles[1].radius as usize].value - 20.0).abs() < 1e-9);
}

/// #43.10 — a curve family's `over (a, b)` bounds its parameter as a spline's knots bound
/// theirs: a contact that ends up past the end is put back and held there for the retry.
#[test]
fn a_curve_family_contact_stays_inside_its_domain() {
    let (e, d) = read(
        "curve quarter(c: circle)(u) over (0, 90) =
           ( c.center.x + c.r * cos(u), c.center.y + c.r * sin(u) )
         point o hint(x: 0, y: 0)
         circle c(center: o) hint(r: 20)
         curve f = quarter(c)
         ground o
         fix c.r
         point p hint(x: 0, y: -20)
         p on f
         p distance(20, along: y) o",
    );
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    let r = solve(&mut sk, SolveOpts::default());
    // (0, −20) is u = 270°, three quadrants past the end: the drawing cannot satisfy both the
    // contact and the rise, and must not claim to
    assert!(!r.success, "a point off the drawn curve reported solved: {r:?}");
    let c = sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap().clone();
    let u = sk.params[c.aux_params()[0] as usize].value;
    assert!((0.0..=90.0).contains(&u), "u = {u} is off the curve");
}
