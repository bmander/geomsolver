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

/// #45.1 — a body is a set (spec P2): a `param` may read one declared below it, at the top
/// level, inside a block and inside a component alike.
#[test]
fn a_param_may_read_one_declared_below_it() {
    let (e, d) = read("param h = w / 2\nparam w = 60\npoint a hint(x: 0, y: 0)\npoint b hint(x: w, y: 0)\na horizontal b\na distance(w) b\npoint c hint(x: 0, y: h)\na vertical c\na distance(h) c\nground a\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(1).0 - 60.0).abs() < 1e-9);
    assert!((sk.point_xy(2).1 - 30.0).abs() < 1e-9);
    // a block's param reads the binder and a param of the enclosing body written after the
    // block; a component's reads its formal and a sibling written below
    let (e, d) = read("repeat 2 as i {\n point p hint(x: k, y: 0)\n param k = base + i * 10\n}\nparam base = 5\ncomponent C(n: Int) {\n param half = whole / 2\n param whole = n * 2\n point q hint(x: half, y: whole)\n}\nc: C(n: 4)\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
    let sk = e.sketch;
    assert!((sk.point_xy(0).0 - 5.0).abs() < 1e-9 && (sk.point_xy(1).0 - 15.0).abs() < 1e-9);
    assert!((sk.point_xy(2).0 - 4.0).abs() < 1e-9 && (sk.point_xy(2).1 - 8.0).abs() < 1e-9);
}

/// #45.2 — an index is an expression over the numbers in scope, at the top level as inside a
/// block: `p[n - 1]` reads the `param n`, wherever the statement stands.
#[test]
fn a_top_level_index_reads_a_param() {
    let (e, d) = read("p[n - 1] distance(10) p[0]\nparam n = 4\ncycle n as i {\n point p hint(x: 40 * cos(i * 90), y: 40 * sin(i * 90))\n}\nground p[0]\np[n / 2] distance(k) p[1]\nparam k = 30\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    // p[3] is 10 from p[0], and p[2] is 30 from p[1]
    let (x0, y0) = sk.point_xy(0);
    let (x3, y3) = sk.point_xy(3);
    assert!(((x3 - x0).hypot(y3 - y0) - 10.0).abs() < 1e-6);
    let (x1, y1) = sk.point_xy(1);
    let (x2, y2) = sk.point_xy(2);
    assert!(((x2 - x1).hypot(y2 - y1) - 30.0).abs() < 1e-6);
    // an index past the copies is still nothing
    let (e, d) = read("param n = 4\ncycle n {\n point p\n}\nground p[n]\n");
    assert!(!e.ok() && d.iter().any(|m| m.contains("no such entity: `p[n]`")), "{d:?}");
}

/// #45.1 — a `param` defined in terms of itself, through however many others, is the cyclic
/// definitional dependency spec §11 names E041; and one that fails is reported once, where it
/// is written, not again at every param that reads it.
#[test]
fn a_cyclic_param_is_e041_and_a_failed_one_is_reported_once() {
    let (e, d) = read("param a = b + 1\nparam b = c * 2\nparam c = a\nparam d = d\nparam e = 60\npoint p hint(x: e, y: 0)\nground p\n");
    assert!(!e.ok());
    let cycles: Vec<&String> = d.iter().filter(|m| m.starts_with("E041")).collect();
    assert_eq!(cycles.len(), 4, "{d:?}");
    assert!(cycles.iter().any(|m| m.contains("`a` is defined in terms of itself, through `b`")), "{d:?}");
    assert!(cycles.iter().any(|m| m.contains("`d` is defined in terms of itself") && !m.contains("through")), "{d:?}");
    // `e` is not in the cycle and is worked out
    assert!((e.sketch.point_xy(0).0 - 60.0).abs() < 1e-9);
    // `h` reads a `w` whose definition failed: the one error is at `w`
    let (_, d) = read("param w = nosuch * 2\nparam h = w / 2\npoint a hint(x: 0, y: 0)\nground a\n");
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("`w`: `nosuch` is not a number here"), "{d:?}");
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

/// #45.7 — a contact *seeded* off the end of its curve is only a seed (spec P3): brought onto
/// the curve before the solve and left free, so a document with one answer reaches it from
/// `t: 2`, `t: -1` or `t: 5` exactly as from `t: 0.5`.  Pinned to the end for the retry, as a
/// solve that *walked* off is, it nailed the point to the curve's last control point.
#[test]
fn a_contact_seeded_off_its_curve_still_solves() {
    let doc = |t: &str| {
        format!(
            "point a hint(x: 0, y: 0)\npoint b hint(x: 10, y: 20)\npoint c hint(x: 30, y: 20)\n\
             point d hint(x: 40, y: 0)\nspline s(a, b, c, d)\nground a\nground b\nground c\nground d\n\
             point p hint(x: 20, y: 14)\np on s hint(t: {t})\np vertical b\n"
        )
    };
    let mut want = None;
    for t in ["0.5", "1.5", "-1", "2", "5"] {
        let (e, d) = read(&doc(t));
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "t: {t}: {}", r.message);
        let (px, py) = sk.point_xy(4);
        assert!((px - 10.0).abs() < 1e-6, "t: {t}: p at ({px}, {py})");
        let c = sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap().clone();
        let u = sk.params[c.aux_params()[0] as usize].value;
        assert!((0.0..=1.0).contains(&u) && !sk.params[c.aux_params()[0] as usize].fixed, "t: {t}: u = {u}");
        // one answer, whatever the seed
        match want {
            None => want = Some(py),
            Some(w) => assert!((py - w).abs() < 1e-6, "t: {t}: {py} vs {w}"),
        }
    }
    // a family's contact likewise, seeded past either end of `over (0, 720)`
    for u in ["900", "-100"] {
        let (e, d) = read(&format!(
            "component spiral(o: point, k: Length, u: Angle) {{\n  point p = ( o.x + k * u / 360 * cos(u), o.y + k * u / 360 * sin(u) )\n}}\n\
             point o hint(x: 0, y: 0)\ncurve f = spiral(o, k: 10).p over u in (0, 720)\npoint t hint(x: 12, y: 3)\nt on f hint(t: {u})\nground o\n"
        ));
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success, "u: {u}");
        let c = sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap().clone();
        let v = sk.params[c.aux_params()[0] as usize].value;
        assert!((0.0..=720.0).contains(&v), "u: {u} → {v}");
    }
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
        "component quarter(c: circle, u: Angle) {
           point p = ( c.center.x + c.r * cos(u), c.center.y + c.r * sin(u) )
         }
         point o hint(x: 0, y: 0)
         circle c(center: o) hint(r: 20)
         curve f = quarter(c).p over u in (0, 90)
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

/// The lexer read `b[i] as char`, so the first byte of a multibyte character passed for a
/// Latin-1 letter and opened an identifier that then consumed nothing: `…` or `→` anywhere in a
/// statement hung the parser, while the same character in a comment was fine.  Both are read
/// now — a letter outside ASCII is an identifier character, and a symbol is a punctuation token
/// reported where it is used, like any other stray character.
#[test]
fn a_non_ascii_character_in_a_statement_does_not_hang_the_lexer() {
    for src in ["point p hint(x: 0, y: 0) …\n", "use a…\n", "point p → q\n", "line l(a, b) — c\n"] {
        let (_, d) = read(src);
        assert!(!d.is_empty(), "{src:?} should be refused, not accepted");
    }
    // a letter outside ASCII is a letter, in a name as in a comment
    let (e, d) = read("point début hint(x: 1, y: 2)\nground début\n");
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(e.sketch.point_xy(0), (1.0, 2.0));
    let (_, d) = read("// an em dash — in a comment\npoint p hint(x: 0, y: 0)\n");
    assert!(d.is_empty(), "{d:?}");
}
