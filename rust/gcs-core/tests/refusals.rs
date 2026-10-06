//! Numbers and statements that used to be accepted and should not have been (issue #43), and
//! two mistakes that used to be reported four ways.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse_legacy as parse;

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
    let (_, d) = read("use std\nin std.front {\no := point\nc := circle(center: o) hint(r: 20)\nradius(-20) c\nfix(x == 0, y == 0) o\n}\n");
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("radius is a magnitude")), "{d:?}");
    let (_, d) = read("use std\nin std.front {\na := point hint(x: 0, y: 0)\nb := point hint(x: 40, y: 0)\na distance(-40) b\n}\n");
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("distance is a magnitude")), "{d:?}");
    // the run is signed from the first point to the second, and −40 is a statement
    let (e, d) = read("use std\nin std.front {\na := point\nb := point hint(x: -40, y: 0)\na distance(-40, along: x) b\nfix(x == 0, y == 0) a\n}\n");
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(1).0 + 40.0).abs() < 1e-9);
    // and the app's own write path says the same
    let (e, _) = read("use std\nin std.front {\no := point hint(x: 0, y: 0)\nc := circle(center: o) hint(r: 20)\nradius(20) c\n}\n");
    let mut sk = e.sketch;
    let id = sk.user_constraints()[0].id;
    assert!(gcs_core::expr::set_dimension(&mut sk, id, "r", "-5").is_err());
    assert!(gcs_core::expr::set_dimension(&mut sk, id, "r", "5").is_ok());
}

/// An arc's length is a magnitude like a radius, refused negative where it is written, and only
/// an arc has one to state this way; `angle`'s parentheses hold one number or the pair of lines
/// of the angle it equals — two numbers there are a mistake, said at the first rather than the
/// second being dropped, and a point in the pair is not a line.
#[test]
fn an_arc_length_and_an_angle_pair_refuse_what_they_cannot_mean() {
    const ARC: &str = "\
use std
in std.front {
o := point
s := point hint(x: 10, y: 0)
e := point hint(x: 0, y: 10)
a := arc(o, s, e)
fix(x == 0, y == 0) o
}
";
    let (_, d) = read(&format!("{ARC}length(-5) a\n"));
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("length is a magnitude")), "{d:?}");
    assert!(read(&format!("{ARC}length(5) a\n")).1.is_empty());
    let (_, d) = read("use std\nin std.front {\no := point hint(x: 0, y: 0)\nc := circle(center: o) hint(r: 5)\nlength(10) c\n}\n");
    assert!(d.iter().any(|m| m.contains("`length` does not apply to a circle")), "{d:?}");
    let (_, d) = read("use std\nin std.front {\na := point hint(x: 0, y: 0)\nb := point hint(x: 5, y: 0)\nl := line(a, b)\nlength(10) l\n}\n");
    assert!(d.iter().any(|m| m.contains("`length` does not apply to a line")), "{d:?}");
    const PAIR: &str = "\
use std
in std.front {
a := point hint(x: 0, y: 0)
b := point hint(x: 10, y: 0)
c := point hint(x: 0, y: 10)
ab := line(a, b)
ac := line(a, c)
}
";
    let (_, d) = read(&format!("{PAIR}ab angle(30, 40) ac\n"));
    assert!(d.iter().any(|m| m.contains("`angle` takes one number, or the two lines")), "{d:?}");
    let (_, d) = read(&format!("{PAIR}ab angle(ab, ac, sense: left) ac\n"));
    assert!(d.iter().any(|m| m == "E040: `sense` is `ccw` or `cw`, not `left`"), "{d:?}");
    let (_, d) = read(&format!("{PAIR}ab angle(a, ac) ac\n"));
    assert!(!d.is_empty(), "a point in the pair is refused");
}

/// #43.13 — a second `param w` is the E001 a second `w := point` is, and the first stands.
#[test]
fn a_param_declared_twice_is_an_error() {
    let (e, d) = read("use std\nw := 60\nw := 80\nin std.front {\na := point\nb := point hint(x: 40, y: 0)\na distance(w) b\nfix(x == 0, y == 0) a\n}\n");
    assert!(d.iter().any(|m| m.starts_with("E001") && m.contains("`w` is declared twice")), "{d:?}");
    assert!(!e.ok());
    // one per body: a component may have its own
    let (e, d) = read("use std\nw := 60\ncomponent C() { w := 5\n p := point hint(x: w, y: 0) }\nin std.front {\nc := C()\n}\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
}

/// #45.1 — a body is a set (spec P2): a `param` may read one declared below it, at the top
/// level, inside a block and inside a component alike.
#[test]
fn a_param_may_read_one_declared_below_it() {
    let (e, d) = read("use std\nh := w / 2\nw := 60\nin std.front {\na := point\nb := point hint(x: w, y: 0)\na horizontal b\na distance(w) b\nc := point hint(x: 0, y: h)\na vertical c\na distance(h) c\nfix(x == 0, y == 0) a\n}\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!((sk.point_xy(1).0 - 60.0).abs() < 1e-9);
    assert!((sk.point_xy(2).1 - 30.0).abs() < 1e-9);
    // a block's param reads the binder and a param of the enclosing body written after the
    // block; a component's reads its formal and a sibling written below
    let (e, d) = read("use std\nin std.front {\nrepeat 2 as i {\n p := point hint(x: k, y: 0)\n k := base + i * 10\n}\n}\nbase := 5\ncomponent C(n: Int) {\n half := whole / 2\n whole := n * 2\n q := point hint(x: half, y: whole)\n}\nin std.front {\nc := C(n: 4)\n}\n");
    assert!(d.is_empty() && e.ok(), "{d:?}");
    let sk = e.sketch;
    assert!((sk.point_xy(0).0 - 5.0).abs() < 1e-9 && (sk.point_xy(1).0 - 15.0).abs() < 1e-9);
    assert!((sk.point_xy(2).0 - 4.0).abs() < 1e-9 && (sk.point_xy(2).1 - 8.0).abs() < 1e-9);
}

/// #45.2 — an index is an expression over the numbers in scope, at the top level as inside a
/// block: `p[n - 1]` reads the `param n`, wherever the statement stands.
#[test]
fn a_top_level_index_reads_a_param() {
    let (e, d) = read("p[n - 1] distance(10) p[0]\nn := 4\ncycle n as i {\n p := point hint(x: 40 * cos(i * 90), y: 40 * sin(i * 90))\n}\nfix(x == 40, y == 0) p[0]\np[n / 2] distance(k) p[1]\nk := 30\n");
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
    let (e, d) = read("n := 4\ncycle n {\n p := point\n}\nfix(x == 0, y == 0) p[n]\n");
    assert!(!e.ok() && d.iter().any(|m| m.contains("no such entity: `p[n]`")), "{d:?}");
}

/// #45.1 — a `param` defined in terms of itself, through however many others, is the cyclic
/// definitional dependency spec §11 names E041; and one that fails is reported once, where it
/// is written, not again at every param that reads it.
#[test]
fn a_cyclic_param_is_e041_and_a_failed_one_is_reported_once() {
    let (e, d) = read("use std\na := b + 1\nb := c * 2\nc := a\nd := d\ne := 60\nin std.front {\np := point\nfix(x == e, y == 0) p\n}\n");
    assert!(!e.ok());
    let cycles: Vec<&String> = d.iter().filter(|m| m.starts_with("E041")).collect();
    assert_eq!(cycles.len(), 4, "{d:?}");
    assert!(cycles.iter().any(|m| m.contains("`a` is defined in terms of itself, through `b`")), "{d:?}");
    assert!(cycles.iter().any(|m| m.contains("`d` is defined in terms of itself") && !m.contains("through")), "{d:?}");
    // `e` is not in the cycle and is worked out
    assert!((e.sketch.point_xy(0).0 - 60.0).abs() < 1e-9);
    // `h` reads a `w` whose definition failed: the one error is at `w`
    let (_, d) = read("use std\nw := nosuch * 2\nh := w / 2\nin std.front {\na := point\nfix(x == 0, y == 0) a\n}\n");
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("`w`: `nosuch` is not a number here"), "{d:?}");
}

/// #43.19 — a bad key in a `hint(…)` is the mistake, not the declaration: one error, and the
/// entity is still declared for everything that names it.
#[test]
fn a_bad_hint_key_is_one_error_and_keeps_the_declaration() {
    let (_, d) = read("use std\nin std.front {\na := point hint(x: 0, y: 0, z: 5)\nb := point hint(x: 40, y: 0)\nab := line(a, b)\nhorizontal ab\na distance(40) b\nfix(x == 0, y == 0) a\n}\n");
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("no `z`"), "{d:?}");
}

/// #43.20 — a value a known property cannot read is a mistake and is said to be one; an
/// unknown property still has no rule, as in CSS.
#[test]
fn an_unreadable_style_value_is_reported() {
    let (_, d) = read("style .weird { color: ; width: nope; dash: }\na := point\nfix(x == 0, y == 0) a\n");
    assert!(d.iter().any(|m| m.contains("`color:` is given no value")), "{d:?}");
    assert!(d.iter().any(|m| m.contains("`width` cannot read `nope`")), "{d:?}");
    assert!(!d.iter().any(|m| m.contains("dash")), "`dash:` states solid: {d:?}");
    let (e, d) = read("style .odd { glow: 3 }\na := point\nfix(x == 0, y == 0) a\n");
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
            "\
use std
in std.front {{
a := point
b := point
c := point
d := point
s := spline(a, b, c, d)
fix(x == 0, y == 0) a
fix(x == 10, y == 20) b
fix(x == 30, y == 20) c
fix(x == 40, y == 0) d
p := point hint(x: 20, y: 14)
p coincident s hint(t: {t})
p vertical b
}}
"
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
        let c = (*sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap()).clone();
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
            "\
use std
component spiral(o: point, k: Length, u: Angle) {{
  p := point(x: o.x + k * u / 360 * cos(u), y: o.y + k * u / 360 * sin(u))
}}
in std.front {{
o := point
}}
f := spiral(o, k: 10).p over u in (0, 720)
in std.front {{
t := point hint(x: 12, y: 3)
t coincident f hint(t: {u})
fix(x == 0, y == 0) o
}}
"
        ));
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success, "u: {u}");
        let c = (*sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap()).clone();
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
        "use std\nin std.front {\no1 := point hint(x: 0, y: 0)\no2 := point hint(x: 70, y: 0)\nc1 := circle(center: o1) hint(r: 15)\nc2 := circle(center: o2) hint(r: 20)\nc1 distance(10) c2\n}\n",
    );
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("concentric")), "{d:?}");
    let (e, d) = read(
        "use std\nin std.front {\no := point\nc1 := circle(center: o) hint(r: 15)\nc2 := circle(center: o) hint(r: 20)\nradius(15) c1\nc1 distance(5) c2\nfix(x == 0, y == 0) o\n}\n",
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
        "\
use std
component quarter(c: circle, u: Angle) {
           p := point(x: c.center.x + c.r * cos(u), y: c.center.y + c.r * sin(u))
         }
         in std.front {
         o := point
         c := circle(center: o) hint(r: 20)
         }
         f := quarter(c).p over u in (0, 90)
         in std.front {
         fix(x == 0, y == 0) o
         fix(r == 20) c
         p := point hint(x: 0, y: -20)
         p coincident f
         p distance(20, along: y) o
         }
",
    );
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    let r = solve(&mut sk, SolveOpts::default());
    // (0, −20) is u = 270°, three quadrants past the end: the drawing cannot satisfy both the
    // contact and the rise, and must not claim to
    assert!(!r.success, "a point off the drawn curve reported solved: {r:?}");
    let c = (*sk.user_constraints().iter().find(|c| !c.aux_params().is_empty()).unwrap()).clone();
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
    for src in ["p := point hint(x: 0, y: 0) …\n", "use a…\n", "p := point → q\n", "l := line(a, b) — c\n"] {
        let (_, d) = read(src);
        assert!(!d.is_empty(), "{src:?} should be refused, not accepted");
    }
    // a letter outside ASCII is a letter, in a name as in a comment
    let (e, d) = read("use std\nin std.front {\ndébut := point\nfix(x == 1, y == 2) début\n}\n");
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(e.sketch.point_xy(0), (1.0, 2.0));
    let (_, d) = read("use std\n// an em dash — in a comment\nin std.front {\np := point hint(x: 0, y: 0)\n}\n");
    assert!(d.is_empty(), "{d:?}");
}

/// **A selector says what it means, or it is refused** (issue #48, item 4).
///
/// Three silences, all in the same handful of words: a key naming no slot was dropped and the
/// statement settled without it; a word outside a slot's set fell through `contact_point`'s
/// `s == "start"` and silently meant the *other* end; and `along: z` made `infix_op` return
/// nothing, which the elaborator reported as a complaint about the operands' kinds — an error
/// about something else, which is the whole shape of what this issue is about.
#[test]
fn a_selector_that_says_nothing_is_refused() {
    const PAIR: &str = "\
unit mm
use std
in std.front {
a := point
b := point hint(x: 40, y: 10)
fix(x == 0, y == 0) a
}
";
    // a key the word has no slot for: dropped in silence, and the statement stood
    let (_, d) = read(&format!("{PAIR}a distance(40, sied: x) b\n"));
    assert!(d.iter().any(|m| m == "E040: `distance` takes no `sied`"), "{d:?}");
    // `along` fills no slot — it chooses the kind — so it is the one key checked by name
    let (_, d) = read(&format!("{PAIR}a distance(40, along: z) b\n"));
    let want = "E040: `along` is `x`, `y`, `u`, `v`, `n`, `right`, `left`, `up` or `down`, not `z`";
    assert!(d.iter().any(|m| m == want), "{d:?}");
    assert!(read(&format!("{PAIR}a distance(40, along: x) b\n")).1.is_empty());

    // a word outside the slot's own set: `banana` used to mean `end`, because nothing checked
    const ARC: &str = "\
unit mm
use std
in std.front {
c := point
s := point hint(x: 12, y: 0)
e := point hint(x: 0, y: 12)
k := arc(c, s, e)
radius(12) k
q := point hint(x: 40, y: 0)
l := line(s, q)
fix(x == 0, y == 0) c
}
";
    let (_, d) = read(&format!("{ARC}k tangent(at: banana) l\n"));
    assert!(d.iter().any(|m| m == "E040: `at` is `start` or `end`, not `banana`"), "{d:?}");
    for w in ["start", "end"] {
        assert!(read(&format!("{ARC}k tangent(at: {w}) l\n")).1.is_empty(), "{w}");
    }
    // and the registry publishes the set, so a front end offers what the core accepts
    let reg = gcs_core::report::registry_json();
    let arc = reg.get("types").unwrap().arr().iter()
        .find(|t| t.get("name").unwrap().as_str() == "TangentArcLine")
        .expect("tangent_arc_line in the registry")
        .clone();
    let words = arc.get("words").unwrap().arr()[2].arr().iter()
        .map(|w| w.as_str().to_string())
        .collect::<Vec<_>>();
    assert_eq!(words, ["start", "end"]);
}

/// **A distance from a line is a magnitude, and which side is a word** (issue #48, item 4).
///
/// The sign used to say the side, and no reader could check it: positive is left of the line's
/// own `p1 → p2`, which is nowhere on the drawing.  Now the number is a magnitude — both sides
/// are solutions, and the seed picks between them, which is what every other sketcher does and
/// what P3 already says a seed may do — and `side:` is how a statement pins one.
#[test]
fn a_distance_from_a_line_is_a_magnitude() {
    const AXIS: &str = "\
unit mm
use std
in std.front {
a := point
b := point
ax := line(a, b)
fix(x == 0, y == 0) a
fix(x == 40, y == 0) b
}
";
    let solved = |src: &str| {
        let (e, d) = read(src);
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        sk.point_xy(2).1
    };
    // seeded below, it stays below; seeded above, above.  One statement, two solutions, and the
    // seed is what chooses — the drawing says where it was drawn
    assert!((solved(&format!("{AXIS}in std.front {{\np := point hint(x: 10, y: -3)\np distance(12) ax\n}}\n")) + 12.0).abs() < 1e-6);
    assert!((solved(&format!("{AXIS}in std.front {{\np := point hint(x: 10, y: 3)\np distance(12) ax\n}}\n")) - 12.0).abs() < 1e-6);
    // pinned, the seed does not get a say
    assert!((solved(&format!("{AXIS}in std.front {{\np := point hint(x: 10, y: 3)\np distance(12, side: right) ax\n}}\n")) + 12.0).abs() < 1e-6);
    assert!((solved(&format!("{AXIS}in std.front {{\np := point hint(x: 10, y: -3)\np distance(12, side: left) ax\n}}\n")) - 12.0).abs() < 1e-6);

    // and the minus is refused *by value*, so a component handed one is caught at the call
    let (_, d) = read(&format!("{AXIS}in std.front {{\np := point hint(x: 10, y: 3)\np distance(-12) ax\n}}\n"));
    let said = "E040: a point_line_distance is a magnitude and cannot be negative, and which \
                way is a word: write `distance(12, side: right)`";
    assert!(d.iter().any(|m| m == said), "{d:?}");
    let (_, d) = read(&format!(
        "{AXIS}component Off(o: point, ax: line, v: Length) {{\n\
           p := point hint(x: o.x, y: o.y + 20mm)\n\
           p distance(v) ax\n\
         }}\n\
         f := Off(a, ax, v: -8mm) in std.front\n"
    ));
    assert!(d.iter().any(|m| m.contains("cannot be negative")), "{d:?}");
}

/// A side may be passed to a component, since a helper that places a point either side of an
/// axis has to be told which — as a word, not as the ±1 it would otherwise multiply by, which is
/// the unreadable idiom one level down.
#[test]
fn a_component_takes_a_side() {
    const SRC: &str = "\
unit mm
use std
component Off(o: point, ax: line, d: Length, s: Side) {
p := point hint(x: o.x, y: o.y + 20mm)
p distance(d, side: s) ax
}
in std.front {
a := point
b := point
ax := line(a, b)
fix(x == 0, y == 0) a
fix(x == 40, y == 0) b
}
";
    let at = |src: &str, i: usize| {
        let (e, d) = read(src);
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        sk.point_xy(i).1
    };
    let both = format!("{SRC}in std.front {{\nl := Off(a, ax, d: 12, s: left)\nr := Off(a, ax, d: 12, s: right)\n}}\n");
    assert!((at(&both, 2) - 12.0).abs() < 1e-6);
    assert!((at(&both, 3) + 12.0).abs() < 1e-6);
    // a word that is not a side, and a number where a side was wanted
    let (_, d) = read(&format!("{SRC}in std.front {{\nl := Off(a, ax, d: 12, s: sideways)\n}}\n"));
    assert!(d.iter().any(|m| m.contains("`s` is a side: `left` or `right`, not `sideways`")), "{d:?}");
    let (_, d) = read(&format!("{SRC}in std.front {{\nl := Off(a, ax, d: 12, s: 1)\n}}\n"));
    assert!(d.iter().any(|m| m.contains("is a side")), "{d:?}");
}

/// The run, the rise and the angle keep their signs — there the sign is arithmetic, and a
/// component computes it — but each gains the word that says the same thing in the open.
#[test]
fn a_direction_and_a_sense_are_words() {
    const PAIR: &str = "\
unit mm
use std
in std.front {
a := point
b := point hint(x: 40, y: 10)
fix(x == 0, y == 0) a
}
";
    let at = |src: &str| {
        let (e, d) = read(src);
        assert!(d.is_empty(), "{d:?}");
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        sk.point_xy(1)
    };
    assert!((at(&format!("{PAIR}a distance(60, along: left) b\n")).0 + 60.0).abs() < 1e-6);
    assert!((at(&format!("{PAIR}a distance(60, along: right) b\n")).0 - 60.0).abs() < 1e-6);
    assert!((at(&format!("{PAIR}a distance(60, along: down) b\n")).1 + 60.0).abs() < 1e-6);
    // `along: x` still means what it always meant, so an old document is untouched
    assert!((at(&format!("{PAIR}a distance(-60, along: x) b\n")).0 + 60.0).abs() < 1e-6);

    // and an angle: `sense: cw` is the minus, and the drawing shows what the statement makes
    let lines = "\
unit mm
use std
in std.front {
o := point
p := point
q := point hint(x: 7, y: 7)
l1 := line(o, p)
l2 := line(o, q)
fix(x == 0, y == 0) o
fix(x == 10, y == 0) p
}
";
    let (e, d) = read(&format!("{lines}l1 angle(30, sense: cw) l2\n"));
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (x, y) = sk.point_xy(2);
    assert!(y < 0.0 && (y.atan2(x).to_degrees() + 30.0).abs() < 1e-6, "({x}, {y})");
    assert_eq!(gcs_core::io::dimension_text(&sk.user_constraints()[0]).as_deref(), Some("-30°"));
}

/// The word says which way whether the number is stated or a free variable's: `side: right`
/// over `k`, a name nothing defines, is the right-hand side wherever the point was seeded, as it
/// is over 30 — the free twin reads the word's sign as the stated kernel does.  It used to drop
/// it, and the point stayed on whichever side its seed was.
#[test]
fn a_free_dimension_keeps_its_word() {
    // `k` is 30, stated by a rise whose sign is arithmetic
    const AXIS: &str = "\
unit mm
use std
in std.front {
a := point
b := point
q := point
ax := line(a, b)
fix(x == 0, y == 0) a
fix(x == 40, y == 0) b
fix(x == 0, y == 30) q
}
param k: Length
in std.front {
a distance(k, along: up) q
}
";
    let at = |src: &str, name: &str| {
        let (e, d) = read(src);
        // `k` is a declared unknown: nothing to say
        assert!(d.is_empty(), "{d:?}");
        let i = e.map.ent_named(name).unwrap().i();
        let mut sk = e.sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        sk.point_xy(i)
    };
    // each read with the number stated and with it free, seeded either side: one answer
    // a point from a line, a line from a line, and a run
    let cases = [
        "p := point hint(x: 10, y: {y})\np distance({d}, side: {w}) ax\n",
        "l := line(hint(x: 0, y: {y}), hint(x: 40, y: {y}))\nl distance({d}, side: {w}) ax\n\
         horizontal l\nl.p1 distance(0, along: x) a\n",
        "p := point hint(x: {y}, y: 5)\na distance({d}, along: {w}) p\n",
    ];
    for (k, case) in cases.iter().enumerate() {
        let words = if k == 2 { ["left", "right"] } else { ["right", "left"] };
        for w in words {
            let place = |d: &str, y: i32| {
                let src = case.replace("{y}", &y.to_string()).replace("{d}", d).replace("{w}", w);
                let moved = if k == 1 { "l.p1" } else { "p" };
                let (x, yy) = at(&format!("{AXIS}in std.front {{\n{src}\n}}\n"), moved);
                if k == 2 { x } else { yy }
            };
            let stated = place("30", 3);
            assert!((stated.abs() - 30.0).abs() < 1e-6, "{case} {w}: {stated}");
            for y in [3, -3] {
                assert!((place("30", y) - stated).abs() < 1e-6, "{case} {w}, stated, seeded at {y}");
                assert!((place("k", y) - stated).abs() < 1e-6, "{case} {w}, free, seeded at {y}");
            }
        }
    }
    // and an angle's sense: `t` is 30 degrees, by a line grounded there
    let lines = "\
unit mm
use std
in std.front {
o := point
p := point
q := point hint(x: 7, y: 7)
r := point
l1 := line(o, p)
l2 := line(o, q)
l3 := line(o, r)
fix(x == 0, y == 0) o
fix(x == 10, y == 0) p
fix(x == 8.660254037844387, y == 5) r
}
param t: Angle
in std.front {
l1 angle(t) l3
}
";
    let (x, y) = at(&format!("{lines}l1 angle(t, sense: cw) l2\n"), "q");
    assert!(y < 0.0 && (y.atan2(x).to_degrees() + 30.0).abs() < 1e-6, "({x}, {y})");
}

/// Across planes a word means space: a word with no meaning there is E062 and a selector that
/// names a direction in a plane is E040 — while the same words inside one plane are what they
/// always were.
#[test]
fn a_word_across_views_with_no_meaning_in_space_is_refused() {
    let views = "use std\n\
                 in std.front {\na := point hint(x: 5, y: 5)\nla := line(hint(x: 0, y: 0), hint(x: 20, y: 5))\n}\n\
                 in std.side {\nb := point hint(x: 10, y: 5)\nlb := line(hint(x: 0, y: 0), hint(x: 20, y: 9))\n}\n";
    for (stmt, code, needle) in [
        ("a horizontal b", "E062", "no meaning in space"),
        ("a vertical b", "E062", "no meaning in space"),
        ("a distance(4, along: y) b", "E062", "no meaning in space"),
        ("la angle(30, sense: cw) lb", "E040", "unsigned"),
        ("la distance(3, side: right) lb", "E040", "no sides"),
    ] {
        let (_, d) = read(&format!("{views}{stmt}\n"));
        assert!(d.iter().any(|m| m.starts_with(code) && m.contains(needle)), "{stmt}: {d:?}");
    }
    // within one view the page's words stand, and across views the ones space has are accepted,
    // the midpoint and the mirror in a line among them
    for stmt in ["a horizontal la.p1", "a distance(4, along: y) la.p1", "a distance(8) b", "la angle(40) lb",
                 "a symmetry(lb) la.p1", "a midpoint lb"] {
        let (_, d) = read(&format!("{views}{stmt}\n"));
        assert!(d.is_empty(), "{stmt}: {d:?}");
    }
}

/// Cones and cylinders take the words they have kernels for and say so for the rest: a
/// line on either (a generator) is said of its points, two cones touch at a point the statement
/// names, and each is built about a line already drawn in a view.
#[test]
fn what_a_cone_or_a_cylinder_cannot_say_is_refused() {
    let views = "use std\n\
                 in std.front {\na := point hint(x: 5, y: 5)\nla := line(hint(x: 0, y: 0), hint(x: 0, y: 20))\n}\n\
                 in std.side {\nlb := line(hint(x: 0, y: 0), hint(x: 20, y: 9))\n}\n\
                 k := cone(axis: la) hint(half: 30deg)\nk2 := cone(axis: lb) hint(half: 20deg)\n\
                 c := cylinder(axis: la) hint(r: 5)\n";
    for (stmt, code, needle) in [
        ("lb coincident k", "E040", "a line on a cone or a cylinder"),
        ("lb coincident c", "E040", "a line on a cone or a cylinder"),
        ("k tangent lb", "E040", "a line touches a cylinder"),
        ("lb tangent c", "E040", "the cylinder first"),
        ("k tangent k2", "E040", "names it"),
        ("angle(20deg) c", "E040", "does not apply to a cylinder"),
        ("radius(-2) c", "E040", "magnitude"),
        ("bad := cone(axis: a)", "E103", "axis is a line"),
        ("bad := cylinder", "E103", "built about a line"),
    ] {
        let (_, d) = read(&format!("{views}{stmt}\n"));
        assert!(d.iter().any(|m| m.starts_with(code) && m.contains(needle)), "{stmt}: {d:?}");
    }
    for stmt in ["a coincident k", "a coincident c", "radius(4) c", "angle(25deg) k", "c tangent lb",
                 "m := point hint(x: 3, y: 3) in std.front\nk tangent(m) k2"] {
        let (_, d) = read(&format!("{views}{stmt}\n"));
        assert!(d.is_empty(), "{stmt}: {d:?}");
    }
}

/// Three of §16.1's codes, each said where its mistake is written: a component reached again
/// while it is still being expanded (E003), `next` with no `cycle` to name a sibling in (E020),
/// and a block's binder over a name already in scope (E002).
#[test]
fn a_cycle_a_stray_next_and_a_shadowing_binder_have_their_codes() {
    use crate::common::refused;
    refused("use std\ncomponent A(p: point) {\n  b := B(p)\n}\ncomponent B(p: point) {\n  a := A(p)\n}\n\
             in std.front {\n  o := point\n  x := A(o)\n}\n",
            "E003", "`A` instantiates itself through `B`", "a := A(p)");
    refused("use std\nin std.front {\n  repeat 3 {\n    p := point\n    p distance(5) next.p\n  }\n}\n",
            "E020", "`next` names the next copy round a `cycle`", "next.p");
    refused("use std\ni := 4\nin std.front {\n  repeat 3 as i {\n    p := point\n  }\n}\n",
            "E002", "`i` is already a name here", "i");
}
