//! **Relation words, and `use M (names)`** (Solvent §9.9, §14.4 [0.48]; issue #94): a word a
//! file defines is its body with the operands and parameters put in, closed over them, reported
//! where it is written and described as written; a `use` reads the names it lists bare.  The
//! standard library's `horizontal` and `vertical` between points are such words.
use gcs_core::constraints::CKind;
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;
use std::collections::BTreeMap;

use crate::common::{ent, read, refused, said};

/// Where the point the document calls `n` stands once solved.
fn solved_at(e: &Elaborated, names: &[&str]) -> Vec<[f64; 3]> {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    names.iter().map(|n| sk.world_point(ent(e, n).i())).collect()
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

const POINTS: &str = "\
in std.front {
  p := point hint((0, 0))
  q := point hint((10, 3))
  r := point hint((12, 9))
  fix((0, 0)) p
  p distance(10) q
";

#[test]
fn std_horizontal_and_vertical_are_imported_words() {
    let e = read(&format!("use std (horizontal, vertical)\n{POINTS}  p horizontal q\n  q vertical r\n  p distance(14) r\n}}\n"));
    let at = solved_at(&e, &["q", "r"]);
    assert!((at[0][2]).abs() < 1e-9 && (at[0][0] - 10.0).abs() < 1e-9, "{at:?}");
    assert!((at[1][0] - 10.0).abs() < 1e-9, "{at:?}");
    // the constraint is the level its body states, and it is described as written
    let levels: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(levels.iter().filter(|k| **k == CKind::Level).count(), 2);
    let s = said(&e);
    assert!(s.contains(&"p horizontal q".to_string()), "{s:?}");
    assert!(s.contains(&"q vertical r".to_string()), "{s:?}");
    // the same statement written in the core's own words reads back in them
    let e = read(&format!("use std\n{POINTS}  p level(up) q\n}}\n"));
    assert!(said(&e).contains(&"p level(up) q".to_string()), "{:?}", said(&e));
}

#[test]
fn a_word_not_imported_is_refused_naming_the_import() {
    refused(
        &format!("use std\n{POINTS}  p horizontal q\n}}\n"),
        "E102",
        "use std (horizontal)",
        "horizontal",
    );
    // `horizontal` of a line is the language's own and needs nothing
    let e = read("use std\nin std.front {\n  a := point\n  l := line(a, hint((5, 1)))\n  horizontal l\n}\n");
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));
    // and a word nothing defines anywhere
    refused(&format!("use std\n{POINTS}  p sideways q\n}}\n"), "E102", "no relation word `sideways`", "sideways");
}

#[test]
fn a_defined_word_with_a_number_reads_its_argument() {
    let src = format!("\
use std (horizontal)
a above(d) b := b distance(d, along: up) a
h := 7
{POINTS}  p horizontal q
  r above(d: h) q
  r level(right) q
}}
");
    let e = read(&src);
    let at = solved_at(&e, &["r"]);
    assert!(near(at[0], [10.0, 0.0, 7.0]), "{at:?}");
    // the dimension is the statement's argument: its callout draws `h`, and `describe` the word
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Ordinate).unwrap();
    assert_eq!(c.written.as_deref(), Some("h"));
    assert!(said(&e).contains(&"r above(d: h) q".to_string()), "{:?}", said(&e));
    // and editing it edits the argument: a number over a param is the param's line
    let lit = src.replace("r above(d: h) q", "r above(d: 7) q");
    let e = read(&lit);
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Ordinate).unwrap();
    let edit = gcs_core::edit::set_dimension(&e, &e.program, c.id, "d", "9");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("r above(d: 9) q"), "{}", edit.text);
}

#[test]
fn a_word_parameter_stands_where_a_word_does() {
    let e = read(&format!("\
use std
a toward(w) b := a level(w) b
{POINTS}  p toward(w: up) q
  q toward(w: right) r
  p distance(13) r
}}
"));
    let at = solved_at(&e, &["q", "r"]);
    assert!(at[0][2].abs() < 1e-9 && (at[1][0] - 10.0).abs() < 1e-9, "{at:?}");
}

#[test]
fn a_prefix_word_and_a_word_in_terms_of_another() {
    let e = read("\
use std
long(d) l := length(d) l
a above(d) b := b distance(d, along: up) a
a twice(d) b := a above(d: d * 2) b
in std.front {
  p := point hint((0, 0))
  q := point hint((2, 9))
  fix((0, 0)) p
  l := line(p, q)
  long(d: 10) l
  q twice(d: 3) p
}
");
    let at = solved_at(&e, &["q"]);
    assert!((at[0][2] - 6.0).abs() < 1e-9 && (at[0][0] - 8.0).abs() < 1e-9, "{at:?}");
    let s = said(&e);
    assert!(s.contains(&"long(d: 10) l".to_string()) && s.contains(&"q twice(d: 3) p".to_string()), "{s:?}");
}

#[test]
fn a_claim_keeps_its_word() {
    let e = read(&format!("use std (horizontal)\n{POINTS}  p distance(0, along: v) q\n}}\n")
        .replace("p distance(0, along: v) q", "p level(up) q\n  claim p horizontal q"));
    assert!(said(&e).contains(&"claim p horizontal q".to_string()), "{:?}", said(&e));
}

#[test]
fn the_arguments_are_given_by_label() {
    let src = |stmt: &str| format!("use std\na above(d) b := b distance(d, along: up) a\n{POINTS}  {stmt}\n}}\n");
    refused(&src("r above(e: 5) q"), "E040", "has no parameter `e`", "e");
    refused(&src("r above q"), "E040", "needs `d`", "above");
    refused(&src("r above(5, d: 5) q"), "E004", "by label where it has more than one", "5");
    // a word of two keeps them labelled, so nothing given in order is ambiguous
    let two = "use std\na offset(dx, dy) b := {\n  b distance(dx, along: right) a\n  b distance(dy, along: up) a\n}\n";
    refused(&format!("{two}{POINTS}  r offset(3, 4) q\n}}\n"), "E004", "`offset(dx: 3, …)`", "3");
    refused(&format!("{two}{POINTS}  r offset(3) q\n}}\n"), "E004", "by label", "3");
    // and a word of none takes nothing
    let none = "use std\na same b := a coincident b\n";
    refused(&format!("{none}{POINTS}  r same(3) q\n}}\n"), "E040", "`same` has no parameters", "3");
}

/// **A word of one parameter takes it bare** (#463), as a built-in takes its one number
/// (`distance(80)`): the same statement as the labelled one, described and edited as written —
/// and passed on bare to another word, a number or an entity.
#[test]
fn a_word_of_one_parameter_takes_it_bare() {
    let src = |stmt: &str| format!("\
use std (horizontal)
a above(d) b := b distance(d, along: up) a
a twice(d) b := a above(d * 2) b
{POINTS}  p horizontal q
  {stmt}
  r level(right) q
}}
");
    for stmt in ["r above(7) q", "r above(d: 7) q", "r twice(3.5) q"] {
        let e = read(&src(stmt));
        let at = solved_at(&e, &["r"]);
        assert!(near(at[0], [10.0, 0.0, 7.0]), "{stmt}: {at:?}");
        assert!(said(&e).contains(&stmt.to_string()), "{stmt}: {:?}", said(&e));
    }
    // the dimension is the bare argument: an edit of it writes there
    let e = read(&src("r above(7) q"));
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Ordinate).unwrap();
    let edit = gcs_core::edit::set_dimension(&e, &e.program, c.id, "d", "9");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("r above(9) q"), "{}", edit.text);
    // an entity, given bare and passed on bare
    let e = read(&format!("\
use std
a both(along) b := {{
  a parallel along
  b parallel along
}}
a level_with(e) b := a both(e) b
{LINES}  l1 both(h) l2
  l2 level_with(std.x) l2
}}
"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!(rise(&sk, &e, "l1").abs() < 1e-9 && rise(&sk, &e, "l2").abs() < 1e-9, "{:?}", said(&e));
}

/* -- the plane an operand is drawn in (#463) ---------------------------------------------- */

/// `std.Spin` as a word: the axis through `o` square to the view `o` is drawn in, which the word
/// reads from its operand, `o.plane` — written with the plane's normal, or square to the plane.
fn spins(square: &str) -> String {
    format!("\
unit mm
use std
f spins(ratio) o := {{
  o coincident f.axis
  {square}
  fix(ratio == ratio, advance == 0mm) f
}}
in std.top {{
  o := point
  fix((0, 0)) o
  c := point
  fix((30, 0)) c
  x := point
  fix((40, 0)) x
}}
arm := frame
arm spins(1) o
planet := frame
planet spins(-3) c
")
}

/// **A word reads the plane its operand is drawn in** (#463): `o.plane` is the view a point is
/// drawn in, so a word about it needs no plane given — and is still closed over what it is
/// given.  The planet is turned about `c` square to the top, three times a radian the other way.
#[test]
fn a_word_reads_the_plane_its_operand_is_drawn_in() {
    use gcs_core::motion::Family;
    for square in ["f.axis parallel o.plane.normal", "f.axis perpendicular o.plane"] {
        let e = read(&spins(square));
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).success, "{square}");
        let planet = sk.motions.iter().position(|m| m.name == "planet").expect("a motion `planet`");
        let family = Family::read(&sk, planet).unwrap();
        let x = sk.world_point(ent(&e, "x").i());
        for deg in [-60.0, 25.0, 90.0] {
            let tau = f64::to_radians(deg);
            let (s, k) = (-3.0 * tau).sin_cos();
            let want = [30.0 + 10.0 * k, 10.0 * s, 0.0];
            let got = family.at(tau).unwrap().point(x);
            assert!(near(got, want), "{square}, τ = {deg}: {got:?} against {want:?}");
        }
    }
}

/// A point standing in space is drawn in no plane: its `plane` is refused where the word is used,
/// and where a relation reads it directly, at the reference.
#[test]
fn a_point_in_space_has_no_plane() {
    let src = format!("{}a := point hint((1, 2, 7))\nbad := frame\nbad spins(2) a\n",
        spins("f.axis parallel o.plane.normal"));
    let (prog, _, _) = gcs_core::library::parse_linked(&src);
    let e = elaborate(&prog);
    let at: Vec<(&str, &str)> = e.errors()
        .map(|d| (d.code.as_str(), d.span.slice(prog.text())))
        .collect();
    assert_eq!(at, [("E040", "bad spins(2) a")], "{:?}", e.diags);
    assert!(e.errors().all(|d| d.message.contains("standing in space is drawn in no plane")));
    refused(
        "unit mm\nuse std\na := point hint((1, 2, 7))\nl := axis hint(dir: (0, 0, 1))\nl perpendicular a.plane\n",
        "E040",
        "drawn in no plane",
        "a.plane",
    );
}

#[test]
fn an_error_inside_the_expansion_is_said_at_the_word() {
    // the body's relation does not relate a point and a line: said where `above` is written
    refused(
        "use std\na above(d) b := b distance(d, along: up) a\nin std.front {\n  p := point\n  l := line\n  p above(d: 3) l\n}\n",
        "E040",
        "",
        "above",
    );
}

#[test]
fn an_operand_that_names_nothing_is_said_once() {
    let (prog, _, _) = gcs_core::library::parse_linked(&format!("use std (horizontal)\n{POINTS}  p horizontal nobody\n}}\n"));
    let e = elaborate(&prog);
    let said: Vec<&str> = e.errors().filter(|d| d.code.as_str() == "E101").map(|d| d.span.slice(prog.text())).collect();
    assert_eq!(said, ["nobody"], "{:?}", e.diags);
}

#[test]
fn a_definition_is_checked_where_it_is_written() {
    let src = |def: &str| format!("use std\n{def}\n{POINTS}}}\n");
    // closed over its operands and parameters
    refused(&src("flat l := l perpendicular z"), "E101", "not an operand or a parameter of `flat`", "z");
    refused(&src("a far b := a distance(w) b"), "E101", "`w` is not an operand", "w");
    // a word of the language's own, of the same fixity; a keyword whatever the fixity
    refused(&src("a level b := a coincident b"), "E071", "already a constraint word", "level");
    refused(&src("radius l := l coincident l"), "E071", "already a constraint word", "radius");
    refused(&src("a line b := a coincident b"), "E071", "a word of the language's own", "line");
    // twice in one file
    refused(&src("a same b := a coincident b\na same b := a coincident b"), "E071", "defined twice", "same");
    // a parameter read as a number and as a word
    refused(&src("a odd(s) b := a distance(s, along: s) b"), "E040", "as a number and as a word", "s");
    // defined in terms of itself: said where it is written
    refused(
        &format!("use std\na loop b := a loop b\n{POINTS}  p loop q\n}}\n"),
        "E003",
        "in terms of itself",
        "loop",
    );
}

#[test]
fn a_word_is_defined_at_the_top_of_a_file() {
    let (_, errs) = parse("component C(a: point, b: point) {\n  a same b := a coincident b\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("at the top of a file")), "{errs:?}");
    let (_, errs) = parse("a same b := a coincident b class shown\n");
    assert!(errs.iter().any(|e| e.message.contains("the relation alone")), "{errs:?}");
}

/* -- entity parameters (#250) ------------------------------------------------------------- */

/// Two lines drawn in the front plane, free to turn, and a third held level.
const LINES: &str = "\
in std.front {
  a := point hint((0, 0))
  fix((0, 0)) a
  b := point hint((9, 2))
  c := point hint((0, 5))
  d := point hint((7, 9))
  h0 := point hint((0, -5))
  fix((0, -5)) h0
  h1 := point hint((8, -4))
  h := line(h0, h1)
  h0 level(up) h1
  h0 distance(8) h1
  l1 := line(a, b)
  l2 := line(c, d)
";

/// How far line `l` turns from the page's right, in the front plane: the sine of its rise.
fn rise(sk: &gcs_core::model::Sketch, e: &Elaborated, l: &str) -> f64 {
    let (p, q) = crate::common::ends(sk, ent(e, l));
    let d = crate::common::unit([q[0] - p[0], q[1] - p[1], q[2] - p[2]]);
    d[2]
}

/// A word's labelled parameter may be an entity: bound as an alias in the body, as an operand
/// is, whatever it is given — a line by its name, an axis by its path.
#[test]
fn an_entity_parameter_is_bound_as_an_operand_is() {
    let word = "a both(along) b := {\n  a parallel along\n  b parallel along\n}\n";
    for given in ["h", "std.x"] {
        let src = format!("use std\n{word}{LINES}  l1 both(along: {given}) l2\n}}\n");
        let e = read(&src);
        let mut sk = e.sketch.clone();
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "{given}: {}", r.message);
        assert!(rise(&sk, &e, "l1").abs() < 1e-9 && rise(&sk, &e, "l2").abs() < 1e-9, "{given}");
        // described as written
        assert!(said(&e).contains(&format!("l1 both(along: {given}) l2")), "{:?}", said(&e));
    }
    // and a direction word stands where the parameter does, as it did before
    let word = "a toward(w) b := a level(w) b";
    let e = read(&format!("use std\n{word}\n{POINTS}  p toward(w: std.z) q\n}}\n"));
    let at = solved_at(&e, &["p", "q"]);
    assert!(at[1][2].abs() < 1e-9, "{at:?}");
}

/// One that names nothing is said once, at the argument; one that is no name is refused there.
#[test]
fn an_entity_parameter_naming_nothing_is_said_at_the_argument() {
    let word = "a both(along) b := {\n  a parallel along\n  b parallel along\n}\n";
    let src = format!("use std\n{word}{LINES}  l1 both(along: nobody) l2\n}}\n");
    let (prog, _, _) = gcs_core::library::parse_linked(&src);
    let e = elaborate(&prog);
    let at: Vec<&str> = e.errors()
        .filter(|d| d.code.as_str() == "E101")
        .map(|d| d.span.slice(prog.text()))
        .collect();
    assert_eq!(at, ["nobody"], "{:?}", e.diags);
    refused(
        &format!("use std\n{word}{LINES}  l1 both(along: 3 + 4) l2\n}}\n"),
        "E040",
        "`along` is an entity in `both`",
        "3 + 4",
    );
}

/// A parameter passed on whole to another word is what that word reads it as: an entity here, a
/// number in the other; and an operand passed on so is an entity.
#[test]
fn a_parameter_passed_on_is_what_the_inner_word_reads() {
    let src = format!("\
use std
a both(along) b := {{
  a parallel along
  b parallel along
}}
a level_with(e) b := a both(along: e) b
a alongside b := a both(along: b) a
p up_by(d) q := p distance(d, along: up) q
p raised(e) q := p up_by(d: e) q
{LINES}  l1 level_with(e: h) l1
  l2 alongside h
  c raised(e: 5) a
}}
");
    let e = read(&src);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    assert!(rise(&sk, &e, "l1").abs() < 1e-9 && rise(&sk, &e, "l2").abs() < 1e-9, "{:?}", said(&e));
    let z = |n: &str| sk.world_point(ent(&e, n).i())[2];
    assert!((z("a") - z("c") - 5.0).abs() < 1e-9);
}

/// A guide `g` along the front plane's right, held, and a plane through its start.
fn square_src(stated: &str) -> String {
    format!("\
use std (square)
in std.front {{
  a := point hint((0, 0))
  b := point hint((10, 0))
  fix((0, 0)) a
  fix((10, 0)) b
  g := line(a, b)
}}
n := plane(u: hint(dir: (0, 1, 0.2)), v: hint(dir: (0.1, 0, 1)))
n.origin coincident a
{stated}
")
}

/// **`P square(up: d) l`** (#250): a plane square to `l`, its `v` along `d` — the three
/// statements a plane's unsquared axes take, said once.  Through a point, nothing is left to move,
/// and it solves where the three written out do.
#[test]
fn square_stands_a_plane_square_to_a_line() {
    let worded = read(&square_src("n square(up: std.z) g"));
    let three = "n.v parallel std.z\nn.u perpendicular g\nn.u perpendicular std.z";
    let written = read(&square_src(three));
    let mut at = Vec::new();
    for e in [&worded, &written] {
        let mut sk = e.sketch.clone();
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "{}", r.message);
        let d = gcs_core::diagnose::diagnose(&mut sk, Default::default());
        assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
        assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
        let b = sk.basis(ent(e, "n").i());
        assert!((b.normal()[0].abs() - 1.0).abs() < 1e-9, "its normal along g: {:?}", b.normal());
        assert!((b.v[2].abs() - 1.0).abs() < 1e-9, "its v along up: {:?}", b.v);
        at.push((b.o, b.u, b.v));
    }
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9);
    let alike = close(at[0].0, at[1].0) && close(at[0].1, at[1].1) && close(at[0].2, at[1].2);
    assert!(alike, "{at:?}");
    assert!(said(&worded).contains(&"n square(up: std.z) g".to_string()), "{:?}", said(&worded));
}

/// A formal named as a direction word is, given on, the formal (`square(up: up)`), and a
/// component standing a plane square to its guide says it once.
#[test]
fn a_formal_given_on_is_the_formal_not_the_direction_word() {
    let station = "\
component Station(g: line, at: point, up: axis) {
  n := plane(u: hint(dir: (0, 1, 0.2)), v: hint(dir: (0.1, 0, 1)))
  n.origin coincident at
  n square(up: up) g
}
";
    let src = square_src("").replace("n := plane", "st := Station(g, a, up: std.z)\nn := plane");
    let e = read(&format!("{station}{src}"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let b = sk.basis(ent(&e, "st.n").i());
    assert!((b.normal()[0].abs() - 1.0).abs() < 1e-9 && (b.v[2].abs() - 1.0).abs() < 1e-9, "{b:?}");
}

/// `up` square to `l` is the drawing's to keep, and no conflict where it is not: the three
/// statements still hold the plane, its normal `l`'s part square to `up`.
#[test]
fn square_to_a_line_up_is_not_square_to_leans_with_it() {
    let e = read(&square_src("n square(up: std.z) g").replace("fix((10, 0)) b", "fix((10, 3)) b"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = gcs_core::diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    let b = sk.basis(ent(&e, "n").i());
    assert!((b.normal()[0].abs() - 1.0).abs() < 1e-9, "{:?}", b.normal());
    assert!((b.v[2].abs() - 1.0).abs() < 1e-9, "{:?}", b.v);
}

/* -- imports ------------------------------------------------------------------------------- */

fn shelf() -> BTreeMap<&'static str, &'static str> {
    let mut m = BTreeMap::new();
    m.insert("lib.words", "\
a above(d) b := b distance(d, along: up) a
gap := 4
dims := {w: 6, h: 2}
component Rung(a: point, b: point, w: Length) {
  a above(d: w) b
}
");
    m.insert("lib.other", "gap := 9\ncomponent Rung(a: point) { }\n");
    // a module importing a word for itself: its importer has not
    m.insert("lib.uses", "use std (horizontal)\ncomponent Flat(a: point, b: point) {\n  a horizontal b\n}\n");
    m
}

fn linked(src: &str) -> (Elaborated, Vec<gcs_core::program::Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    let diags = link(&mut prog, &mut |name| {
        shelf.get(name).map(|t| t.to_string()).or_else(|| gcs_core::library::resolve(name))
    });
    (elaborate(&prog), diags)
}

fn ok(src: &str) -> Elaborated {
    let (e, diags) = linked(src);
    assert!(diags.is_empty(), "{diags:?}");
    let saw: Vec<String> = e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect();
    assert!(e.ok(), "{saw:?}\n{src}");
    e
}

fn errs(src: &str) -> Vec<(String, String, String)> {
    let (e, diags) = linked(src);
    e.diags
        .iter()
        .chain(&diags)
        .filter(|d| d.code.as_str().starts_with('E'))
        .map(|d| (d.code.as_str().to_string(), d.message.clone(), d.span.slice(e.program.text()).to_string()))
        .collect()
}

fn has(src: &str, code: &str, needle: &str, at: &str) {
    let saw = errs(src);
    assert!(
        saw.iter().any(|(c, m, s)| c == code && m.contains(needle) && s == at),
        "expected {code} `{needle}` at `{at}`\n{src}\n{saw:#?}"
    );
}

const BODY: &str = "in std.front {\n  p := point hint((0, 0))\n  q := point hint((3, 5))\n  fix((0, 0)) p\n";

#[test]
fn any_name_a_module_defines_may_be_imported() {
    // a word, a component, a value and a group, bare beside their full paths
    let e = ok(&format!("\
use std
use lib.words (above, Rung, gap, dims)
w := gap + dims.w + lib.words.dims.h
{BODY}  q above(d: lib.words.gap) p
  q distance(w, along: u) p
}}
"));
    let at = solved_at(&e, &["q"]);
    assert!(near(at[0], [-12.0, 0.0, 4.0]), "{at:?}");
    let e = ok(&format!("use std\nuse lib.words (Rung)\n{BODY}  r := Rung(q, p, w: 2)\n  q level(right) p\n}}\n"));
    let at = solved_at(&e, &["q"]);
    assert!(near(at[0], [0.0, 0.0, 2.0]), "{at:?}");
    // the full path still works with nothing imported, for a component and a value
    ok(&format!("use std\nuse lib.words\n{BODY}  r := lib.words.Rung(q, p, w: lib.words.gap)\n}}\n"));
}

#[test]
fn an_import_is_per_file_and_there_is_no_prelude() {
    // `lib.uses` imports `horizontal` for its own body; the document has not
    ok(&format!("use std\nuse lib.uses\n{BODY}  f := lib.uses.Flat(p, q)\n}}\n"));
    has(&format!("use std\nuse lib.uses\n{BODY}  p horizontal q\n}}\n"), "E102", "use std (horizontal)", "horizontal");
    // a word is only ever bare: imported, or not written (`std` defines an `above` too, and the
    // file uses it first)
    has(&format!("use std\nuse lib.words\n{BODY}  q above(d: 1) p\n}}\n"), "E102", "use std (above)", "above");
}

#[test]
fn an_import_is_refused_where_it_is_written() {
    has(&format!("use std\nuse lib.words (nothing)\n{BODY}}}\n"), "E101", "`lib.words` defines no `nothing`", "nothing");
    has(&format!("use std\nuse lib.words (gap)\nuse lib.other (gap)\n{BODY}}}\n"), "E071", "from `lib.words` and from `lib.other`", "gap");
    has(&format!("use std\nuse lib.words (gap, gap)\n{BODY}}}\n"), "E071", "imported twice", "gap");
    has(&format!("use std\nuse lib.words (gap)\ngap := 3\n{BODY}}}\n"), "E071", "defined in this file", "gap");
    has(&format!("use std\nuse lib.words (Rung)\ncomponent Rung(a: point) {{ }}\n{BODY}}}\n"), "E071", "defined in this file", "Rung");
    has(&format!("use std (horizontal)\na horizontal b := a coincident b\n{BODY}}}\n"), "E071", "defined in this file", "horizontal");
}

#[test]
fn a_built_in_word_is_imported_by_no_one() {
    let mut shelf = shelf();
    shelf.insert("lib.pi", "pi := 3\n");
    let (mut prog, _) = parse("use lib.pi (pi)\n");
    link(&mut prog, &mut |n| shelf.get(n).map(|t| t.to_string()));
    let e = elaborate(&prog);
    assert!(
        e.errors().any(|d| d.code.as_str() == "E071" && d.message.contains("word of the language's own")),
        "{:?}",
        e.diags
    );
}

#[test]
fn a_module_reads_its_own_words_bare() {
    // `std` writes `horizontal` in its own components with no import of its own
    let std = gcs_core::library::module("std").unwrap();
    assert!(std.contains("a horizontal b := a level(up) b"));
    assert!(std.contains("k.center horizontal q"));
    // `lib.words`' `Rung` writes `above` bare, and a file importing only `Rung` draws it
    ok(&format!("use std\nuse lib.words (Rung)\n{BODY}  r := Rung(q, p, w: 2)\n}}\n"));
}
