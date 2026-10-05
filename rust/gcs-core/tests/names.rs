//! One namespace for a number's names, and **an unknown is declared** (issue #77).
//!
//! A number is named one way, `w := 60` — or, as one of the document's inputs, `param w := 60` —
//! and a dimension's number only reads names.  An unknown the solve answers for is declared too:
//! an input nothing binds (`param w: Length`) or a component's formal no call binds.  A name
//! nothing declares is E101 wherever it is read — in a dimension, a fold or a pin — and never an
//! unknown of its own making, which turned a misspelling into a degree of freedom.

use std::collections::BTreeMap;

use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;

const BASE: &str = "\
use std
in std.front {
a := point
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 40)
fix(x == 0, y == 0) a
a horizontal b
b vertical c
}
";

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

fn solved(e: Elaborated) -> gcs_core::model::Sketch {
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    sk
}

fn dist(sk: &gcs_core::model::Sketch, i: usize, j: usize) -> f64 {
    let (px, py) = sk.point_xy(i);
    let (qx, qy) = sk.point_xy(j);
    ((px - qx).powi(2) + (py - qy).powi(2)).sqrt()
}

/// An input is read like any other number: by dimensions, and by the values defined over it.
#[test]
fn an_input_is_read_like_any_number() {
    let src = format!("{BASE}param w := 60\na distance(w) b\nh := w / 2\nb distance(h) c\n");
    let (e, d) = read(&src);
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 1, 2) - 30.0).abs() < 1e-9);
}

/// A name defined as a value and as an input is declared twice, and that is the one thing said.
#[test]
fn a_name_defined_both_ways_is_declared_twice() {
    let (_, d) = read(&format!("w := 60\n{BASE}param w := 60\na distance(w) b\n"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].starts_with("E001") && d[0].contains("`w` is declared twice"), "{d:?}");
}

/// A bare name a component never declares is E101: a component's unknowns are its formals, and a
/// formal no call binds is the instance's own — `t1.w`, `t2.w` — so two instances have two.
#[test]
fn a_name_a_component_never_declares_is_refused() {
    let doc = format!(
        "component T(p: point, q: point) {{ p distance(w) q }}\n{BASE}d := point hint(x: 0, y: 40)\n\
         t1 := T(a, b)\nt2 := T(b, c)\nt3 := T(c, d)\n"
    );
    let (e, d) = read(&doc);
    assert!(d.iter().any(|m| m.starts_with("E101") && m.contains("`w` is not defined")), "{d:?}");
    assert!(e.sketch.free_vars.is_empty(), "{:?}", e.sketch.free_vars);
    // the same drawing with `w` a formal left unbound has one unknown per instance, and says
    // nothing: a declared formal is no misspelling
    let formal = doc.replace("q: point)", "q: point, w: Length)");
    let (e2, d2) = read(&formal);
    assert!(d2.is_empty(), "{d2:?}");
    let names: Vec<String> = e2.sketch.free_vars.keys().cloned().collect();
    assert_eq!(names, ["t1.w", "t2.w", "t3.w"]);
}

/// An input nothing binds is one unknown, however many dimensions read it, and is no warning.
#[test]
fn an_unbound_input_is_one_unknown() {
    let (e, d) = read(&format!(
        "{BASE}param w: Length\na distance(w) b\nb distance(w) c\na distance(2 * w) c\n"
    ));
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(e.sketch.free_vars.keys().collect::<Vec<_>>(), vec!["w"]);
    // undeclared, the same three dimensions are three refusals and no unknown
    let (e, d) = read(&format!("{BASE}a distance(w) b\nb distance(w) c\na distance(2 * w) c\n"));
    assert_eq!(d.iter().filter(|m| m.starts_with("E101")).count(), 3, "{d:?}");
    assert!(e.sketch.free_vars.is_empty());
}

/// The document's numbers reach a component through an explicit numeric argument.
#[test]
fn a_component_receives_an_input_as_an_argument() {
    let doc = format!(
        "component T(p: point, q: point, w: Length) {{ p distance(w / 2) q }}\n\
         {BASE}param w := 60\na distance(w) b\nt := T(b, c, w: w)\n"
    );
    let (e, d) = read(&doc);
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 1, 2) - 30.0).abs() < 1e-9);
}

/// A module's component cannot reach into the document that draws it: `w` there is the
/// instance's own unknown, whatever the caller calls its dimensions.
#[test]
fn a_modules_component_does_not_read_the_callers_names() {
    let mut shelf: BTreeMap<&str, &str> = BTreeMap::new();
    shelf.insert("lib.t", "component T(p: point, q: point, w: Length) { p distance(w) q }\n");
    let src = format!("use lib.t\n{BASE}param w := 60\na distance(w) b\nt := lib.t.T(b, c)\n");
    let (mut prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = link(&mut prog, &mut |name| shelf.get(name).map(|t| t.to_string()).or_else(|| gcs_core::library::resolve(name)));
    assert!(linked.is_empty(), "{linked:?}");
    let e = elaborate(&prog);
    // a formal left unbound is declared, so nothing is said of it
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    assert!(e.sketch.free_vars.contains_key("t.w"), "{:?}", e.sketch.free_vars);
}

/// An instance's unknown — a formal its call left unbound, `u.t.w` — is read by dotted path from
/// the body around it and from the sheet, like anything else an instance makes; a dotted name
/// nothing made is E101, as a bare one is.
#[test]
fn an_instances_unknown_is_read_by_its_dotted_path() {
    let doc = format!(
        "component T(p: point, q: point, w: Length) {{ p distance(w) q }}\n\
         component U(p: point, q: point, r: point) {{ t := T(p, q)\n  q distance(t.w / 2) r }}\n\
         {BASE}in std.front {{\nd := point hint(x: 0, y: 40)\n}}\nu := U(a, b, c)\nc distance(u.t.w) d\n\
         a distance(60) b\n"
    );
    let (e, d) = read(&doc);
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 0, 1) - 60.0).abs() < 1e-9);
    assert!((dist(&sk, 1, 2) - 30.0).abs() < 1e-9);
    assert!((dist(&sk, 2, 3) - 60.0).abs() < 1e-9);
    let (e, d) = read(&doc.replace("c distance(u.t.w) d", "c distance(u.t.ww) d"));
    assert!(d.iter().any(|m| m.starts_with("E101") && m.contains("`u.t.ww`")), "{d:?}");
    assert!(!e.sketch.free_vars.contains_key("u.t.ww"));
}

/// A name declared inside a `cycle` is each copy's own — `#N.k.w` — so a value defined in a
/// block is defined once per copy rather than N times over, and the document's unknown read in
/// the block is one, shared by every copy.
#[test]
fn a_block_copy_declares_its_own_names_and_shares_the_documents_unknowns() {
    let (e, d) = read(
        "\
use std
in std.front {
o := point
fix(x == 0, y == 0) o
}
param s: Length
in std.front {
cycle 2 { z := point hint(x: 5, y: 5)
  y := point hint(x: 9, y: 2)
x := point hint(x: 3, y: 8)
  w := 60
o distance(w) z
  o distance(w / 2) y
  o distance(s) x }
}
",
    );
    assert!(d.is_empty(), "{d:?}");
    // one shared unknown `s`, and no complaint about `w`
    assert_eq!(e.sketch.free_vars.keys().collect::<Vec<_>>(), vec!["s"]);
    let sk = solved(e);
    for k in 0..2 {
        assert!((dist(&sk, 0, 1 + 3 * k) - 60.0).abs() < 1e-9);
        assert!((dist(&sk, 0, 2 + 3 * k) - 30.0).abs() < 1e-9);
    }
    assert!((dist(&sk, 0, 3) - dist(&sk, 0, 6)).abs() < 1e-9);
}

/// An instance inside a block leaving a formal unbound is `#N.k.t.w`, which the expression
/// graph now reads — it used to stop at the `#`.
#[test]
fn an_unbound_formal_inside_a_block_is_a_name_the_graph_reads() {
    let (e, d) = read(
        "\
use std
component T(p: point, q: point, w: Length) { p distance(w) q }
in std.front {
o := point
fix(x == 0, y == 0) o
cycle 3 { a := point hint(x: 10, y: 0)
  t := T(o, a) }
}
",
    );
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(e.sketch.free_vars.len(), 3, "{:?}", e.sketch.free_vars);
    assert!(e.sketch.free_vars.keys().all(|k| k.starts_with('#') && k.ends_with(".t.w")));
}

/// A value may read an unknown, and is then that unknown scaled (`q := s * 2`), read by
/// dimensions as the unknown is; a value reading a name nothing declares is refused.
#[test]
fn a_value_over_an_unknown_is_the_unknown_scaled() {
    let src = format!("{BASE}param s: Length\nq := s * 2\na distance(q) b\nb distance(s) c\n");
    let (e, d) = read(&src);
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 0, 1) - 2.0 * dist(&sk, 1, 2)).abs() < 1e-9);
    let (_, d) = read(&format!("{BASE}q := s * 2\na distance(q) b\n"));
    assert!(!d.is_empty(), "{d:?}");
}

/// **A name declared over a built-in is said** (issue #48, item 2).  `tau := 35deg` read
/// 35° where the flattener substituted the text and a full turn where `expr::eval` worked a
/// number out, so one name had two values and the lever it turned stood at 360° with nothing
/// said.  W112 at every declaration of a number's name: a `param`, a formal, a block's index.
#[test]
fn a_name_that_shadows_a_built_in_is_said() {
    let w112 = |src: &str| -> Vec<String> {
        read(src).1.into_iter().filter(|m| m.starts_with("W112")).collect()
    };
    let d = w112(&format!("tau := 35deg\n{BASE}"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("`tau`") && d[0].contains("a `param`"), "{d:?}");
    // a formal, in a component nothing instantiates: the name is wrong wherever it is written
    let d = w112("component Lever(o: point, tau: Angle) { q := point hint(x: o.x + cos(tau)) }\n");
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("a formal"), "{d:?}");
    // a block's index
    let d = w112(&format!("{BASE}cycle 3 as pi {{ p := point hint(x: 10 * pi, y: 0) }}\n"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].contains("a block's index"), "{d:?}");
    // a function's name is built in as much as a constant's
    let d = w112(&format!("min := 3\n{BASE}"));
    assert!(d[0].contains("a built-in function"), "{d:?}");
    // an input is a declaration like any other
    let d = w112(&format!("param tau := 35deg\n{BASE}"));
    assert_eq!(d.len(), 1, "{d:?}");
    // and every other name is a name: said once, at the declaration, and about nothing else
    assert!(w112(&format!("taut := 35deg\npit := 2\n{BASE}")).is_empty());
}

/// **A dimension reads no geometry outside a trace** (§6.5).  `distance(c.r)` names the circle's
/// radius, which the expression graph cannot read: it used to mint `c.r` a free variable of its
/// own, warned and solved, so a drawn instance and the trace of the same component disagreed
/// about what the body says.  Refused where the text is settled, on the sheet and in a drawn
/// body alike; a traced body still reads it (`gear_trace.sv`'s `c.r * u`).
#[test]
fn a_dimension_reading_geometry_is_refused_outside_a_trace() {
    let sheet = format!("{BASE}k := circle(center: a) hint(r: 10)\nradius(10) k\na distance(k.r) b\n");
    let (_, d) = read(&sheet);
    assert!(d.iter().any(|m| m.starts_with("E103") && m.contains("`k.r` is a number of the geometry")), "{d:?}");
    let drawn = format!(
        "component L(k: circle, p: point, q: point) {{ p distance(k.r) q }}\n\
         {BASE}k := circle(center: a) hint(r: 10)\nradius(10) k\nl := L(k, a, b)\n"
    );
    let (_, d) = read(&drawn);
    assert!(d.iter().any(|m| m.starts_with("E103") && m.contains("`k.r` is a number of the geometry")), "{d:?}");

    let traced = format!(
        "component L(k: circle, u: Length) {{ p := point hint(x: 1, y: 1)\n  \
         horizontal line(k.center, p)\n  k.center distance(k.r + u) p }}\n\
         {BASE}k := circle(center: a) hint(r: 10)\nradius(10) k\ne := L(k).p over u in (0, 5)\n"
    );
    let (_, d) = read(&traced);
    assert!(d.is_empty(), "{d:?}");
}

/// **An input stands at the top of a document and says what it is** (§6.3, issue #77).  One with
/// no value is an unknown, so it names its type — a length, an angle or a plain number, never a
/// count — and a seed is for an unknown alone.  A component's inputs are its formals, a block's
/// copies share the document's, and a module's numbers are read by documents that do not draw
/// it, so an unknown there has no drawing to belong to.  A preview is the top of its document.
#[test]
fn an_input_stands_at_the_top_and_says_what_it_is() {
    let said = |src: &str, code: &str, what: &str| {
        let (_, d) = read(src);
        assert!(d.iter().any(|m| m.starts_with(code) && m.contains(what)), "{code} {what}: {d:?}");
    };
    said(&format!("{BASE}param w\na distance(w) b\n"), "E040", "say what it is");
    said(&format!("{BASE}param n: Int\n"), "E040", "is an unknown, so a `Length`");
    said(&format!("{BASE}param w := 60 hint(50)\n"), "E040", "a seed is for an unknown");
    said(&format!("{BASE}param w: Length hint(30deg)\na distance(w) b\n"), "E103", "30deg");
    said(&format!("{BASE}param w: Length := 30deg\na distance(w) b\n"), "E103", "Length");
    let comp = "component T(p: point) { param w := 3 }\n";
    said(comp, "syntax", "a component's inputs are its formals");
    said(&format!("{BASE}cycle 2 {{ param w := 3 }}\n"), "syntax", "it stands at the top level");
    said(&format!("{BASE}param w: point\n"), "syntax", "a `param` is a number");
    // a preview's statements are its document's top level
    let (_, d) = read(&format!("{BASE}preview {{\n  param w := 60\n  a distance(w) b\n}}\n"));
    assert!(d.is_empty(), "{d:?}");
    // and a module's number has a value, or no drawing to belong to
    let mut shelf: BTreeMap<&str, &str> = BTreeMap::new();
    shelf.insert("lib.u", "param k: Length\nparam j := 4\n");
    let (mut prog, errs) = parse(&format!("use lib.u\n{BASE}a distance(lib.u.j) b\n"));
    assert!(errs.is_empty(), "{errs:?}");
    assert!(link(&mut prog, &mut |name| shelf.get(name).map(|t| t.to_string()).or_else(|| gcs_core::library::resolve(name))).is_empty());
    let e = elaborate(&prog);
    let d: Vec<String> =
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    let refused = d.iter().any(|m| m.starts_with("E040") && m.contains("belongs to the document"));
    assert!(refused, "{d:?}");
}

/// **A solve writes an unknown's seed back where it is declared**, as it writes a point's: the
/// literal inside `param w: Length hint(…)` becomes the number the solve came to, in the unit it
/// was written in, and nothing else in the text moves.
#[test]
fn a_solve_writes_an_unknowns_seed_back_to_its_declaration() {
    let src = format!("{BASE}param w: Length hint(10)\na distance(w) b\nb distance(w / 2) c\n\
                       fix(x == 80, y == 0) b\n");
    let (e, d) = read(&src);
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let ed = gcs_core::edit::commit_seeds(&e, &sk, &e.program);
    assert!(ed.text.contains("param w: Length hint(80)"), "{}", ed.text);
    assert!(ed.text.contains("a distance(w) b\nb distance(w / 2) c"), "{}", ed.text);
    // an unknown with no seed written gets one, where the clause would go
    let (e, _) = read(&src.replace(" hint(10)", ""));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let ed = gcs_core::edit::commit_seeds(&e, &sk, &e.program);
    assert!(ed.text.contains("param w: Length hint(80)\n"), "{}", ed.text);
}

/// **An unknown starts at its seed**, wherever it is read: a dimension's unknown at the number in
/// its `hint(…)` rather than where a walk from the pose would put it, and one turning a plane's
/// ray likewise.  An angle misspelling its unknown is E101 and nothing more — the plane stands,
/// so what is drawn in it is not refused after it.
#[test]
fn an_unknown_starts_at_its_seed_and_a_misspelt_one_is_said_once() {
    let (e, d) = read(&format!("{BASE}param w: Length hint(25)\na distance(w) b\n"));
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(e.sketch.params[e.sketch.free_vars["w"] as usize].value, 25.0);
    let views = "use std\nparam beta: Angle hint(20deg)\n";
    let side = "tilt := ray hint(x: 0, y: 0.9396926207859084, z: 0.3420201433256687)\n\
                std.x perpendicular tilt\nstd.y angle(beta) tilt\n\
                side := plane(u: std.x, v: tilt)\nfix(x == 0, y == 0, z == 0) side\n\
                p := point hint(x: 5, y: 5) in side\n";
    let (e, d) = read(&format!("{views}{side}"));
    assert!(d.is_empty(), "{d:?}");
    assert!((e.sketch.params[e.sketch.free_vars["beta"] as usize].value - 20.0).abs() < 1e-12);
    let (_, d) = read(&format!("{views}{}", side.replace("angle(beta)", "angle(betta)")));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].starts_with("E101") && d[0].contains("`betta`"), "{d:?}");
}
