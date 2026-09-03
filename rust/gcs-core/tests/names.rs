//! One namespace for the three ways a number gets a name (issue #47, item 7).
//!
//! `param w = 60`, `a distance(w = 60) b` and a bare `w` nothing defines used to be resolved by
//! two machineries with different rules: a `param` was the flattener's, lexically scoped, and
//! a named dimension the expression graph's, one global table — so a `param` could not read a
//! named dimension, a name defined both ways collided as a stray `=`, and a bare name inside a
//! component reached whatever the document happened to call that.  Now a named dimension
//! declares its name in its body exactly as a `param` does, and a bare name a body never
//! declares is an unknown of the instance, as an unbound formal already was.

use std::collections::BTreeMap;

use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

const BASE: &str = "point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
ground a
a horizontal b
b vertical c
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

/// A `param` reads a named dimension: the two are one kind of definition.
#[test]
fn a_param_reads_a_named_dimension() {
    let (e, d) = read(&format!("{BASE}a distance(w = 60) b\nparam h = w / 2\nb distance(h) c\n"));
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 1, 2) - 30.0).abs() < 1e-9);
}

/// A name defined as a `param` and as a dimension is declared twice, and that is the one
/// thing said — not the stray `=` that folding the param's number over the dimension's own
/// name used to leave behind.
#[test]
fn a_name_defined_both_ways_is_declared_twice() {
    let (_, d) = read(&format!("param w = 60\n{BASE}a distance(w = 60) b\n"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].starts_with("E001") && d[0].contains("`w` is declared twice"), "{d:?}");
}

/// A bare name inside a component is an unknown of the instance — `t1.w`, `t2.w` — exactly
/// as a formal left unbound is, so two instances have two unknowns and not one shared one.
#[test]
fn a_bare_name_in_a_component_is_the_instances_own_unknown() {
    let doc = format!(
        "component T(p: point, q: point) {{ p distance(w) q }}\n{BASE}point d hint(x: 0, y: 40)\n\
         t1: T(a, b)\nt2: T(b, c)\nt3: T(c, d)\n"
    );
    let (e, d) = read(&doc);
    let free: Vec<&String> = d.iter().filter(|m| m.starts_with("W111")).collect();
    assert_eq!(free.len(), 3, "{d:?}");
    for n in ["t1.w", "t2.w", "t3.w"] {
        assert!(free.iter().any(|m| m.contains(&format!("`{n}`"))), "{d:?}");
        assert!(e.sketch.free_vars.contains_key(n), "{:?}", e.sketch.free_vars);
    }
    // the same drawing with `w` a formal left unbound says the same thing in the same words
    let formal = doc.replace("q: point)", "q: point, w: Length)");
    let (e2, d2) = read(&formal);
    assert_eq!(d2, d);
    let names = |e: &Elaborated| e.sketch.free_vars.keys().cloned().collect::<Vec<_>>();
    assert_eq!(names(&e2), names(&e));
}

/// A file's named dimensions are in scope in the components it defines, as its params are.
#[test]
fn a_component_reads_the_named_dimensions_of_its_own_file() {
    let doc = format!(
        "component T(p: point, q: point) {{ p distance(w / 2) q }}\n\
         {BASE}a distance(w = 60) b\nt: T(b, c)\n"
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
    shelf.insert("lib.t", "component T(p: point, q: point) { p distance(w) q }\n");
    let src = format!("use lib.t\n{BASE}a distance(w = 60) b\nt: T(b, c)\n");
    let (mut prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = link(&mut prog, &mut |name| shelf.get(name).map(|t| t.to_string()));
    assert!(linked.is_empty(), "{linked:?}");
    let e = elaborate(&prog);
    let free: Vec<String> =
        e.diags.iter().filter(|d| d.code.as_str() == "W111").map(|d| d.message.clone()).collect();
    assert_eq!(free.len(), 1, "{free:?}");
    assert!(free[0].contains("`t.w`"), "{free:?}");
    assert!(e.sketch.free_vars.contains_key("t.w"));
}

/// A named dimension inside a component is the instance's — `u.t.w` — and is read by dotted
/// path from the body around it and from the sheet, like anything else an instance makes.
#[test]
fn a_named_dimension_in_an_instance_is_read_by_its_dotted_path() {
    let doc = format!(
        "component T(p: point, q: point) {{ p distance(w = 60) q }}\n\
         component U(p: point, q: point, r: point) {{ t: T(p, q)\n  q distance(t.w / 2) r }}\n\
         {BASE}point d hint(x: 0, y: 40)\nu: U(a, b, c)\nc distance(u.t.w) d\n"
    );
    let (e, d) = read(&doc);
    assert!(d.is_empty(), "{d:?}");
    let sk = solved(e);
    assert!((dist(&sk, 0, 1) - 60.0).abs() < 1e-9);
    assert!((dist(&sk, 1, 2) - 30.0).abs() < 1e-9);
    assert!((dist(&sk, 2, 3) - 60.0).abs() < 1e-9);
    // and the drawing says so in the constraint list
    let said: Vec<String> =
        sk.user_constraints().iter().map(|c| gcs_core::io::describe(c)).collect();
    assert!(said.iter().any(|s| s.contains("u.t.w = 60")), "{said:?}");
    assert!(said.iter().any(|s| s.contains("u.t.w / 2")), "{said:?}");
}

/// A name declared inside a `cycle` is each copy's own — `#N.k.w` — so a dimension named in a
/// block is defined once per copy rather than N times over, and a bare name in the block is
/// the enclosing instance's, shared by every copy.
#[test]
fn a_block_copy_declares_its_own_names_and_shares_the_bodys_unknowns() {
    let (e, d) = read(
        "point o hint(x: 0, y: 0)\nground o\n\
         cycle 2 { point z hint(x: 5, y: 5)\n  point y hint(x: 9, y: 2)\n\
         point x hint(x: 3, y: 8)\n\
         o distance(w = 60) z\n  o distance(w / 2) y\n  o distance(s) x }\n",
    );
    let free: Vec<&String> = d.iter().filter(|m| m.starts_with("W111")).collect();
    assert_eq!(d.len(), free.len(), "{d:?}");
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
        "component T(p: point, q: point, w: Length) { p distance(w) q }\n\
         point o hint(x: 0, y: 0)\nground o\ncycle 3 { point a hint(x: 10, y: 0)\n  t: T(o, a) }\n",
    );
    assert!(d.iter().all(|m| m.starts_with("W111")), "{d:?}");
    assert_eq!(e.sketch.free_vars.len(), 3, "{:?}", e.sketch.free_vars);
    assert!(e.sketch.free_vars.keys().all(|k| k.starts_with('#') && k.ends_with(".t.w")));
}

/// A `param` reading a free variable is refused with the cause: nothing in scope gives the
/// name a number.
#[test]
fn a_param_may_not_read_a_free_variable() {
    let (_, d) = read(&format!("{BASE}a distance(s) b\nparam q = s * 2\n"));
    let said = d.iter().any(|m| m.starts_with("E103") && m.contains("`s` is not a number here"));
    assert!(said, "{d:?}");
}
