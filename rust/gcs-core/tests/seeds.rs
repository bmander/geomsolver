//! Seeds that read geometry (§6.4): `hint(x: k.center.x + k.r, y: pin.y)` reads the *seeds* of
//! the scalars it names, `hint(at: pin)` and `hint(at: k, bearing: b)` name a place outright, and a
//! seed inside a child slot is settled over a component's parameters like any other.  A seed is
//! where a solve begins and nothing more, so none of this changes what a document says.

use gcs_core::edit;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn refused(src: &str, needle: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.errors().any(|d| d.message.contains(needle)),
        "expected `{needle}`\n{}",
        e.diags.iter().map(|d| d.message.clone()).collect::<Vec<_>>().join("\n")
    );
}

fn xy(e: &Elaborated, name: &str) -> (f64, f64) {
    // `q[1]` is copy 1 of a block's `q`, which the map calls `#<stmt>.1.q`
    let r = match name.split_once('[') {
        Some((leaf, k)) => {
            let tail = format!(".{}.{leaf}", k.trim_end_matches(']'));
            *e.map
                .names
                .iter()
                .find(|(_, ns)| ns.iter().any(|n| n.ends_with(&tail)))
                .map(|(r, _)| r)
                .unwrap_or_else(|| panic!("no `{name}`"))
        }
        None => e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")),
    };
    e.sketch.point_xy(r.i())
}

#[test]
fn a_seed_reads_the_seed_of_what_it_names() {
    let e = read(
        "point a hint(x: 5, y: 1)\n\
         circle k(center: a) hint(r: 4)\n\
         point b hint(x: a.x + 10, y: k.r)\n\
         point c hint(at: a)\n\
         point d hint(at: k, bearing: 90deg)\n\
         point e hint(x: k.center.x + k.r, y: 0)\n\
         line l(hint(x: a.x, y: 1), hint(x: a.x + 1, y: 1))\n",
    );
    assert_eq!(xy(&e, "b"), (15.0, 4.0));
    assert_eq!(xy(&e, "c"), (5.0, 1.0));
    let (dx, dy) = xy(&e, "d");
    assert!((dx - 5.0).abs() < 1e-9 && (dy - 5.0).abs() < 1e-9, "{dx} {dy}");
    assert_eq!(xy(&e, "e"), (9.0, 0.0));
    assert_eq!(xy(&e, "l.p2"), (6.0, 1.0));
}

#[test]
fn a_component_seeds_from_its_formals_geometry() {
    let e = read(
        "component Off(p: point, d: Length) {\n\
           point q hint(x: p.x + d, y: p.y)\n\
           line s(p, hint(x: p.x + d / 2, y: p.y + d))\n\
         }\n\
         point a hint(x: 10, y: 20)\n\
         o: Off(a, d: 6)\n",
    );
    assert_eq!(xy(&e, "o.q"), (16.0, 20.0));
    assert_eq!(xy(&e, "o.s.p2"), (13.0, 26.0));
}

#[test]
fn a_component_reads_the_files_top_level_params() {
    let e = read(
        "param w = 30\n\
         component Bar(a: point) {\n\
           param h = w / 2\n\
           point b hint(x: a.x + w, y: a.y + h)\n\
           line l(a, b)\n\
           a distance(w) b\n\
         }\n\
         point o hint(x: 0, y: 0)\n\
         r: Bar(o)\n",
    );
    assert_eq!(xy(&e, "r.b"), (30.0, 15.0));
    // a formal of the same name shadows the file's param
    let e = read(
        "param w = 30\n\
         component Bar(a: point, w: Length) {\n\
           point b hint(x: a.x + w, y: a.y)\n\
         }\n\
         point o hint(x: 0, y: 0)\n\
         r: Bar(o, w: 7)\n",
    );
    assert_eq!(xy(&e, "r.b"), (7.0, 0.0));
}

#[test]
fn a_geometric_seed_follows_the_scope_it_was_written_in() {
    // the tangent span: its seeds name the circles it is written over, through the formals
    let e = read(
        "unit mm\n\
         component Span(k1: circle, k2: circle, side: Scalar) {\n\
           point a hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)\n\
           point b hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)\n\
           line s(a, b)\n\
         }\n\
         point o hint(x: 0, y: 0)\n\
         point c hint(x: 100, y: 0)\n\
         circle k1(center: o) hint(r: 10)\n\
         circle k2(center: c) hint(r: 20)\n\
         up: Span(k1, k2, side: 1)\n\
         dn: Span(k1, k2, side: -1)\n",
    );
    let (ax, ay) = xy(&e, "up.a");
    assert!((ax - 0.0).abs() < 1e-9 && (ay - 10.0).abs() < 1e-9, "{ax} {ay}");
    let (bx, by) = xy(&e, "dn.b");
    assert!((bx - 100.0).abs() < 1e-9 && (by + 20.0).abs() < 1e-9, "{bx} {by}");
    // and inside a block, a copy's own
    let e = read(
        "repeat 2 as i {\n\
           point p hint(x: i * 10, y: 0)\n\
           point q hint(x: p.x + 1, y: p.y)\n\
         }\n",
    );
    assert_eq!(xy(&e, "q[1]"), (11.0, 0.0));
}

#[test]
fn a_seed_that_reads_geometry_is_never_written_back() {
    let src = "point a hint(x: 5, y: 1)\npoint b hint(x: a.x + 10, y: 2)\npoint c hint(at: a)\na distance(3) b\n";
    let e = read(src);
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let out = edit::commit_seeds(&e, &sk, &e.program);
    // `b`'s x is an expression and `c` is a place: neither is spliced; only the moved numbers are
    assert!(out.text.contains("hint(x: a.x + 10, y: "), "{}", out.text);
    assert!(out.text.contains("point c hint(at: a)"), "{}", out.text);
}

#[test]
fn what_a_seed_may_read_is_checked() {
    refused("point a hint(x: 0, y: 0)\npoint b hint(x: a.z, y: 0)\n", "has no `z`");
    refused("point a hint(x: 0, y: 0)\npoint b hint(x: nobody.x, y: 0)\n", "no such entity");
    refused("point a hint(x: 0, y: 0)\ncircle k(center: a) hint(r: 3)\npoint d hint(at: k)\n", "where on the edge");
    refused("point a hint(x: 0, y: 0)\ncircle k(center: a) hint(r: a.x)\nline l(a, hint(x: 1, y: 1)) hint(at: a)\n", "only a point");
    // a `param` is not a seed: it feeds constraints, and a seed may not change what a document says
    refused("point a hint(x: 0, y: 0)\nparam w = a.x + 3\n", "not a number here");
}

#[test]
fn a_geometry_read_is_a_length_where_the_document_names_a_unit() {
    read("unit mm\npoint a hint(x: 5, y: 1)\npoint b hint(x: a.x + 10mm, y: 0)\n");
    refused("unit mm\npoint a hint(x: 5, y: 1)\npoint b hint(x: a.x + 10, y: 0)\n", "cannot be added");
    // and a bare number where it does not — there is nothing else a number could be
    read("point a hint(x: 5, y: 1)\npoint b hint(x: a.x + 10, y: 0)\n");
}

/// **A place is two keys of the one seed clause** (issue #47, item 2): `at:` and `bearing:`
/// beside `x`, `y` and `r`, in any order, and the printer spells the clause back as it was
/// read.  A clause naming a place carries no scalar, a bearing needs a place, `at:` names and
/// is not a pair, and the grammar the keys replaced says what it became — each refused where
/// it is written, since the mistake is the key's and not the declaration's.
#[test]
fn a_place_is_two_keys_of_the_seed_clause() {
    let e = read(
        "point a hint(x: 5, y: 1)\n\
         circle k(center: a) hint(r: 4)\n\
         point d hint(bearing: 90deg, at: k)\n",
    );
    let (dx, dy) = xy(&e, "d");
    assert!((dx - 5.0).abs() < 1e-9 && (dy - 5.0).abs() < 1e-9, "{dx} {dy}");
    // the printer's one spelling of the clause is the keyed one
    let (prog, errs) = parse("point d hint(at: k, bearing: 90deg)\npoint c hint(at: a.p1)\n");
    assert!(errs.is_empty(), "{errs:?}");
    let printed: Vec<String> = prog
        .root()
        .body
        .iter()
        .map(|st| {
            let mut out = String::new();
            gcs_core::syntax::write_stmt_to(&mut out, &st.kind);
            // the printer pads the kind to a column; the clause is what is under test
            out.split_whitespace().collect::<Vec<_>>().join(" ")
        })
        .collect();
    assert_eq!(printed, ["point d hint(at: k, bearing: 90deg)", "point c hint(at: a.p1)"]);
    for (src, want) in [
        ("point a hint(x: 0, y: 0)\npoint q hint(x: 3, at: a)\n", "carries no scalar"),
        ("point r hint(bearing: 30)\n", "needs `at:`"),
        ("point s hint(at: (3, 4))\n", "a coordinate seed is `hint(x: …, y: …)`"),
        ("point a hint(x: 0, y: 0)\npoint q hint(at: a, at: a)\n", "written twice"),
        ("point a hint(x: 0, y: 0)\npoint q hint at a\n", "a place is keyed now"),
        ("point a hint(x: 0, y: 0)\npoint q hint at a bearing (30)\n", "a place is keyed now"),
    ] {
        let (_, errs) = parse(src);
        assert!(
            errs.iter().any(|e| e.message.contains(want)),
            "expected `{want}` from {src:?}, got {:?}",
            errs.iter().map(|e| &e.message).collect::<Vec<_>>()
        );
    }
}
