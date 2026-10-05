//! Iterating over a named chain's edges: `repeat e in rack.profile { … }` is one copy of the body
//! per link, in traversal order, with `e` naming that copy's edge.
//!
//! It is a `repeat` in every respect but its count and its binding — the copies are the
//! flattener's (`#<id>.<k>.`), indexed from outside as `s[0]`, `s[1]`, and a statement inside
//! is one statement however many copies it makes — so what is worth testing is the binding:
//! that `e` is the link and nothing else, wherever the chain is reached from, and that anything
//! that is not a named chain is refused at the reference.

use gcs_core::constraints::{Constraint, CKind};
use gcs_core::diagnose;
use gcs_core::edit::{self, Kind};
use gcs_core::model::{EntKind, EntRef};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{self, SolveOpts};
use gcs_core::syntax::{highlight, parse, render_flat, Tint};

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn reconciled(e: &mut Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

/// Every error an elaboration reports, as `code: message`.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    e.errors().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect()
}

fn refused(src: &str, code: &str, needle: &str) {
    let errs = errors(src);
    assert!(
        errs.iter().any(|m| m.starts_with(code) && m.contains(needle)),
        "expected {code} `{needle}`\n{src}\n{errs:?}"
    );
}

/// A rectangle, 40 by 30, as a named chain of four lines: two dimensions, two directions and
/// the corners, so the figure is rigid once `a` is grounded and turned.
const SQUARE: &str = "\
unit mm
a := point
b := point hint(x: 40, y: 0)
c := point hint(x: 40, y: 30)
d := point hint(x: 0, y: 30)
fix(x == 0, y == 0) a
square := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
horizontal ab
vertical bc
horizontal cd
vertical da
distance(40) ab
distance(30) bc
";

/// The case the construct is for: a point at every edge's midpoint.  Four copies, each point
/// the midpoint of its own edge in traversal order, and the midpoints add their coordinates and
/// take exactly as many equations — the DOF is the rectangle's, and it is zero.
#[test]
fn a_point_at_every_edges_midpoint() {
    let src = format!("{SQUARE}repeat e in square as i {{\n  m := point\n  m midpoint e\n}}\n");
    let mut e = read(&src);
    assert_eq!(e.sketch.points.len(), 8, "four corners and four midpoints");
    assert_eq!(
        e.sketch.user_constraints().iter().filter(|c| c.kind == CKind::Midpoint).count(),
        4
    );
    let r = solve::solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{r:?}");
    let d = diagnose::diagnose(&mut e.sketch, Default::default());
    assert_eq!(d.dof, 0);
    // in traversal order: copy k's point is the middle of the k-th link
    let at = |name: &str| {
        let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
        let [x, y] = e.sketch.point_params(p.i());
        [e.sketch.params[x as usize].value, e.sketch.params[y as usize].value]
    };
    // the copies' names are the block's, `#<id>.<k>.m`, in copy order
    let mut ms: Vec<String> =
        e.map.names.values().flatten().filter(|n| n.ends_with(".m")).cloned().collect();
    ms.sort();
    assert_eq!(ms.len(), 4, "{ms:?}");
    let want = [[20.0, 0.0], [40.0, 15.0], [20.0, 30.0], [0.0, 15.0]];
    for (k, w) in want.iter().enumerate() {
        let got = at(&ms[k]);
        assert!((got[0] - w[0]).abs() < 1e-9 && (got[1] - w[1]).abs() < 1e-9, "{k}: {got:?}");
    }
}

/// A copy is reached from outside by index, as any block's is, and `e` itself is a reference
/// like any other: a field (`e.p1`), a constraint operand, an argument to an instance.
#[test]
fn the_edge_is_a_reference_like_any_other() {
    let src = format!(
        "{SQUARE}\
component Tick(l: line) {{
  t := point
  t distance(0mm) l
  l.p1 distance(5mm) t
}}
repeat e in square as i {{
  m := point
  m midpoint e
  k := Tick(e)
  e.p2 distance(i * 1mm + 10mm) m
}}
m[0] distance(1mm) m[1]
"
    );
    let e = read(&src);
    // the instance given `e` aliases the link: its formal's line is the chain's k-th edge
    let links = ["ab", "bc", "cd", "da"].map(|n| e.map.ent_named(n).unwrap());
    let ticks: Vec<&str> = e.map.names.values().flatten()
        .filter(|n| n.ends_with(".k.t")).map(String::as_str).collect();
    assert_eq!(ticks.len(), 4, "{ticks:?}");
    // `e.p2 distance(…) m` names the edge's own end: every such constraint's first point is a
    // corner, and the four corners are each link's p2 in turn
    let ds: Vec<&Constraint> = e.sketch.user_constraints().into_iter()
        .filter(|c| c.kind == CKind::Distance && (9.5..13.5).contains(&c.args[2].num()))
        .collect();
    let mut ends: Vec<EntRef> = ds.iter().map(|c| c.args[0].ent()).collect();
    ends.sort();
    let mut want: Vec<EntRef> = links.iter().map(|l| e.sketch.children(*l)[1]).collect();
    want.sort();
    assert_eq!(ends, want);
}

/// A revolved profile, one surface per edge — the pattern `spiral_bevel/verification.sv` wrote
/// out by hand.  The chain is reached through an instance (`r.profile`), through a component's
/// group formal, and through a group bundling it, and each way gives one surface per link on
/// that link.
#[test]
fn a_surface_per_edge_of_a_revolved_profile() {
    let src = "\
unit mm
o := point
q := point
fix(x == 0, y == 0) o
fix(x == 0, y == 10) q
axis := line(o, q)
component Rect(w: Length, h: Length) {
  a := point hint(x: 2, y: 0)
  b := point hint(x: 2mm + w, y: 0)
  c := point hint(x: 2mm + w, y: h)
  d := point hint(x: 2, y: h)
  profile := (bottom := line(a, b)) -> (right := line(b, c)) -> (top := line(c, d)) -> (left := line(d, a)) -> close
  horizontal bottom
  vertical right
  horizontal top
  vertical left
  distance(w) bottom
  distance(h) right
}
r := Rect(w: 4mm, h: 3mm)
r.a distance(2mm, along: x) o
r.a distance(0mm, along: y) o
ring := solid(r.profile, about: axis)
repeat e in r.profile as i {
  s := surface(ring, edge: e)
}
component Walls(body: solid, rect: group) {
  repeat e in rect.profile {
    wall := surface(body, e)
  }
}
w := Walls(ring, r)
g := {profile: r.profile}
repeat f in g.profile {
  t := surface(ring, f)
}
";
    let mut e = read(src);
    assert!(solve::solve(&mut e.sketch, SolveOpts::default()).success);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert_eq!(e.sketch.surfaces.len(), 12, "four surfaces, three ways");
    let links =
        ["bottom", "right", "top", "left"].map(|n| e.map.ent_named(&format!("r.{n}")).unwrap());
    for (k, s) in e.sketch.surfaces.iter().enumerate() {
        assert_eq!(s.edge, links[k % 4], "surface {k} ({}) is on the wrong edge", s.name);
        assert_eq!(s.solid, e.map.ent_named("ring").unwrap().i() as u32);
    }
    // each surface is an ordinary named spatial entity, reached by its copy's name
    for s in &e.sketch.surfaces {
        let r = e.map.ent_named(&s.name).unwrap_or_else(|| panic!("`{}` does not resolve", s.name));
        assert_eq!(r.kind, EntKind::Surface);
    }
    // and the component's copies are under its instance
    assert!(e.sketch.surfaces[4..8].iter().all(|s| s.name.starts_with("w.")));
}

/// A `cycle` over a chain closes as a `cycle N` does: `next` is the following edge's copy, and
/// the last copy's is the first's — a diamond through the rectangle's midpoints.
#[test]
fn a_cycle_over_a_closed_chain_reads_next() {
    let src = format!(
        "{SQUARE}cycle e in square {{\n  m := point\n  m midpoint e\n  line(m, next.m)\n}}\n"
    );
    let mut e = read(&src);
    assert_eq!(e.sketch.lines.len(), 8, "four sides and a diamond");
    assert!(solve::solve(&mut e.sketch, SolveOpts::default()).success);
    // each diamond side joins the midpoints of two neighbouring edges: its length is half
    // the rectangle's diagonal, 25
    for l in &e.sketch.lines[4..] {
        let [p, q] = [l.p1, l.p2].map(|p| {
            let [x, y] = e.sketch.point_params(p as usize);
            [e.sketch.params[x as usize].value, e.sketch.params[y as usize].value]
        });
        assert!(((p[0] - q[0]).hypot(p[1] - q[1]) - 25.0).abs() < 1e-9);
    }
}

/// An open chain is iterated by `repeat` like a closed one, and a chain declared *after* the
/// block — or inside an instance written after it — is found all the same (P2).
#[test]
fn an_open_chain_and_a_forward_reference() {
    let src = "\
unit mm
repeat e in trail {
  m := point
  m midpoint e
}
repeat e in late.p {
  n := point
  n midpoint e
}
trail := (t1 := line(hint(x: 0, y: 0), hint(x: 10, y: 0))) ->
  (t2 := line(hint(x: 10, y: 0), hint(x: 10, y: 10))) -> (t3 := line)
component Late() {
  p := line(hint(x: 0, y: 20), hint(x: 5, y: 20)) -> line
}
late := Late()
";
    let e = read(src);
    assert_eq!(e.sketch.lines.len(), 5);
    let ms = e.sketch.user_constraints().iter().filter(|c| c.kind == CKind::Midpoint).count();
    assert_eq!(ms, 5, "three edges of the trail and two of the late chain");
}

/// Blocks nest: a `repeat` over a chain inside a copy of another, and a chain *made* by one
/// block's copies iterated by a block of its own.
#[test]
fn chain_blocks_nest() {
    let src = format!(
        "{SQUARE}\
repeat e in square as i {{
  repeat 2 as j {{
    m := point
    m coincident e
  }}
}}
repeat 2 as k {{
  z := line(hint(x: 0, y: 50 + k), hint(x: 10, y: 50 + k)) -> line
  repeat e in z {{
    n := point
    n midpoint e
  }}
}}
"
    );
    let e = read(&src);
    let on = e.sketch.user_constraints().iter().filter(|c| c.kind == CKind::PointOnLine).count();
    assert_eq!(on, 8);
    let mid = e.sketch.user_constraints().iter().filter(|c| c.kind == CKind::Midpoint).count();
    assert_eq!(mid, 4);
}

/// What is not a named chain is refused at the reference, with the code that says which
/// mistake it is: a name that is something else, a name nothing declares, a private chain
/// reached from outside, a `cycle` over a chain that does not close, and a body declaration
/// called what the copies call their edge.
#[test]
fn what_is_not_a_named_chain_is_refused() {
    refused(&format!("{SQUARE}repeat e in a {{ }}\n"), "E103", "`a` is not a named chain");
    refused(&format!("{SQUARE}repeat e in ab {{ }}\n"), "E103", "`ab` is not a named chain");
    refused(&format!("{SQUARE}repeat e in nothing {{ }}\n"), "E101", "no such entity: `nothing`");
    refused(
        "open := line(hint(x: 0, y: 0), hint(x: 1, y: 0)) -> line\ncycle e in open { }\n",
        "E103",
        "is an open chain",
    );
    let twice = format!("{SQUARE}repeat e in square {{ e := point }}\n");
    refused(&twice, "E001", "`e` is declared twice");
    refused(
        "component Hid() {\n  private p := line(hint(x: 0, y: 0), hint(x: 1, y: 0)) -> line\n}\n\
         h := Hid()\nrepeat e in h.p { }\n",
        "E101",
        "private member",
    );
    // and the reference is where it is said
    let src = format!("{SQUARE}repeat e in a {{ }}\n");
    let (prog, _) = parse(&src);
    let el = elaborate(&prog);
    let d = el.errors().next().unwrap();
    assert_eq!(&src[d.span.lo as usize..d.span.hi as usize], "a");
}

/// The source is canonical: the canonical flat printer refuses the block as it refuses every
/// `repeat`, and an edit beside it splices around it, leaving the block as written — the block
/// is parsed back to the same statements.
#[test]
fn the_block_is_kept_as_written() {
    let src = format!(
        "{SQUARE}repeat e in square as i {{\n  m := point hint(x: 1, y: 1)\n  m midpoint e\n}}\n"
    );
    // the block standing before the chain it runs over, so the printer meets it first
    let early = "repeat e in sq {\n  m := point\n  m midpoint e\n}\n\
                 sq := line(hint(x: 0, y: 0), hint(x: 1, y: 0)) -> line\n";
    let (mut prog, _) = parse(early);
    assert_eq!(render_flat(&mut prog).unwrap_err().construct, "repeat and cycle blocks");
    assert_eq!(prog.text(), early, "and the source is kept");
    // a gesture appends beside it, and the block text is unchanged
    let mut e = read(&src);
    let p = e.sketch.point(-10.0, 5.0, false, "");
    let q = e.sketch.point(-20.0, 5.0, false, "");
    e.sketch.line(p, q);
    let ed = reconciled(&mut e);
    assert_eq!(ed.kind, Kind::Structural, "{:?}", ed.refused);
    let block = "repeat e in square as i {\n  m := point hint(x: 1, y: 1)\n";
    assert!(ed.text.contains(block), "{}", ed.text);
    let back = read(&ed.text);
    assert_eq!(back.sketch.points.len(), e.sketch.points.len());
    assert_eq!(back.sketch.lines.len(), e.sketch.lines.len());
}

/// An edit inside the body is an edit inside a `repeat`: a seed reached by four copies has no
/// one pose to write, and a gesture on one copy is refused with the cause.
#[test]
fn edits_inside_the_body_behave_as_in_a_repeat() {
    let src = format!("{SQUARE}repeat e in square {{\n  m := point hint(x: 1, y: 1)\n  m coincident e\n}}\n");
    let (prog, _) = parse(&src);
    let mut e = elaborate(&prog);
    assert!(solve::solve(&mut e.sketch, SolveOpts::default()).success);
    let ed = edit::commit_seeds(&e, &e.sketch, &prog);
    assert!(ed.text.contains("m := point hint(x: 1, y: 1)"), "four poses, one seed:\n{}", ed.text);
    let copy = EntRef::point(4);
    let copy_name = e.map.name_of(copy).cloned().unwrap();
    assert!(copy_name.starts_with('#') && copy_name.ends_with(".0.m"), "{copy_name}");
    e.sketch.add(Constraint::distance(EntRef::point(0), copy, 7.0));
    let ed = reconciled(&mut e);
    assert_eq!(ed.kind, Kind::None);
    assert!(ed.refused.expect("refused, and says why").contains("block"));
}

/// The colouring reads the edge's name as a name the block declares, as it reads `as i`.
#[test]
fn the_edge_is_coloured_as_a_declaration() {
    let src = "repeat e in square as i { }\n";
    let runs = highlight(src);
    let tint_at = |at: usize| runs.iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| *t);
    assert_eq!(tint_at(0), Some(Tint::Word));
    assert_eq!(tint_at(7), Some(Tint::Def), "{runs:?}");
    assert_eq!(tint_at(22), Some(Tint::Def), "the index too: {runs:?}");
}

/// The copies stand where the block is written, though they are made once the chain is found:
/// a point declared before the block is built before them and one after it after them, whatever
/// line the chain itself is on.
#[test]
fn the_copies_stand_where_the_block_is_written() {
    let src = "\
a := point hint(x: 0, y: 0)
b := point hint(x: 10, y: 0)
c := point hint(x: 0, y: 10)
before := point hint(x: -5, y: -5)
repeat e in tri {
  m := point
  m midpoint e
}
after := point hint(x: 5, y: 5)
tri := line(a, b) -> line(b, c) -> line(c, a) -> close
";
    let e = read(src);
    let at = |n: &str| e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`")).i();
    assert_eq!((at("before"), at("after")), (3, 7));
    for k in 4..7 {
        let name = e.map.name_of(EntRef::point(k)).unwrap();
        assert!(name.ends_with(&format!(".{}.m", k - 4)), "{k}: {name}");
    }
}
