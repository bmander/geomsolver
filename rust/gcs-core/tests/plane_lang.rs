//! The language half of multiview sketching (§6.7): `plane` declarations over axes, the `in`
//! clause, the `project` operator, and the writeback of each.
use gcs_core::constraints::CKind;
use gcs_core::edit::{self, Kind};
use gcs_core::model::{EntKind, EntRef};
use gcs_core::plane::Basis;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, write_stmt_to, Tint};
use crate::common::parse;
use gcs_core::io;

use crate::common::read;

/// Elaborates with an error carrying `code`, whose message contains `needle`.
fn refused(src: &str, code: &str, needle: &str) {
    let (prog, errs, _) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    let hit = e.errors().any(|d| d.code.as_str() == code && d.message.contains(needle));
    assert!(
        hit,
        "expected {code} `{needle}`\n{src}\n{:?}",
        e.diags.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
}

/// The entity's index, by name.
fn at(e: &Elaborated, n: &str) -> usize {
    e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`")).i()
}

/// A part designed in one place (§6.7): a component whose body carries `in view { … }` blocks
/// over planes it was handed, with the projection tying its views inside it — and a view left
/// undrawn by `repeat 0`.
#[test]
fn a_component_carries_its_views_in_blocks() {
    let src = "\
use std
component Peg(f: plane, r: plane, cf: point, cr: point, draw_r: Int) {
  in f {
    a := point hint((cf.x, cf.y + 10))
    cf vertical a
    cf distance(10, along: y) a
  }
  repeat draw_r {
    in r {
      b := point hint((cr.x + 5, cr.y + 10))
      cr distance(5, along: x) b
    }
    a project b[0]
  }
}
p := Peg(std.front, std.side, std.front.origin, std.side.origin, draw_r: 1)
q := Peg(std.front, std.side, std.front.origin, std.side.origin, draw_r: 0)
";
    let e = read(src);
    let bare = read("use std\n");
    assert_eq!(e.sketch.points.len(), bare.sketch.points.len() + 3, "a and b of p, a of q");
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let b = e.map.ent_named("p.b").or_else(|| {
        e.map.names.iter().find(|(_, ns)| ns.iter().any(|n| n.ends_with(".0.b"))).map(|(r, _)| *r)
    }).expect("p.b");
    let (bx, by) = sk.point_xy(b.i());
    assert!((bx - 5.0).abs() < 1e-6 && (by - 10.0).abs() < 1e-6, "{bx} {by}");
    // and inside a root block the clause is still written per declaration
    misparses("use std\nrepeat 2 { in std.front { p := point } }\n", "in a component");
}

fn misparses(src: &str, needle: &str) {
    let (_, errs) = parse(src);
    let msgs: Vec<String> = errs.into_iter().map(|e| e.message).collect();
    assert!(msgs.iter().any(|m| m.contains(needle)), "expected `{needle}`\n{src}\n{msgs:?}");
}

fn reconciled(e: &mut Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

/// Three planes of the document's own over the standard axes, held at the origin.
const VIEWS: &str = "\
use std
front := plane(u: std.x, v: std.z)
top := plane(u: std.x, v: std.y)
right := plane(u: std.y, v: std.z)
fix(origin == (0, 0, 0)) front
fix(origin == (0, 0, 0)) top
fix(origin == (0, 0, 0)) right
";

#[test]
fn every_spelling_prints_back() {
    let src = "\
use std
r := axis hint(dir: (0.6, 0.8, 0))
front := plane(u: std.x, v: std.z)
top := plane(u: std.x, v: std.y)
p := plane(u: r, v: std.z) hint(origin: (1, 2, 3))
a := point in top hint((10, 5))
l := line(a, hint((3, 4))) in top
b := point hint((2, 2)) in front
s := point hint((1, 2, 3))
a project b
claim a project b
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    for st in &prog.root().body {
        write_stmt_to(&mut out, &st.kind).unwrap();
        out.push('\n');
    }
    let squash = |s: &str| s.split_whitespace().collect::<Vec<_>>().join(" ");
    // `in` is a trailer and prints after `hint`; every other statement prints as written, and
    // the `use` is the program's, not a statement of its body
    let want = src.replace("a := point in top hint((10, 5))", "a := point hint((10, 5)) in top")
        .replace("use std\n", "");
    assert_eq!(squash(&out), squash(&want));
    let e = read(src);
    assert_eq!(e.sketch.planes.len(), 4 + 3);
    assert_eq!(e.sketch.user_constraints().len(), 2);
    assert!(e.sketch.user_constraints()[1].claim);
}

#[test]
fn a_plane_reads_its_basis_off_its_rays() {
    let e = read(&format!("{VIEWS}r := axis hint(dir: (0.8660254037844387, 0.5, 0))\n\
                           fix(dir == (0.8660254037844387, 0.5, 0)) r\n\
                           aux := plane(u: r, v: std.z)\n"));
    let b = |n: &str| e.sketch.basis(at(&e, n));
    let near = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    assert_eq!(b("front"), Basis::page());
    assert!(near(b("top").u, [1.0, 0.0, 0.0]) && near(b("top").v, [0.0, 1.0, 0.0]));
    assert!(near(b("right").u, [0.0, 1.0, 0.0]) && near(b("right").v, [0.0, 0.0, 1.0]));
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    assert!(near(b("aux").u, [c, s, 0.0]) && near(b("aux").v, [0.0, 0.0, 1.0]));
    // a plane may be declared before the axes it stands on
    let e = read("aux := plane(u: r, v: w)\nr := axis hint(dir: (1, 0, 0))\nw := axis hint(dir: (0, 1, 0))\n");
    assert!(near(e.sketch.basis(at(&e, "aux")).v, [0.0, 1.0, 0.0]));
    // and its axes need be neither unit nor square: `v` is what is left of the second
    let e = read("a := axis hint(dir: (2, 0, 0))\nb := axis hint(dir: (1, 0, 3))\np := plane(u: a, v: b)\n");
    assert!(near(e.sketch.basis(at(&e, "p")).v, [0.0, 0.0, 1.0]));
}

#[test]
fn plane_refusals_carry_their_codes() {
    refused("use std\np := point hint((1, 2, 3))\na := plane(u: p, v: std.z)\n", "E103",
            "an axis or a line");
    refused("use std\na := plane(u: nope, v: std.z)\n", "E101", "no such entity");
    refused("use std\na := plane(u: std.x, v: std.z, origin: std.origin)\n", "E103",
            "a plane's origin is its own");
    misparses("f := plane(u: a, v: b) in top\n", "has none of its own");
}

#[test]
fn in_on_every_kind() {
    let e = read(&format!(
        "{VIEWS}\
a := point in top
l := line in top
c := circle in right
k := arc in right
k0 := point hint((0, 0))
k1 := point hint((1, 0))
k2 := point hint((2, 1))
k3 := point hint((3, 0))
s := spline(k0, k1, k2, k3) in front
"
    ));
    let sk = &e.sketch;
    let on = |name: &str| -> Vec<Option<usize>> {
        let r = e.map.ent_named(name).unwrap();
        let pts = if r.kind == EntKind::Point { vec![r] } else { sk.children(r) };
        pts.iter().map(|p| sk.plane_of(p.i())).collect()
    };
    let (front, top, right) = (at(&e, "front"), at(&e, "top"), at(&e, "right"));
    assert_eq!(on("a"), vec![Some(top)]);
    assert_eq!(on("l"), vec![Some(top), Some(top)]);
    assert_eq!(on("c"), vec![Some(right)]);
    assert_eq!(on("k"), vec![Some(right); 3]);
    assert_eq!(on("s"), vec![Some(front); 4]);
    // a point named by a line in one plane and declared in another is one image on two planes
    refused(&format!("{VIEWS}a := point in front\nl := line(a, top.origin) in top\n"), "E060",
            "already in `front`");
    // agreement is not a conflict
    read(&format!("{VIEWS}a := point in top\nl := line(a, top.origin) in top\n"));
    refused("a := point in nope\n", "E101", "no such entity");
    refused("use std\na := point in l\nin std.front {\nl := line\n}\n", "E040", "`in` names a plane");
    misparses("p := plane in top\n", "has none of its own");
    misparses("p := point in top in front\n", "already in a plane");
}

#[test]
fn unit_in_still_parses_and_in_is_not_a_name() {
    let e = read("unit in\nuse std\np := point hint((3in, 0)) in std.front\n");
    assert!((e.sketch.point_xy(at(&e, "p")).0 - 3.0).abs() < 1e-12);
    // a point cannot be called `in`: the word is a clause's, and the parser says so
    let (_, errs) = parse("point in\n");
    assert!(!errs.is_empty());
    let e = read("use std\npoint in front\nfront := plane(u: std.x, v: std.z)\n");
    let front = at(&e, "front");
    let drawn = e.sketch.points.iter().filter(|p| p.plane == Some(front as u32)).count();
    assert_eq!(drawn, 2, "the plane's origin and an anonymous point, in it");
}

#[test]
fn project_settles_refuses_and_claims() {
    let e = read(&format!("{VIEWS}a := point in front\nb := point in top\na project b\n"));
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(c.kind, CKind::Project);
    assert_eq!(c.entities()[2], EntRef::plane(at(&e, "front")));
    assert_eq!(c.entities()[3], EntRef::plane(at(&e, "top")));
    // each refusal at the statement's own span
    let (prog, _, _) =
        gcs_core::library::parse_linked(&format!("{VIEWS}a := point in front\nb := point\na project b\n"));
    let e = elaborate(&prog);
    let d = e.errors().find(|d| d.code.as_str() == "E061").expect("refused");
    assert!(d.message.contains("no plane"), "{}", d.message);
    assert_eq!(d.span.slice(prog.text()), "a project b");
    refused(&format!("{VIEWS}a := point in front\nb := point in front\na project b\n"), "E061", "itself");
    refused(
        &format!("{VIEWS}front2 := plane\nfix(origin == (0, -5, 0)) front2\nfix(dir == (1, 0, 0)) front2.u\nfix(dir == (0, 0, 1)) front2.v\n\
                  a := point in front\nb := point in front2\na project b\n"),
        "E061",
        "parallel",
    );
    refused(&format!("{VIEWS}l := line\nm := line\nl project m\n"), "E040", "");
    let e = read(&format!("{VIEWS}a := point in front\nb := point in top\nclaim a project b\n"));
    assert!(e.sketch.user_constraints()[0].claim);
}

#[test]
fn describe_and_write_skip_the_planes() {
    let e = read(&format!("{VIEWS}a := point in front\nb := point in top\na project b\n"));
    let c = &e.sketch.user_constraints()[0];
    let named = io::describe_with(c, &|r| e.map.name_of(r).cloned());
    assert_eq!(named, "a project b");
    let want = format!("P{} project P{}", at(&e, "a"), at(&e, "b"));
    assert_eq!(io::describe(c), want, "and positionally, with the planes left out");
}

#[test]
fn reconcile_writes_membership_a_plane_and_a_projection() {
    let mut e = read(VIEWS);
    let top = at(&e, "top");
    // a point drawn in the current plane: its statement says so
    let p = e.sketch.point(20.0, 110.0, false, "new");
    e.sketch.set_plane(p, Some(top));
    let out = reconciled(&mut e);
    assert_eq!(out.kind, Kind::Structural);
    assert!(out.text.contains("p0 := point hint((20, 110)) in top"), "{}", out.text);
    // an anonymous plane is named the moment a point is put in it
    let mut e = read("use std\nplane(u: std.x, v: std.y)\n");
    let pl = 0;   // the document's own, numbered before the standard datums
    let p = e.sketch.point(3.0, 4.0, false, "new");
    e.sketch.set_plane(p, Some(pl));
    let out = reconciled(&mut e);
    assert!(out.text.contains("\nv0 := plane("), "{}", out.text);
    assert!(out.text.contains(" in v0"), "{}", out.text);
    // a plane made by a gesture is written over its axes, and a projection with two operands
    let mut e = read(&format!("{VIEWS}a := point in front\nb := point in top\n"));
    let aux = Basis { u: [0.6, 0.8, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] };
    e.sketch.fixed_plane(aux, "aux");
    let (a, b) = (e.map.ent_named("a").unwrap(), e.map.ent_named("b").unwrap());
    let c = gcs_core::constraints::Constraint::project(&e.sketch, a, b).unwrap();
    e.sketch.add(c);
    let out = reconciled(&mut e);
    assert!(out.text.contains(" := plane(u: x"), "{}", out.text);
    assert!(out.text.contains("\nx0 := axis\n"), "a held direction is its own seed: {}", out.text);
    assert!(out.text.contains("fix(dir == (0.6, 0.8, 0), origin == (0, 0, 0)) x0\n"),
        "{}", out.text);
    assert!(out.text.contains("\na project b\n"), "{}", out.text);
    assert!(!out.text.contains("origin :="), "an origin is the plane's: {}", out.text);
    assert!(!out.text.contains("project("), "the planes are never spelled: {}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.planes.len(), e.sketch.planes.len());
    // a plane made through the edit API, over two axes and given a name
    let (prog, _, _) = gcs_core::library::parse_linked(VIEWS);
    let out = edit::add_plane(&prog, &["std.y".to_string(), "std.x".to_string()], Some("aux"));
    assert!(out.text.contains("aux := plane(u: std.y, v: std.x)"), "{}", out.text);
    read(&out.text);
    let out = edit::add_plane(&prog, &[], Some("front"));
    assert!(out.refused.is_some(), "a taken name is refused");
    let out = edit::add_plane(&prog, &[], Some("in"));
    assert!(out.refused.is_some(), "a reserved word is refused");
}

#[test]
fn commit_seeds_writes_a_rays_direction_and_a_planes_place() {
    let src = "use std\nr := axis hint(dir: (1, 0, 0))\np := plane(u: r, v: std.z)\n";
    let e = read(src);
    let mut sk = e.sketch.clone();
    let (r, p) = (at(&e, "r"), at(&e, "p"));
    for (k, x) in [0.6, 0.8, 0.0].iter().enumerate() {
        sk.params[sk.axes[r].d[k] as usize].value = *x;
    }
    for (k, x) in [1.0, 2.0, 3.0].iter().enumerate() {
        sk.params[sk.planes[p].o[k] as usize].value = *x;
    }
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("r := axis hint(dir: (0.6, 0.8, 0))"), "{}", out.text);
    assert!(out.text.contains("p := plane(u: r, v: std.z) hint(origin: (1, 2, 3))"), "{}", out.text);
    read(&out.text);
}

#[test]
fn remove_a_plane() {
    let src = format!(
        "{VIEWS}\
a := point in front hint((3, 4))
b := point hint((5, 105)) in top
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("top := plane"), "{}", out.text);
    assert!(!out.text.contains(") top\n"), "its fix goes too: {}", out.text);
    assert!(!out.text.contains("project"), "{}", out.text);
    assert!(out.text.contains("b := point hint((5, 105))\n"), "the clause came out: {}", out.text);
    assert!(out.text.contains("a := point in front hint((3, 4))"), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.planes.len(), 4 + 2);
    let b = at(&back, "b");
    assert!(back.sketch.points[b].z.is_some(), "a point in space now");
}

#[test]
fn an_in_block_is_the_clause_written_once() {
    let e = read(&format!(
        "{VIEWS}\
in top {{
  a := point hint((10, 90))
  l := line
  cycle 4 {{ (s := line) -> perpendicular equal }}
  a project b
}}
b := point in front hint((10, 5))
"
    ));
    let sk = &e.sketch;
    let top = at(&e, "top") as u32;
    // the origin, a, the line's two minted ends, and the cycle's four corners: every declaration
    // in the block, a nested block's copies included, is drawn in the plane
    assert_eq!(sk.points.iter().filter(|p| p.plane == Some(top)).count(), 1 + 7);
    // a constraint inside the block passes through unchanged
    assert_eq!(sk.user_constraints().iter().filter(|c| c.kind == CKind::Project).count(), 1);
    // the statements are the body's own, and none of them spells a clause it did not write
    let mut out = String::new();
    for st in e.program.root().body.iter().filter(|s| !matches!(s.kind, gcs_core::syntax::StmtKind::Block(_))) {
        write_stmt_to(&mut out, &st.kind).unwrap();
        out.push('\n');
    }
    assert_eq!(out.matches(" in ").count(), 1, "only b's own clause: {out}");
    // the one-line form reads too
    let e = read("use std\nin std.front { c := point hint((1, 2)) }\n");
    assert_eq!(e.sketch.plane_of(at(&e, "c")), Some(at(&e, "std.front")));
}

#[test]
fn an_in_block_refuses_what_it_cannot_mean() {
    refused("in nope { a := point }\n", "E101", "no such entity");
    refused(
        &format!("{VIEWS}a := point in front\nin top {{ l := line(a, hint((1, 2))) }}\n"),
        "E060",
        "already in `front`",
    );
    misparses("in front { f := plane(u: a, v: b) }\n", "has none of its own");
    misparses("in front { a := point in front }\n", "already in a plane");
    misparses("cycle 2 { in front { a := point } }\n", "stands at the top level");
    misparses("in front { in front { a := point } }\n", "stands at the top level");
    misparses("in front point a\n", "an `in` block is");
}

#[test]
fn removing_the_plane_unwraps_its_block() {
    let src = format!(
        "{VIEWS}\
in top {{
  a := point hint((10, 90))
  l := line
}}
b := point in front hint((1, 1))
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("in top"), "{}", out.text);
    assert!(!out.text.contains('{') && !out.text.contains('}'), "{}", out.text);
    assert!(out.text.contains("a := point hint((10, 90))"), "the statements stay: {}", out.text);
    assert!(out.text.contains("l := line"), "{}", out.text);
    assert!(!out.text.contains("project"), "{}", out.text);
    let back = read(&out.text);
    let a = at(&back, "a");
    assert_eq!(back.sketch.plane_of(a), None, "a point in space now");
    let b = at(&back, "b");
    assert_eq!(back.sketch.plane_of(b), Some(at(&back, "front")), "its own clause stands");
}

#[test]
fn a_seed_inside_a_block_splices_in_place() {
    let src = format!("{VIEWS}in top {{\n  a := point hint((10, 90))\n}}\n");
    let e = read(&src);
    let mut sk = e.sketch.clone();
    let a = e.map.ent_named("a").unwrap();
    let [px, py] = sk.point_params(a.i());
    sk.params[px as usize].value = 12.0;
    sk.params[py as usize].value = 95.0;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("a := point hint((12, 95))"), "{}", out.text);
    assert!(out.text.contains("in top {"), "the block is untouched: {}", out.text);
    read(&out.text);
}

/// A block over a plane reached by a path — `in std.front { … }` — is that plane to the gesture
/// as it is to the build.  Reconcile once read the clause's root alone (`std`), found no plane,
/// and took every point in the block for one moved out of it: an edit was refused on load.
#[test]
fn a_block_over_a_module_plane_is_not_a_membership_to_change() {
    let src = "unit mm\nuse std\nin std.front {\n  a := point\n}\n\
               fix((10, 5)) a\nb := point in std.front\nfix((3, 4)) b\n";
    let mut e = read(src);
    let a = e.map.ent_named("a").unwrap();
    assert_eq!(e.sketch.plane_of(a.i()), e.map.ent_named("std.front").map(|p| p.i()));
    let out = reconciled(&mut e);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert_eq!(out.text, src, "nothing to write");
}

#[test]
fn the_words_are_tinted() {
    let src = "top := plane(u: r, v: s)\nr := axis\na := point in top hint((3in, 0))\na project b\nunit in\n";
    let tints: Vec<(Tint, &str)> =
        highlight(src).into_iter().map(|(t, s)| (t, s.slice(src))).collect();
    let has = |t: Tint, w: &str| tints.iter().any(|(x, s)| *x == t && *s == w);
    assert!(has(Tint::Word, "plane"), "{tints:?}");
    assert!(has(Tint::Word, "axis"), "{tints:?}");
    assert!(has(Tint::Label, "u"), "{tints:?}");
    assert!(has(Tint::Relation, "project"), "{tints:?}");
    assert!(has(Tint::Type, "in"), "`unit in` names a unit: {tints:?}");
    let ins: Vec<&(Tint, &str)> = tints.iter().filter(|(_, s)| *s == "in").collect();
    assert!(ins.iter().any(|(t, _)| *t == Tint::Word), "the clause: {tints:?}");
    assert_eq!(ins.len(), 2, "`3in` stays plain: {tints:?}");
}

const SLOT: &str = "\
component Slot(p: Point, w: Length) {
  a := point hint((0, 0))
  b := point hint((10, 0))
  a distance(w) b
  l := line(a, b)
  arm := line(p, a)
}
";

#[test]
fn an_instance_may_be_drawn_in_a_view() {
    let e = read(&format!(
        "{VIEWS}{SLOT}\
x := point hint((5, 5)) in top
s1 := Slot(x, w: 12) in top
"
    ));
    let sk = &e.sketch;
    let top = at(&e, "top");
    let on = |n: &str| sk.plane_of(at(&e, n));
    assert_eq!(on("s1.a"), Some(top));
    assert_eq!(on("s1.b"), Some(top));
    assert_eq!(on("x"), Some(top), "the aliased argument joins through `arm`, and agrees");
    // the statement prints as written
    let (prog, errs) = parse("s1 := Slot(x, w: 12) in top\n");
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    write_stmt_to(&mut out, &prog.root().body[0].kind).unwrap();
    assert_eq!(out.trim(), "s1 := Slot(x, w: 12) in top");
    // and inside an `in` block the instance takes the block's plane
    let e = read(&format!(
        "{VIEWS}{SLOT}x := point hint((5, 5)) in front\nin front {{ s3 := Slot(x, w: 12) }}\n"
    ));
    assert_eq!(e.sketch.plane_of(at(&e, "s3.a")), Some(at(&e, "front")));
}

#[test]
fn an_instance_in_a_view_refuses_what_it_cannot_mean() {
    // an argument already on another plane is one image on two planes
    refused(
        &format!("{VIEWS}{SLOT}y := point hint((1, 1)) in front\ns3 := Slot(y, w: 5) in top\n"),
        "E060",
        "already in `front`",
    );
    // a plane given twice: a clause under an enclosing block, or under an outer instance
    misparses(
        &format!("{SLOT}q := point\nin front {{ s4 := Slot(q, w: 3) in front }}\n"),
        "already in a plane",
    );
    refused(
        &format!(
            "{VIEWS}{SLOT}\
component Two(p: Point, a: axis, b: axis) {{
  mine := plane(u: a, v: b)
  inner := Slot(p, w: 4) in mine
}}
q := point hint((1, 1)) in front
t := Two(q, std.x, std.y) in front
"
        ),
        "E103",
        "already in a plane",
    );
    // a plane inside is left alone: its origin is its own
    let e = read(&format!(
        "{VIEWS}component D(a: axis, b: axis) {{\n  f := plane(u: a, v: b)\n  c := point hint((1, 2))\n}}\nd1 := D(std.x, std.y) in front\n"
    ));
    assert_eq!(e.sketch.plane_of(at(&e, "d1.c")), Some(at(&e, "front")));
    let f = at(&e, "d1.f");
    assert_eq!(e.sketch.plane_of(e.sketch.planes[f].origin as usize), Some(f));
}

#[test]
fn removing_the_plane_takes_an_instances_clause() {
    let src = format!("{VIEWS}{SLOT}x := point hint((5, 5))\ns1 := Slot(x, w: 12) in top\n");
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(out.text.contains("s1 := Slot(x, w: 12)\n"), "the instance stays: {}", out.text);
    assert!(!out.text.contains("in top"), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.plane_of(at(&back, "s1.a")), None);
}

/// A statement expanded by `flatten` keeps the id of the statement it came from, so a plane
/// declared in a component is several planes from one id — each over the axis *its* copy was
/// given.  Keyed by that id, every copy read the first one's basis and came out silently wrong
/// (no diagnostic, just the wrong geometry).
#[test]
fn every_copy_of_a_plane_gets_its_own_basis() {
    let e = read("\
use std
component V(r: axis, up: axis) {
  v := plane(u: r, v: up)
}
x1 := V(std.x, std.z)
x2 := V(std.y, std.z)
");
    let near = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    let b = |n: &str| e.sketch.basis(at(&e, n));
    assert!(near(b("x1.v").u, [1.0, 0.0, 0.0]), "{:?}", b("x1.v"));
    assert!(near(b("x2.v").u, [0.0, 1.0, 0.0]), "the second copy over its own axis: {:?}", b("x2.v"));
}

/// A line between a point in a view and a point in space is a declaration that *names* its
/// points, and says nothing about planes — the case the `names_all` escape exists for.  Refused
/// there, `reconcile` returned the refusal for ever after and the source silently stopped
/// tracking the drawing (`syncSource` only reports it).
#[test]
fn a_line_across_two_views_does_not_jam_the_source() {
    let mut e = read(&format!("{VIEWS}a := point hint((5, 5)) in front\nb := point hint((9, 9))\n"));
    let (ai, bi) = (at(&e, "a"), at(&e, "b"));
    let mut sk = std::mem::take(&mut e.sketch);
    sk.line(ai, bi);
    e.sketch = sk;
    let first = reconciled(&mut e);
    assert_eq!(first.kind, Kind::Structural, "{:?}", first.refused);
    assert!(first.text.contains("l0 := line(a, b)"), "{}", first.text);
    let again = reconciled(&mut e);
    assert_eq!(again.kind, Kind::None, "the source keeps up: {:?}", again.refused);
    assert_eq!(again.refused, None);
}

/// #45.4 — an instance's `in PLANE` is written at the instance, in the caller's scope, and
/// resolves there: a declaration inside the component that happens to bear the plane's name
/// does not take it — and neither does one in a component nested below, where the clause
/// was carried down from the outermost instance.
#[test]
fn an_instance_in_plane_resolves_in_the_callers_scope() {
    let e = read(&format!(
        "{VIEWS}\ncomponent Dot(o: point) {{\n top := point hint((5, 5))\n o distance(5) top\n \
         o horizontal top\n}}\nk := Dot(top.origin) in top\n"
    ));
    let top = at(&e, "top");
    assert_eq!(e.sketch.plane_of(at(&e, "k.top")), Some(top));
    let e = read(&format!(
        "{VIEWS}\ncomponent Dot(o: point) {{\n top := point hint((5, 5))\n o distance(5) top\n}}\n\
         component Pair(o: point) {{\n top := point hint((9, 9))\n d := Dot(o)\n}}\n\
         k := Pair(top.origin) in top\n"
    ));
    let top = at(&e, "top");
    assert_eq!(e.sketch.plane_of(at(&e, "k.top")), Some(top));
    assert_eq!(e.sketch.plane_of(at(&e, "k.d.top")), Some(top));
    // a plane the caller cannot see is still nothing, named as written
    refused(
        &format!("{VIEWS}\ncomponent Dot(o: point) {{\n p := point hint((5, 5))\n}}\nk := Dot(top.origin) in nowhere\n"),
        "E101",
        "`nowhere`",
    );
}

/// Where a plane stands is document data like its axes, so a record of the sketch carries it:
/// a place survives `dumps`/`loads` and the graft.
#[test]
fn a_planes_place_survives_json_and_the_graft() {
    let e = read("unit mm\nuse std\nq := plane hint(origin: (0, 0, 12))\n\
                  fix(origin == (0, 0, 12)) q\nfix(dir == (1, 0, 0)) q.u\n\
                  fix(dir == (0, 1, 0)) q.v\na := point hint((5, 0)) in q\n");
    let q = at(&e, "q");
    let b = e.sketch.basis(q);
    assert!((b.along_normal() - 12.0).abs() < 1e-12, "stood off: {b:?}");
    let text = io::dumps(&e.sketch, Some(1));
    let back = io::loads(&text).unwrap();
    assert_eq!(back.basis(q), b);
    let copied = io::copy(&e.sketch, &[e.map.ent_named("a").unwrap(), EntRef::plane(q)]);
    assert_eq!(copied.planes.len(), 1);
    assert_eq!(copied.basis(0), b);
}
