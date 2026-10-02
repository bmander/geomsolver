//! The language half of multiview sketching (§6.7): `plane` declarations and their attitude,
//! the `in` clause, the `project` operator, and the writeback of each.
use gcs_core::constraints::CKind;
use gcs_core::edit::{self, Kind};
use gcs_core::model::{EntKind, EntRef};
use gcs_core::plane::Basis;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, parse, write_stmt_to, Tint};
use gcs_core::io;

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

/// Elaborates with an error carrying `code`, whose message contains `needle`.
fn refused(src: &str, code: &str, needle: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    let hit = e.errors().any(|d| d.code.as_str() == code && d.message.contains(needle));
    assert!(
        hit,
        "expected {code} `{needle}`\n{src}\n{:?}",
        e.diags.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
}

/// A part designed in one place (§6.7): a component whose body carries `in view { … }` blocks
/// over planes it was handed, with the projection tying its views inside it — and a view left
/// undrawn by `repeat 0`.
#[test]
fn a_component_carries_its_views_in_blocks() {
    let src = "\
Af := point in front
qf := point
front := plane(origin: Af, toward: qf)
Ar := point in right
qr := point
right := plane(origin: Ar, toward: qr, from: front, fold: -90deg)
fix(x == 0, y == 0) Af
fix(x == 40, y == 0) qf
fix(x == 150, y == 0) Ar
fix(x == 150, y == -40) qr
component Peg(f: plane, r: plane, cf: point, cr: point, draw_r: Int) {
  in f {
    a := point hint(x: cf.x, y: cf.y + 10)
    cf distance(0, along: x) a
    cf distance(10, along: y) a
  }
  repeat draw_r {
    in r {
      b := point hint(x: cr.x + 5, y: cr.y + 10)
      cr distance(5, along: x) b
    }
    a project b[0]
  }
}
p := Peg(front, right, Af, Ar, draw_r: 1)
q := Peg(front, right, Af, Ar, draw_r: 0)
";
    let e = read(src);
    assert_eq!(e.sketch.points.len(), 4 + 3, "a and b of p, a of q");
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let b = e.map.ent_named("p.b").or_else(|| {
        e.map.names.iter().find(|(_, ns)| ns.iter().any(|n| n.ends_with(".0.b"))).map(|(r, _)| *r)
    }).expect("p.b");
    let (bx, by) = sk.point_xy(b.i());
    assert!((bx - 155.0).abs() < 1e-6 && (by - 10.0).abs() < 1e-6, "{bx} {by}");
    // and inside a root block the clause is still written per declaration
    misparses(
        "o := point\nf := plane(origin: o, toward: hint(x: 1, y: 0))\nrepeat 2 { in f { p := point } }\n",
        "in a component",
    );
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

const VIEWS: &str = "\
o := point
q := point
o2 := point
q2 := point
o3 := point
q3 := point
front := plane(origin: o, toward: q)
top := plane(origin: o2, toward: q2, from: front, fold: 0deg)
right := plane(origin: o3, toward: q3, from: front, fold: -90deg)
fix(x == 0, y == 0) o
fix(x == 1, y == 0) q
fix(x == 0, y == 100) o2
fix(x == 1, y == 100) q2
fix(x == 150, y == 0) o3
fix(x == 150, y == -1) q3
";

#[test]
fn every_spelling_prints_back() {
    let src = "\
o := point hint(x: 0, y: 0)
q := point hint(x: 1, y: 0)
front := plane(origin: o, toward: q)
top := plane(origin: o, toward: q, from: front, fold: 0deg)
right := plane(from: front, fold: -90deg)
aux := plane(origin: o, toward: q, from: front, fold: 30deg)
p := plane(origin: o, toward: q, u: (0.6, 0.8, 0), v: (0, 0, 1))
a := point in top hint(x: 10, y: 5)
l := line(a, hint(x: 3, y: 4)) in top
b := point hint(x: 2, y: 2) in front
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
    // `in` is a trailer and prints after `hint`, and a plane's rotor prints as a frame's does;
    // every other statement prints as written
    let want = src.replace("a := point in top hint(x: 10, y: 5)", "a := point hint(x: 10, y: 5) in top");
    assert_eq!(squash(&out.replace(" hint(c: 0, s: 0)", "")), squash(&want));
    let e = read(src);
    assert_eq!(e.sketch.planes.len(), 5);
    assert_eq!(e.sketch.user_constraints().len(), 2);
    assert!(e.sketch.user_constraints()[1].claim);
}

#[test]
fn a_fold_chain_gives_the_bases() {
    let e = read(&format!("{VIEWS}aux := plane(origin: o, toward: q, from: top, fold: 30deg)\n"));
    let b = |n: usize| e.sketch.basis(n);
    let near = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    assert_eq!(b(0), Basis::page());
    assert!(near(b(1).u, [1.0, 0.0, 0.0]) && near(b(1).v, [0.0, 1.0, 0.0]));
    assert!(near(b(2).u, [0.0, 0.0, -1.0]) && near(b(2).v, [0.0, 1.0, 0.0]));
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    assert!(near(b(3).u, [c, s, 0.0]) && near(b(3).v, [0.0, 0.0, -1.0]));
    // a fold may read a parameter, and a plane may be declared before the one it folds from.
    // **`fold:` is written**: `from:` alone no longer means `fold: 0deg` — it says which plane
    // this one is derived from, and the clause beside it says how (§6.7).
    let e = read("\
tilt := 30deg
aux := plane(from: top, fold: tilt)
top := plane(from: front, fold: 0deg)
front := plane
");
    assert!(near(e.sketch.basis(0).u, [c, s, 0.0]));
    // and an explicit basis is orthonormalised on the way in
    let e = read("p := plane(u: (2, 0, 0), v: (1, 0, 3))\n");
    assert!(near(e.sketch.basis(0).v, [0.0, 0.0, 1.0]));
}

#[test]
fn attitude_refusals_carry_their_codes() {
    refused("a := plane(from: b)\nb := plane(from: a)\n", "E041", "folded from itself");
    refused("a := plane(from: a)\n", "E041", "folded from itself");
    refused("a := plane(from: nope)\n", "E101", "no such entity");
    refused("p := point\na := plane(from: p)\n", "E040", "`from` names a plane");
    refused("a := plane(u: (1, 0, 0), v: (2, 0, 0))\n", "E103", "do not span");
    refused("unit mm\na := plane(from: b, fold: 3mm)\nb := plane\n", "E103", "`fold` is Angle");
    misparses("a := plane(fold: 30deg)\n", "say `from:` too");
    misparses("a := plane(u: (1, 0, 0))\n", "both `u:` and `v:`");
    misparses("a := plane(from: b, u: (1, 0, 0), v: (0, 1, 0))\n", "not two of the three");
    // `from:` with neither clause is a plane *stood off* another, and one with both is refused
    misparses("a := plane(from: b, fold: 0deg, offset: 5)\n", "not both");
    misparses("a := plane(offset: 5)\n", "say `from:` too");
    misparses("p := line(from: b)\n", "has no attitude to give");
    misparses("a := plane(from: b, from: c)\n", "given twice");
}

#[test]
fn in_on_every_kind() {
    let e = read(&format!(
        "{VIEWS}\
a := point in top
l := line in top
c := circle in right
k := arc in right
k0 := point hint(x: 0, y: 0)
k1 := point hint(x: 1, y: 0)
k2 := point hint(x: 2, y: 1)
k3 := point hint(x: 3, y: 0)
s := spline(k0, k1, k2, k3) in front
"
    ));
    let sk = &e.sketch;
    let on = |name: &str| -> Vec<Option<usize>> {
        let r = e.map.ent_named(name).unwrap();
        let pts = if r.kind == EntKind::Point { vec![r] } else { sk.children(r) };
        pts.iter().map(|p| sk.plane_of(p.i())).collect()
    };
    assert_eq!(on("a"), vec![Some(1)]);
    assert_eq!(on("l"), vec![Some(1), Some(1)]);
    assert_eq!(on("c"), vec![Some(2)]);
    assert_eq!(on("k"), vec![Some(2); 3]);
    assert_eq!(on("s"), vec![Some(0); 4]);
    // a point named by a line in one plane and declared in another is one image on two planes
    refused(
        &format!("{VIEWS}a := point in front\nl := line(a, o) in top\n"),
        "E060",
        "already in `front`",
    );
    // agreement is not a conflict
    read(&format!("{VIEWS}a := point in top\nl := line(a, o2) in top\n"));
    refused("a := point in nope\n", "E101", "no such entity");
    refused("a := point in l\nl := line\n", "E040", "`in` names a plane");
    misparses("f := plane in top\n", "has none of its own");
    misparses("p := plane in top\n", "has none of its own");
    misparses("p := point in top in front\n", "already in a plane");
}

#[test]
fn unit_in_still_parses_and_in_is_not_a_name() {
    let e = read("unit in\np := point hint(x: 3in, y: 0)\nfront := plane\n");
    assert!((e.sketch.point_xy(0).0 - 3.0).abs() < 1e-12);
    // a point cannot be called `in`: the word is a clause's, and the parser says so
    let (_, errs) = parse("point in\n");
    assert!(!errs.is_empty());
    let (prog, errs) = parse("point in front\nfront := plane\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok());
    assert_eq!(e.sketch.plane_of(0), Some(0), "an anonymous point, in a plane");
}

#[test]
fn project_settles_refuses_and_claims() {
    let e = read(&format!("{VIEWS}a := point in front\nb := point in top\na project b\n"));
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(c.kind, CKind::Project);
    assert_eq!(c.entities()[2], EntRef::plane(0));
    assert_eq!(c.entities()[3], EntRef::plane(1));
    // each refusal at the statement's own span
    let (prog, _) = parse(&format!("{VIEWS}a := point in front\nb := point\na project b\n"));
    let e = elaborate(&prog);
    let d = e.errors().find(|d| d.code.as_str() == "E061").expect("refused");
    assert!(d.message.contains("no plane"), "{}", d.message);
    assert_eq!(d.span.slice(prog.text()), "a project b");
    refused(&format!("{VIEWS}a := point in front\nb := point in front\na project b\n"), "E061", "itself");
    refused(
        &format!("{VIEWS}front2 := plane\na := point in front\nb := point in front2\na project b\n"),
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
    assert_eq!(io::describe(c), "P6 project P7", "and positionally, with the planes left out");
}

#[test]
fn reconcile_writes_membership_a_plane_and_a_projection() {
    let mut e = read(VIEWS);
    let top = e.map.ent_named("top").unwrap().i();
    // a point drawn in the current plane: its statement says so
    let p = e.sketch.point(20.0, 110.0, false, "new");
    e.sketch.set_plane(p, Some(top));
    let out = reconciled(&mut e);
    assert_eq!(out.kind, Kind::Structural);
    assert!(out.text.contains("p0 := point hint(x: 20, y: 110) in top"), "{}", out.text);
    // an anonymous plane is named the moment a point is put in it
    let mut e = read("plane(origin: hint(x: 0, y: 0), toward: hint(x: 1, y: 0))\n");
    let p = e.sketch.point(3.0, 4.0, false, "new");
    e.sketch.set_plane(p, Some(0));
    let out = reconciled(&mut e);
    assert!(out.text.starts_with("v0 := plane("), "{}", out.text);
    assert!(out.text.contains(" in v0"), "{}", out.text);
    // a plane made by a gesture is written with its basis, and a projection with two operands
    let mut e = read(&format!("{VIEWS}a := point in front\nb := point in top\n"));
    let o = e.sketch.point(300.0, 0.0, false, "o4");
    let t = e.sketch.point(301.0, 0.0, false, "t4");
    e.sketch.plane(o, t, Basis::page().fold(0.5), "aux");
    let (a, b) = (e.map.ent_named("a").unwrap(), e.map.ent_named("b").unwrap());
    let c = gcs_core::constraints::Constraint::project(&e.sketch, a, b).unwrap();
    e.sketch.add(c);
    let out = reconciled(&mut e);
    assert!(out.text.contains("v0 := plane(origin: p0, toward: p1, u: ("), "{}", out.text);
    assert!(out.text.contains("\na project b\n"), "{}", out.text);
    assert!(!out.text.contains("project("), "the planes are never spelled: {}", out.text);
    read(&out.text);
    // a plane made through the edit API, folded from another and given a name
    let (prog, _) = parse(VIEWS);
    let out = edit::add_plane(
        &prog,
        &[],
        gcs_core::syntax::Attitude::From {
            plane: gcs_core::syntax::Ref::new("front"),
            fold: gcs_core::syntax::Arg::Dim { text: "30deg".into(), span: Default::default() },
        },
        Some("aux"),
        &[(0.0, 0.0), (40.0, 0.0)],
    );
    assert!(
        out.text.contains("aux := plane(origin: hint(x: 0, y: 0), toward: hint(x: 40, y: 0), from: front, fold: 30deg)"),
        "{}",
        out.text
    );
    read(&out.text);
    let out = edit::add_plane(&prog, &[], Default::default(), Some("front"), &[]);
    assert!(out.refused.is_some(), "a taken name is refused");
    let out = edit::add_plane(&prog, &[], Default::default(), Some("in"), &[]);
    assert!(out.refused.is_some(), "a reserved word is refused");
}

#[test]
fn commit_seeds_replaces_the_list() {
    let src = "front := plane\nright := plane(from: front, fold: -90deg)\n";
    let e = read(src);
    let mut sk = e.sketch.clone();
    // move the minted points, so the pose has to be written into the source
    for i in 0..sk.points.len() {
        let [x, y] = sk.point_params(i);
        sk.params[x as usize].value += 7.0;
        sk.params[y as usize].value += 1.0;
    }
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    let line = out.text.lines().nth(1).unwrap();
    // one list — the two minted points seeded in it, and the attitude kept — and the rotor
    assert_eq!(line.matches("plane").count(), 1, "{line}");
    assert_eq!(line.matches("from: front, fold: -90deg").count(), 1, "{line}");
    assert_eq!(line.matches("origin: hint(").count(), 1, "{line}");
    assert!(line.contains(") hint(c: "), "{line}");
    read(&out.text);
}

#[test]
fn remove_a_plane() {
    let src = format!(
        "{VIEWS}\
aux := plane(from: top, fold: 30deg)
a := point in front hint(x: 3, y: 4)
b := point hint(x: 5, y: 105) in top
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("top := plane"), "{}", out.text);
    assert!(!out.text.contains("aux := plane"), "a plane folded from it goes too: {}", out.text);
    assert!(!out.text.contains("project"), "{}", out.text);
    assert!(out.text.contains("b := point hint(x: 5, y: 105)\n"), "the clause came out: {}", out.text);
    assert!(out.text.contains("a := point in front hint(x: 3, y: 4)"), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.planes.len(), 2);
    assert_eq!(back.sketch.points.len(), 8);
}

#[test]
fn an_in_block_is_the_clause_written_once() {
    let e = read(&format!(
        "{VIEWS}\
in top {{
  a := point hint(x: 10, y: 90)
  l := line
  cycle 4 {{ (s := line) -> perpendicular equal }}
  a project b
}}
b := point in front hint(x: 10, y: 5)
"
    ));
    let sk = &e.sketch;
    let top = e.map.ent_named("top").unwrap().i() as u32;
    // a, the line's two minted ends, and the cycle's four corners: every declaration in the
    // block, a nested block's copies included, is drawn in the view
    assert_eq!(sk.points.iter().filter(|p| p.plane == Some(top)).count(), 7);
    // a constraint inside the block passes through unchanged
    assert_eq!(sk.user_constraints().iter().filter(|c| c.kind == CKind::Project).count(), 1);
    // the statements are the body's own, and none of them spells a clause it did not write
    let mut out = String::new();
    for st in e.program.stmts().filter(|s| !matches!(s.kind, gcs_core::syntax::StmtKind::Block(_))) {
        write_stmt_to(&mut out, &st.kind).unwrap();
        out.push('\n');
    }
    assert_eq!(out.matches(" in ").count(), 1, "only b's own clause: {out}");
    // the one-line form reads too (the declared point builds first: index 0)
    let e = read("front := plane\nin front { c := point hint(x: 1, y: 2) }\n");
    let c = e.map.ent_named("c").unwrap();
    assert_eq!(e.sketch.plane_of(c.i()), Some(0));
}

#[test]
fn an_in_block_refuses_what_it_cannot_mean() {
    refused("in nope { a := point }\n", "E101", "no such entity");
    refused(
        &format!("{VIEWS}a := point in front\nin top {{ l := line(a, hint(x: 1, y: 2)) }}\n"),
        "E060",
        "already in `front`",
    );
    misparses("front := plane\nin front { f := plane }\n", "has none of its own");
    misparses("front := plane\nin front { a := point in front }\n", "already in a plane");
    misparses("front := plane\ncycle 2 { in front { a := point } }\n", "stands at the top level");
    misparses("front := plane\nin front { in front { a := point } }\n", "stands at the top level");
    misparses("front := plane\nin front point a\n", "an `in` block is");
}

#[test]
fn removing_the_plane_unwraps_its_block() {
    let src = format!(
        "{VIEWS}\
in top {{
  a := point hint(x: 10, y: 90)
  l := line
}}
b := point in front hint(x: 1, y: 1)
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("in top"), "{}", out.text);
    assert!(!out.text.contains('{') && !out.text.contains('}'), "{}", out.text);
    assert!(out.text.contains("a := point hint(x: 10, y: 90)"), "the statements stay: {}", out.text);
    assert!(out.text.contains("l := line"), "{}", out.text);
    assert!(!out.text.contains("project"), "{}", out.text);
    let back = read(&out.text);
    let front = back.map.ent_named("front").unwrap().i();
    let a = back.map.ent_named("a").unwrap().i();
    assert_eq!(back.sketch.plane_of(a), None, "page geometry now");
    let b = back.map.ent_named("b").unwrap().i();
    assert_eq!(back.sketch.plane_of(b), Some(front), "its own clause stands");
}

#[test]
fn a_seed_inside_a_block_splices_in_place() {
    let src = format!("{VIEWS}in top {{\n  a := point hint(x: 10, y: 90)\n}}\n");
    let e = read(&src);
    let mut sk = e.sketch.clone();
    let a = e.map.ent_named("a").unwrap();
    let [px, py] = sk.point_params(a.i());
    sk.params[px as usize].value = 12.0;
    sk.params[py as usize].value = 95.0;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("a := point hint(x: 12, y: 95)"), "{}", out.text);
    assert!(out.text.contains("in top {"), "the block is untouched: {}", out.text);
    read(&out.text);
}

#[test]
fn the_words_are_tinted() {
    let src = "top := plane(origin: o, toward: q, from: front, fold: -90deg)\na := point in top hint(x: 3in, y: 0)\na project b\nunit in\n";
    let tints: Vec<(Tint, &str)> =
        highlight(src).into_iter().map(|(t, s)| (t, s.slice(src))).collect();
    let has = |t: Tint, w: &str| tints.iter().any(|(x, s)| *x == t && *s == w);
    assert!(has(Tint::Word, "plane"), "{tints:?}");
    assert!(has(Tint::Label, "from"), "{tints:?}");
    assert!(has(Tint::Label, "fold"), "{tints:?}");
    assert!(has(Tint::Relation, "project"), "{tints:?}");
    assert!(has(Tint::Type, "in"), "`unit in` names a unit: {tints:?}");
    let ins: Vec<&(Tint, &str)> = tints.iter().filter(|(_, s)| *s == "in").collect();
    assert!(ins.iter().any(|(t, _)| *t == Tint::Word), "the clause: {tints:?}");
    assert_eq!(ins.len(), 2, "`3in` stays plain: {tints:?}");
}

const SLOT: &str = "\
component Slot(p: Point, w: Length) {
  a := point hint(x: 0, y: 0)
  b := point hint(x: 10, y: 0)
  a distance(w) b
  l := line(a, b)
  arm := line(p, a)
}
";

#[test]
fn an_instance_may_be_drawn_in_a_view() {
    let e = read(&format!(
        "{VIEWS}{SLOT}\
x := point hint(x: 5, y: 5) in top
s1 := Slot(x, w: 12) in top
s2 := Slot(x, w: 12)
"
    ));
    let sk = &e.sketch;
    let top = e.map.ent_named("top").unwrap().i();
    let at = |n: &str| sk.plane_of(e.map.ent_named(n).unwrap().i());
    assert_eq!(at("s1.a"), Some(top));
    assert_eq!(at("s1.b"), Some(top));
    assert_eq!(at("s2.a"), None, "an instance with no clause stays where it was written");
    assert_eq!(at("x"), Some(top), "the aliased argument joins through `arm`, and agrees");
    // the statement prints as written
    let (prog, errs) = parse("s1 := Slot(x, w: 12) in top\n");
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    write_stmt_to(&mut out, &prog.root().body[0].kind).unwrap();
    assert_eq!(out.trim(), "s1 := Slot(x, w: 12) in top");
    // and inside an `in` block the instance takes the block's plane
    let e = read(&format!(
        "{VIEWS}{SLOT}x := point hint(x: 5, y: 5) in front\nin front {{ s3 := Slot(x, w: 12) }}\n"
    ));
    let front = e.map.ent_named("front").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("s3.a").unwrap().i()), Some(front));
}

#[test]
fn an_instance_in_a_view_refuses_what_it_cannot_mean() {
    // an argument already on another plane is one image on two planes
    refused(
        &format!("{VIEWS}{SLOT}y := point hint(x: 1, y: 1) in front\ns3 := Slot(y, w: 5) in top\n"),
        "E060",
        "already in `front`",
    );
    // a plane given twice: a clause under an enclosing block, or under an outer instance
    misparses(
        &format!("{SLOT}front := plane\nq := point\nin front {{ s4 := Slot(q, w: 3) in front }}\n"),
        "already in a plane",
    );
    refused(
        &format!(
            "{SLOT}\
component Two(p: Point) {{
  mine := plane
  inner := Slot(p, w: 4) in mine
}}
front := plane
q := point hint(x: 1, y: 1)
t := Two(q) in front
"
        ),
        "E103",
        "already in a plane",
    );
    // a datum inside is left alone: it has no points of its own to put on the plane
    let e = read(
        "component D() {\n  f := plane\n  c := point hint(x: 1, y: 2)\n}\nfront := plane\nd1 := D() in front\n",
    );
    let front = e.map.ent_named("front").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("d1.c").unwrap().i()), Some(front));
    let f = e.map.ent_named("d1.f").unwrap();
    for p in e.sketch.children(f) {
        assert_eq!(e.sketch.plane_of(p.i()), None, "a frame's points are the datum's own");
    }
}

#[test]
fn removing_the_plane_takes_an_instances_clause() {
    let src = format!("{VIEWS}{SLOT}x := point hint(x: 5, y: 5)\ns1 := Slot(x, w: 12) in top\n");
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(out.text.contains("s1 := Slot(x, w: 12)\n"), "the instance stays: {}", out.text);
    assert!(!out.text.contains("in top"), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.plane_of(back.map.ent_named("s1.a").unwrap().i()), None);
}

/// A statement expanded by `flatten` keeps the id of the statement it came from, so a plane
/// declared in a component is several planes from one id — each folded by the angle *its* copy
/// was given.  Keyed by that id, every copy read the first one's basis and came out silently
/// wrong (no diagnostic, just the wrong geometry).
#[test]
fn every_copy_of_a_plane_gets_its_own_basis() {
    let e = read("\
component V(base: plane, a: Angle) {
  vo := point hint(x: 0, y: 0)
  vq := point hint(x: 1, y: 0)
  v := plane(origin: vo, toward: vq, from: base, fold: a)
}
base := plane
x1 := V(base, a: 0deg)
x2 := V(base, a: 90deg)
");
    let near = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    let b = |n: &str| e.sketch.basis(e.map.ent_named(n).unwrap().i());
    assert!(near(b("x1.v").u, [1.0, 0.0, 0.0]), "{:?}", b("x1.v"));
    assert!(near(b("x2.v").u, [0.0, 0.0, 1.0]), "the 90° copy folds its own way: {:?}", b("x2.v"));
    // and a plane in a `cycle`, where the fold is the binder
    let e = read("base := plane\ncycle 3 as i { w := plane(from: base, fold: i * 30deg) }\n");
    assert_eq!(e.sketch.planes.len(), 4);
    let us: Vec<f64> = (1..e.sketch.planes.len()).map(|p| e.sketch.basis(p).u[2]).collect();
    assert!(us[0] < us[1] && us[1] < us[2], "each copy folds further: {us:?}");
}

/// A line between a point in a view and a point on the page is a declaration that *names* its
/// points, and says nothing about planes — the case the `names_all` escape exists for.  Refused
/// there, `reconcile` returned the refusal for ever after and the source silently stopped
/// tracking the drawing (`syncSource` only reports it).
#[test]
fn a_line_across_two_views_does_not_jam_the_source() {
    let mut e = read(&format!("{VIEWS}a := point hint(x: 5, y: 5) in front\nb := point hint(x: 9, y: 9)\n"));
    let (ai, bi) = (e.map.ent_named("a").unwrap().i(), e.map.ent_named("b").unwrap().i());
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
        "{VIEWS}\ncomponent Dot(o: point) {{\n top := point hint(x: 5, y: 5)\n o distance(5) top\n \
         o horizontal top\n}}\nk := Dot(o2) in top\n"
    ));
    let top = e.map.ent_named("top").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.top").unwrap().i()), Some(top));
    let e = read(&format!(
        "{VIEWS}\ncomponent Dot(o: point) {{\n top := point hint(x: 5, y: 5)\n o distance(5) top\n}}\n\
         component Pair(o: point) {{\n top := point hint(x: 9, y: 9)\n d := Dot(o)\n}}\n\
         k := Pair(o2) in top\n"
    ));
    let top = e.map.ent_named("top").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.top").unwrap().i()), Some(top));
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.d.top").unwrap().i()), Some(top));
    // a plane the caller cannot see is still nothing, named as written
    refused(
        &format!("{VIEWS}\ncomponent Dot(o: point) {{\n p := point hint(x: 5, y: 5)\n}}\nk := Dot(o2) in nowhere\n"),
        "E101",
        "`nowhere`",
    );
}

/// Where a plane stands along its normal is document data like its directions, so a record of
/// the sketch carries it: a derived offset survives `dumps`/`loads` and the graft, and a view
/// standing at the origin writes no `"o"` at all, so its record is what it always was.
#[test]
fn an_offset_plane_keeps_its_origin_through_json_and_the_graft() {
    let e = read("unit mm\np := plane\na := point hint(x: 5, y: 0)\nb := point hint(x: 30, y: 0)\n\
        q := plane(origin: a, toward: b, from: p, offset: 12mm)\n");
    let q = e.map.ent_named("q").unwrap().i();
    let b = e.sketch.basis(q);
    assert!((b.along_normal() - 12.0).abs() < 1e-12, "stood off by the offset: {b:?}");
    let text = io::dumps(&e.sketch, Some(1));
    assert_eq!(text.matches("\"o\"").count(), 1, "only the plane off the origin writes one");
    let back = io::loads(&text).unwrap();
    assert_eq!(back.basis(q), b);
    let pts = [e.map.ent_named("a").unwrap(), e.map.ent_named("b").unwrap()];
    let copied = io::copy(&e.sketch, &[pts[0], pts[1], EntRef::plane(q)]);
    assert_eq!(copied.planes.len(), 1);
    assert_eq!(copied.basis(0), b);
}
