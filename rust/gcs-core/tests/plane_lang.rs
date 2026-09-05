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
point Af hint(x: 0, y: 0) in front
point qf hint(x: 40, y: 0)
plane front(origin: Af, toward: qf)
point Ar hint(x: 150, y: 0) in right
point qr hint(x: 150, y: -40)
plane right(origin: Ar, toward: qr, from: front, fold: -90deg)
ground Af
ground qf
ground Ar
ground qr
component Peg(f: plane, r: plane, cf: point, cr: point, draw_r: Int) {
  in f {
    point a hint(x: cf.x, y: cf.y + 10)
    cf distance(0, along: x) a
    cf distance(10, along: y) a
  }
  repeat draw_r {
    in r {
      point b hint(x: cr.x + 5, y: cr.y + 10)
      cr distance(5, along: x) b
    }
    a project b[0]
  }
}
p: Peg(front, right, Af, Ar, draw_r: 1)
q: Peg(front, right, Af, Ar, draw_r: 0)
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
        "point o\nplane f(origin: o, toward: hint(x: 1, y: 0))\nrepeat 2 { in f { point p } }\n",
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
point o hint(x: 0, y: 0)
point q hint(x: 1, y: 0)
point o2 hint(x: 0, y: 100)
point q2 hint(x: 1, y: 100)
point o3 hint(x: 150, y: 0)
point q3 hint(x: 150, y: -1)
plane front(origin: o, toward: q)
plane top(origin: o2, toward: q2, from: front, fold: 0deg)
plane right(origin: o3, toward: q3, from: front, fold: -90deg)
ground o
ground q
ground o2
ground q2
ground o3
ground q3
";

#[test]
fn every_spelling_prints_back() {
    let src = "\
point o hint(x: 0, y: 0)
point q hint(x: 1, y: 0)
plane front(origin: o, toward: q)
plane top(origin: o, toward: q, from: front, fold: 0deg)
plane right(from: front, fold: -90deg)
plane aux(origin: o, toward: q, from: front, fold: 30deg)
plane p(origin: o, toward: q, u: (0.6, 0.8, 0), v: (0, 0, 1))
point a in top hint(x: 10, y: 5)
line l(a, hint(x: 3, y: 4)) in top
point b hint(x: 2, y: 2) in front
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
    let want = src.replace("point a in top hint(x: 10, y: 5)", "point a hint(x: 10, y: 5) in top");
    assert_eq!(squash(&out.replace(" hint(c: 0, s: 0)", "")), squash(&want));
    let e = read(src);
    assert_eq!(e.sketch.planes.len(), 5);
    assert_eq!(e.sketch.user_constraints().len(), 2);
    assert!(e.sketch.user_constraints()[1].claim);
}

#[test]
fn a_fold_chain_gives_the_bases() {
    let e = read(&format!("{VIEWS}plane aux(origin: o, toward: q, from: top, fold: 30deg)\n"));
    let b = |n: usize| e.sketch.planes[n].basis;
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
param tilt = 30deg
plane aux(from: top, fold: tilt)
plane top(from: front, fold: 0deg)
plane front
");
    assert!(near(e.sketch.planes[0].basis.u, [c, s, 0.0]));
    // and an explicit basis is orthonormalised on the way in
    let e = read("plane p(u: (2, 0, 0), v: (1, 0, 3))\n");
    assert!(near(e.sketch.planes[0].basis.v, [0.0, 0.0, 1.0]));
}

#[test]
fn attitude_refusals_carry_their_codes() {
    refused("plane a(from: b)\nplane b(from: a)\n", "E041", "folded from itself");
    refused("plane a(from: a)\n", "E041", "folded from itself");
    refused("plane a(from: nope)\n", "E101", "no such entity");
    refused("point p\nplane a(from: p)\n", "E040", "`from` names a plane");
    refused("plane a(u: (1, 0, 0), v: (2, 0, 0))\n", "E103", "do not span");
    refused("unit mm\nplane a(from: b, fold: 3mm)\nplane b\n", "E103", "`fold` is Angle");
    misparses("plane a(fold: 30deg)\n", "say `from:` too");
    misparses("plane a(u: (1, 0, 0))\n", "both `u:` and `v:`");
    misparses("plane a(from: b, u: (1, 0, 0), v: (0, 1, 0))\n", "not two of the three");
    // `from:` with neither clause is a plane *stood off* another, and one with both is refused
    misparses("plane a(from: b, fold: 0deg, offset: 5)\n", "not both");
    misparses("plane a(offset: 5)\n", "say `from:` too");
    misparses("point p(from: b)\n", "has no attitude to give");
    misparses("plane a(from: b, from: c)\n", "given twice");
}

#[test]
fn in_on_every_kind() {
    let e = read(&format!(
        "{VIEWS}\
point a in top
line l in top
circle c in right
arc k in right
point k0 hint(x: 0, y: 0)
point k1 hint(x: 1, y: 0)
point k2 hint(x: 2, y: 1)
point k3 hint(x: 3, y: 0)
spline s(k0, k1, k2, k3) in front
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
        &format!("{VIEWS}point a in front\nline l(a, o) in top\n"),
        "E060",
        "already in `front`",
    );
    // agreement is not a conflict
    read(&format!("{VIEWS}point a in top\nline l(a, o2) in top\n"));
    refused("point a in nope\n", "E101", "no such entity");
    refused("point a in l\nline l\n", "E040", "`in` names a plane");
    misparses("plane f in top\n", "has none of its own");
    misparses("plane p in top\n", "has none of its own");
    misparses("point p in top in front\n", "already in a plane");
}

#[test]
fn unit_in_still_parses_and_in_is_not_a_name() {
    let e = read("unit in\npoint p hint(x: 3in, y: 0)\nplane front\n");
    assert!((e.sketch.point_xy(0).0 - 3.0).abs() < 1e-12);
    // a point cannot be called `in`: the word is a clause's, and the parser says so
    let (_, errs) = parse("point in\n");
    assert!(!errs.is_empty());
    let (prog, errs) = parse("point in front\nplane front\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok());
    assert_eq!(e.sketch.plane_of(0), Some(0), "an anonymous point, in a plane");
}

#[test]
fn project_settles_refuses_and_claims() {
    let e = read(&format!("{VIEWS}point a in front\npoint b in top\na project b\n"));
    let c = &e.sketch.user_constraints()[0];
    assert_eq!(c.kind, CKind::Project);
    assert_eq!(c.entities()[2], EntRef::plane(0));
    assert_eq!(c.entities()[3], EntRef::plane(1));
    // each refusal at the statement's own span
    let (prog, _) = parse(&format!("{VIEWS}point a in front\npoint b\na project b\n"));
    let e = elaborate(&prog);
    let d = e.errors().find(|d| d.code.as_str() == "E061").expect("refused");
    assert!(d.message.contains("no plane"), "{}", d.message);
    assert_eq!(d.span.slice(prog.text()), "a project b");
    refused(&format!("{VIEWS}point a in front\npoint b in front\na project b\n"), "E061", "itself");
    refused(
        &format!("{VIEWS}plane front2\npoint a in front\npoint b in front2\na project b\n"),
        "E061",
        "parallel",
    );
    refused(&format!("{VIEWS}line l\nline m\nl project m\n"), "E040", "");
    let e = read(&format!("{VIEWS}point a in front\npoint b in top\nclaim a project b\n"));
    assert!(e.sketch.user_constraints()[0].claim);
}

#[test]
fn describe_and_write_skip_the_planes() {
    let e = read(&format!("{VIEWS}point a in front\npoint b in top\na project b\n"));
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
    assert!(out.text.contains("point   p0 hint(x: 20, y: 110) in top"), "{}", out.text);
    // an anonymous plane is named the moment a point is put in it
    let mut e = read("plane(origin: hint(x: 0, y: 0), toward: hint(x: 1, y: 0))\n");
    let p = e.sketch.point(3.0, 4.0, false, "new");
    e.sketch.set_plane(p, Some(0));
    let out = reconciled(&mut e);
    assert!(out.text.starts_with("plane v0("), "{}", out.text);
    assert!(out.text.contains(" in v0"), "{}", out.text);
    // a plane made by a gesture is written with its basis, and a projection with two operands
    let mut e = read(&format!("{VIEWS}point a in front\npoint b in top\n"));
    let o = e.sketch.point(300.0, 0.0, false, "o4");
    let t = e.sketch.point(301.0, 0.0, false, "t4");
    e.sketch.plane(o, t, Basis::page().fold(0.5), "aux");
    let (a, b) = (e.map.ent_named("a").unwrap(), e.map.ent_named("b").unwrap());
    let c = gcs_core::constraints::Constraint::project(&e.sketch, a, b).unwrap();
    e.sketch.add(c);
    let out = reconciled(&mut e);
    assert!(out.text.contains("plane   v0(origin: p0, toward: p1, u: ("), "{}", out.text);
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
        out.text.contains("plane   aux(origin: hint(x: 0, y: 0), toward: hint(x: 40, y: 0), from: front, fold: 30deg)"),
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
    let src = "plane front\nplane right(from: front, fold: -90deg)\n";
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
plane aux(from: top, fold: 30deg)
point a in front hint(x: 3, y: 4)
point b hint(x: 5, y: 105) in top
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("plane top"), "{}", out.text);
    assert!(!out.text.contains("plane aux"), "a plane folded from it goes too: {}", out.text);
    assert!(!out.text.contains("project"), "{}", out.text);
    assert!(out.text.contains("point b hint(x: 5, y: 105)\n"), "the clause came out: {}", out.text);
    assert!(out.text.contains("point a in front hint(x: 3, y: 4)"), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.planes.len(), 2);
    assert_eq!(back.sketch.points.len(), 8);
}

#[test]
fn an_in_block_is_the_clause_written_once() {
    let e = read(&format!(
        "{VIEWS}\
in top {{
  point a hint(x: 10, y: 90)
  line l
  cycle 4 {{ line s -> perpendicular equal }}
  a project b
}}
point b in front hint(x: 10, y: 5)
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
    let e = read("plane front\nin front { point c hint(x: 1, y: 2) }\n");
    let c = e.map.ent_named("c").unwrap();
    assert_eq!(e.sketch.plane_of(c.i()), Some(0));
}

#[test]
fn an_in_block_refuses_what_it_cannot_mean() {
    refused("in nope { point a }\n", "E101", "no such entity");
    refused(
        &format!("{VIEWS}point a in front\nin top {{ line l(a, hint(x: 1, y: 2)) }}\n"),
        "E060",
        "already in `front`",
    );
    misparses("plane front\nin front { plane f }\n", "has none of its own");
    misparses("plane front\nin front { point a in front }\n", "already in a plane");
    misparses("plane front\ncycle 2 { in front { point a } }\n", "stands at the top level");
    misparses("plane front\nin front { in front { point a } }\n", "stands at the top level");
    misparses("plane front\nin front point a\n", "an `in` block is");
}

#[test]
fn removing_the_plane_unwraps_its_block() {
    let src = format!(
        "{VIEWS}\
in top {{
  point a hint(x: 10, y: 90)
  line l
}}
point b in front hint(x: 1, y: 1)
a project b
"
    );
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(!out.text.contains("in top"), "{}", out.text);
    assert!(!out.text.contains('{') && !out.text.contains('}'), "{}", out.text);
    assert!(out.text.contains("point a hint(x: 10, y: 90)"), "the statements stay: {}", out.text);
    assert!(out.text.contains("line l"), "{}", out.text);
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
    let src = format!("{VIEWS}in top {{\n  point a hint(x: 10, y: 90)\n}}\n");
    let e = read(&src);
    let mut sk = e.sketch.clone();
    let a = e.map.ent_named("a").unwrap();
    let [px, py] = sk.point_params(a.i());
    sk.params[px as usize].value = 12.0;
    sk.params[py as usize].value = 95.0;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("point a hint(x: 12, y: 95)"), "{}", out.text);
    assert!(out.text.contains("in top {"), "the block is untouched: {}", out.text);
    read(&out.text);
}

#[test]
fn the_words_are_tinted() {
    let src = "plane top(origin: o, toward: q, from: front, fold: -90deg)\npoint a in top hint(x: 3in, y: 0)\na project b\nunit in\n";
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
  point a hint(x: 0, y: 0)
  point b hint(x: 10, y: 0)
  a distance(w) b
  line l(a, b)
  line arm(p, a)
}
";

#[test]
fn an_instance_may_be_drawn_in_a_view() {
    let e = read(&format!(
        "{VIEWS}{SLOT}\
point x hint(x: 5, y: 5) in top
s1: Slot(x, w: 12) in top
s2: Slot(x, w: 12)
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
    let (prog, errs) = parse("s1: Slot(x, w: 12) in top\n");
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    write_stmt_to(&mut out, &prog.root().body[0].kind).unwrap();
    assert_eq!(out.trim(), "s1: Slot(x, w: 12) in top");
    // and inside an `in` block the instance takes the block's plane
    let e = read(&format!(
        "{VIEWS}{SLOT}point x hint(x: 5, y: 5) in front\nin front {{ s3: Slot(x, w: 12) }}\n"
    ));
    let front = e.map.ent_named("front").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("s3.a").unwrap().i()), Some(front));
}

#[test]
fn an_instance_in_a_view_refuses_what_it_cannot_mean() {
    // an argument already on another plane is one image on two planes
    refused(
        &format!("{VIEWS}{SLOT}point y hint(x: 1, y: 1) in front\ns3: Slot(y, w: 5) in top\n"),
        "E060",
        "already in `front`",
    );
    // a plane given twice: a clause under an enclosing block, or under an outer instance
    misparses(
        &format!("{SLOT}plane front\npoint q\nin front {{ s4: Slot(q, w: 3) in front }}\n"),
        "already in a plane",
    );
    refused(
        &format!(
            "{SLOT}\
component Two(p: Point) {{
  plane mine
  inner: Slot(p, w: 4) in mine
}}
plane front
point q hint(x: 1, y: 1)
t: Two(q) in front
"
        ),
        "E103",
        "already in a plane",
    );
    // a datum inside is left alone: it has no points of its own to put on the plane
    let e = read(
        "component D() {\n  plane f\n  point c hint(x: 1, y: 2)\n}\nplane front\nd1: D() in front\n",
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
    let src = format!("{VIEWS}{SLOT}point x hint(x: 5, y: 5)\ns1: Slot(x, w: 12) in top\n");
    let e = read(&src);
    let top = e.map.ent_named("top").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[top], &[]);
    assert_eq!(out.kind, Kind::Structural, "{:?}", out.refused);
    assert!(out.text.contains("s1: Slot(x, w: 12)\n"), "the instance stays: {}", out.text);
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
component V(a: Angle) {
  point vo hint(x: 0, y: 0)
  point vq hint(x: 1, y: 0)
  plane v(origin: vo, toward: vq, from: base, fold: a)
}
plane base
x1: V(a: 0deg)
x2: V(a: 90deg)
");
    let near = |a: [f64; 3], c: [f64; 3]| (0..3).all(|i| (a[i] - c[i]).abs() < 1e-12);
    let b = |n: &str| e.sketch.planes[e.map.ent_named(n).unwrap().i()].basis;
    assert!(near(b("x1.v").u, [1.0, 0.0, 0.0]), "{:?}", b("x1.v"));
    assert!(near(b("x2.v").u, [0.0, 0.0, 1.0]), "the 90° copy folds its own way: {:?}", b("x2.v"));
    // and a plane in a `cycle`, where the fold is the binder
    let e = read("plane base\ncycle 3 as i { plane w(from: base, fold: i * 30deg) }\n");
    assert_eq!(e.sketch.planes.len(), 4);
    let us: Vec<f64> = e.sketch.planes[1..].iter().map(|p| p.basis.u[2]).collect();
    assert!(us[0] < us[1] && us[1] < us[2], "each copy folds further: {us:?}");
}

/// A line between a point in a view and a point on the page is a declaration that *names* its
/// points, and says nothing about planes — the case the `names_all` escape exists for.  Refused
/// there, `reconcile` returned the refusal for ever after and the source silently stopped
/// tracking the drawing (`syncSource` only reports it).
#[test]
fn a_line_across_two_views_does_not_jam_the_source() {
    let mut e = read(&format!("{VIEWS}point a hint(x: 5, y: 5) in front\npoint b hint(x: 9, y: 9)\n"));
    let (ai, bi) = (e.map.ent_named("a").unwrap().i(), e.map.ent_named("b").unwrap().i());
    let mut sk = std::mem::take(&mut e.sketch);
    sk.line(ai, bi);
    e.sketch = sk;
    let first = reconciled(&mut e);
    assert_eq!(first.kind, Kind::Structural, "{:?}", first.refused);
    assert!(first.text.contains("line    l0(a, b)"), "{}", first.text);
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
        "{VIEWS}\ncomponent Dot(o: point) {{\n point top hint(x: 5, y: 5)\n o distance(5) top\n \
         o horizontal top\n}}\nk: Dot(o2) in top\n"
    ));
    let top = e.map.ent_named("top").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.top").unwrap().i()), Some(top));
    let e = read(&format!(
        "{VIEWS}\ncomponent Dot(o: point) {{\n point top hint(x: 5, y: 5)\n o distance(5) top\n}}\n\
         component Pair(o: point) {{\n point top hint(x: 9, y: 9)\n d: Dot(o)\n}}\n\
         k: Pair(o2) in top\n"
    ));
    let top = e.map.ent_named("top").unwrap().i();
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.top").unwrap().i()), Some(top));
    assert_eq!(e.sketch.plane_of(e.map.ent_named("k.d.top").unwrap().i()), Some(top));
    // a plane the caller cannot see is still nothing, named as written
    refused(
        &format!("{VIEWS}\ncomponent Dot(o: point) {{\n point p hint(x: 5, y: 5)\n}}\nk: Dot(o2) in nowhere\n"),
        "E101",
        "`nowhere`",
    );
}
