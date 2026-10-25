//! **A plane's normal, `P.normal`**: an axis along `u × v` through the plane's origin, minted where
//! the document reads it (`Sketch::plane_normal`) and held so by the plane's own rows — square to
//! each axis on `u × v`'s side, through the origin — with no freedom of its own.  Held here against `u × v` as
//! solved, sense and all, against the freedoms it adds (none), the places it may be read (a
//! relation, a later plane's slot, a component's formal, a `hint(dir:)`), a plane declared after
//! the reading, JSON and a lifted program, and the workspace, which draws it nowhere.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::overview::scene3d;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, norm, sub};

use crate::common::{ent, read, refused, unit};

fn solved(sk: &Sketch) -> Sketch {
    let mut sk = sk.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

fn dof(sk: &Sketch) -> i64 {
    diagnose(&mut sk.clone(), DiagnoseOptions::default()).dof
}

fn dir(sk: &Sketch, r: usize) -> [f64; 3] {
    sk.axes[r].d.map(|p| sk.params[p as usize].value)
}

fn place(sk: &Sketch, r: usize) -> [f64; 3] {
    sk.axes[r].a.map(|p| sk.params[p as usize].value)
}

/// Plane `pi`'s normal axis, which must have been minted.
fn normal(sk: &Sketch, pi: usize) -> usize {
    sk.frames[pi].normal.expect("the plane's normal was read, so it was minted") as usize
}

/// The normal stands along `u × v` — the sense the plane's `n` has — and through its origin.
fn is_u_cross_v(sk: &Sketch, pi: usize) {
    let n = normal(sk, pi);
    let pl = &sk.frames[pi];
    let want = unit(cross(dir(sk, pl.u as usize), dir(sk, pl.v as usize)));
    assert!(norm(sub(unit(dir(sk, n)), want)) < 1e-8, "{:?} is not u × v, {want:?}", dir(sk, n));
    let o = pl.o.map(|q| sk.params[q as usize].value);
    let d = unit(dir(sk, n));
    let off = norm(cross(sub(o, place(sk, n)), d));
    assert!(off < 1e-8, "the normal misses the plane's origin by {off}");
}

/// The front is x and z, so its normal is x × z: toward −y, the viewer's side of it.  A line
/// parallel to it runs along y, and the normal takes no freedom of the drawing's.
#[test]
fn a_held_plane_has_its_normal_along_u_cross_v() {
    let src = "use std\na := point hint((1, 2, 3))\nb := point hint((2, 7, 4))\nl := line(a, b)\n";
    let without = read(src);
    let e = read(&format!("{src}l parallel std.front.normal\n"));
    let sk = solved(&e.sketch);
    let front = ent(&e, "std.front").i();
    is_u_cross_v(&sk, front);
    assert!(norm(sub(dir(&sk, normal(&sk, front)), [0.0, -1.0, 0.0])) < 1e-9);
    let l = &sk.lines[ent(&e, "l").i()];
    let run = sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize));
    assert!(norm(cross(run, [0.0, 1.0, 0.0])) < 1e-8 * norm(run), "{run:?} is not along y");
    assert_eq!(ent(&e, "std.front.normal"), EntRef::new(EntKind::Axis, normal(&sk, front)));
    assert_eq!(dof(&e.sketch), dof(&without.sketch) - 2, "the parallel takes two, the normal none");
    // and a plane nothing reads the normal of mints none
    assert!(without.sketch.frames.iter().all(|p| p.normal.is_none()));
}

/// A plane its relations turn carries its normal with it, square to both axes and on the side
/// `u × v` says — a solve that left it pointing back would satisfy every row but the sense.
#[test]
fn the_normal_turns_with_its_plane() {
    let e = read("\
use std
P := plane(u: hint(dir: (1, 0, 0.1)), v: hint(dir: (0.1, 1, 0)))
fix(origin == (5, 0, 0)) P
k := point hint((10, 10, 10))
fix(x == 10, y == 10, z == 10) k
m := line(P.origin, k)
P.u parallel m
P.v perpendicular std.z
P.v perpendicular m
c := point hint((0, 0, 0))
d := point hint((1, 1, 1))
s := line(c, d)
s parallel P.normal
");
    let sk = solved(&e.sketch);
    let pi = ent(&e, "P").i();
    is_u_cross_v(&sk, pi);
    let s = &sk.lines[ent(&e, "s").i()];
    let run = sub(sk.world_point(s.p2 as usize), sk.world_point(s.p1 as usize));
    assert!(norm(cross(run, dir(&sk, normal(&sk, pi)))) < 1e-8 * norm(run));
}

/// The normal's sense is u × v as the holds leave the axes, not as they were seeded: a plane
/// minted facing the front and held facing along x has its normal along x, not against it.
#[test]
fn the_normal_is_seeded_after_the_holds() {
    let e = read("\
use std
P := plane
fix(dir == (0, 1, 0)) P.u
fix(dir == (0, 0, 1)) P.v
fix(origin == (0, 0, 0)) P
a := point hint((0, 0, 0))
b := point hint((3, 1, 1))
l := line(a, b)
l parallel P.normal
");
    let sk = solved(&e.sketch);
    let pi = ent(&e, "P").i();
    is_u_cross_v(&sk, pi);
    assert!(norm(sub(dir(&sk, normal(&sk, pi)), [1.0, 0.0, 0.0])) < 1e-9);
}

/// A normal held against the way its plane was seeded turns the plane over, never itself alone
/// (#629): `P` seeded facing up, its `u` held along x and its normal down, has its `v` along −y.
/// Square to both axes either way round, the normal turned alone would satisfy every row and
/// leave `u × v` up, so what stands on the plane would stand mirrored.
#[test]
fn a_normal_held_against_its_seed_turns_its_plane_over() {
    let e = read("\
use std
P := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 1, 0)))
fix(origin == (0, 0, 0)) P
fix(dir == (1, 0, 0)) P.u
P.v perpendicular P.u
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
P.normal parallel(sense: same) down
");
    let sk = solved(&e.sketch);
    let pi = ent(&e, "P").i();
    is_u_cross_v(&sk, pi);
    assert!(norm(sub(dir(&sk, sk.frames[pi].v as usize), [0.0, -1.0, 0.0])) < 1e-9);
    assert!(norm(sub(dir(&sk, normal(&sk, pi)), [0.0, 0.0, -1.0])) < 1e-9);
    assert_eq!(dof(&e.sketch), 0);
}

/// A later plane may stand on it — `plane(u: P.normal, …)` — a component may read its plane
/// formal's, and `hint(dir: P.normal)` reads what `hint(dir: P.n)` does.
#[test]
fn the_normal_is_read_where_an_axis_is() {
    let e = read("\
use std
P := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0.8, 0.6)))
Q := plane(u: P.normal, v: hint(dir: (1, 0, 0.2)))
Q.v parallel P.u
R := plane(u: hint(dir: P.normal), v: hint(dir: P.n, turn: 90deg, about: P.u))
");
    let sk = solved(&e.sketch);
    let (p, q) = (ent(&e, "P").i(), ent(&e, "Q").i());
    is_u_cross_v(&sk, p);
    assert_eq!(sk.frames[q].u as usize, normal(&sk, p), "Q's `u` is P's normal itself");
    assert!(sk.frames[q].normal.is_none());
    // read where P stands once the statements about the planes have placed it
    let r = &e.sketch.frames[ent(&e, "R").i()];
    let n = unit(e.sketch.basis(p).normal());
    assert!(norm(sub(unit(dir(&e.sketch, r.u as usize)), n)) < 1e-9, "`normal` seeds as `n` does");

    let e = read("\
use std
component Post(f: plane) {
  a := point
  b := point hint((1, 2, 30))
  l := line(a, b)
  l parallel f.normal
  a coincident f.origin
  a distance(10) b
}
p := Post(std.top)
");
    let sk = solved(&e.sketch);
    let top = ent(&e, "std.top").i();
    is_u_cross_v(&sk, top);
    let b = sk.world_point(ent(&e, "p.b").i());
    assert!(norm(sub(b, [0.0, 0.0, 10.0])) < 1e-8, "the post stands up off the top: {b:?}");
}

/// A plane's normal is minted with it, so a declaration before the plane cannot name it.
#[test]
fn a_normal_read_before_its_plane_is_refused() {
    refused("use std\nQ := plane(u: P.normal, v: std.x)\nP := plane\n", "E040",
            "`P` is declared after this", "P.normal");
}

/// Across JSON the normal stays its plane's — its rows put back by the reader — and a lifted
/// program names it as the plane's member rather than declaring an axis.
#[test]
fn the_normal_survives_json_and_lift() {
    let e = read("\
use std
P := plane(u: hint(dir: (1, 0, 0.3)), v: hint(dir: (0, 1, 0)))
a := point hint((0, 0, 0))
b := point hint((1, 2, 30))
l := line(a, b)
l parallel P.normal
");
    let before = dof(&e.sketch);
    let back = io::from_json(&io::to_json(&e.sketch)).expect("reads back");
    let pi = ent(&e, "P").i();
    assert_eq!(back.frames[pi].normal, e.sketch.frames[pi].normal);
    assert_eq!(dof(&back), before);
    is_u_cross_v(&solved(&back), pi);
    let text = gcs_core::program::dumps(&e.sketch);
    let lifted = text.lines().filter(|l| l.contains(".normal")).count();
    assert!(lifted > 0, "{text}");
    let again = read(&text);
    assert_eq!(dof(&again.sketch), before, "{text}");
}

/// The workspace draws it nowhere: not among what is drawn, not in the scene in space, and it
/// is never what a click picks.
#[test]
fn the_normal_is_not_drawn() {
    let e = read("use std\na := point\nb := point hint((1, 2, 3))\nl := line(a, b)\nl parallel std.top.normal\n");
    let n = EntRef::new(EntKind::Axis, normal(&e.sketch, ent(&e, "std.top").i()));
    assert!(!e.sketch.drawn().contains(&n));
    assert!(scene3d(&e.sketch, 0.0).iter().all(|it| it.of != Some(n)));
    // every other axis still is
    let axes = e.sketch.drawn().iter().filter(|r| r.kind == EntKind::Axis).count();
    assert_eq!(axes, e.sketch.axes.len() - 1);
}

/// **A normal is the plane's, and no axis a gesture drew** (#463): `o.plane.normal` is minted
/// after every axis the elaboration made and is no statement's, so a reconcile after a solve —
/// what the page does after every edit — writes nothing for it.  It once appended an
/// `x0 := axis`, and the text then elaborated to six more parameters than the drawing held,
/// which every worker given the page's values refused (the engine's timing drive).
#[test]
fn a_normal_read_writes_nothing_back() {
    for reading in ["a.plane.normal", "std.top.normal"] {
        let src = format!("use std\na := point in std.top\nb := point hint((1, 2, 3))\n\
            l := line(a, b)\nl parallel {reading}\n");
        let mut e = read(&src);
        let sk = solved(&e.sketch);
        let out = gcs_core::edit::reconcile(&mut e, &sk);
        assert!(out.refused.is_none(), "{reading}: {:?}", out.refused);
        assert!(!out.text.contains(":= axis"), "{reading}: reconcile wrote {}", out.text);
        let again = read(&out.text);
        assert_eq!(again.sketch.params.len(), sk.params.len(), "{reading}: {}", out.text);
    }
}
