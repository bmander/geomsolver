//! **Membership is one rule** (§6.7, §6.21, #105): a plane is a set of points, so `p coincident
//! P` and `p := point in P` say one thing, and the elaborator lowers both to the point drawn in
//! `P` — the same rows, the same topology.  A point whose declaration or statements say it
//! stands in space keeps the row in space.
use gcs_core::constraints::CKind;
use gcs_core::edit;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{dot, sub};

use crate::common::{ent, read};

/// A side plane standing square to the front through the origin, with a point drawn in it.
const SIDE: &str = "\
unit mm
use std
side := plane(u: std.z, v: std.y)
fix(origin == (0, 0, 0)) side
a := point hint((10, 20)) in side
fix((10, 20)) a
";

fn kinds(e: &Elaborated) -> Vec<CKind> {
    e.sketch.user_constraints().iter().map(|c| c.kind).collect()
}

fn plane_of(e: &Elaborated, n: &str) -> Option<usize> {
    e.sketch.plane_of(ent(e, n).i())
}

fn solved(e: &Elaborated) -> Sketch {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

/// How far a point stands off the side plane (the world's `x = 0`).
fn off_side(sk: &Sketch, e: &Elaborated, n: &str) -> f64 {
    let p = sk.world_point(ent(e, n).i());
    dot(sub(p, [0.0, 0.0, 0.0]), [1.0, 0.0, 0.0]).abs()
}

/// `p coincident side` is `p := point in side`: the point is drawn in the plane, two unknowns
/// and no row, and every relation over it reads as it would there.
#[test]
fn coincident_with_a_plane_is_membership() {
    let said_in = read(&format!("{SIDE}p := point hint((30, 5)) in side\np distance(25) a\n"));
    let said_on =
        read(&format!("{SIDE}p := point hint((30, 5))\np coincident side\np distance(25) a\n"));
    let side = Some(ent(&said_in, "side").i());
    assert_eq!(plane_of(&said_in, "p"), side);
    assert_eq!(plane_of(&said_on, "p"), side);
    // the distance between two points of one view is the plane's own, not the one in space
    assert_eq!(kinds(&said_in), vec![CKind::Distance]);
    assert_eq!(kinds(&said_on), vec![CKind::Distance]);
    assert_eq!(said_on.sketch.params.len(), said_in.sketch.params.len());
    assert_eq!(said_on.sketch.topology_key(), said_in.sketch.topology_key());
    let sk = solved(&said_on);
    assert!(off_side(&sk, &said_on, "p") < 1e-9);
    // read either way round
    let flipped = read(&format!("{SIDE}p := point\nside coincident p\n"));
    assert_eq!(plane_of(&flipped, "p"), side);
    assert!(kinds(&flipped).is_empty());
}

/// A line is its ends: `l coincident P` draws each one in `P`.
#[test]
fn a_line_on_a_plane_is_drawn_in_it() {
    let e = read(&format!("{SIDE}l := line(hint((0, 0)), hint((20, 10)))\nl coincident side\n"));
    let side = Some(ent(&e, "side").i());
    assert_eq!(plane_of(&e, "l.p1"), side);
    assert_eq!(plane_of(&e, "l.p2"), side);
    assert!(kinds(&e).is_empty());
    // an end already in the plane is in it: the other joins it
    let e = read(&format!("{SIDE}l := line(a, hint((20, 10)))\nl coincident side\n"));
    assert_eq!(plane_of(&e, "l.p2"), side);
    assert!(kinds(&e).is_empty());
}

/// The first plane a point is put on is the one it is drawn in; another is a row in space, and
/// a point already drawn in a view is put on another plane in space.
#[test]
fn the_first_plane_draws_and_another_is_a_row() {
    let e = read(&format!("{SIDE}p := point\np coincident side\np coincident std.top\n"));
    assert_eq!(plane_of(&e, "p"), Some(ent(&e, "side").i()));
    assert_eq!(kinds(&e), vec![CKind::PointOnPlane]);
    let sk = solved(&e);
    assert!(off_side(&sk, &e, "p") < 1e-9);
    assert!(sk.world_point(ent(&e, "p").i())[2].abs() < 1e-9);
    let e = read(&format!("{SIDE}b := point hint((5, 5)) in std.front\nb coincident side\n"));
    assert_eq!(kinds(&e), vec![CKind::PointOnPlane]);
}

/// A point its declaration or statements put in space stays there, and the plane holds it by a
/// row: a seed with a height, a hold of its three numbers, a claim, a tangency's contact.
#[test]
fn a_point_said_to_stand_in_space_keeps_the_row() {
    for (decl, more) in [
        ("p := point hint(x: 3, y: 4, z: 5)", ""),
        ("p := point", "fix((0, 4, 5)) p"),
    ] {
        let e = read(&format!("{SIDE}{decl}\np coincident side\n{more}\n"));
        assert_eq!(plane_of(&e, "p"), None, "{decl} {more}");
        assert_eq!(kinds(&e), vec![CKind::PointOnPlane], "{decl} {more}");
    }
    let e = read(&format!("{SIDE}p := point\nclaim p coincident side\n"));
    assert_eq!(plane_of(&e, "p"), None);
}

/// A set's body is read at the point it is used on: a plane in it draws the point there — and
/// a set written in place whose body is a circle's is that circle, the point put on it.
#[test]
fn a_sets_plane_draws_the_point_it_is_used_on() {
    let e = read(&format!(
        "{SIDE}q := point\nq coincident {{ p | p coincident side; p distance(12) std.origin }}\n"
    ));
    assert_eq!(plane_of(&e, "q"), Some(ent(&e, "side").i()));
    assert_eq!(kinds(&e), vec![CKind::Distance3]);
    let sk = solved(&e);
    assert!(off_side(&sk, &e, "q") < 1e-9);
    let e = read(&format!("{SIDE}q := point hint((0, 30)) in side\n\
                           q coincident {{ p | p coincident side; p distance(12) a }}\n"));
    assert_eq!(e.sketch.circles.len(), 1);
    assert_eq!(kinds(&e), vec![CKind::Radius, CKind::PointOnCircle]);
}

/// What a circle is drawn about may be put on its plane by `coincident`: it is drawn there.
#[test]
fn a_circle_about_a_point_on_a_plane_is_drawn_in_it() {
    let e = read(&format!("{SIDE}o := point\no coincident side\nk := circle(center: o) hint(r: 5)\n"));
    assert_eq!(plane_of(&e, "o"), Some(ent(&e, "side").i()));
}

/// A solve written back says the point's two coordinates in its plane, and reads back the same.
#[test]
fn a_written_pose_keeps_the_point_in_its_plane() {
    let src = format!("{SIDE}p := point\np coincident side\np distance(25) a\n");
    let e = read(&src);
    let sk = solved(&e);
    let text = edit::commit_seeds(&e, &sk, &e.program).text;
    assert!(text.contains("p := point hint(("), "{text}");
    let again = read(&text);
    assert_eq!(plane_of(&again, "p"), Some(ent(&again, "side").i()));
    assert_eq!(again.sketch.topology_key(), e.sketch.topology_key());
}

/// A circle centred on the front plane's origin, written as `circle` or as a set, with a point
/// on it and a disc swept from it.
fn disc(circle: &str) -> String {
    format!(
        "unit mm\nuse std\no := point in std.front\nfix((0, 0)) o\n{circle}\n\
         q := point hint((20, 10)) in std.front\nq coincident k\n\
         f := face(k)\ndisc := solid(f, depth: 4mm)\n"
    )
}

const WRITTEN: &str = "k := circle(center: o) hint(r: 25)\nradius(25) k";
const AS_SET: &str = "k := { p | p coincident std.front; p distance(25) o }";

/// What a sketch's figures and constraints are, read in order: kinds, entities and numbers.
fn said(sk: &Sketch) -> Vec<String> {
    sk.constraints.iter().map(|c| format!("{:?} {:?}", c.kind, c.args)).collect()
}

/// **A circle written as a set is the circle** (#105): `{ p | p coincident P; p distance(r) o }`
/// elaborates to what `circle(center: o)` with `radius(r)` does — the same entities, rows and
/// topology, decomposed alike, solved alike, its radius drawn `R25`, dragged alike and swept to
/// the same solid, exported to the same file.
#[test]
fn a_circle_written_as_a_set_is_the_circle() {
    use gcs_core::brep::{export, sweep::Say};
    let written = read(&disc(WRITTEN));
    let set = read(&disc(AS_SET));
    assert_eq!(set.sketch.circles.len(), 1);
    assert_eq!(set.sketch.topology_key(), written.sketch.topology_key());
    assert_eq!(said(&set.sketch), said(&written.sketch));
    assert_eq!(kinds(&set), vec![CKind::Radius, CKind::PointOnCircle]);
    assert_eq!(set.map.name_of(ent(&set, "k")).map(String::as_str), Some("k"));
    let plan = |sk: &Sketch| format!("{:?}", gcs_core::cgraph::build(sk).unsupported);
    assert_eq!(plan(&set.sketch), plan(&written.sketch));
    let (a, b) = (solved(&set), solved(&written));
    assert_eq!(a.params.iter().map(|p| p.value).collect::<Vec<_>>(),
               b.params.iter().map(|p| p.value).collect::<Vec<_>>());
    let callouts = |sk: &Sketch| gcs_core::callout::layout(sk, 0.1).into_iter()
        .map(|c| c.text).collect::<Vec<_>>();
    assert_eq!(callouts(&a), vec!["R25".to_string()]);
    assert_eq!(callouts(&a), callouts(&b));
    // a drag of the point on it walks the rim, as on the written circle
    let drag = |e: &Elaborated, mut sk: Sketch| {
        let q = ent(e, "q").i();
        let mut d = gcs_core::decompose::PlanDrag::new(&sk, q, 20.0, 10.0, None, 0.05);
        assert!(d.move_to(&mut sk, None, 0.0, 40.0).success);
        sk.point_xy(q)
    };
    assert_eq!(drag(&set, a.clone()), drag(&written, b.clone()));
    let say = Say { stage: &|_| {}, mark: &|_| {} };
    let step = |sk: &Sketch| {
        let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "disc").unwrap();
        let exact = export::exact(sk, body, None, None, &say).unwrap();
        export::step(&exact, "disc", None, &say).unwrap()
    };
    assert_eq!(step(&a), step(&b));
}

/// A family of them: `Round(o, std.front, r: 25)` is the circle, its radius the instance's.
#[test]
fn an_instance_of_a_family_of_circles_is_the_circle() {
    let family = "component Round(center: point, on: plane, r: Length) := { p |\n  \
                  p coincident on\n  p distance(r) center\n}\nk := Round(o, std.front, r: 25)";
    let set = read(&disc(family));
    let written = read(&disc(WRITTEN));
    assert_eq!(set.sketch.topology_key(), written.sketch.topology_key());
    assert_eq!(kinds(&set), kinds(&written));
    let sk = solved(&set);
    let callouts: Vec<String> =
        gcs_core::callout::layout(&sk, 0.1).into_iter().map(|c| c.text).collect();
    assert_eq!(callouts, vec!["R25".to_string()]);
    // drawn where the call gives it, and edited there
    let id = set.sketch.constraints.iter().find(|c| c.kind == CKind::Radius).unwrap().id;
    let out = edit::set_dimension(&set, &set.program, id, "r", "30");
    assert!(out.text.contains("Round(o, std.front, r: 30)"), "{}", out.text);
}

/// The radius drawn is the set's number, where the set writes it: editing it edits the set.
#[test]
fn a_set_circles_radius_is_edited_where_the_set_writes_it() {
    let e = read(&disc(AS_SET));
    let id = e.sketch.constraints.iter().find(|c| c.kind == CKind::Radius).unwrap().id;
    let out = edit::set_dimension(&e, &e.program, id, "r", "30");
    assert!(out.text.contains("p distance(30) o"), "{}", out.text);
}

/// A body that is no circle here is the set it says: a point on a line at a distance from a
/// point, or a sphere's points on a plane its centre stands off.  Nothing is drawn, and a point
/// put on it is put on the body.
#[test]
fn a_set_that_is_no_circle_here_stays_a_set() {
    let on_line = read(&format!(
        "{SIDE}l := line(hint((0, 0)), hint((40, 0))) in side\nfix((0, 0)) l.p1\nfix((40, 0)) l.p2\n\
         k := {{ p | p coincident l; p distance(25) a }}\nq := point hint((30, 0)) in side\n\
         q coincident k\n"));
    assert!(on_line.sketch.circles.is_empty());
    assert_eq!(kinds(&on_line), vec![CKind::PointOnLine, CKind::Distance]);
    let off = read(&format!(
        "{SIDE}c := point hint((5, 0)) in std.front\nfix((5, 0)) c\n\
         k := {{ p | p coincident side; p distance(13) c }}\nq := point\nq coincident k\n"));
    assert!(off.sketch.circles.is_empty());
    assert_eq!(plane_of(&off, "q"), Some(ent(&off, "side").i()));
    assert_eq!(kinds(&off), vec![CKind::Distance3]);
    let sk = solved(&off);
    let q = sk.world_point(ent(&off, "q").i());
    assert!(q[0].abs() < 1e-9 && (gcs_core::space::norm(sub(q, [5.0, 0.0, 0.0])) - 13.0).abs() < 1e-9);
}

/// A line touching a circle written as a set touches the set's body: the tangency reads the
/// body's rows, whatever the set is drawn as.
#[test]
fn a_line_tangent_to_a_set_circle_touches_it() {
    let e = read(&format!(
        "{}l := line(hint((-40, 30)), hint((40, 28))) in std.front\nhorizontal l\nl tangent k\n",
        disc(AS_SET).replace("use std\n", "use std (horizontal)\nuse std\n")));
    let sk = solved(&e);
    let l = &sk.lines[ent(&e, "l").i()];
    assert!((sk.point_xy(l.p1 as usize).1.abs() - 25.0).abs() < 1e-6);
}

/// A gesture on a point a `coincident` drew in its plane writes no `in` clause: the statement
/// says where it is drawn, and a clause beside it would say it twice.
#[test]
fn a_gesture_writes_no_clause_a_coincident_says() {
    let src = format!("{SIDE}p := point hint((3, 4))\np coincident side\n\
                       l := line(hint((0, 0)), hint((20, 10)))\nl coincident side\n");
    let mut e = read(&src);
    let mut sk = solved(&e);
    let p = ent(&e, "p").i();
    let x = sk.points[p].x as usize;
    sk.params[x].value += 5.0;
    let out = edit::reconcile(&mut e, &sk);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    let said = |n: &str| out.text.lines().find(|l| l.starts_with(n)).unwrap().to_string();
    assert!(!said("p :=").contains(" in ") && !said("l :=").contains(" in "), "{}", out.text);
    assert!(said("p :=").contains("hint((8, 4))"), "{}", out.text);
    let again = read(&out.text);
    assert_eq!(plane_of(&again, "p"), Some(ent(&again, "side").i()));
    assert!(kinds(&again).is_empty());
}

/// A set drawn as a circle has no clause of its own to write a solve into, and a gesture adds
/// nothing about it: its text stands as written.
#[test]
fn a_set_circles_text_stands_through_a_solve_and_a_gesture() {
    let mut e = read(&disc(AS_SET));
    let mut sk = solved(&e);
    let committed = edit::commit_seeds(&e, &sk, &e.program).text;
    assert!(committed.contains(AS_SET), "{committed}");
    let q = ent(&e, "q").i();
    let y = sk.points[q].y as usize;
    sk.params[y].value += 1.0;
    let out = edit::reconcile(&mut e, &sk);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(out.text.contains(AS_SET), "{}", out.text);
    assert_eq!(read(&out.text).sketch.topology_key(), e.sketch.topology_key());
}

/// A circle-shaped set written in a component is one circle per instance, and one per copy of a
/// block, each under its own name.
#[test]
fn a_set_circle_is_made_per_instance_and_per_copy() {
    let e = read("unit mm\nuse std\n\
        component Hub(o: point, on: plane) {\n  k := { p | p coincident on; p distance(5) o }\n}\n\
        a := point hint((0, 0)) in std.front\nb := point hint((20, 0)) in std.front\n\
        h1 := Hub(a, std.front)\nh2 := Hub(b, std.front)\n\
        repeat 2 {\n  m := { p | p coincident std.front; p distance(3) a }\n}\n");
    assert_eq!(e.sketch.circles.len(), 4);
    for n in ["h1.k", "h2.k"] {
        assert_eq!(ent(&e, n).kind, gcs_core::model::EntKind::Circle);
    }
    assert_eq!(kinds(&e), vec![CKind::Radius; 4]);
}

/// A body putting its point on another set is no plane's circle: it stays a set.
#[test]
fn a_set_on_a_set_is_no_circle() {
    let e = read(&format!(
        "{SIDE}ball := std.Sphere(a, r: 20)\nk := {{ p | p coincident ball; p distance(5) a }}\n\
         q := point\nq coincident k\n"));
    assert!(e.sketch.circles.is_empty());
    assert_eq!(kinds(&e).len(), 2);
}

/// Whether a set is a circle is judged once every membership is in, those another set's use draws
/// included (`lowering`, #140): `o coincident on`, `on` a set on the front plane, is what draws
/// `o` there, and so what makes `k` the circle about it — as `o coincident std.front` would.
#[test]
fn a_centre_another_sets_use_draws_is_a_circle() {
    let doc = |o: &str| format!(
        "unit mm\nuse std\non := {{ p | p coincident std.front }}\no := point\n{o}\n\
         k := {{ p | p coincident std.front; p distance(25) o }}\n\
         q := point hint((20, 10)) in std.front\nq coincident k\n");
    let by_set = read(&doc("o coincident on"));
    let said = read(&doc("o coincident std.front"));
    let front = Some(ent(&by_set, "std.front").i());
    assert_eq!(plane_of(&by_set, "o"), front);
    assert_eq!(by_set.sketch.circles.len(), 1);
    assert_eq!(kinds(&by_set), vec![CKind::Radius, CKind::PointOnCircle]);
    assert_eq!(by_set.sketch.topology_key(), said.sketch.topology_key());
    let sk = solved(&by_set);
    let callouts: Vec<String> =
        gcs_core::callout::layout(&sk, 0.1).into_iter().map(|c| c.text).collect();
    assert_eq!(callouts, vec!["R25".to_string()]);
}

/// A refusal is final (`lowering`): `k1`'s centre stands off the front plane, so it is walked as
/// a set, and `k2`'s centre `q`, put on `k1`, is in space when both are judged — so `k2` is walked
/// as a set too, though `k1`'s body then draws `q` in front.  Either way is one drawing: it solves.
#[test]
fn a_refusal_is_final() {
    let e = read("unit mm\nuse std\na := point\nfix((0, 3, 0)) a\n\
        k1 := { p | p coincident std.front; p distance(5) a }\n\
        q := point\nq coincident k1\n\
        k2 := { p | p coincident std.front; p distance(3) q }\n\
        s := point\ns coincident k2\n");
    assert!(e.sketch.circles.is_empty());
    let front = Some(ent(&e, "std.front").i());
    assert_eq!(plane_of(&e, "q"), front);
    assert_eq!(plane_of(&e, "s"), front);
    let sk = solved(&e);
    let at = |n: &str| sk.world_point(ent(&e, n).i());
    let d = |a: [f64; 3], b: [f64; 3]| dot(sub(a, b), sub(a, b)).sqrt();
    assert!((d(at("q"), at("a")) - 5.0).abs() < 1e-6);
    assert!((d(at("s"), at("q")) - 3.0).abs() < 1e-6);
}
