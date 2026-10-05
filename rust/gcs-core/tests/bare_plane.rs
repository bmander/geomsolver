//! A bare `p := plane` (#84): its origin and two axes it mints, `p.u` and `p.v`, all free — three
//! numbers of place and two of direction each, seven in all — placed by the relations every
//! other entity takes.  A plane's axes pass through its origin, so two axes `coincident` with
//! `p.u` and `p.v` hold the plane where they meet; two held axes that do not meet are refused.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::{EntKind, Sketch};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};
use gcs_core::syntax::write_stmt_to;

use crate::common::{ent, read, refused, unit};

fn dof(sk: &mut Sketch) -> i64 {
    diagnose(sk, DiagnoseOptions::default()).dof
}

/// Axis `n` where it stands: a point on it and its direction.
fn line(e: &Elaborated, sk: &Sketch, n: &str) -> ([f64; 3], [f64; 3]) {
    let r = &sk.axes[ent(e, n).i()];
    (r.a.map(|q| sk.params[q as usize].value), unit(r.d.map(|q| sk.params[q as usize].value)))
}

/// How far `x` stands off the line through `a` along unit `d`.
fn off(x: [f64; 3], (a, d): ([f64; 3], [f64; 3])) -> f64 {
    norm(cross(sub(x, a), d))
}

#[test]
fn a_bare_plane_is_seven_freedoms() {
    let e = read("use std\np := plane\n");
    let mut sk = e.sketch.clone();
    assert_eq!(dof(&mut sk), 3 + 2 + 2, "a place, and a direction for each axis");
    for n in ["p.u", "p.v"] {
        assert_eq!(ent(&e, n).kind, EntKind::Axis, "{n} is an axis the plane minted");
    }
    // seeded as the front plane is: u right and v up
    let b = sk.basis(ent(&e, "p").i());
    assert_eq!((b.u, b.v), ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    // where the numbers landed is reported under the axes' dotted names
    let names: Vec<String> =
        gcs_core::report::positions(&sk, &e.map).into_iter().map(|(n, _)| n).collect();
    for n in ["p.u.x", "p.u.px", "p.v.z"] {
        assert!(names.iter().any(|m| m == n), "`{n}` in {names:?}");
    }
}

/// One slot written and the other minted: `plane(u: std.x)` takes its u and makes its v.
#[test]
fn a_plane_mints_only_the_axis_it_is_not_given() {
    let e = read("use std\np := plane(u: std.x)\n");
    assert_eq!(e.sketch.planes[ent(&e, "p").i()].u as usize, ent(&e, "std.x").i());
    assert_eq!(ent(&e, "p.v").kind, EntKind::Axis);
    let mut sk = e.sketch.clone();
    // the origin slides along std.x, and v turns about it
    assert_eq!(dof(&mut sk), 1 + 2);
}

const FRAME: &str = "\
u := axis
v := axis
u perpendicular v
p := plane
u coincident p.u
v coincident p.v
";

/// Two square axes `coincident` with a bare plane's: the plane is the frame they make, standing
/// where they meet — a rigid body in space, six freedoms, and no row the others imply.
#[test]
fn two_square_axes_make_a_frame() {
    let e = read(FRAME);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 6, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    let b = sk.basis(ent(&e, "p").i());
    let (u, v) = (line(&e, &sk, "u"), line(&e, &sk, "v"));
    assert!(off(b.o, u) < 1e-9 && off(b.o, v) < 1e-9, "the origin is where u and v meet");
    assert!(norm(cross(b.normal(), cross(u.1, v.1))) < 1e-9, "its normal is u × v");
    assert!(dot(u.1, v.1).abs() < 1e-9);
}

/// Held, the two axes hold the plane: it stands where they meet, with nothing left to solve.
#[test]
fn two_held_axes_hold_the_plane_where_they_meet() {
    let e = read(&format!("{FRAME}fix(x == 0, y == 1, z == 0, px == 3, py == 0, pz == 4) u\n\
                           fix(x == 0, y == 0, z == 1, px == 3, py == 0, pz == 0) v\n"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    let b = sk.basis(ent(&e, "p").i());
    assert!(norm(sub(b.o, [3.0, 0.0, 4.0])) < 1e-9, "{:?}", b.o);
    assert!(norm(sub(b.normal(), [1.0, 0.0, 0.0])) < 1e-9, "y × z: {:?}", b.normal());
}

/// Held axes a plane cannot stand on: two that miss each other, two that run alike, and a plane
/// held off its own held axes.
#[test]
fn a_plane_over_held_axes_that_do_not_meet_is_refused() {
    let held = "use std\na := axis\nfix(x == 1, y == 0, z == 0, px == 0, py == 0, pz == 0) a\nb := axis\n";
    refused(&format!("{held}fix(x == 0, y == 1, z == 0, px == 0, py == 0, pz == 5) b\n\
                      p := plane(u: a, v: b)\n"),
            "E067", "do not meet", "p := plane(u: a, v: b)");
    refused(&format!("{held}fix(x == 1, y == 0, z == 0, px == 0, py == 2, pz == 0) b\n\
                      p := plane(u: a, v: b)\n"),
            "E067", "run alike", "p := plane(u: a, v: b)");
    refused("use std\np := plane(u: std.x, v: std.z)\nfix(x == 0, y == -5, z == 0) p\n",
            "E067", "is held where its axis `p.u` is not", "p := plane(u: std.x, v: std.z)");
}

/// A relation that read an axis's place, removed, leaves the place held again: no freedom stays
/// behind for a place nothing reads.
#[test]
fn removing_what_placed_an_axis_holds_its_place_again() {
    let e = read(FRAME);
    let mut sk = e.sketch.clone();
    let before = dof(&mut sk);
    let u = ent(&e, "u");
    let c = sk.user_constraints().iter()
        .find(|c| c.kind == CKind::AxisCoincident && c.entities().contains(&u)).unwrap().id;
    sk.remove(c);
    assert!(!sk.axes[u.i()].placed);
    // four rows gone, and with them the two freedoms of where u stood
    assert_eq!(dof(&mut sk), before + 4 - 2);
}

/// **A plane's own axis takes a seed** in its slot, as a line's own point does: the direction
/// it starts from, so `parallel` begins where it should and picks the sense written.
#[test]
fn a_plane_slot_seeds_the_axis_it_mints() {
    let e = read("use std\nb := plane(v: hint(x: 0, y: -1, z: 0)) hint(x: 0, y: 0, z: 12)\n\
                  b.u parallel std.x\nb.v parallel std.y\n");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (_, v) = line(&e, &sk, "b.v");
    assert!(norm(sub(v, [0.0, -1.0, 0.0])) < 1e-9, "the sense written: {v:?}");
    let b = sk.basis(ent(&e, "b").i());
    assert!(norm(sub(b.normal(), [0.0, 0.0, -1.0])) < 1e-9, "x × -y looks down: {:?}", b.normal());
}

/// A child's seed prints back as written, and a point's takes no `z`.
#[test]
fn a_plane_slot_seed_prints_back_and_a_point_refuses_z() {
    let decl = "p := plane(u: hint(x: 0, y: 1, z: 0), v: hint(x: 0, y: 0, z: 1))";
    let (prog, errs, _) = gcs_core::library::parse_linked(&format!("use std\n{decl}\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let printed: Vec<String> = prog.root().body.iter().map(|st| {
        let mut out = String::new();
        write_stmt_to(&mut out, &st.kind).unwrap();
        out
    }).collect();
    assert!(printed.iter().any(|p| p == decl), "{printed:?}");
    let e = read(&format!("use std\n{decl}\n"));
    let (_, u) = line(&e, &e.sketch, "p.u");
    assert!(norm(sub(u, [0.0, 1.0, 0.0])) < 1e-12);
    refused("use std\nin std.front {\n  l := line(hint(x: 1, y: 2, z: 3), hint(x: 4, y: 5))\n}\n",
            "E040", "an anonymous point has no `z`", "3");
    refused("use std\np := plane(u: hint(x: 0, y: 0, z: 0))\n",
            "E103", "points nowhere", "hint(x: 0, y: 0, z: 0)");
}

/// **A solve that turns a plane's own axes writes where they point** into its slots — the
/// list where the source wrote none, the numbers in place where it wrote a seed — and the text
/// written reads back to the same plane.
#[test]
fn a_solve_writes_a_plane_s_own_axes_back() {
    let e = read("use std\nw := axis hint(x: 0.6, y: 0.8, z: 0)\nfix(x == 0.6, y == 0.8, z == 0) w\n\
                  p := plane\np.u parallel w\n");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let out = gcs_core::edit::commit_seeds(&e, &sk, &e.program);
    assert!(out.text.contains("p := plane(u: hint("), "{}", out.text);
    let back = read(&out.text);
    let (_, u) = line(&back, &back.sketch, "p.u");
    let (_, was) = line(&e, &sk, "p.u");
    assert!(norm(sub(u, was)) < 1e-9, "{u:?} against {was:?}\n{}", out.text);

    let e = read("use std\np := plane(u: hint(x: 1, y: 0.1, z: 0))\np.u parallel std.y\n");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let out = gcs_core::edit::commit_seeds(&e, &sk, &e.program);
    let back = read(&out.text);
    let (_, u) = line(&back, &back.sketch, "p.u");
    assert!(norm(cross(u, [0.0, 1.0, 0.0])) < 1e-9, "spliced in place: {}", out.text);
}

/// **`P parallel Q`**: two planes facing alike, either way — their normals parallel, two rows.
/// It says nothing of where either stands or how either turns within itself, so a plane held
/// parallel to the top and standing where it is held still turns about its normal, and its `v`
/// still leans within it.
#[test]
fn two_planes_parallel_face_alike_either_way() {
    let e = read("use std\nb := plane(u: hint(x: 1, y: 0.2, z: 0.3), v: hint(x: 0.1, y: -1, z: 0.2)) \
                  hint(x: 0, y: 0, z: 12)\nb parallel std.top\nfix(x == 0, y == 0, z == 12) b\n");
    let mut sk = e.sketch.clone();
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(kinds, [CKind::PlaneParallel]);
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let n = sk.basis(ent(&e, "b").i()).normal();
    assert!(norm(cross(n, [0.0, 0.0, 1.0])) < 1e-9, "{n:?}");
    // the seed's sense: u × v points down
    assert!(n[2] < 0.0, "{n:?}");
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 2, "a turn about its normal and v's lean: {}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    // and read either way round
    let e = read("use std\nb := plane\nstd.top parallel b\n");
    assert_eq!(e.sketch.user_constraints()[0].kind, CKind::PlaneParallel);
}
