//! **A directed angle about an axis** (#558, Solvent 0.174): `angle(θ, about: a)` is the turn
//! about `a`, by the right-hand rule and on the full turn, from the first direction's shadow on
//! the plane square to `a` to the second's; `perpendicular(sense: ccw|cw, about: a)` is that turn
//! at a quarter.  One row, one root: where the unsigned `angle` and the bare `perpendicular` in
//! space hold either way round, these say which.  In a plane the axis is its normal, unsaid.
use gcs_core::constraints::{CKind, Constraint};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{dot, norm, sub};

use crate::common::{dir, dof, ent, kinds, read, refused, set_dir as seed, solved, unit};

/// The sketch's one row of kind `k`.
fn row(sk: &Sketch, k: CKind) -> Constraint {
    sk.user_constraints().into_iter().find(|c| c.kind == k).cloned().expect("the row")
}

/// Two axes held through the origin, `z` along (0, 0, 1) and `n` along (0, 1, 0), and a free
/// axis `x` with the statement over it.
fn held(stated: &str) -> String {
    format!("unit mm\nuse std\nz := axis\nfix(dir == (0, 0, 1), origin == (0, 0, 0)) z\n\
             n := axis\nfix(dir == (0, 1, 0), origin == (0, 0, 0)) n\n\
             x := axis hint(dir: (1, 0.2, 0.1))\nfix(origin == (0, 0, 0)) x\n\
             x perpendicular z\n{stated}\n")
}

/// The bearing of `d` about `z` from `x`: the right-handed turn, on (−π, π].
fn bearing(d: [f64; 3]) -> f64 {
    d[1].atan2(d[0])
}

/// In a plane, `about:` its normal is the plane's own `angle`: the same rows, the same solve —
/// and for the sided `perpendicular`, the same.
#[test]
fn in_a_plane_about_its_normal_is_the_planes_angle() {
    let doc = |said: &str| format!("unit mm\nuse std\nin std.front {{\na := point\n\
        b := point hint((10, 0))\nc := point hint((1, 1))\nd := point hint((3, -8))\n\
        fix((0, 0)) a\nfix((10, 0)) b\nfix((1, 1)) c\nab := line(a, b)\ncd := line(c, d)\n\
        c distance(10) d\n}}\nab {said} cd\n");
    for (plain, about) in [
        ("angle(40deg)", "angle(40deg, about: std.front.normal)"),
        ("angle(40deg, sense: cw)", "angle(40deg, sense: cw, about: std.front.normal)"),
        ("perpendicular(sense: ccw)", "perpendicular(sense: ccw, about: std.front.normal)"),
        ("perpendicular(sense: cw)", "perpendicular(sense: cw, about: std.front.normal)"),
    ] {
        let (p, q) = (read(&doc(plain)), read(&doc(about)));
        assert_eq!(kinds(&p), kinds(&q), "{about}");
        let d = |e: &Elaborated| solved(e).world_point(ent(e, "d").i());
        let (p0, q0) = (d(&p), d(&q));
        assert!(norm(sub(p0, q0)) < 1e-9, "{about}: {q0:?} against {p0:?}");
    }
}

/// One root: started on either side of the stated turn, and from the pose square to it, the
/// solve lands on the turn said — where the unsigned angle's other root is no root of it.
#[test]
fn a_turn_about_an_axis_has_one_root() {
    for (said, want) in [("angle(30deg, about: z)", 30.0), ("angle(30deg, sense: cw, about: z)", -30.0),
                         ("angle(150deg, about: z)", 150.0),
                         ("perpendicular(sense: ccw, about: z)", 90.0),
                         ("perpendicular(sense: cw, about: z)", -90.0)] {
        let e = read(&held(&format!("n {said} x")));
        assert!(kinds(&e).iter().any(|k| matches!(k, CKind::AngleAbout | CKind::PerpendicularAbout)));
        // the turn from `n` to `x` about z is the bearing of x less n's (90°)
        let x = ent(&e, "x").i();
        for start in [-170.0f64, -100.0, -20.0, 10.0, 80.0, 120.0, 175.0] {
            let mut sk = e.sketch.clone();
            let s = start.to_radians();
            seed(&mut sk, x, [s.cos(), s.sin(), 0.0]);
            assert!(solve(&mut sk, SolveOpts::default()).success, "{said} from {start}");
            let got = (bearing(dir(&sk, x)) - 90f64.to_radians()).to_degrees();
            let gap = (got - want).rem_euclid(360.0);
            assert!(gap.min(360.0 - gap) < 1e-6, "{said} from {start}: {got}");
        }
    }
    // the unsigned angle's other root, the turn the other way, is no root of the directed one
    let e = read(&held("n angle(30deg, about: z) x"));
    let mut sk = e.sketch.clone();
    let x = ent(&e, "x").i();
    let other = 60f64.to_radians();
    seed(&mut sk, x, [other.cos(), other.sin(), 0.0]);
    let c = row(&sk, CKind::AngleAbout);
    let r = c.residual(&sk, &c.local_values(&sk))[0];
    assert!((r.abs() - 60f64.to_radians()).abs() < 1e-9, "{r}");
}

/// Neither direction need be square to the axis: only their shadows are read.
#[test]
fn only_the_shadows_on_the_plane_square_to_the_axis_are_read() {
    let e = read("unit mm\nuse std\nz := axis\nfix(dir == (0, 0, 1), origin == (0, 0, 0)) z\n\
                  b := axis\nfix(dir == (1, 0, 1), origin == (0, 0, 0)) b\n\
                  a := axis hint(dir: (0.2, 0.5, 0.8))\nfix(origin == (0, 0, 0)) a\n\
                  a angle(60deg) z\nb angle(120deg, about: z) a\n");
    let sk = solved(&e);
    let a = unit(dir(&sk, ent(&e, "a").i()));
    assert!((bearing(a).to_degrees() - 120.0).abs() < 1e-7, "{a:?}");
    assert!((a[2] - 0.5).abs() < 1e-9, "{a:?}");
}

/// A frame mated by its origin, its z along a line the way it runs and its x a quarter turn about
/// z from a direction: three statements, six freedoms, no redundancy.
#[test]
fn a_frame_mated_about_its_z_is_held_wholly() {
    let src = "unit mm\nuse std\ncomponent Post() {\n  in std.front {\n    a := point\n    \
               a coincident std.origin\n    b := point hint((3, 40))\n    \
               b distance(3mm, along: u) std.front\n    a distance(40mm) b\n  }\n}\n\
               p := Post()\nseat := point\nfix((10, 20, 30)) seat\nrod := axis\n\
               fix(dir == (0, 1, 0), origin == (10, 0, 30)) rod\nwall := axis\n\
               fix(dir == (1, 0, 0), origin == (0, 0, 0)) wall\n\
               p.std.origin coincident seat\np.std.z parallel(sense: same) rod\n\
               p.std.x perpendicular(sense: ccw, about: p.std.z) wall\n";
    let e = read(src);
    let u = e.sketch.frames[ent(&e, "p#frame").i()].u as usize;
    for x0 in [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [1.0, 0.1, 0.2]] {
        let mut sk = e.sketch.clone();
        seed(&mut sk, u, x0);
        assert!(solve(&mut sk, SolveOpts::default()).success, "from {x0:?}");
        // z along the rod, and the wall a quarter turn about it from x: x = wall × z
        let want = [0.0, 0.0, 1.0];
        let got = unit(dir(&sk, u));
        assert!(dot(got, want) > 1.0 - 1e-9, "from {x0:?}: x {got:?}");
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 0);
        assert!(d.over.is_empty() && d.implied.is_empty(), "{:?} {:?}", d.over, d.implied);
    }
}

/// A direction along the axis casts no shadow: the row reads a residual there, never a NaN, and
/// a solve started there leaves it.
#[test]
fn a_direction_along_the_axis_reads_no_nan() {
    let e = read(&held("n perpendicular(sense: ccw, about: z) x"));
    let mut sk = e.sketch.clone();
    let x = ent(&e, "x").i();
    seed(&mut sk, x, [0.0, 0.0, 1.0]);
    let c = row(&sk, CKind::PerpendicularAbout);
    let v = c.local_values(&sk);
    let r = c.residual(&sk, &v);
    assert!(r.iter().all(|x| x.is_finite()) && r[0].abs() > 1.0, "{r:?}");
    let j = c.jacobian(&sk, &v);
    assert!(j.iter().all(|x| x.is_finite()), "{j:?}");
}

/// The refusals: a sense in space with no axis says where one goes; in a plane an axis other
/// than its normal; a perpendicular about an axis that says no sense; `about:` on a word that
/// turns nothing.
#[test]
fn a_turn_about_an_axis_is_refused_where_it_says_nothing() {
    refused(&held("n perpendicular(sense: ccw) x"), "E040", "about: a", "sense");
    refused(&held("n angle(30deg, sense: cw) x"), "E040", "about: a", "sense");
    refused(&held("n perpendicular(about: z) x"), "E040", "says no `sense:`", "about");
    refused(&held("n parallel(about: z) x"), "E040", "takes no `about`", "about");
    let plane = |about: &str| format!("unit mm\nuse std\nz := axis\n\
        fix(dir == (0, 0, 1), origin == (0, 0, 0)) z\nin std.front {{\na := point\n\
        b := point hint((10, 0))\nc := point hint((1, 1))\nd := point hint((3, -8))\n\
        ab := line(a, b)\ncd := line(c, d)\n}}\nab angle(40deg, about: {about}) cd\n");
    refused(&plane("z"), "E040", "a plane's sense is its normal's", "about");
    refused(&plane("std.top.normal"), "E040", "a plane's sense is its normal's", "about");
}

/// A turn about an axis prints back as written, and reads back to the same pose.
#[test]
fn a_turn_about_an_axis_prints_back_as_written() {
    use gcs_core::program::to_program;
    let src = held("n angle(30deg, sense: cw, about: z) x");
    let e = read(&src);
    let mut sk = solved(&e);
    let mut lifted = to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
    assert!(text.contains("angle(30deg, sense: cw, about: "), "{text}");
    let back = solved(&read(&text));
    let x = |sk: &Sketch| unit(dir(sk, ent(&e, "x").i()));
    assert!(norm(sub(x(&sk), x(&back))) < 1e-9, "{text}");
    assert_eq!(dof(&mut sk), 0);
}

/// A copy keeps the axis a turn is read about, as the file does: the row and its slots alike,
/// and the same pose from a start turned round.
#[test]
fn a_copy_keeps_the_axis_a_turn_is_read_about() {
    use gcs_core::{graft, io};
    let e = read(&held("n perpendicular(sense: cw, about: z) x"));
    let sk = solved(&e);
    let x = ent(&e, "x").i();
    for mut back in [io::from_json(&io::to_json(&sk)).expect("reads back"),
                     graft::copy(&sk, &sk.primitives())] {
        let args = |sk: &Sketch| row(sk, CKind::PerpendicularAbout).args;
        assert_eq!(args(&back), args(&sk));
        seed(&mut back, x, [0.0, 1.0, 0.0]);
        assert!(solve(&mut back, SolveOpts::default()).success);
        assert!(norm(sub(unit(dir(&back, x)), unit(dir(&sk, x)))) < 1e-9);
    }
}
