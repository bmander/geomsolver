//! `k minimizes …` compiled to stationarity (#121): the KKT rows' Jacobian against finite
//! differences, the multipliers' seed, the group's leading rule, and the refusals.

use gcs_core::constraints::CKind;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

/// A rope of `n` written control points between (0, 0) and (100, 0), `L` long, seeded sagging.
pub fn rope(n: usize, l: f64) -> String {
    let mut s = String::from("use std\nin std.front {\na := point\nb := point\nfix((0, 0)) a\nfix((100, 0)) b\n");
    let mut names = vec!["a".to_string()];
    for i in 1..n - 1 {
        let x = 100.0 * i as f64 / (n - 1) as f64;
        let y = -40.0 * (std::f64::consts::PI * x / 100.0).sin();
        s += &format!("k{i} := point hint(({x}, {y}))\n");
        names.push(format!("k{i}"));
    }
    names.push("b".into());
    s += &format!("rope := spline({})\nlength({l}) rope\n}}\n", names.join(", "));
    s += "rope minimizes integral(p.y over p)\n";
    s
}

#[test]
fn a_rope_of_written_points_solves() {
    let (mut e, d) = read(&rope(8, 150.0));
    assert!(e.ok(), "{d:?}");
    let n_stat = e.sketch.constraints.iter().filter(|c| c.kind == CKind::Stationary).count();
    assert_eq!(n_stat, 1);
    let n_gauge = e.sketch.constraints.iter().filter(|c| c.kind == CKind::SplineGauge).count();
    assert_eq!(n_gauge, 8 - 4);
    crate::common::fd_jacobian(&e.sketch, 1e-4);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
}

/// The catenary through (0, 0) and (100, 0) of length `l`: `y = a cosh((x − 50)/a) + c`, with
/// `2a sinh(50/a) = l` solved by bisection here, never asked of the core.
pub fn catenary(l: f64) -> (f64, f64) {
    let (mut lo, mut hi): (f64, f64) = (1.0, 1e4);
    for _ in 0..200 {
        let a = 0.5 * (lo + hi);
        if 2.0 * a * (50.0 / a).sinh() > l { lo = a } else { hi = a }
    }
    let a = 0.5 * (lo + hi);
    (a, -a * (50.0 / a).cosh())
}

/// How far the solved spline `i` strays from the catenary, over 400 samples.
pub fn off_catenary(sk: &gcs_core::model::Sketch, i: usize, l: f64) -> f64 {
    let (a, c) = catenary(l);
    let (t0, t1) = gcs_core::curve::domain(sk, i);
    (0..=400)
        .map(|k| {
            let (x, y) = gcs_core::curve::point_at(sk, i, t0 + (t1 - t0) * k as f64 / 400.0);
            (y - (a * ((x - 50.0) / a).cosh() + c)).abs()
        })
        .fold(0.0, f64::max)
}

#[test]
fn a_sixteen_point_rope_hangs_as_the_catenary() {
    let (mut e, d) = read(&rope(16, 150.0));
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let off = off_catenary(&e.sketch, 0, 150.0);
    assert!(off < 1e-2, "{off}");
    // the energy closes exactly the rope's own freedoms: the ends were held, so nothing is left
    let dg = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!((dg.dof, dg.numeric_rank), (0, Some(dg.n_equations)), "{dg:?}");
}

fn solved(src: &str) -> Elaborated {
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn points(e: &Elaborated) -> Vec<(f64, f64)> {
    (0..e.sketch.points.len()).map(|i| e.sketch.point_xy(i)).collect()
}

fn same(a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
    a.iter().zip(b).map(|(p, q)| (p.0 - q.0).hypot(p.1 - q.1)).fold(0.0, f64::max)
}

/// At a solution the multipliers are the ones that make the stationarity hold, so seeding them
/// again from the solved pose moves no residual.
#[test]
fn the_multipliers_seed_where_the_solve_left_them() {
    let mut e = solved(&rope(10, 150.0));
    gcs_core::variational::seed_multipliers(&mut e.sketch);
    let mut sys = gcs_core::system::System::new(&e.sketch);
    let z = sys.z0(&e.sketch);
    assert!(sys.max_relative_residual(&z) < 1e-8, "{}", sys.max_relative_residual(&z));
}

/// Two statements over one rope are one energy: the first carries the rows, the second none, and
/// the two halves hang the rope as the whole does.  Taking the first away leaves the second to
/// carry them.
#[test]
fn two_energies_over_one_curve_add() {
    let whole = solved(&rope(10, 150.0));
    let halves = rope(10, 150.0).replace(
        "rope minimizes integral(p.y over p)",
        "rope minimizes 0.5 * integral(p.y over p)\nrope minimizes integral(p.y / 2 over p)",
    );
    let mut e = solved(&halves);
    assert!(same(&points(&whole), &points(&e)) < 1e-6);
    let ids: Vec<u32> = e.sketch.constraints.iter()
        .filter(|c| c.kind == CKind::Stationary).map(|c| c.id).collect();
    assert_eq!(ids.len(), 2);
    let rows: Vec<usize> = ids.iter().map(|&i| e.sketch.constraint(i).unwrap().rows_in(&e.sketch)).collect();
    assert_eq!(rows, vec![16, 0]);
    e.sketch.remove(ids[0]);
    assert_eq!(e.sketch.constraint(ids[1]).unwrap().rows_in(&e.sketch), 16);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
}

/// The ends are what the rope hangs between, not what it varies: free, they keep their freedom,
/// and the energy closes only the rope's own.
#[test]
fn free_ends_keep_their_freedom() {
    let src = rope(10, 150.0).replace("fix((100, 0)) b\n", "fix(y == 0) b\n");
    let mut e = solved(&src);
    let dg = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(dg.dof, 1, "{dg:?}");
}

/// A save, and a copy, carry the energy and mint its gauge rows and multipliers again.
#[test]
fn a_saved_rope_hangs_again() {
    let e = solved(&rope(10, 150.0));
    let back = gcs_core::io::loads(&gcs_core::io::dumps(&e.sketch, None)).unwrap();
    assert_eq!(back.constraints.iter().filter(|c| c.kind == CKind::SplineGauge).count(), 6);
    let mut back = back;
    let r = solve(&mut back, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let after: Vec<(f64, f64)> = (0..back.points.len()).map(|i| back.point_xy(i)).collect();
    assert!(same(&points(&e), &after) < 1e-9);
    let all: Vec<_> = (0..e.sketch.splines.len()).map(gcs_core::model::EntRef::spline).collect();
    let mut copied = gcs_core::io::copy(&e.sketch, &all);
    assert!(copied.constraints.iter().any(|c| c.kind == CKind::Stationary));
    let r = solve(&mut copied, SolveOpts::default());
    assert!(r.success, "{}", r.message);
}

/// What an energy cannot be over, or read, is refused where it is written.
#[test]
fn energies_refused() {
    let refused = |src: &str, code: &str| {
        let (_, d) = read(src);
        assert!(d.iter().any(|m| m.starts_with(code)), "{src}\n{d:?}");
    };
    let base = rope(8, 150.0);
    // a circle has no shape to vary
    refused(&base.replace("}\nrope minimizes", "o := point\nc := circle(o) hint(r: 5)\n}\nc minimizes"),
            "E040");
    // nor a curve nothing declares
    refused(&base.replace("rope minimizes", "cord minimizes"), "E101");
    // a name no scope declares
    refused(&base.replace("p.y over", "p.y + h over"), "E101");
    // and a point's own field that is not one
    refused(&base.replace("p.y over", "p.z over"), "E101");
    // nor, where the document names a unit, is an angle a power of length
    let mm = format!("unit mm\n{base}");
    refused(&mm.replace("p.y over", "atan2(p.y, p.x) * 1deg over"), "E103");
    refused(&mm.replace("p.y over", "p.y + 1 over"), "E103");
}

/// Numbers in scope are written into the integrand, under whatever name the point is given.
#[test]
fn an_integrand_reads_the_scope() {
    let whole = solved(&rope(10, 150.0));
    let e = solved(&rope(10, 150.0).replace(
        "rope minimizes integral(p.y over p)",
        "h := 7\nrope minimizes integral(q.y - h over q)",
    ));
    // a constant added to the height is the length's multiplier moved, not the shape
    assert!(same(&points(&whole), &points(&e)) < 1e-6);
}

/// An integrand reading the tangent — Dido's area, `(x·t_y − y·t_x)/2` — with a length an
/// unknown sets: the rows' Jacobian is the system's own derivative, the multipliers' columns and
/// the free length's included.
#[test]
fn a_tangent_reading_energy_differentiates_exactly() {
    let src = rope(9, 150.0)
        .replace("length(150) rope", "length(L) rope")
        .replace("rope minimizes integral(p.y over p)",
                 "param L: Length hint(150)\nrope maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t)) + integral(p.y over p)");
    let (mut e, d) = read(&src);
    assert!(e.ok(), "{d:?}");
    gcs_core::variational::seed_multipliers(&mut e.sketch);
    crate::common::fd_jacobian(&e.sketch, 1e-4);
}

/// Deleting the rope takes its energy with it, as it takes its length; deleting the energy leaves
/// the rope, its gauge rows going with it.
#[test]
fn an_energy_goes_with_its_curve() {
    let e = solved(&rope(8, 150.0));
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[gcs_core::model::EntRef::spline(0)], &[]);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(!out.text.contains("minimizes") && !out.text.contains("length("), "{}", out.text);
    let stat = e.sketch.constraints.iter().find(|c| c.kind == CKind::Stationary).unwrap().id;
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[], &[stat]);
    assert!(!out.text.contains("minimizes") && out.text.contains("length(150) rope"), "{}", out.text);
    let (e2, d) = read(&out.text);
    assert!(e2.ok(), "{d:?}");
    assert!(!e2.sketch.constraints.iter().any(|c| c.kind == CKind::SplineGauge));
}

/// The statement prints as it was written, so a document lifted from the drawing keeps it.
#[test]
fn an_energy_prints_as_written() {
    let src = "rope minimizes 2 * integral(p.y over p) - integral((p.x * t.y) over (p, t))";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let st = &prog.root().body[0];
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
    assert_eq!(out, src);
}

/// The word stands after the curve it is about, a dotted one too, and is no name.
#[test]
fn an_energy_is_said_of_its_curve() {
    let src = "inst.rope maximizes integral(p.x over p)";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &prog.root().body[0].kind).unwrap();
    assert_eq!(out, src);
    assert!(!gcs_core::syntax::is_name("minimizes") && !gcs_core::syntax::is_name("maximizes"));
    assert!(!parse("minimize integral(p.y over p)").1.is_empty());
}
