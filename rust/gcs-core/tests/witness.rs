use gcs_core::constraints::{CKind, Constraint};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::linalg::absmax;
use gcs_core::system::{System, RANK_TOL};
use gcs_core::witness::{analyze, make_witness};

#[test]
fn stage2_residue_is_diagnosed_with_a_culprit() {
    let mut sk = examples::polygon_chain(8, 50.0);
    let d = diagnose(&mut sk, DiagnoseOptions { witness: true, ..Default::default() });
    let rep = d.witness.as_ref().unwrap();
    assert_eq!(d.structural_rank, 24);
    assert_eq!(rep.numeric_rank, 23);
    assert_eq!(rep.dependencies.len(), 1);
    let dep = &rep.dependencies[0];
    assert_eq!(sk.constraint(dep.constraint).unwrap().kind, CKind::EqualLength);
    assert!(dep.theorem);
    assert!(!dep.implied_by.is_empty());
    assert!(dep.implied_by.iter().all(|&c| matches!(
        sk.constraint(c).unwrap().kind,
        CKind::EqualLength | CKind::Coincident
    )));
    assert_eq!((rep.n_dof(), rep.n_internal_dof()), (7, 7)); // one point fixed: no rigid modes
}

#[test]
fn concurrent_altitudes_theorem() {
    let mut sk = examples::altitudes();
    let d = diagnose(&mut sk, DiagnoseOptions { witness: true, ..Default::default() });
    let rep = d.witness.as_ref().unwrap();
    assert_eq!(d.structural_rank, 6);
    assert_eq!(rep.numeric_rank, 5); // the graph is blind to the concurrency
    assert_eq!(rep.dependencies.len(), 1);
    assert!(rep.dependencies[0].theorem);
    assert_eq!(sk.constraint(rep.dependencies[0].constraint).unwrap().kind, CKind::PointOnLine);
    assert!(rep.dependencies[0]
        .implied_by
        .iter()
        .any(|&c| sk.constraint(c).unwrap().kind == CKind::Perpendicular));
    assert_eq!(rep.n_internal_dof(), 3); // the three feet slide along their altitudes
    assert!(!rep.used_current); // P did not satisfy the incidences: a witness was built
}

#[test]
fn witness_of_well_constrained_is_current_and_full_rank() {
    for name in ["rect_fillets", "slotted_link", "truss"] {
        let mut sk = examples::example(name).unwrap();
        let rep = analyze(&mut sk, None, 0);
        assert!(rep.used_current, "{name}");
        assert_eq!(rep.n_dof(), 0, "{name}");
        assert!(rep.dependencies.is_empty(), "{name}");
    }
}

#[test]
fn rigid_body_modes_are_separated() {
    let mut sk = examples::truss_floating(4);
    let rep = analyze(&mut sk, None, 0);
    assert_eq!((rep.n_dof(), rep.n_internal_dof()), (3, 0));
    assert!(rep.motions.iter().all(|m| m.rigid));
}

#[test]
fn motions_are_localised_and_unit_scaled() {
    let mut sk = examples::rect_fillets_under();
    let rep = analyze(&mut sk, None, 0);
    assert_eq!((rep.n_dof(), rep.n_internal_dof()), (1, 1));
    let m = &rep.motions[0];
    assert_eq!(absmax(&m.velocity), 1.0);
    let mut names: Vec<String> = rep
        .moving_params(m, 1e-3)
        .iter()
        .map(|&p| sk.params[p as usize].name.clone())
        .collect();
    names.sort();
    assert_eq!(names, ["b2.x", "c_br.x", "c_tr.x", "r1.x", "r2.x", "t1.x"]);
}

#[test]
fn make_witness_restores_the_sketch_and_generalises_dimensions() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let x0 = sk.get_x();
    let dims: Vec<f64> = sk
        .constraints
        .iter()
        .filter(|c| c.kind == CKind::Distance)
        .map(|c| c.args[2].num())
        .collect();
    let xw = make_witness(&mut sk, 1, 0.05, 1e-8);
    assert_eq!(sk.get_x(), x0);
    let after: Vec<f64> = sk
        .constraints
        .iter()
        .filter(|c| c.kind == CKind::Distance)
        .map(|c| c.args[2].num())
        .collect();
    assert_eq!(after, dims);
    assert!(xw.iter().zip(&x0).any(|(a, b)| (a - b).abs() > 1e-8));
    let rep = analyze(&mut sk, Some(xw), 0);
    assert_eq!(rep.numeric_rank, 26);
    assert_eq!(rep.n_dof(), 0);
}

#[test]
fn reported_dependencies_are_genuinely_redundant() {
    for mut sk in [
        examples::polygon_chain(8, 50.0),
        examples::altitudes(),
        examples::rect_fillets(100.0, 60.0, 10.0, 0.0),
    ] {
        let rep = analyze(&mut sk, None, 0);
        let xw = rep.x_witness.clone();
        sk.set_x(&xw);
        let mut s = System::new(&sk);
        let z = s.z0(&sk);
        // the matrix the core judged, at the tolerance it judged it — not a raw Jacobian
        let j = s.conditioned(&z);
        let (_, rows_c) = s.structure();
        let full = j.rank_rrqr(RANK_TOL);
        for dep in &rep.dependencies {
            let keep: Vec<usize> =
                (0..rows_c.len()).filter(|&i| rows_c[i] != dep.constraint).collect();
            assert_eq!(j.select_rows(&keep).rank_rrqr(RANK_TOL), full);
            assert!(!dep.implied_by.contains(&dep.constraint));
        }
    }
}

#[test]
fn dimension_jitter_follows_the_constraint_declarations() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let dimensioned: Vec<&str> = sk
        .hard_constraints()
        .iter()
        .filter(|c| !c.dimensions().is_empty())
        .map(|c| c.type_name())
        .collect();
    let uniq: std::collections::BTreeSet<&str> = dimensioned.into_iter().collect();
    assert_eq!(uniq, ["Distance", "Radius"].into_iter().collect());
}

/// `analyze` standing alone used to pass an empty over-block, so every dependency it found came
/// back `theorem: true` — "invisible to the graph" — including ones the matching sees perfectly
/// well.  The app's witness panel reads this report, so it labelled everything theorem-type.
#[test]
fn a_dependency_the_graph_can_see_is_not_called_theorem_type() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    sk.add(Constraint::coincident(EntRef::point(a), EntRef::point(b)));
    sk.add(Constraint::coincident(EntRef::point(b), EntRef::point(a))); // a plain duplicate

    let rep = analyze(&mut sk, None, 0);
    assert_eq!(rep.dependencies.len(), 1, "{:?}", rep.dependencies);
    assert!(!rep.dependencies[0].theorem, "a duplicate is not a theorem");

    // and a genuinely theorem-type one still is
    let mut alt = examples::altitudes();
    let rep = analyze(&mut alt, None, 0);
    assert_eq!(rep.dependencies.len(), 1);
    assert!(rep.dependencies[0].theorem);
}
