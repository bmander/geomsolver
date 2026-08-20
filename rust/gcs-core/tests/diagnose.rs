use gcs_core::constraints::{CKind, Constraint};
use gcs_core::diagnose::{diagnose, distance_rigidity, minimal_conflict_set, DiagnoseOptions, State};
use gcs_core::examples;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::newton::Method;
use gcs_core::solve::{solve, SolveOpts};

fn names(sk: &Sketch, ps: &[u32]) -> Vec<String> {
    let mut v: Vec<String> = ps.iter().map(|&p| sk.params[p as usize].name.clone()).collect();
    v.sort();
    v
}

#[test]
fn dof_counts_what_can_actually_move_not_what_the_matching_sees() {
    let mut sk = examples::altitudes();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.geometric_dependency, 1);
    assert_eq!(d.structural_dof, 2); // what the matching alone believes
    assert_eq!(d.dof, 3); // what is actually free to move
    assert_eq!(d.dof, d.n_params as i64 - d.numeric_rank.unwrap() as i64);
    assert!(d.under_params.len() as i64 >= d.dof);
}

#[test]
fn redundancy_the_matching_cannot_see_is_counted_and_named() {
    let mut sk = examples::altitudes();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.structural_n_redundant, 0);
    assert_eq!(d.n_redundant, 1);
    assert_eq!(d.status, State::Over);
    assert_eq!(d.over.len(), 6);
    let mut kinds: Vec<&str> =
        d.over.iter().map(|&c| sk.constraint(c).unwrap().type_name()).collect();
    kinds.sort();
    assert_eq!(kinds, ["Perpendicular", "Perpendicular", "Perpendicular", "PointOnLine", "PointOnLine", "PointOnLine"]);
    let over: std::collections::BTreeSet<u32> = d.over.iter().copied().collect();
    let w = diagnose(&mut sk, DiagnoseOptions { witness: true, ..Default::default() }).witness.unwrap();
    assert!(!w.dependencies.is_empty());
    let dep = &w.dependencies[0];
    assert!(over.contains(&dep.constraint));
    assert!(dep.implied_by.iter().all(|c| over.contains(c)));
}

#[test]
fn a_dependency_with_nothing_to_remove_is_not_called_over_constrained() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let centre = sk.point(10.0, 0.0, false, "c");
    let line = sk.line(a, centre);
    let s = sk.point(13.0, 4.0, false, "s");
    let e = sk.point(13.0, -4.0, false, "e");
    let arc = sk.arc(centre, s, e, "arc");
    let _ = arc;
    sk.add(Constraint::new(
        CKind::Symmetric,
        vec![
            gcs_core::constraints::Arg::Ent(EntRef::point(s)),
            gcs_core::constraints::Arg::Ent(EntRef::point(e)),
            gcs_core::constraints::Arg::Ent(EntRef::line(line)),
        ],
    ));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.geometric_dependency, 1);
    assert_eq!(d.n_redundant, 1); // the deficiency is real...
    assert!(d.over.is_empty()); // ...but nothing is removable
    assert_eq!(d.status, State::Under);
    assert!(d.dof > 0);
    assert!(d.entity_state.values().all(|&s| s != State::Over));
}

#[test]
fn a_wholly_implied_constraint_is_still_named() {
    let mut sk = examples::truss(4, 20.0, 15.0, true);
    let (ax, ay) = sk.point_xy(0);
    let (bx, by) = sk.point_xy(2);
    let extra = sk.add(Constraint::distance(
        EntRef::point(0),
        EntRef::point(2),
        (ax - bx).hypot(ay - by),
    ));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Over);
    assert_eq!(d.n_redundant, 1);
    assert!(d.over.contains(&extra));
}

#[test]
fn well_constrained_examples() {
    for name in ["rect_fillets", "slotted_link", "truss"] {
        let mut sk = examples::example(name).unwrap();
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.status, State::Well, "{name}");
        assert_eq!((d.dof, d.n_redundant), (0, 0), "{name}");
        assert!(d.warnings.is_empty(), "{name}: {:?}", d.warnings);
        assert!(d.entity_state.values().all(|&s| s == State::Well), "{name}");
    }
}

#[test]
fn conflict_set_is_the_two_distances() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let l = &sk.lines[0];
    let (p1, p2) = (l.p1 as usize, l.p2 as usize);
    let extra = sk.add(Constraint::distance(EntRef::point(p1), EntRef::point(p2), 50.0));
    let width = sk
        .constraints
        .iter()
        .find(|c| c.kind == CKind::Distance && (c.args[2].num() - 80.0).abs() < 1e-9)
        .unwrap()
        .id;
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict);
    assert_eq!(d.n_redundant, 1);
    let conf: std::collections::BTreeSet<u32> = d.conflicts.clone().unwrap().into_iter().collect();
    assert_eq!(conf, [extra, width].into_iter().collect());
    assert_eq!(d.entity_state[&EntRef::line(0)], State::Conflict);
}

#[test]
fn under_constrained_reports_free_params_and_components() {
    let mut sk = examples::slotted_link(80.0, 15.0, 6.0);
    sk.constraints.retain(|c| c.kind != CKind::Distance);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Under);
    assert_eq!(d.dof, 1);
    assert_eq!(names(&sk, &d.under_params), ["b1.x", "c2.x", "t2.x"]);
    let su = names(&sk, &d.structural_under_params);
    assert!(su.contains(&"c2.x".to_string()) && su.contains(&"c2.y".to_string()));
    let mut dofs: Vec<i64> = d.components.iter().map(|c| c.dof).collect();
    dofs.sort();
    assert_eq!(dofs, [0, 0, 1]);
    assert_eq!(d.entity_state[&EntRef::point(1)], State::Under);
    assert_eq!(d.entity_state[&EntRef::point(0)], State::Well);
}

#[test]
fn null_space_pins_the_left_side_of_an_undimensioned_rect() {
    let mut sk = examples::rect_fillets_under();
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 1);
    assert_eq!(names(&sk, &d.under_params), ["b2.x", "c_br.x", "c_tr.x", "r1.x", "r2.x", "t1.x"]);
    assert_eq!(d.entity_state[&EntRef::line(3)], State::Well);
    assert_eq!(d.entity_state[&EntRef::arc(2)], State::Well);
    assert_eq!(d.entity_state[&EntRef::arc(3)], State::Well);
    for i in 0..3 {
        assert_eq!(d.entity_state[&EntRef::line(i)], State::Under);
    }
}

#[test]
fn theorem_type_dependency_is_logged() {
    let mut sk = examples::polygon_chain(8, 50.0);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.numeric_rank, Some(d.structural_rank - 1));
    assert!(!d.warnings.is_empty());
}

#[test]
fn minimal_conflict_set_on_an_infeasible_triangle() {
    let mut sk = examples::impossible_triangle();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.n_redundant, 0); // the graph sees nothing wrong...
    assert_eq!(d.status, State::Conflict); // ...but the numbers do
    let conf = minimal_conflict_set(&mut sk, None, 1e-6, Method::DogLeg, 60);
    assert!((2..=3).contains(&conf.len()));
    assert!(conf.iter().all(|&c| sk.constraint(c).unwrap().kind == CKind::Distance));
}

#[test]
fn distance_rigidity_merges_coincident_points() {
    let sk = examples::polygon_chain(5, 50.0);
    assert_eq!(distance_rigidity(&sk).0.len(), 0);
    let mut sk = Sketch::new();
    let l1 = sk.line_xy(0.0, 0.0, 10.0, 0.0, "a");
    let l2 = sk.line_xy(10.0, 0.0, 5.0, 8.0, "b");
    let l3 = sk.line_xy(5.0, 8.0, 0.0, 0.0, "c");
    for (a, b) in [(l1, l2), (l2, l3), (l3, l1)] {
        let p = sk.lines[a].p2 as usize;
        let q = sk.lines[b].p1 as usize;
        sk.add(Constraint::coincident(EntRef::point(p), EntRef::point(q)));
    }
    for l in [l1, l2, l3] {
        let (p, q) = (sk.lines[l].p1 as usize, sk.lines[l].p2 as usize);
        let d = sk.line_length(l);
        sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), d));
    }
    let (clusters, red) = distance_rigidity(&sk);
    assert_eq!(clusters.len(), 1);
    assert_eq!(clusters[0].len(), 6);
    assert!(red.is_empty());
}

#[test]
fn under_params_are_per_axis_not_per_point() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(0.0, 10.0, true, "b");
    let p = sk.point(0.0, 4.0, false, "p");
    let l = sk.line(a, b);
    sk.add(Constraint::new(
        CKind::PointOnLine,
        vec![
            gcs_core::constraints::Arg::Ent(EntRef::point(p)),
            gcs_core::constraints::Arg::Ent(EntRef::line(l)),
        ],
    ));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    let (px, py) = (sk.points[p].x, sk.points[p].y);
    assert!(d.under_params.contains(&py) && !d.under_params.contains(&px));
}

#[test]
fn conflict_set_on_a_large_truss_from_good_geometry() {
    let mut sk = examples::truss(30, 20.0, 15.0, true);
    let bad = sk.add(Constraint::distance(EntRef::point(0), EntRef::point(3), 999.0));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict);
    assert!(d.conflicts.clone().unwrap().contains(&bad));
    let mut sk = examples::truss(30, 20.0, 15.0, true);
    let bad = sk.add(Constraint::distance(EntRef::point(0), EntRef::point(3), 21.0));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    let c = d.conflicts.clone().unwrap();
    assert!(c.contains(&bad) && (3..=13).contains(&c.len()), "{}", c.len());
}
