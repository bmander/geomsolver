use gcs_core::constraints::{Arg, CKind, Constraint, SpecKind};
use gcs_core::diagnose::{diagnose, distance_rigidity, minimal_conflict_set, summary, DiagnoseOptions, State};
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
fn redundancy_the_matching_cannot_see_is_counted_and_named_as_implied() {
    // the altitudes concur: a theorem among pure relations.  Counted (DOF is the truth), named
    // (any of the six could go), but not `over` — nothing can ever break it, so there is nothing
    // to fix and no reason to paint the sketch red
    let mut sk = examples::altitudes();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.structural_n_redundant, 0);
    assert_eq!(d.n_redundant, 1);
    assert_eq!(d.status, State::Under);
    assert!(d.over.is_empty());
    assert_eq!(d.implied.len(), 6);
    let mut kinds: Vec<&str> =
        d.implied.iter().map(|&c| sk.constraint(c).unwrap().type_name()).collect();
    kinds.sort();
    assert_eq!(kinds, ["Perpendicular", "Perpendicular", "Perpendicular", "PointOnLine", "PointOnLine", "PointOnLine"]);
    assert!(d.entity_state.values().all(|&s| s != State::Over));
    let implied: std::collections::BTreeSet<u32> = d.implied.iter().copied().collect();
    let w = diagnose(&mut sk, DiagnoseOptions { witness: true, ..Default::default() }).witness.unwrap();
    assert!(!w.dependencies.is_empty());
    let dep = &w.dependencies[0];
    assert!(implied.contains(&dep.constraint));
    assert!(dep.implied_by.iter().all(|c| implied.contains(c)));
}

#[test]
fn a_relation_only_theorem_is_implied_not_over() {
    // two arcs on one centre, the centre on a line, equal radii, and an endpoint of each mirrored
    // about the line.  Mirroring about a line through the centre preserves distance to it, so
    // EqualRadius follows — and so does the centre being on the line (it is on the chord's
    // perpendicular bisector).  Each is wholly implied, neither involves a dimension: the user
    // can drag the sketch anywhere and it stays consistent, so this is a remark, not a fault.
    let mut sk = Sketch::new();
    let a = sk.point(-20.0, 0.0, false, "a");
    let b = sk.point(40.0, 0.0, false, "b");
    let line = sk.line(a, b);
    let centre = sk.point(10.0, 0.0, true, "c");
    let (s1, e1) = (sk.point(18.0, 6.0, false, "s1"), sk.point(4.0, 8.0, false, "e1"));
    let (s2, e2) = (sk.point(4.0, -8.0, false, "s2"), sk.point(18.0, -6.0, false, "e2"));
    let arc1 = sk.arc(centre, s1, e1, "arc1");
    let arc2 = sk.arc(centre, s2, e2, "arc2");
    let on_line = sk.add(Constraint::new(
        CKind::PointOnLine,
        vec![Arg::Ent(EntRef::point(centre)), Arg::Ent(EntRef::line(line))],
    ));
    let equal = sk.add(Constraint::new(
        CKind::EqualRadius,
        vec![Arg::Ent(EntRef::arc(arc1)), Arg::Ent(EntRef::arc(arc2))],
    ));
    sk.add(Constraint::new(
        CKind::Symmetric,
        vec![
            Arg::Ent(EntRef::point(s2)),
            Arg::Ent(EntRef::point(e1)),
            Arg::Ent(EntRef::line(line)),
        ],
    ));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.geometric_dependency, 1);
    assert_eq!(d.n_redundant, 1);
    assert_eq!(d.status, State::Under);
    assert!(d.over.is_empty());
    let implied: std::collections::BTreeSet<u32> = d.implied.iter().copied().collect();
    assert_eq!(implied, [on_line, equal].into_iter().collect());
    assert!(d.violated.is_empty());
    assert!(d.entity_state.values().all(|&s| s != State::Over));
}

#[test]
fn a_dependency_that_involves_a_dimension_is_still_over() {
    // the same kind of theorem — two equal distances make EqualLength follow — but the rows
    // that take part carry dimensions, and editing either of them is a conflict.  That is worth
    // flagging now, and the relation the dimensions imply is named along with them.
    let mut sk = Sketch::new();
    let p = sk.point(0.0, 0.0, true, "p");
    let q = sk.point(5.0, 0.0, false, "q");
    let r = sk.point(5.0, 5.0, false, "r");
    let (l1, l2) = (sk.line(p, q), sk.line(q, r));
    sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), 5.0));
    sk.add(Constraint::distance(EntRef::point(q), EntRef::point(r), 5.0));
    let equal =
        sk.add(Constraint::two_line(CKind::EqualLength, EntRef::line(l1), EntRef::line(l2)));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.geometric_dependency, 1);
    assert_eq!(d.status, State::Over);
    assert_eq!(d.over.len(), 3);
    assert!(d.over.contains(&equal));
    assert!(d.implied.is_empty());
}

#[test]
fn implied_and_over_are_told_apart_per_constraint_not_per_sketch() {
    // both of the above in one sketch: the left null space then mixes a theorem with a fragile
    // dependency, and each constraint still has to land on its own side
    let mut sk = Sketch::new();
    let a = sk.point(-20.0, 0.0, false, "a");
    let b = sk.point(40.0, 0.0, false, "b");
    let line = sk.line(a, b);
    let centre = sk.point(10.0, 0.0, true, "c");
    let (s1, e1) = (sk.point(18.0, 6.0, false, "s1"), sk.point(4.0, 8.0, false, "e1"));
    let (s2, e2) = (sk.point(4.0, -8.0, false, "s2"), sk.point(18.0, -6.0, false, "e2"));
    let arc1 = sk.arc(centre, s1, e1, "arc1");
    let arc2 = sk.arc(centre, s2, e2, "arc2");
    let on_line = sk.add(Constraint::new(
        CKind::PointOnLine,
        vec![Arg::Ent(EntRef::point(centre)), Arg::Ent(EntRef::line(line))],
    ));
    let equal_r = sk.add(Constraint::new(
        CKind::EqualRadius,
        vec![Arg::Ent(EntRef::arc(arc1)), Arg::Ent(EntRef::arc(arc2))],
    ));
    sk.add(Constraint::new(
        CKind::Symmetric,
        vec![
            Arg::Ent(EntRef::point(s2)),
            Arg::Ent(EntRef::point(e1)),
            Arg::Ent(EntRef::line(line)),
        ],
    ));
    let p = sk.point(100.0, 0.0, true, "p");
    let q = sk.point(105.0, 0.0, false, "q");
    let r = sk.point(105.0, 5.0, false, "r");
    let (l1, l2) = (sk.line(p, q), sk.line(q, r));
    let d1 = sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), 5.0));
    let d2 = sk.add(Constraint::distance(EntRef::point(q), EntRef::point(r), 5.0));
    let equal_l =
        sk.add(Constraint::two_line(CKind::EqualLength, EntRef::line(l1), EntRef::line(l2)));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.geometric_dependency, 2);
    assert_eq!(d.status, State::Over);
    let set = |v: &[u32]| v.iter().copied().collect::<std::collections::BTreeSet<u32>>();
    assert_eq!(set(&d.implied), set(&[on_line, equal_r]));
    assert_eq!(set(&d.over), set(&[d1, d2, equal_l]));
}

#[test]
fn a_theorem_that_tips_the_matching_still_reads_implied_not_over() {
    // A rectangle drawn with three perpendiculars *and* two horizontals and two verticals —
    // three of the seven follow from the others.  With one side dimensioned the matching still
    // places every equation, so the theorem is caught numerically; dimensioning the second side
    // tips the count (nine equations on eight coordinates), and the structural over-block blames
    // the whole rectangle, both side lengths included.  The dependency the numbers see lives on
    // the seven angular relations alone: a theorem, so nothing is over, neither dimension is
    // named, and the sketch stays under — it can still translate.
    let mut sk = Sketch::new();
    let a = sk.point(167.7, 4.83, false, "a");
    let b = sk.point(175.3, 4.83, false, "b");
    let c = sk.point(175.3, -35.2, false, "c");
    let d = sk.point(167.7, -35.2, false, "d");
    let (top, right, bottom, left) = (sk.line(a, b), sk.line(b, c), sk.line(c, d), sk.line(d, a));
    let mut angular = Vec::new();
    for (l1, l2) in [(top, right), (right, bottom), (bottom, left)] {
        let pp = Constraint::two_line(CKind::Perpendicular, EntRef::line(l1), EntRef::line(l2));
        angular.push(sk.add(pp));
    }
    angular.push(sk.add(Constraint::one_line(CKind::Vertical, EntRef::line(left))));
    angular.push(sk.add(Constraint::one_line(CKind::Vertical, EntRef::line(right))));
    angular.push(sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(bottom))));
    angular.push(sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(top))));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 40.0));
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 25.0));
    let res = solve(&mut sk, SolveOpts::default());
    assert!(res.success, "{}", res.message);
    let dg = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(dg.status, State::Under);
    assert!(dg.over.is_empty());
    assert_eq!(dg.geometric_dependency, 2);
    assert_eq!(dg.n_redundant, 3);
    let implied: std::collections::BTreeSet<u32> = dg.implied.iter().copied().collect();
    assert_eq!(implied, angular.iter().copied().collect());
    assert!(dg.entity_state.values().all(|&s| s != State::Over));
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
            Arg::Ent(EntRef::point(s)),
            Arg::Ent(EntRef::point(e)),
            Arg::Ent(EntRef::line(line)),
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
    // the width the case states, and a second number on the *same pair*: what makes this a
    // minimal conflict of two is that both name one length, not that both are lengths
    let stated = sk
        .constraints
        .iter()
        .find(|c| c.kind == CKind::Distance && (c.args[2].num() - 100.0).abs() < 1e-9)
        .expect("the width dimension")
        .clone();
    let width = stated.id;
    let (p1, p2) = (stated.args[0].ent(), stated.args[1].ent());
    let extra = sk.add(Constraint::distance(p1, p2, 50.0));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict);
    assert_eq!(d.n_redundant, 1);
    let conf: std::collections::BTreeSet<u32> = d.conflicts.clone().unwrap().into_iter().collect();
    assert_eq!(conf, [extra, width].into_iter().collect());
    assert_eq!(d.entity_state[&p1], State::Conflict, "the points the two numbers measure between");
    assert_eq!(d.entity_state[&p2], State::Conflict);
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
            Arg::Ent(EntRef::point(p)),
            Arg::Ent(EntRef::line(l)),
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

/// Kernels are not all written to the same power of length: a `Radius` residual is a length, a
/// `Distance` residual a length squared.  Judging both against one `1e-6 * extent²` threshold
/// makes every linear constraint's tolerance grow with the sketch, so on a large sketch two
/// contradictory radii are "solved" and diagnosis sees no conflict.
#[test]
fn a_large_sketch_does_not_loosen_the_linear_constraints() {
    let mut sk = Sketch::new();
    sk.point(0.0, 0.0, true, "a");
    sk.point(1000.0, 1000.0, true, "b"); // extent 1000: scale² would be 1e6
    let c = sk.point(500.0, 500.0, false, "c");
    let ci = sk.circle(c, 50.0, "circle");
    sk.add(Constraint::radius(EntRef::circle(ci), 50.0));
    sk.add(Constraint::radius(EntRef::circle(ci), 50.9));

    let r = solve(&mut sk, SolveOpts::default());
    assert!(!r.success, "contradictory radii reported solved: {r:?}");
    // the residual is reported in the row's own units — a length over the extent — so the
    // 0.45 the two radii cannot agree on reads as 0.45 / 1000
    assert!(r.max_residual > 0.4 / 1000.0, "{r:?}");

    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict, "{d:?}");
    assert_eq!(d.violated.len(), 2, "{:?}", d.violated); // the solve split the difference
    assert!(d.conflicts.is_some(), "no minimal conflict set: {d:?}");
}

/// A consistent duplicate makes a structural over-block that has nothing to do with an
/// infeasibility elsewhere.  Confining the conflict search to the over-block leaves the real
/// conflict among the constraints held fixed, so no candidate can ever be satisfied and the first
/// one tried is reported — here, a harmless second `Horizontal`.
#[test]
fn the_conflict_search_is_not_confined_to_the_over_block() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(10.0, 0.0, false, "b");
    let ln = sk.line(a, b);
    let h1 = sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(ln)));
    let h2 = sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(ln)));

    // an unrelated, genuinely impossible triangle: 1 + 1 < 10
    let p = sk.point(100.0, 0.0, false, "p");
    let q = sk.point(101.0, 0.0, false, "q");
    let r = sk.point(100.5, 1.0, false, "r");
    let d1 = sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), 1.0));
    let d2 = sk.add(Constraint::distance(EntRef::point(q), EntRef::point(r), 1.0));
    let d3 = sk.add(Constraint::distance(EntRef::point(p), EntRef::point(r), 10.0));

    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Conflict, "{d:?}");
    let conflicts = d.conflicts.clone().expect("a conflict set");
    assert!(!conflicts.is_empty(), "{d:?}");
    for c in &conflicts {
        assert!(*c != h1 && *c != h2, "blamed a harmless Horizontal: {conflicts:?}");
        assert!([d1, d2, d3].contains(c), "unexpected culprit {c}: {conflicts:?}");
    }
    // the over-block is still reported for what it is
    assert!(d.over.contains(&h1) || d.over.contains(&h2), "{:?}", d.over);
}

/// Diagnosis runs after every edit, so its bookkeeping has to stay linear in the constraint
/// count.  This is the same answer as the small cases, at a size where a per-constraint scan of
/// the constraint list would show.
#[test]
fn diagnosis_scales_to_a_large_sketch() {
    let mut sk = examples::truss(40, 20.0, 15.0, true);
    assert!(sk.constraints.len() > 150, "{}", sk.constraints.len());
    let d = diagnose(&mut sk, DiagnoseOptions { numeric: Some(false), ..Default::default() });
    assert_eq!(d.status, State::Well, "{}", summary(&d));
    assert_eq!(d.dof, 0);
    assert!(d.violated.is_empty());
    assert_eq!(d.components.len(), 1);
    assert_eq!(d.components[0].constraints.len(), sk.hard_constraints().len());
}

/// A line through a point on a circle touches it there, at a maximum of the distance from the
/// centre — so `PointOnCircle` and `TangentLineCircle` are first-order dependent at every
/// solution (a double root), and the singular value that says so measures how far the solve
/// stopped from the exact pose, nothing about the rest of the drawing.
fn tangent_at_a_point_on_the_circle(far_dimension: bool) -> Sketch {
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, true, "C");
    let circle = sk.circle(c, 17.0, "circle");
    let p = sk.point(17.0, 0.0, false, "P");
    let q = sk.point(17.0, 30.0, false, "Q");
    let line = sk.line(p, q);
    sk.add(Constraint::new(
        CKind::Radius,
        vec![Arg::Ent(EntRef::circle(circle)), Arg::Num(17.0)],
    ));
    sk.add(Constraint::point_on_circle(EntRef::point(p), EntRef::circle(circle), false));
    sk.add(Constraint::new(
        CKind::TangentLineCircle,
        vec![Arg::Ent(EntRef::line(line)), Arg::Ent(EntRef::circle(circle)), Arg::Int(1)],
    ));
    if far_dimension {
        // a separate figure: one point held hint at a distance from the centre, touching nothing
        let far = sk.point(60.0, 0.0, false, "far");
        sk.add(Constraint::distance(EntRef::point(far), EntRef::point(c), 60.0));
    }
    sk
}

#[test]
fn an_unrelated_dimension_cannot_change_the_verdict() {
    // a rank relative to the largest singular value read the circle as over-constrained when
    // the far dimension's row (a squared distance: gradient 2·60) was in the matrix and as fine
    // without it.  The conditioned Jacobian is dimensionless, the tolerance absolute, and the
    // verdict on the circle is the circle's alone.
    let mut reads = Vec::new();
    for far in [false, true] {
        let mut sk = tangent_at_a_point_on_the_circle(far);
        solve(&mut sk, SolveOpts::default());
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert!(d.over.is_empty(), "far={far}: {:?}", d.over);
        assert_ne!(d.status, State::Over, "far={far}");
        reads.push((d.geometric_dependency, d.implied.len(), d.n_redundant));
    }
    assert_eq!(reads[0], reads[1]);
}

/// Every coordinate, radius and length dimension times `k`: the same drawing, `k` times the
/// size, and still solved.
fn scaled(sk: &Sketch, k: f64) -> Sketch {
    let mut s = sk.clone();
    for p in s.params.iter_mut() {
        p.value *= k;
    }
    let lengths: Vec<(u32, &'static str)> = s
        .constraints
        .iter()
        .flat_map(|c| {
            c.dimensions()
                .into_iter()
                .filter(|&(_, _, kind)| kind == SpecKind::Length)
                .map(move |(_, name, _)| (c.id, name))
        })
        .collect();
    for (id, name) in lengths {
        let v = s.constraint(id).unwrap().get_num(name).unwrap();
        assert!(s.set_constraint_num(id, name, v * k));
    }
    s
}

#[test]
fn the_verdict_does_not_depend_on_the_drawing_s_size() {
    // the conditioned Jacobian is invariant under a uniform rescale of the drawing, so what the
    // numeric cross-check finds — and files as `over` or `implied` — is the figure's, at any
    // size it is drawn
    let mut over = Sketch::new();
    {
        let p = over.point(0.0, 0.0, true, "p");
        let q = over.point(5.0, 0.0, false, "q");
        let r = over.point(5.0, 5.0, false, "r");
        let (l1, l2) = (over.line(p, q), over.line(q, r));
        over.add(Constraint::distance(EntRef::point(p), EntRef::point(q), 5.0));
        over.add(Constraint::distance(EntRef::point(q), EntRef::point(r), 5.0));
        over.add(Constraint::two_line(CKind::EqualLength, EntRef::line(l1), EntRef::line(l2)));
    }
    let cases = [
        ("altitudes", examples::altitudes()),
        ("rect_fillets", examples::rect_fillets(100.0, 60.0, 10.0, 0.0)),
        ("polygon_chain", examples::polygon_chain(8, 50.0)),
        ("equal_length_over", over),
        ("tangent", tangent_at_a_point_on_the_circle(true)),
    ];
    let verdict = |sk: &mut Sketch| {
        let d = diagnose(sk, DiagnoseOptions::default());
        (d.numeric_rank, d.geometric_dependency, d.over, d.implied, d.status)
    };
    for (name, mut sk) in cases {
        solve(&mut sk, SolveOpts::default());
        let base = verdict(&mut sk);
        for k in [1e3, 0.1] {
            let mut s = scaled(&sk, k);
            assert_eq!(verdict(&mut s), base, "{name} x{k}");
        }
    }
}

#[test]
fn tangency_stated_at_its_contact_is_regular() {
    // the same figure as `tangent_at_a_point_on_the_circle`, stated the new way: the endpoint on
    // the circle, and the tangency *at* that endpoint (the radius perpendicular there).  No
    // double root: full numeric rank, honest DOF, nothing to warn about.
    let mut sk = Sketch::new();
    let c = sk.point(0.0, 0.0, true, "C");
    let circle = sk.circle(c, 17.0, "circle");
    let p = sk.point(17.0, 0.0, false, "P");
    let q = sk.point(17.0, 30.0, false, "Q");
    let line = sk.line(p, q);
    sk.add(Constraint::new(
        CKind::Radius,
        vec![Arg::Ent(EntRef::circle(circle)), Arg::Num(17.0)],
    ));
    sk.add(Constraint::point_on_circle(EntRef::point(p), EntRef::circle(circle), false));
    sk.add(Constraint::new(
        CKind::TangentLineCircleAt,
        vec![Arg::Ent(EntRef::line(line)), Arg::Ent(EntRef::circle(circle)), Arg::Str("p1".into())],
    ));
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.numeric_rank, Some(d.structural_rank));
    assert_eq!(d.geometric_dependency, 0);
    assert_eq!(d.shaky, 0);
    assert!(d.warnings.is_empty(), "{:?}", d.warnings);
    // the contact slides around the circle, and the far end slides along the line
    assert_eq!(d.dof, 2);
}

#[test]
fn a_tangential_contact_is_rigid_not_under() {
    // each first-order "motion" is an endpoint swimming along the line — blocked at second
    // order, so the settle test counts it out and the sketch reads as what it is: rigid
    let mut sk = examples::belt_tangency();
    solve(&mut sk, SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "shaky={} numeric={:?} structural={}", d.shaky, d.numeric_rank, d.structural_rank);
    assert_eq!(d.status, State::Well);
    assert!(d.over.is_empty() && d.implied.is_empty());
    assert_eq!(d.geometric_dependency, 0);
    assert!(d.under_params.is_empty(), "{:?}", d.under_params);
}

/// A pose the solver merely stopped at is no verdict.  The four-bar linkage is consistent and
/// full rank; asked to solve in one iteration it does not get there, and what the diagnosis
/// says about that pose is *unsolved* — not a conflict naming three innocent statements, which
/// is what reading the unsatisfied rows as evidence about the geometry did (issue #43).
#[test]
fn a_solve_that_stopped_short_is_unsolved_not_a_conflict() {
    let (prog, _) = gcs_core::syntax::parse(
        "point a hint(x: 0, y: 0)
         point d hint(x: 60, y: 0)
         point b hint(x: 8, y: 24)
         point c hint(x: 52, y: 30)
         line ground_link(a, d)
         line crank(a, b)
         line coupler(b, c)
         line rocker(d, c)
         a distance(25) b
         b distance(45) c
         d distance(30) c
         ground a
         ground d
         crank angle(70) ground_link",
    );
    let mut sk = gcs_core::program::elaborate(&prog).sketch;
    let r = solve(&mut sk, SolveOpts { max_iter: 1, retry: false, ..SolveOpts::default() });
    assert!(!r.success, "one iteration was enough: {r:?}");
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Unsolved, "{}", summary(&d));
    assert!(!d.violated.is_empty(), "the unsatisfied rows are still reported");
    assert!(d.conflicts.is_none(), "no conflict set is searched for: {:?}", d.conflicts);
    assert!(summary(&d).contains("UNSOLVED"), "{}", summary(&d));
    assert!(
        d.entity_state.values().any(|&s| s == State::Unsolved)
            && d.entity_state.values().all(|&s| s != State::Conflict),
        "{:?}",
        d.entity_state
    );

    // solved, the same figure is well-constrained and nothing is violated
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.status, State::Well, "{}", summary(&d));
}
