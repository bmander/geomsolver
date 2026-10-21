//! A parent solved through its instances (#454, `solve::solve_through`): the prototype of a
//! reusable solve boundary on the locus's machinery, against `docs/solve-boundaries-plan.md`.
//!
//! Each case is one document solved twice, flat and with instances named from Rust as
//! boundaries, and the two held to agree (FL1, FL2), with the rank adding up (FL3).  Each
//! boundary's derivative, the implicit function theorem on its own factorisation, is held to a
//! central difference of its own evaluations at the scale `tests/jacobians.rs` holds kernels to
//! (DD1).  The belt drive of the direction paper (§6.1) solves with both of its runs as
//! boundaries to the flat answer.
use gcs_core::boundary;
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, solve_through, Boundaries, Boundary, Reason, SolveOpts, Through};
use gcs_core::system::{System, RANK_TOL};

/// `src` elaborated and solved flat: the document, and its answer.
fn flat(src: &str) -> (Elaborated, Sketch) {
    let e = crate::common::read(src);
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success && r.settled(), "the flat solve does not settle: {}", r.message);
    (e, sk)
}

/// The instances named, as boundaries of `e`'s drawing.
fn bounds(e: &Elaborated, names: &[&str]) -> Vec<Boundary> {
    names.iter().map(|n| boundary::instance(e, &e.sketch, n)
        .unwrap_or_else(|| panic!("no instance `{n}`"))).collect()
}

/// `e`'s drawing solved through the instances named, from the same start as the flat solve.
fn through(e: &Elaborated, names: &[&str]) -> (Through, Sketch) {
    let mut sk = e.sketch.clone();
    let t = solve_through(&mut sk, &bounds(e, names), SolveOpts::default())
        .unwrap_or_else(|why| panic!("not a boundary: {why}"));
    (t, sk)
}

/// The flat document's rank at a pose, as the diagnosis reads it.
fn rank(sk: &Sketch) -> usize {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    sys.rank(&z, RANK_TOL, true)
}

/// FL1–FL3 for one document: solved through `names`, it settles with every flat row holding, at
/// the flat solve's pose to `tol`; the flat solve started there does not move; and the flat
/// rank is the boundaries' own unknowns and the rank of the parent through them.
fn agrees(src: &str, names: &[&str], tol: f64) -> (Elaborated, Through, Sketch) {
    let (e, want) = flat(src);
    let (t, got) = through(&e, names);
    assert!(t.result.success, "through {names:?}: {} ({:?})", t.result.message, t.failure);
    assert!(t.result.max_residual < 1e-9, "{}", t.result.max_residual);
    assert!(t.inlined.is_empty(), "{:?}", t.inlined);
    for (name, n) in &t.evaluations {
        assert!(*n > 0, "`{name}` was never evaluated");
    }
    // FL1: the same pose, parameter by parameter
    for (i, (a, b)) in got.params.iter().zip(&want.params).enumerate() {
        assert!((a.value - b.value).abs() <= tol * (1.0 + b.value.abs()),
                "`{}` (#{i}): {} through the boundaries, {} flat", b.name, a.value, b.value);
    }
    // FL2: the flat solve from there has nothing left to do
    let mut sys = System::new(&got);
    let z = sys.z0(&got);
    assert!(sys.max_relative_residual(&z) < 1e-9);
    let mut again = got.clone();
    assert!(solve(&mut again, SolveOpts::default()).success);
    for (a, b) in again.params.iter().zip(&got.params) {
        assert!((a.value - b.value).abs() <= 1e-9 * (1.0 + b.value.abs()), "`{}` moved", a.name);
    }
    // FL3: the rank adds up
    assert_eq!(rank(&got), t.boundary_unknowns + t.reduced_rank,
               "flat rank against {} of the boundaries' and {} of the parent's",
               t.boundary_unknowns, t.reduced_rank);
    (e, t, got)
}

/// DD1: every unknown of the boundary against every input, the derivative of its evaluation
/// against a central difference of two more, each resumed from the last.  `h` is relative to
/// each input's size, the tolerance relative to each entry's.
fn derivatives_hold(sk: &Sketch, b: &Boundary) {
    let mut bs = Boundaries::new(sk, std::slice::from_ref(b)).expect("a boundary");
    let inputs = bs.inputs(&b.name).unwrap();
    assert!(!inputs.is_empty(), "`{}` reads no input the solve moves", b.name);
    let x0: Vec<f64> = inputs.iter().map(|&p| sk.params[p as usize].value).collect();
    let at = bs.evaluate(&b.name, &x0).unwrap_or_else(|f| panic!("{}", f.text()));
    for j in 0..x0.len() {
        let h = 1e-6 * x0[j].abs().max(1.0);
        let mut x = x0.clone();
        x[j] = x0[j] + h;
        let up = bs.evaluate(&b.name, &x).unwrap_or_else(|f| panic!("{}", f.text()));
        x[j] = x0[j] - h;
        let down = bs.evaluate(&b.name, &x).unwrap_or_else(|f| panic!("{}", f.text()));
        for i in 0..at.y.len() {
            let fd = (up.y[i] - down.y[i]) / (2.0 * h);
            let d = at.dydx.at(i, j);
            assert!((fd - d).abs() <= 1e-5 * d.abs().max(1.0),
                    "`{}`: d y{i} / d x{j} is {d}, differenced {fd}", b.name);
        }
    }
}

/* -- the two-root point, placed through its output --------------------------------------------- */

/// The review's two-root point as a component, its seed written from its operands, and a parent
/// that places `b` by where the point lands: `one.x` 6.5 from `c`.  The parent's one unknown
/// left, `b`'s run along the line from `a`, reaches the point only through the boundary.
const APEX: &str = "\
unit mm
use std (horizontal)
component Apex(view: plane, a: point, b: point) {
  in view {
    x := point hint(at: a, toward: b, by: 0.8333, turn: 53.13deg)
    x distance(5mm) a
    x distance(5mm) b
  }
}
in std.front {
  a := point
  b := point hint((6, 0))
  c := point
  fix((0, 0)) a
  fix((3, 10)) c
  a horizontal b
}
one := Apex(std.front, a, b)
one.x distance(6.5mm) c
";

#[test]
fn the_apex_placed_through_its_output_solves_to_the_flat_answer() {
    let (e, t, sk) = agrees(APEX, &["one"], 1e-9);
    assert_eq!((t.parent_unknowns, t.boundary_unknowns), (2, 2));
    assert_eq!(t.reduced_rank, 2);
    // the root the seeds chose, up: x = (d/2, √(25 − d²/4)), 6.5 from (3, 10)
    let (x, y) = sk.point_xy(crate::common::ent(&e, "one.x").i());
    assert!(y > 0.0 && ((x - 3.0).powi(2) + (y - 10.0).powi(2)).sqrt() - 6.5 < 1e-9);
    assert!((x * x + y * y - 25.0).abs() < 1e-9);
}

/// One boundary reading another's output (IO4): a second apex over the first's point and `c`,
/// and the parent placing `b` by where the second lands.  The two are evaluated in that order,
/// and the parent's derivative is chained through both.
#[test]
fn an_apex_over_another_apex_solves_through_both() {
    let src = APEX.replace("one.x distance(6.5mm) c\n", "\
two := Apex(std.front, one.x, c)
in std.front {
  d := point
  fix((0, 12)) d
}
two.x distance(4mm) d
");
    // named the other way round, they are still evaluated in the order they read each other
    let (_, t, _) = agrees(&src, &["two", "one"], 1e-9);
    assert_eq!(t.evaluations.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(), ["one", "two"]);
    assert_eq!((t.parent_unknowns, t.boundary_unknowns, t.reduced_rank), (2, 4, 2));
}

#[test]
fn the_apex_derivative_is_its_difference() {
    // nothing held, so every operand's coordinate is an input
    let src = APEX.replace("  fix((0, 0)) a\n", "");
    let (e, sk) = flat(&src.replace("one.x distance(6.5mm) c\n", ""));
    let b = boundary::instance(&e, &sk, "one").unwrap();
    let bs = Boundaries::new(&sk, std::slice::from_ref(&b)).unwrap();
    assert_eq!(bs.inputs("one").unwrap().len(), 4, "a and b, two coordinates each");
    assert_eq!(bs.unknowns("one").unwrap().len(), 2);
    derivatives_hold(&sk, &b);
}

/// The same instance in a document a thousand times larger elsewhere: the parent's extent
/// changes the units every row is judged in, and the value still agrees (CL1, to the inner
/// solve's tolerance; the boundary's own scale is #455's).
#[test]
fn the_apex_does_not_read_the_size_of_the_rest_of_the_document() {
    let (e, _, small) = agrees(APEX, &["one"], 1e-9);
    let far = APEX.replace("  fix((3, 10)) c\n",
                           "  fix((3, 10)) c\n  far := point\n  fix((6000, 0)) far\n");
    let (e2, _, large) = agrees(&far, &["one"], 1e-9);
    let at = |e: &Elaborated, sk: &Sketch| sk.point_xy(crate::common::ent(e, "one.x").i());
    let ((x1, y1), (x2, y2)) = (at(&e, &small), at(&e2, &large));
    assert!((x1 - x2).abs() < 1e-9 && (y1 - y2).abs() < 1e-9, "({x1}, {y1}) against ({x2}, {y2})");
}

/* -- the belt drive ---------------------------------------------------------------------------- */

/// The direction paper's belt drive (§6.1), its turning frames left out: two pulleys, the upper
/// and lower runs tangent to both, and where the runs cross on the line of centres.  `Crossing`
/// stays the parent's, so its point is placed by rows over both runs' outputs.  The second shaft
/// is seeded off its place, so the parent moves it and the runs follow.
fn belt(centres: &str) -> String {
    format!("\
unit mm
use std (horizontal)

drive := {{r_small: 20mm, r_large: 45mm, centres: {centres}, bore: 4mm, width: 10mm}}

component Pulley(view: plane, centre: point, r: Length, bore: Length, width: Length) {{
  in view {{
    rim := circle(center: centre) hint(r: r)
    radius(r) rim
    hole := circle(center: centre) hint(r: bore)
    radius(bore) hole
  }}
  body := solid(face(rim, holes: hole), from: 0mm, to: width)
}}

component Run(view: plane, a: circle, b: circle, up: Scalar) {{
  in view {{
    s := point hint((a.center.x, a.center.y + up * a.r))
    e := point hint((b.center.x, b.center.y + up * b.r))
    span := line(s, e)
    s coincident a
    e coincident b
    span tangent(at: p1) a
    span tangent(at: p2) b
  }}
}}

component Crossing(view: plane, a: line, b: line, l: line) {{
  in view {{
    x := point hint((-60, 0))
    x coincident a
    x coincident b
    claim x coincident l
  }}
}}

in std.front {{
  o1 := point
  fix((0, 0)) o1
  o2 := point hint((100, 10))
  o1 horizontal o2
  o1 distance(drive.centres) o2
  construction centres := line(o1, o2)
}}

small := Pulley(std.front, o1, r: drive.r_small, bore: drive.bore, width: drive.width)
large := Pulley(std.front, o2, r: drive.r_large, bore: drive.bore, width: drive.width)
upper := Run(std.front, small.rim, large.rim, up: 1)
lower := Run(std.front, small.rim, large.rim, up: -1)
meet := Crossing(std.front, upper.span, lower.span, centres)
")
}

#[test]
fn the_belt_drive_solves_through_its_runs_to_the_flat_answer() {
    let (e, t, sk) = agrees(&belt("120mm"), &["upper", "lower"], 1e-9);
    assert_eq!(t.evaluations.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
               ["upper", "lower"]);
    assert_eq!(t.boundary_unknowns, 8, "two runs, two points each");
    assert!(t.result.iterations > 1, "the parent had nothing to do");
    // the runs, produced, meet at x = −r₁c/(r₂ − r₁) = −20 · 120 / 25
    let (x, y) = sk.point_xy(crate::common::ent(&e, "meet.x").i());
    assert!((x + 96.0).abs() < 1e-7 && y.abs() < 1e-7, "({x}, {y})");
}

/// A run's inputs are exactly what its rows read and it did not make: the two rims' centres and
/// radii, less what the document holds (`o1`, fixed).  Nothing outside them (CL2).
#[test]
fn a_run_reads_its_two_rims_and_nothing_else() {
    let (e, sk) = flat(&belt("120mm"));
    let bs = Boundaries::new(&sk, &bounds(&e, &["upper"])).unwrap();
    let mut want: Vec<u32> = ["o1", "o2", "small.rim", "large.rim"].iter()
        .flat_map(|n| sk.own_params(crate::common::ent(&e, n)))
        .filter(|&p| !sk.params[p as usize].fixed)
        .collect();
    want.sort_unstable();
    want.dedup();
    let mut got = bs.inputs("upper").unwrap();
    got.sort_unstable();
    assert_eq!(got, want);
    let mut own = bs.unknowns("upper").unwrap();
    own.sort_unstable();
    let mut made: Vec<u32> = ["upper.s", "upper.e"].iter()
        .flat_map(|n| sk.own_params(crate::common::ent(&e, n))).collect();
    made.sort_unstable();
    assert_eq!(own, made);
}

#[test]
fn a_run_derivative_is_its_difference() {
    let (e, sk) = flat(&belt("120mm"));
    for name in ["upper", "lower"] {
        derivatives_hold(&sk, &boundary::instance(&e, &sk, name).unwrap());
    }
}

/// Pulleys whose centres stand closer than the difference of their radii have no common tangent.
/// Asked to stand there, the parent stops short at the edge — each step past it is a run whose
/// inner solve does not settle or does not continue, and is rejected (FA2, DD5) — and says which
/// boundary held it back.  At the pose it stops at, every run's own rows still hold to the inner
/// solve's acceptance: no evaluation it accepted was a nearby value.
#[test]
fn a_parent_asked_to_nest_the_pulleys_stops_short_and_says_which_run() {
    let e = crate::common::read(&belt("10mm"));
    let (t, sk) = through(&e, &["upper", "lower"]);
    assert!(!t.result.success && t.rejected > 0);
    assert_eq!(t.failure.as_ref().map(|f| f.boundary.as_str()), Some("upper"), "{:?}", t.failure);
    let (x, _) = sk.point_xy(crate::common::ent(&e, "o2").i());
    assert!(x > 25.0, "the centres {x} apart: one rim holds the other");
    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    let errors = sys.constraint_errors(&z);
    let own: Vec<u32> = bounds(&e, &["upper", "lower"]).iter().flat_map(|b| b.rows.clone())
        .collect();
    for (cid, err) in sys.cids.iter().zip(&errors) {
        if own.contains(cid) {
            // within the inner solve's acceptance, 1e-8: at the edge the block is ill-conditioned
            assert!(*err < 1e-8, "a run's row #{cid} is off by {err}");
        }
    }
    assert!(t.result.max_residual > 1e-3, "{}", t.result.max_residual);
}

/// Started with the pulleys nested, the runs fail at the first evaluation: the parent's residual
/// is not finite and the solve names the run and why (FA1, FA3).
#[test]
fn a_run_between_nested_pulleys_fails_and_the_parent_says_which() {
    let e = crate::common::read(&belt("10mm").replace("hint((100, 10))", "hint((10, 0))"));
    let (t, _) = through(&e, &["upper", "lower"]);
    assert!(!t.result.success);
    let f = t.failure.as_ref().expect("a failure");
    assert_eq!((f.boundary.as_str(), f.reason), ("upper", Reason::Unsettled), "{f:?}");
    assert!(t.result.max_residual.is_nan());
    assert!(t.result.message.contains("`upper` failed: unsettled"), "{}", t.result.message);
}

/* -- a common perpendicular in space ----------------------------------------------------------- */

/// The common perpendicular of two skew lines, the other case the issue starts from: `p` on `l1`,
/// `q` on `l2`, and the line between them square to both.  The parent turns the second line about
/// a held end until the perpendicular is 7 long.
const PERPENDICULAR: &str = "\
unit mm
use std
component Perp(l1: line, l2: line) {
  p := point hint((0, 0, 0))
  q := point hint((0, 0, 10))
  n := line(p, q)
  p coincident l1
  q coincident l2
  n perpendicular l1
  n perpendicular l2
}
a1 := point
b1 := point
a2 := point
b2 := point hint((0, 30, 8))
fix(x == -20, y == 0, z == 0) a1
fix(x == 20, y == 0, z == 0) b1
fix(x == 0, y == -20, z == 10) a2
b2 distance(50mm) a2
fix(x == 0) b2
l1 := line(a1, b1)
l2 := line(a2, b2)
c := Perp(l1, l2)
length(7mm) c.n
";

#[test]
fn the_common_perpendicular_solves_through_to_the_flat_answer() {
    let (e, t, sk) = agrees(PERPENDICULAR, &["c"], 1e-8);
    assert_eq!(t.reduced_rank, t.parent_unknowns);
    let (p, q) = crate::common::ends(&sk, crate::common::ent(&e, "c.n"));
    assert!((gcs_core::space::distance(p, q) - 7.0).abs() < 1e-9);
}

#[test]
fn the_common_perpendicular_derivative_is_its_difference() {
    let (e, sk) = flat(PERPENDICULAR);
    derivatives_hold(&sk, &boundary::instance(&e, &sk, "c").unwrap());
}

/* -- what is not a boundary -------------------------------------------------------------------- */

/// A body that leaves its own point a freedom states fewer rows than it has unknowns, and is
/// refused before anything is solved (FR1).
#[test]
fn a_body_with_a_freedom_is_refused() {
    let e = crate::common::read("\
unit mm
use std
component Loose(view: plane, a: point) {
  in view {
    x := point hint(at: a, toward: a, by: 1, turn: 0deg)
    x distance(5mm) a
  }
}
in std.front {
  a := point
  fix((0, 0)) a
}
one := Loose(std.front, a)
");
    let why = solve_through(&mut e.sketch.clone(), &bounds(&e, &["one"]), SolveOpts::default())
        .err().expect("refused");
    assert!(why.contains("`one` is not square: 2 unknowns of its own against 1 equations"),
            "{why}");
}

/// Two boundaries that read each other's unknowns are one coupled system, and the parent's: both
/// are solved in it, flat, and said to be (IO4).  Made from Rust, since the language cannot say
/// it: `p` placed from `a` and `q`, `q` from `b` and `p`.
#[test]
fn boundaries_that_read_each_other_are_solved_in_the_parent() {
    let src = "\
unit mm
use std (horizontal)
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((8, 0)) b
  p := point hint((1, 4))
  q := point hint((9, 4))
  p distance(5mm) a
  p distance(5mm) q
  q distance(5mm) b
  q horizontal p
}
";
    let (e, want) = flat(src);
    let sk = &e.sketch;
    let params = |n: &str| sk.own_params(crate::common::ent(&e, n));
    let reads = |c: &gcs_core::constraints::Constraint, n: &str| {
        params(n).iter().any(|p| c.params(sk).contains(p))
    };
    let rows = |of: &str, with: &[&str]| -> std::collections::BTreeSet<u32> {
        sk.constraints.iter().filter(|c| reads(c, of) && with.iter().any(|w| reads(c, w)))
            .map(|c| c.id).collect()
    };
    let kind = |id: &u32| sk.constraint(*id).map(|c| c.kind);
    // P: `p distance a`, `p distance q`; Q: `q distance b`, `q horizontal p` (a `level` row)
    let p_rows = rows("p", &["a", "q"]).into_iter()
        .filter(|id| kind(id) == Some(CKind::Distance)).collect();
    let q_rows = rows("q", &["b", "p"]).into_iter()
        .filter(|id| kind(id) == Some(CKind::Level) || rows("q", &["b"]).contains(id))
        .collect();
    let bs = [
        Boundary { name: "P".into(), own: params("p").into_iter().collect(), rows: p_rows },
        Boundary { name: "Q".into(), own: params("q").into_iter().collect(), rows: q_rows },
    ];
    let mut got = sk.clone();
    let t = solve_through(&mut got, &bs, SolveOpts::default()).unwrap();
    assert_eq!(t.inlined, ["P", "Q"]);
    assert!(t.evaluations.is_empty());
    assert!(t.result.success, "{}", t.result.message);
    for (a, b) in got.params.iter().zip(&want.params) {
        assert!((a.value - b.value).abs() <= 1e-9 * (1.0 + b.value.abs()), "`{}`", a.name);
    }
}
