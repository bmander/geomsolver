//! A curve family defined by constraints — `trace p where { … }` — solved against.
//!
//! The involute here is written the way Wikipedia writes it: a point `t` on the base circle at
//! bearing `u`, the string leaving perpendicular to the radius there, and taut — as long as the
//! arc it unwound.  No closed form appears in any constraint; the seeds carry one, because a
//! seed is where the search starts and which winding the string takes, and that is all a seed
//! is ever trusted with.  The tests below check the block against the closed form it never
//! states, check the kernel's implicit-function-theorem Jacobian against a finite difference of
//! the assembled system, and check that an ill-posed block is a diagnostic rather than a curve.

use gcs_core::program::elaborate;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;
use gcs_core::system::System;

/// The taut-string involute, and its closed form beside it, both over one base circle.  The
/// trace's seeds are deliberately *wrong by a factor of three* in the string term: a seed picks
/// the branch, and the constraints do the rest — if the block were decoration, `p` would stay
/// where the seed put it and every comparison below would fail.
const DOC: &str = "\
curve involute(c: circle, phase: Angle)(u) over (5, 90) =
  ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )

curve unwind(c: circle, datum: line, phase: Angle)(u) over (5, 90) =
  trace p where {
    point t hint(x: c.center.x + c.r * cos(u + phase), y: c.center.y + c.r * sin(u + phase))
    point p hint(x: c.center.x + c.r * (cos(u + phase) + 3 * u * pi / 180 * sin(u + phase)), \
                 y: c.center.y + c.r * (sin(u + phase) - 3 * u * pi / 180 * cos(u + phase)))
    line rad(c.center, t)
    line s(t, p)
    t on c
    rad perpendicular s
    datum angle(u + phase) rad
    t distance(c.r * u * pi / 180) p
  }

point  o hint(x: 0, y: 0)
point  ax hint(x: 1, y: 0)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 20) class construction

curve  formula = involute(base, phase: 0) over (5, 60)
curve  string = unwind(base, datum, phase: 0) over (5, 60)

radius(20) base
ground o
ground ax
";

fn build(src: &str) -> gcs_core::program::Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    elaborate(&prog)
}

/// Where the involute is at `u`, worked out here rather than asked of the core.
fn involute_at(cx: f64, cy: f64, rb: f64, u_deg: f64) -> (f64, f64) {
    let r = u_deg.to_radians();
    (cx + rb * (r.cos() + r * r.sin()), cy + rb * (r.sin() - r * r.cos()))
}

/// **The taut string traces the involute.**  Nothing in the `unwind` family states the closed
/// form (its seeds are wrong by design), yet the curve it draws is the involute to solver
/// precision, everywhere along the domain.
#[test]
fn the_taut_string_traces_the_involute() {
    let e = build(DOC);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    assert_eq!(e.sketch.curve_defs.len(), 2);
    assert_eq!(e.sketch.curves.len(), 2);
    for k in 0..=10 {
        let u = 5.0 + 55.0 * k as f64 / 10.0;
        let want = e.sketch.curve_point(0, u);
        let got = e.sketch.curve_point(1, u);
        assert!(
            (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
            "at u = {u}: formula {want:?}, trace {got:?}",
        );
    }
    // and the drawn polyline runs along it too — the sweep is one march, not many cold solves
    let poly = e.sketch.curve_polyline(1);
    assert!(poly.len() > 32);
    let last = poly[poly.len() - 1];
    let end = involute_at(0.0, 0.0, 20.0, 60.0);
    assert!((last.0 - end.0).abs() < 1e-8 && (last.1 - end.1).abs() < 1e-8);
}

/// **A point solves onto the traced curve**, and what it lands on really is the taut string:
/// perpendicular to the radius at the tangent point, as long as the arc unwound.
#[test]
fn a_point_lands_on_a_traced_curve() {
    let src = format!("{DOC}point q hint(x: 28, y: 22)\nq on string hint(u: 30)\n");
    let mut e = build(&src);
    assert!(e.ok());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let q = e.map.ent_named("q").unwrap();
    let got = e.sketch.point_xy(q.i());
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let u = e.sketch.params[c.aux_params()[0] as usize].value;
    let want = involute_at(0.0, 0.0, 20.0, u);
    assert!(
        (got.0 - want.0).abs() < 1e-7 && (got.1 - want.1).abs() < 1e-7,
        "at u = {u}: involute {want:?}, point {got:?}",
    );
    // the string, measured directly: t is on the circle at bearing u, the string is
    // perpendicular to the radius there and as long as the arc unwound
    let t = (20.0 * u.to_radians().cos(), 20.0 * u.to_radians().sin());
    let string = (got.0 - t.0, got.1 - t.1);
    assert!((t.0 * string.0 + t.1 * string.1).abs() < 1e-5, "perpendicular at the tangent point");
    let arc = 20.0 * u.to_radians();
    assert!((string.0.hypot(string.1) - arc).abs() < 1e-6, "taut: |string| = arc unwound");
}

/// **The implicit-function-theorem Jacobian is the system's own derivative.**  A finite
/// difference of the assembled residuals against the assembled Jacobian, so the inner solve,
/// the IFT, the chain through the derived value's tape and `params_on`'s column order are all
/// checked at once.
#[test]
fn the_trace_jacobian_matches_a_finite_difference() {
    let src = format!("{DOC}point q hint(x: 28, y: 22)\nq on string hint(u: 30)\n");
    let e = build(&src);
    assert!(e.ok());
    let mut sys = System::new(&e.sketch);
    let z = sys.z0(&e.sketch);
    // only the contact's rows: the radius row is exact and unrelated
    let dense = sys.jacobian_dense(&z);
    let m = sys.n_res;
    let n = z.len();
    for j in 0..n {
        let h = 1e-6 * z[j].abs().max(1.0);
        let (mut lo, mut hi) = (z.clone(), z.clone());
        lo[j] -= h;
        hi[j] += h;
        let (a, b) = (sys.residuals(&lo), sys.residuals(&hi));
        for i in 0..m {
            let fd = (b[i] - a[i]) / (2.0 * h);
            let got = dense.at(i, j);
            assert!(
                (got - fd).abs() <= 1e-4 * fd.abs().max(1.0),
                "d r{i} / d z{j}: kernel {got}, finite difference {fd}",
            );
        }
    }
}

/// The curve moves when the geometry it is written over does, and the point comes with it —
/// `∂C/∂θ` through the inner solve.  Note the centre is also the datum line's root, so its
/// columns are reached down *two* formal paths and the merged Jacobian must still be right.
#[test]
fn moving_the_circle_carries_the_traced_curve() {
    let doc = DOC.replace("radius(20) base", "radius(26) base");
    let src = format!("{doc}point q hint(x: 28, y: 22)\nq on string hint(u: 30)\n");
    let mut e = build(&src);
    assert!(e.ok());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let q = e.map.ent_named("q").unwrap();
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let u = e.sketch.params[c.aux_params()[0] as usize].value;
    let want = involute_at(0.0, 0.0, 26.0, u);
    let got = e.sketch.point_xy(q.i());
    assert!(
        (got.0 - want.0).abs() < 1e-6 && (got.1 - want.1).abs() < 1e-6,
        "the point followed the resized circle: {got:?} against {want:?}",
    );
}

/// A block that does not determine its points is a diagnostic naming the family — an
/// under-constrained locus is a curve that does not exist, and it must not elaborate quietly.
#[test]
fn an_underconstrained_block_is_refused() {
    let src = "\
curve wander(c: circle)(u) over (0, 90) =
  trace p where {
    point t
    point p
    t on c
    t distance(c.r * u * pi / 180) p
  }
point  o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve  w = wander(base)
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty());
    let e = elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("must determine")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}

/// The statements a block cannot hold are refused with a reason, not elaborated into nonsense.
#[test]
fn a_block_holds_declarations_and_constraints_only() {
    let src = "\
curve odd(c: circle)(u) =
  trace p where {
    point p
    ground p
    p coincident c.center
  }
point  o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve  w = odd(base)
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty());
    let e = elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("declarations and constraints")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}

/// **A gear whose flanks are traced, not computed.**  The shipped example document: `gear.sv`
/// with the involute family swapped for the taut-string trace — the solver finds the two rolls
/// where each flank crosses the root and tip circles, and every point of the flank in between.
/// Twelve teeth, so the test also runs in the stub-tooth regime where the root stands clear of
/// the base circle.
#[test]
fn a_gear_runs_on_a_traced_involute() {
    let n = 12usize;
    let mut e = build(gcs_core::examples::GEAR_TRACE);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    assert_eq!(e.sketch.curves.len(), 2 * n, "two traced flanks per tooth");

    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);

    // the wheel's numbers, worked out here rather than asked of the core
    let (m, ded, phi) = (3.0f64, 1.0f64, 25.0f64);
    let r_pitch = m * n as f64 / 2.0;
    let rb = r_pitch * phi.to_radians().cos();
    let rt = r_pitch + m;
    let rr = (r_pitch - ded * m).max(rb * 1.02);

    // every flank end landed on the circle its statement named — the rolls are the solver's
    let (mut on_root, mut on_tip) = (0, 0);
    for i in 2..e.sketch.points.len() {
        let (x, y) = e.sketch.point_xy(i);
        let rad = x.hypot(y);
        if (rad - rr).abs() < 1e-6 {
            on_root += 1;
        } else if (rad - rt).abs() < 1e-6 {
            on_tip += 1;
        } else {
            panic!("a flank end at radius {rad}, which is neither {rr} nor {rt}");
        }
    }
    assert_eq!(on_root, 2 * n);
    assert_eq!(on_tip, 2 * n);

    // and every flank is an involute: the string test, sampled along each traced curve
    for ci in 0..e.sketch.curves.len() {
        let ph = e.sketch.curves[ci].values[0];
        let (u0, u1) = e.sketch.curve_domain(ci);
        for k in 0..=6 {
            let u = u0 + (u1 - u0) * k as f64 / 6.0;
            let (x, y) = e.sketch.curve_point(ci, u);
            let a = (u + ph).to_radians();
            let t = (rb * a.cos(), rb * a.sin());
            let string = (x - t.0, y - t.1);
            assert!(
                (t.0 * string.0 + t.1 * string.1).abs() < 1e-6 * rb * rb,
                "curve {ci} at u = {u}: the string is not perpendicular to the radius",
            );
            let arc = rb * u.to_radians().abs();
            assert!(
                (string.0.hypot(string.1) - arc).abs() < 1e-6,
                "curve {ci} at u = {u}: string {} against arc {arc}",
                string.0.hypot(string.1),
            );
        }
    }

    // fully constrained: the wheel has no freedom left, and nothing is over- or under-drawn
    let d = gcs_core::diagnose::diagnose(
        &mut e.sketch,
        gcs_core::diagnose::DiagnoseOptions::default(),
    );
    assert_eq!(d.status, gcs_core::diagnose::State::Well, "{:?}", d.status);
    assert_eq!(d.dof, 0, "no degree of freedom is left");
}

/// **An evaluation is a function of what it is asked, and not of what it was asked before.**
///
/// A contact's pose is remembered so the next evaluation may *resume* the continuation rather
/// than replay it from the home (`locus::eval_at`), which is what keeps a traced drawing
/// solvable — but a warm start is only ever an optimisation if the answer is the same one.  A
/// resume that slipped onto the mirror branch would still converge, still satisfy every
/// residual, and be wrong; and it would show up here, because the same parameters asked in a
/// different order would come back different.
///
/// So: the same list of poses, evaluated in two orders and once as a scrambled walk that jumps
/// the length of the domain between neighbours — worst case for a warm start, since every step
/// is far.  The block carries a `ccw`, the case that never trusts a direct solve.  It is kept
/// **on purpose** now that `angle` is directed and pins the side by itself: what is under test
/// here is the resume, and a block with a predicate is the branch of it worth testing.  The
/// shipped documents shed theirs; this one holds that path open.
#[test]
fn an_evaluation_does_not_depend_on_what_was_evaluated_before() {
    let src = "\
curve involute(c: circle, datum: line, phase: Angle)(u) over (5, 60) =
  trace p from (90 - phase) where {
    point t
    point p
    line rad(c.center, t)
    line s(t, p)
    t on c
    datum angle(u + phase) rad
    rad perpendicular s
    p distance(-(c.r * u * pi / 180)) rad
    ccw(datum.p1, datum.p2, t)
  }

point  o hint(x: 0, y: 0)
point  ax hint(x: 1, y: 0)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 20) class construction
curve  w = involute(base, datum, phase: 0) over (5, 60)
radius(20) base
ground o
ground ax
point q hint(x: 28, y: 22)
q on w hint(u: 30)
";
    let e = build(src);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    let mut sys = System::new(&e.sketch);
    let z0 = sys.z0(&e.sketch);

    // the contact: its parameter is a column of the system, and it owns the two rows `q - C(u)`
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let col = sys.col_of[c.aux_params()[0] as usize] as usize;
    let row = sys.row_of(c.id).expect("the contact is compiled into the plan");

    // poses spread over the whole domain
    let us: Vec<f64> = (0..24).map(|k| 6.0 + 52.0 * k as f64 / 23.0).collect();
    let at = |sys: &mut System, u: f64| {
        let mut z = z0.clone();
        z[col] = u;
        sys.residuals(&z)
    };

    let forward: Vec<Vec<f64>> = us.iter().map(|&u| at(&mut sys, u)).collect();
    let mut backward: Vec<Vec<f64>> = us.iter().rev().map(|&u| at(&mut sys, u)).collect();
    backward.reverse();
    // a walk that jumps the domain each step: 0, 23, 1, 22, … — every warm start is a far one
    let mut scrambled = vec![Vec::new(); us.len()];
    for k in 0..us.len() {
        let i = if k % 2 == 0 { k / 2 } else { us.len() - 1 - k / 2 };
        scrambled[i] = at(&mut sys, us[i]);
    }

    let qxy = e.sketch.point_xy(e.map.ent_named("q").unwrap().i());
    for (k, &u) in us.iter().enumerate() {
        for (tag, got) in [("backward", &backward[k]), ("scrambled", &scrambled[k])] {
            for i in 0..sys.n_res {
                assert!(
                    (got[i] - forward[k][i]).abs() < 1e-9,
                    "at u = {u}, row {i}: {tag} read {}, forward read {}",
                    got[i],
                    forward[k][i],
                );
            }
        }
        // and it is the involute, not merely a repeatable answer: the row is q - C(u), handed
        // out over its own units (`System::row_scale`)
        let want = involute_at(0.0, 0.0, 20.0, u);
        let got = (
            qxy.0 - forward[k][row] * sys.row_scale[row],
            qxy.1 - forward[k][row + 1] * sys.row_scale[row + 1],
        );
        assert!(
            (got.0 - want.0).abs() < 1e-7 && (got.1 - want.1).abs() < 1e-7,
            "at u = {u}: involute {want:?}, trace {got:?}",
        );
    }
}

/// **The march is the fallback, and it carries the branch.**  This family's seeds collapse onto
/// the circle's centre past `u = 30` — where `point_on_circle`'s gradient vanishes and the
/// direct solve cannot even factorise — so evaluating at `u = 60` *must* march from the domain's
/// low end, where the seeds still stand, warm-starting step by step.  The curve still comes out
/// the involute.
#[test]
fn the_march_carries_a_branch_past_bad_seeds() {
    let src = "\
curve limp(c: circle, datum: line, phase: Angle)(u) over (5, 60) =
  trace p where {
    point t hint(x: c.center.x + c.r * cos(u + phase) * max(0, 1 - u / 30), \
                 y: c.center.y + c.r * sin(u + phase) * max(0, 1 - u / 30))
    point p hint(x: c.center.x + c.r * cos(u + phase) * max(0, 1 - u / 30), \
                 y: c.center.y + c.r * (sin(u + phase) - u * pi / 90) * max(0, 1 - u / 30))
    line rad(c.center, t)
    line s(t, p)
    t on c
    rad perpendicular s
    datum angle(u + phase) rad
    t distance(c.r * u * pi / 180) p
  }

point  o hint(x: 0, y: 0)
point  ax hint(x: 1, y: 0)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 20) class construction
curve  w = limp(base, datum, phase: 0) over (5, 60)
";
    let e = build(src);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    let want = involute_at(0.0, 0.0, 20.0, 60.0);
    let got = e.sketch.curve_point(0, 60.0);
    assert!(
        (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
        "marched to u = 60: involute {want:?}, trace {got:?}",
    );
}

/// A flat form that cannot be read comes back NaN and not-ok — a residual through it must never
/// read as satisfied, and `System` treats NaN as "not converged", never as "no error".
#[test]
fn a_malformed_flat_is_nan_not_a_curve() {
    let mut s = gcs_core::locus::Scratch::new();
    for junk in [&[][..], &[3.0, 1.0][..], &[f64::NAN; 8][..], &[1e300; 12][..]] {
        let v = gcs_core::locus::eval_flat(junk, &[0.0; 4], 0.0, &mut s);
        assert!(!v.ok && v.x.is_nan() && v.y.is_nan(), "{junk:?}");
    }
}

/// Over-determined is refused the same way under-determined is: a locus with more equations
/// than coordinates is a curve that does not exist.
#[test]
fn an_overconstrained_block_is_refused() {
    let doc = DOC.replace(
        "    t distance(c.r * u * pi / 180) p",
        "    t distance(c.r * u * pi / 180) p\n    p on datum",
    );
    let (prog, errs) = parse(&doc);
    assert!(errs.is_empty());
    let e = elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("must determine")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}

/// A circle declared inside a block: its radius is an unknown of the block like a coordinate,
/// and a dimension may tie it to `u` through the free twin.
#[test]
fn a_block_may_draw_a_circle_of_its_own() {
    let src = "\
curve dot(c: circle)(u) over (0, 10) =
  trace p where {
    point p hint(x: 1, y: 1)
    circle k(center: p) hint(r: 2)
    p coincident c.center
    radius(5 + u) k
  }
point  o hint(x: 3, y: -2)
circle base(center: o) hint(r: 20)
curve  w = dot(base)
";
    let e = build(src);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    // the traced point is the centre at every u; the circle's radius is the block's own affair
    for u in [0.0, 4.0, 10.0] {
        let got = e.sketch.curve_point(0, u);
        assert!((got.0 - 3.0).abs() < 1e-9 && (got.1 + 2.0).abs() < 1e-9, "at u = {u}: {got:?}");
    }
}

/// The block's own mistakes each say what is wrong: a traced point that is not the block's, a
/// name declared twice, a reference to nothing, and an inferred flag left unstated.
#[test]
fn a_blocks_mistakes_are_named() {
    let cases = [
        ("trace c where {\n  point p\n  p coincident c.center\n}",
         "must be a point the block declares"),
        ("trace p where {\n  point p\n  point p\n  p coincident c.center\n}",
         "declared twice"),
        ("trace p where {\n  point p\n  line l(p, zzz)\n  p coincident c.center\n}",
         "no such entity"),
        ("trace p where {\n  point p\n  point q\n  line l(p, q)\n\
          l tangent c\n  p coincident c.center\n}",
         "must be stated"),
    ];
    for (body, want) in cases {
        let src = format!(
            "curve b(c: circle)(u) =\n  {body}\npoint o hint(x: 0, y: 0)\n\
             circle base(center: o) hint(r: 5)\ncurve w = b(base)\n"
        );
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{want}: {errs:?}");
        let e = elaborate(&prog);
        assert!(!e.ok(), "{want}: elaborated cleanly");
        assert!(
            e.errors().any(|d| d.message.contains(want)),
            "wanted `{want}` in {:?}",
            e.errors().map(|d| &d.message).collect::<Vec<_>>()
        );
    }
}

/// **A seed is a place, and a place is named geometrically.**  `at c bearing (u)` is the point
/// at the edge of the circle — said the way a draughtsman says it, lowered to the same tapes the
/// trigonometry would be — and `at t` is wherever another point starts.  The bearing may read
/// `u`, which is what lets one seed follow the whole rim.
#[test]
fn a_seed_is_a_place_named_geometrically() {
    let src = "\
curve rim(c: circle, datum: line)(u) over (0, 350) =
  trace p where {
    point t hint at c bearing (u)
    point p hint at t
    line rad(c.center, t)
    t on c
    datum angle(u) rad
    p coincident t
  }
point  o hint(x: 2, y: 1)
point  ax hint(x: 3, y: 1)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 7)
curve  w = rim(base, datum)
";
    let e = build(src);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    // the full circle, mod-180 branch and all: the bearing seed reads u, so every sample starts
    // on the right side
    for u in [0.0f64, 90.0, 200.0, 350.0] {
        let want = (2.0 + 7.0 * u.to_radians().cos(), 1.0 + 7.0 * u.to_radians().sin());
        let got = e.sketch.curve_point(0, u);
        assert!(
            (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
            "at u = {u}: rim {want:?}, trace {got:?}",
        );
    }
}

/// A geometric seed's own mistakes each say what is wrong.
#[test]
fn a_geometric_seeds_mistakes_are_named() {
    let cases = [
        ("trace p where {\n  point q\n  point p hint at q bearing (0)\n\
          p coincident c.center\n  q coincident c.center\n}",
         "a bearing needs a circle"),
        ("trace p where {\n  point p hint at c\n  p coincident c.center\n}",
         "says the bearing"),
        ("trace p where {\n  point p\n  point q\n  line l(p, q) hint at c\n\
          p coincident c.center\n  q coincident c.center\n}",
         "only a point takes a geometric seed"),
        ("trace p where {\n  point p hint at zzz\n  p coincident c.center\n}",
         "no such entity"),
        // a point may only seed at one already declared: names enter scope in order
        ("trace p where {\n  point p hint at q\n  point q\n\
          p coincident c.center\n  q coincident c.center\n}",
         "no such entity: `q`"),
    ];
    for (body, want) in cases {
        let src = format!(
            "curve b(c: circle)(u) =\n  {body}\npoint o hint(x: 0, y: 0)\n\
             circle base(center: o) hint(r: 5)\ncurve w = b(base)\n"
        );
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{want}: {errs:?}");
        let e = elaborate(&prog);
        assert!(!e.ok(), "{want}: elaborated cleanly");
        assert!(
            e.errors().any(|d| d.message.contains(want)),
            "wanted `{want}` in {:?}",
            e.errors().map(|d| &d.message).collect::<Vec<_>>()
        );
    }
}

/// Outside a trace block a seed is a number a solve writes back, which a place named by
/// reference is not — so the geometric form is refused there, and says where it belongs.
#[test]
fn a_geometric_seed_outside_a_trace_block_is_refused() {
    let src = "point o hint(x: 0, y: 0)\ncircle c0(center: o) hint(r: 5)\npoint q hint at c0 bearing (30)\n";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("lives in a trace block")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}

/// **A branch is a stated fact.**  No seed anywhere: the block's point starts wherever the
/// deterministic restarts land it, and `ccw(datum.p1, datum.p2, t)` — an orientation predicate,
/// the spec's own instrument for selecting among discrete solution components — says which of
/// the circle's two crossings of the vertical `x = centre.x + r·cos u` is meant.  `from (90)`
/// anchors the reading a quarter turn from either crossing, where the two stand farthest apart.
///
/// The point of the test is the second half of §6.5.1's rule: a predicate is read **at the home
/// only**, and an implementation MUST NOT re-enforce it elsewhere — so the branch is carried on
/// to where the predicate itself reads *false*, and the curve stays on it.
///
/// **The datum is deliberately not the mirror axis, and levelling it would gut this test.**  The
/// two solutions are mirrored in the horizontal through the centre (that is what the
/// `horizontal_distance` row says); the predicate is read against `datum`, tilted to (2, 1).
/// Because the two lines differ, the chosen branch crosses the datum at `tan u = 1/2` while the
/// pair itself never merges — so the predicate flips partway along a branch that stays
/// unambiguous.  Were the datum horizontal, a predicate flip and a branch fold would be the same
/// event (an *angle* stated mod 180 was the old fixture, and that is exactly how it read), the
/// domain would have to stop short of both, and the test would pass without ever exercising the
/// rule in its name.
#[test]
fn a_branch_is_a_stated_fact() {
    let family = "\
curve rim(c: circle, datum: line)(u) over (10, 170) =
  trace t from (90) where {
    point t
    t on c
    c.center distance(c.r * cos(u), along: x) t
    ccw(datum.p1, datum.p2, t)
  }
";
    // `ax` puts the datum along (2, 1).  Not chosen by eye: at 45° the restart path reflects the
    // point to where the `horizontal_distance` gradient is parallel to the circle's and the home
    // solve has a singular Jacobian.
    let doc = "\
point  o hint(x: 2, y: 1)
point  ax hint(x: 4, y: 2)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 7)
curve  w = rim(base, datum)
";
    // `ccw(o, ax, t)` read on the traced point: the datum's direction crossed into o→t.  Taken
    // off the elaborated sketch rather than written out, so that levelling the datum in `doc`
    // changes this reading too — which is what makes the assertion below a real guard.
    let reads_true = |sk: &gcs_core::model::Sketch, t: (f64, f64)| {
        let l = &sk.lines[0];
        let (ax, ay) = sk.point_xy(l.p1 as usize);
        let (bx, by) = sk.point_xy(l.p2 as usize);
        (bx - ax) * (t.1 - ay) - (by - ay) * (t.0 - ax) > 0.0
    };
    for (pred, flip) in [("ccw", 1.0f64), ("cw", -1.0)] {
        let src = format!("{}{doc}", family.replace("ccw(", &format!("{pred}(")));
        let e = build(&src);
        assert!(
            e.ok(),
            "{pred}: {:?}",
            e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
        );
        // the whole arc, on the component the predicate names: cw is the mirror reading, and
        // picks the bearing on the other side of the centre
        let mut seen = (false, false);
        for u in [10.0f64, 90.0, 170.0] {
            let b = (flip * u).to_radians();
            let want = (2.0 + 7.0 * b.cos(), 1.0 + 7.0 * b.sin());
            let got = e.sketch.curve_point(0, u);
            assert!(
                (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
                "{pred} at u = {u}: rim {want:?}, trace {got:?}",
            );
            match reads_true(&e.sketch, got) {
                true => seen.0 = true,
                false => seen.1 = true,
            }
        }
        // and the branch was carried past the predicate's own truth: had the datum been levelled
        // onto the mirror axis the predicate would read one way for the whole domain, the curve
        // above would still be traced correctly, and this test would guard nothing
        assert_eq!(seen, (true, true), "{pred}: the predicate never flips along the branch");
    }
}

/// An orientation's own mistakes each say what is wrong.
#[test]
fn an_orientations_mistakes_are_named() {
    let cases = [
        ("trace p where {\n  point p\n  ccw(c.center, p)\n  p coincident c.center\n}",
         "names three points"),
        ("trace p where {\n  point p\n  ccw(c, c.center, p)\n  p coincident c.center\n}",
         "about points"),
        ("trace p where {\n  point p\n  ccw(p, c.center, c.center)\n\
          p coincident c.center\n}",
         "must be one the block places"),
    ];
    for (body, want) in cases {
        let src = format!(
            "curve b(c: circle)(u) =\n  {body}\npoint o hint(x: 0, y: 0)\n\
             circle base(center: o) hint(r: 5)\ncurve w = b(base)\n"
        );
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{want}: {errs:?}");
        let e = elaborate(&prog);
        assert!(!e.ok(), "{want}: elaborated cleanly");
        assert!(
            e.errors().any(|d| d.message.contains(want)),
            "wanted `{want}` in {:?}",
            e.errors().map(|d| &d.message).collect::<Vec<_>>()
        );
    }
}
