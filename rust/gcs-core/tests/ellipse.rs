//! The ellipse as a drawing element: it saves, grafts, picks and takes a rim contact.
use gcs_core::constraints::Constraint;
use gcs_core::model::{pick, EntKind, EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{diagnose, ellipse, io};

fn with_ellipse() -> (Sketch, usize) {
    let mut sk = Sketch::new();
    let c = sk.point(10.0, 5.0, false, "c");
    let m = sk.point(18.0, 5.0, false, "m");
    let e = sk.ellipse(c, m, 3.0, "e");
    (sk, e)
}

/// In the ellipse's own frame the rim satisfies (x/a)² + (y/b)² = 1.
fn on_rim(sk: &Sketch, e: usize, x: f64, y: f64) -> f64 {
    let g = ellipse::geom(sk, e);
    let (wx, wy) = (x - g.cx, y - g.cy);
    let xx = (wx * g.ux + wy * g.uy) / g.a;
    let yy = (g.ux * wy - g.uy * wx) / g.a;
    (xx / g.a) * (xx / g.a) + (yy / g.b) * (yy / g.b) - 1.0
}

#[test]
fn round_trips_through_json() {
    let (mut sk, e) = with_ellipse();
    sk.ellipses[e].class = gcs_core::style::Classes::one("construction");
    let bp = sk.ellipses[e].minor as usize;
    sk.params[bp].fixed = true;
    let p = sk.point(13.0, 7.0, false, "p");
    sk.add(Constraint::point_on_ellipse(&sk, EntRef::point(p), EntRef::ellipse(e)));
    let back = io::loads(&io::dumps(&sk, None)).unwrap();
    assert_eq!(back.ellipses.len(), 1);
    let be = &back.ellipses[0];
    assert!(be.class.has("construction"));
    assert_eq!(back.params[be.minor as usize].value, 3.0);
    assert!(back.params[be.minor as usize].fixed);
    assert_eq!(back.constraints.len(), 1);
    assert_eq!(back.constraints[0].type_name(), "PointOnEllipse");
}

#[test]
fn a_copy_keeps_the_ellipse_and_its_contact() {
    let (mut sk, e) = with_ellipse();
    let p = sk.point(13.0, 7.0, false, "p");
    sk.add(Constraint::point_on_ellipse(&sk, EntRef::point(p), EntRef::ellipse(e)));
    let clip = io::copy(&sk, &[EntRef::ellipse(e), EntRef::point(p)]);
    assert_eq!(clip.ellipses.len(), 1);
    assert_eq!(clip.constraints.len(), 1);
    // deleting the centre takes the ellipse and the contact with it
    let cut = io::without(&sk, &[EntRef::point(sk.ellipses[e].center as usize)], &[]);
    assert_eq!(cut.ellipses.len(), 0);
    assert_eq!(cut.constraints.len(), 0);
}

#[test]
fn the_rim_is_picked_and_bounded() {
    let (sk, e) = with_ellipse();
    // the top of the rim is a minor radius above the centre
    let hit = pick(&sk, 10.0, 8.05, 0.2).expect("the rim is within reach");
    assert_eq!(hit, EntRef::ellipse(e));
    assert!(pick(&sk, 12.0, 5.8, 0.2).is_none(), "the inside of an ellipse is empty space");
    let (x0, y0, x1, y1) = sk.bounds(EntRef::ellipse(e));
    assert!((x0 - 2.0).abs() < 1e-12 && (x1 - 18.0).abs() < 1e-12);
    assert!((y0 - 2.0).abs() < 1e-12 && (y1 - 8.0).abs() < 1e-12);
}

#[test]
fn a_point_solves_onto_the_rim() {
    let (mut sk, e) = with_ellipse();
    sk.fix_point(sk.ellipses[e].center as usize, true);
    sk.fix_point(sk.ellipses[e].major as usize, true);
    let bp = sk.ellipses[e].minor as usize;
    sk.params[bp].fixed = true;
    let p = sk.point(11.0, 9.0, false, "p");
    sk.add(Constraint::point_on_ellipse(&sk, EntRef::point(p), EntRef::ellipse(e)));
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (x, y) = sk.point_xy(p);
    assert!(on_rim(&sk, e, x, y).abs() < 1e-6, "left the rim by {}", on_rim(&sk, e, x, y));
}

#[test]
fn a_bare_ellipse_has_its_five_dof() {
    let (mut sk, _) = with_ellipse();
    // the diagnosis needs an equation to look at; a rim contact spends the point's 2 less its 1
    let p = sk.point(13.0, 7.0, false, "p");
    sk.add(Constraint::point_on_ellipse(&sk, EntRef::point(p), EntRef::ellipse(0)));
    let d = diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.dof, 6, "5 for the ellipse, net 1 for a point held to its rim");
    assert_eq!(d.entity_state.get(&EntRef::ellipse(0)).map(|s| s.as_str()), Some("under"));
}

#[test]
fn minor_to_puts_the_rim_through_the_target() {
    let (mut sk, e) = with_ellipse();
    let b = ellipse::minor_to(10.0, 5.0, 18.0, 5.0, 12.0, 7.5).unwrap();
    let bp = sk.ellipses[e].minor as usize;
    sk.params[bp].value = b;
    assert!(on_rim(&sk, e, 12.0, 7.5).abs() < 1e-9);
    // past the end of the major axis nothing reaches the target; the answer stays finite
    assert!(ellipse::minor_to(10.0, 5.0, 18.0, 5.0, 19.0, 5.1).unwrap().is_finite());
    assert!(ellipse::minor_to(10.0, 5.0, 10.0, 5.0, 12.0, 7.5).is_none());
}

#[test]
fn a_line_solves_tangent_to_the_rim() {
    let (mut sk, e) = with_ellipse();
    sk.fix_point(sk.ellipses[e].center as usize, true);
    sk.fix_point(sk.ellipses[e].major as usize, true);
    let bp = sk.ellipses[e].minor as usize;
    sk.params[bp].fixed = true;
    // a line above the ellipse, level; both endpoints free to fall onto the rim
    let a = sk.point(4.0, 10.0, false, "a");
    let b = sk.point(16.0, 10.0, false, "b");
    let l = sk.line(a, b);
    sk.add(Constraint::ellipse_tangent_line(&sk, EntRef::ellipse(e), EntRef::line(l)));
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    // tangent: the whole rim is on one side of the line and just touches it
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    let (dx, dy) = (bx - ax, by - ay);
    let len = dx.hypot(dy);
    let mut min_d: f64 = f64::INFINITY;
    let mut max_d: f64 = f64::NEG_INFINITY;
    for (x, y) in ellipse::sample(&sk, e, 256) {
        let d = (dx * (y - ay) - dy * (x - ax)) / len;
        min_d = min_d.min(d);
        max_d = max_d.max(d);
    }
    // touching from one side: the signed distances all share a sign and the nearest is ~0
    assert!(min_d.abs().min(max_d.abs()) < 1e-6, "gap {} .. {}", min_d, max_d);
    assert!(min_d * max_d >= -1e-9, "the line crosses the rim: {} .. {}", min_d, max_d);
}

#[test]
fn a_circle_solves_onto_the_osculating_circle() {
    let (mut sk, e) = with_ellipse();
    sk.fix_point(sk.ellipses[e].center as usize, true);
    sk.fix_point(sk.ellipses[e].major as usize, true);
    let bp = sk.ellipses[e].minor as usize;
    sk.params[bp].fixed = true;
    // a circle near the major end, centre and radius free
    let cc = sk.point(16.0, 5.5, false, "cc");
    let ci = sk.circle(cc, 2.0, "c");
    sk.add(Constraint::ellipse_curvature(&sk, EntRef::ellipse(e), EntRef::circle(ci)));
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    // the circle is the rim's own: centre at the centre of curvature, radius the rim's radius
    let t = sk.constraints[0].args[2].value(&sk);
    let g = ellipse::geom(&sk, e);
    let p = g.point_at(t);
    let (d1, d2) = g.derivs(t);
    let q = d1.0 * d1.0 + d1.1 * d1.1;
    let turn = d1.0 * d2.1 - d1.1 * d2.0;
    let want_r = q.powf(1.5) / turn.abs();
    let want_c = (p.0 + (q / turn) * -d1.1, p.1 + (q / turn) * d1.0);
    let (cx, cy) = sk.point_xy(cc);
    assert!((cx - want_c.0).abs() < 1e-6 && (cy - want_c.1).abs() < 1e-6);
    assert!((sk.radius_value(EntRef::circle(ci)).abs() - want_r).abs() < 1e-6);
}

#[test]
fn distance_is_measured_to_the_rim() {
    let (sk, e) = with_ellipse();
    let p = EntRef::point(0); // the centre point, 3 world units inside the rim at the top
    let d = gcs_core::model::distance_between(&sk, p, EntRef::ellipse(e));
    assert!((d - 3.0).abs() < 1e-9, "centre to rim is the minor radius, got {d}");
    assert_eq!(sk.count(EntKind::Ellipse), 1);
}

/// A contact's parameter is scaled by what one unit of it is worth in world length, and that is
/// read off the ellipse at *compile* time — not off the seed the Param kept from when the
/// constraint was added.  Resize the ellipse and the next System must scale the t column by the
/// new size, or a tangency that converges at one size stalls at ten times it.
#[test]
fn a_rim_parameters_scale_is_a_fact_about_the_compile() {
    let (mut sk, e) = with_ellipse();
    let p = sk.point(11.0, 9.0, false, "p");
    sk.add(Constraint::point_on_ellipse(&sk, EntRef::point(p), EntRef::ellipse(e)));
    let t = sk.constraints[0].parametric_contact().expect("a rim contact").1;
    let col = |sk: &Sketch| {
        let sys = gcs_core::system::System::new(sk);
        let i = sys.free.iter().position(|&f| f == t as i32).expect("t is free");
        sys.col_scale[i]
    };
    let before = col(&sk);
    // ten times the size, without touching the constraint the seed lives on
    let (mx, my) = sk.point_xy(sk.ellipses[e].major as usize);
    sk.params[sk.points[sk.ellipses[e].major as usize].x as usize].value = 10.0 * (mx - 10.0) + 10.0;
    let _ = my;
    sk.params[sk.ellipses[e].minor as usize].value = 30.0;
    let after = col(&sk);
    assert!(after > 5.0 * before, "scale stayed at the seed: {before} -> {after}");
}

/// Every pair whose second entity has to be *swept* to be measured goes through one arm, and
/// that arm sits above the ones that reach for a centre and a radius.  A line against a curve
/// used to fall past it into the round arm and ask a spline for a centre it does not have.
#[test]
fn a_swept_kind_is_measured_against_a_line() {
    let (mut sk, e) = with_ellipse();
    let a = sk.point(0.0, 30.0, false, "a");
    let b = sk.point(40.0, 30.0, false, "b");
    let l = EntRef::line(sk.line(a, b));
    // the rim's top is at y = 8, so a level line at y = 30 is 22 away
    let d = gcs_core::model::distance_between(&sk, l, EntRef::ellipse(e));
    assert!((d - 22.0).abs() < 0.05, "line to rim, got {d}");
    let ctrl: Vec<usize> = (0..4)
        .map(|i| sk.point(4.0 * i as f64, 50.0 + 2.0 * (i % 2) as f64, false, "k"))
        .collect();
    let sp = EntRef::spline(sk.spline(&ctrl).expect("four control points make a cubic"));
    let d = gcs_core::model::distance_between(&sk, l, sp);
    assert!(d.is_finite() && d > 0.0, "line to curve, got {d}");
}
