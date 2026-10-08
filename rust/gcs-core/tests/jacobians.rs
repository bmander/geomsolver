//! Every constraint's analytic Jacobian must agree with finite differences at random points.
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::examples;
use gcs_core::expr::Expr;
use gcs_core::fdcheck::{check_constraint, check_sketch};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::rng::Rng;
use gcs_core::system::System;

/// A sketch carrying one of every kind of entity, plus one of every constraint type on them.
fn all_constraints(seed: u32) -> Sketch {
    let mut rng = Rng::new(seed + 1);
    let mut sk = Sketch::new();
    let r = |rng: &mut Rng| rng.uniform(-10.0, 10.0);
    let (x, y) = (r(&mut rng), r(&mut rng));
    let p = sk.point(x, y, false, "p");
    let (x, y) = (r(&mut rng), r(&mut rng));
    let q = sk.point(x, y, false, "q");
    let pt = |sk: &mut Sketch, rng: &mut Rng| {
        let (x, y) = (rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0));
        sk.point(x, y, false, "z")
    };
    let (a1, b1) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let l1 = sk.line(a1, b1);
    let (a2, b2) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let l2 = sk.line(a2, b2);
    let cc1 = pt(&mut sk, &mut rng);
    let c1 = sk.circle(cc1, rng.uniform(1.0, 11.0), "c1");
    let cc2 = pt(&mut sk, &mut rng);
    let c2 = sk.circle(cc2, rng.uniform(1.0, 11.0), "c2");
    let (ac, as_, ae) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let arc = sk.arc(ac, as_, ae, "a");
    // six control points: three spans, so a contact is checked on an interior span too
    let ctrl: Vec<usize> = (0..6).map(|_| pt(&mut sk, &mut rng)).collect();
    let sp = sk.spline(&ctrl).unwrap();
    // the same polygon weighted: the rational basis (`curve::weigh`) under every contact kernel
    let spw = sk.spline_weighted(&ctrl, None, Some(vec![1.0, 2.5, 0.6, 1.8, 0.9, 1.3])).unwrap();
    let fr = sk.fixed_plane(gcs_core::plane::Basis::page(), "f");
    // a plane held where it stands and one over two free axes, with `p` and `q` as images on
    // them — so a projection's planes are inferred exactly as a document's would be
    let r3 = |rng: &mut Rng| [0; 3].map(|_| rng.uniform(-1.0, 1.0));
    let held = gcs_core::plane::Basis { o: r3(&mut rng), ..gcs_core::plane::Basis::page() };
    let pa = sk.fixed_plane(held, "pa");
    // a free plane, its axes knocked off the unit sphere and its place moved — so the lift is
    // checked where a solve passes, not only where it rests
    let free = |sk: &mut Sketch, rng: &mut Rng, name: &str| {
        let (u, v) = (sk.axis(r3(rng), &format!("{name}.u")), sk.axis(r3(rng), &format!("{name}.v")));
        let pl = sk.plane(u, v, r3(rng), name);
        for k in sk.axes[u].d.into_iter().chain(sk.axes[v].d) {
            sk.params[k as usize].value = rng.uniform(-1.5, 1.5);
        }
        pl
    };
    let pb = free(&mut sk, &mut rng, "pb");
    sk.set_plane(p, Some(pa));
    sk.set_plane(q, Some(pb));
    let pc = free(&mut sk, &mut rng, "pc");
    let (lc, lq) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    sk.set_plane(lc, Some(pc));
    sk.set_plane(lq, Some(pb));
    sk.lift_point(lc).unwrap();
    sk.lift_point(lq).unwrap();
    // a point in space, its own lift
    let sp3 = sk.point3(r3(&mut rng).map(|x| 10.0 * x), false, "s3");
    // and entities drawn in those views for the relations in space to read: a line across two
    // views and one inside a third, and a circle in a solved view and one in a stated view
    let m1 = sk.line(p, lc);
    let m2 = sk.line(q, lq);
    let (cpc, cpb) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    sk.set_plane(cpc, Some(pc));
    sk.set_plane(cpb, Some(pa));
    let kc = sk.circle(cpc, rng.uniform(1.0, 11.0), "kc");
    let kb = sk.circle(cpb, rng.uniform(1.0, 11.0), "kb");
    let (me1, me2) = (EntRef::line(m1), EntRef::line(m2));
    // and two axes, their directions knocked off the unit sphere — the first placed, so its
    // point is free and read, the second read only as a direction
    let dir = |rng: &mut Rng| [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
    let ra = sk.axis(dir(&mut rng), "ra");
    let rb = sk.axis(dir(&mut rng), "rb");
    sk.place_axis(ra);
    for k in sk.axes[ra].d.into_iter().chain(sk.axes[ra].a).chain(sk.axes[rb].d) {
        sk.params[k as usize].value = rng.uniform(-1.5, 1.5);
    }
    let (axis_a, axis_b) = (EntRef::new(gcs_core::model::EntKind::Axis, ra), EntRef::new(gcs_core::model::EntKind::Axis, rb));
    // a line drawn in the free plane for a mirror to be taken in
    let lc2 = pt(&mut sk, &mut rng);
    sk.set_plane(lc2, Some(pc));
    let fold_line = EntRef::line(sk.line(lc, lc2));
    let (pce, pbe, pae) = (EntRef::plane(pc), EntRef::plane(pb), EntRef::plane(pa));
    // and a held plane of its own for a projection to a free one, and one between two held
    let held = gcs_core::plane::Basis {
        u: [0.0, 0.6, 0.8], v: [1.0, 0.0, 0.0], o: r3(&mut rng),
    };
    let pd = sk.fixed_plane(held, "pd");
    let dp = pt(&mut sk, &mut rng);
    sk.set_plane(dp, Some(pd));
    let s3 = EntRef::point(sp3);
    // and a line drawn in the held view `p` is in, for an ordinate along a line of its own view;
    // `cpb` is a second point of that view
    let (ia, ib) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    sk.set_plane(ia, Some(pa));
    sk.set_plane(ib, Some(pa));
    let in_pa = EntRef::line(sk.line(ia, ib));
    let (pa_u, pa_v) = (EntRef::new(gcs_core::model::EntKind::Axis, sk.planes[pa].u as usize),
        EntRef::new(gcs_core::model::EntKind::Axis, sk.planes[pa].v as usize));
    let cpbe = EntRef::point(cpb);
    // and the free plane's origin and axes, for an ordinate along its frame from its origin
    let pc_o = EntRef::point(sk.planes[pc].origin as usize);
    let (pc_u, pc_v) = (EntRef::new(gcs_core::model::EntKind::Axis, sk.planes[pc].u as usize),
        EntRef::new(gcs_core::model::EntKind::Axis, sk.planes[pc].v as usize));
    let worded = |mut c: Constraint, w: &str| {
        let i = c.kind.word_slot().expect("an ordinate");
        c.args[i] = Arg::Str(w.to_string());
        c
    };

    let (pe, qe) = (EntRef::point(p), EntRef::point(q));
    let (le1, le2) = (EntRef::line(l1), EntRef::line(l2));
    let (ce1, ce2, ae) = (EntRef::circle(c1), EntRef::circle(c2), EntRef::arc(arc));
    let spe = EntRef::spline(sp);
    let spwe = EntRef::spline(spw);
    let e = |x: EntRef| Arg::Ent(x);
    // a dimension written as an expression, standing at the number it is worth until it is
    // evaluated: `Sketch::add` binds it to the free variable it names
    let fx = |kind: CKind, mut args: Vec<Arg>, text: &str, value: f64| {
        args.push(Arg::Expr(Expr::new(text, value)));
        Constraint::new(kind, args)
    };
    // the same constraint with a side named — the signed kernel rather than the magnitude one
    let sided = |mut c: Constraint, w: &str| {
        let i = c.kind.side_slot().expect("a type with a side");
        c.args[i] = Arg::Str(w.to_string());
        c
    };
    let _ = fr;
    let cs = vec![
        // an ordinate in every form its operands give it: along its view's own axes (the run
        // and the rise), along a line drawn there, along an axis and a line in space, and along a
        // plane's normal — and its zero over the same
        Constraint::ordinate(pe, cpbe, pa_u, -2.5),
        Constraint::ordinate(cpbe, pe, pa_v, 1.5),
        Constraint::ordinate(pe, cpbe, in_pa, 2.0),
        Constraint::ordinate(qe, s3, axis_b, 1.2),
        Constraint::ordinate(EntRef::point(lc), qe, me1, -0.7),
        Constraint::ordinate(qe, EntRef::point(lc), pce, 1.2),
        Constraint::ordinate(s3, pe, pae, -0.8),
        Constraint::level(pe, cpbe, pa_u),
        Constraint::level(pe, cpbe, pa_v),
        Constraint::level(pe, cpbe, in_pa),
        Constraint::level(qe, s3, axis_a),
        Constraint::level(qe, EntRef::point(lc), pce),
        worded(Constraint::ordinate(pc_o, qe, pc_u, 1.5), "u"),
        worded(Constraint::ordinate(pc_o, s3, pc_v, -0.5), "v"),
        worded(Constraint::level(pc_o, qe, pc_v), "v"),
        {
            // the run said the other way, `along: left`
            let mut c = Constraint::ordinate(pe, cpbe, pa_u, 2.5);
            c.args[4] = Arg::Str("left".into());
            c
        },
        Constraint::coincident(pe, qe),
        Constraint::distance(pe, qe, 3.0),
        Constraint::new(CKind::Midpoint, vec![e(pe), e(le1)]),
        Constraint::drag_target(pe, 1.0, 2.0, 0.3),
        // a point in space, seen by an eye turned three quarters round and lifted
        Constraint::drag_seen(s3, 1.0, 2.0, 0.3, 0.6, 0.4),
        Constraint::one_line(CKind::Horizontal, le1),
        Constraint::one_line(CKind::Vertical, le1),
        Constraint::two_line(CKind::Parallel, le1, le2),
        Constraint::two_line(CKind::Perpendicular, le1, le2),
        Constraint::new(CKind::Angle, vec![e(le1), e(le2), Arg::Num(0.7)]),
        // an angle stated as another, turning the same way and the other (`sense: cw`)
        Constraint::new(CKind::EqualAngle, vec![e(le1), e(le2), e(le2), e(me1)]),
        Constraint::new(
            CKind::EqualAngle,
            vec![e(le2), e(le1), e(le1), e(me2), Arg::Str("cw".into())],
        ),
        // an arc's length along itself
        Constraint::new(CKind::ArcLength, vec![e(ae), Arg::Num(7.5)]),
        // a spline's whole length, polynomial and rational
        Constraint::new(CKind::SplineLength, vec![e(spe), Arg::Num(30.0)]),
        Constraint::new(CKind::SplineLength, vec![e(spwe), Arg::Num(30.0)]),
        // two neighbouring spans equally long: the first pair, and the weighted last
        Constraint::new(CKind::SplineGauge, vec![e(spe), Arg::Int(3)]),
        Constraint::new(CKind::SplineGauge, vec![e(spwe), Arg::Int(4)]),
        Constraint::two_line(CKind::EqualLength, le1, le2),
        Constraint::new(CKind::PointOnLine, vec![e(pe), e(le1)]),
        Constraint::point_on_circle(pe, ce1, false),
        Constraint::point_on_circle(pe, ae, false),
        Constraint::radius(ce1, 2.0),
        Constraint::new(CKind::EqualRadius, vec![e(ce1), e(ae)]),
        Constraint::tangent_line_circle(&sk, le1, ce1, None),
        Constraint::tangent_line_circle(&sk, le1, ce1, Some(-1)),
        Constraint::new(CKind::TangentCircleCircle, vec![e(ce1), e(ce2), Arg::Bool(true)]),
        Constraint::new(CKind::TangentCircleCircle, vec![e(ce1), e(ce2), Arg::Bool(false)]),
        Constraint::new(CKind::TangentArcLine, vec![e(ae), e(le1), Arg::Str("start".into())]),
        Constraint::new(CKind::TangentArcLine, vec![e(ae), e(le2), Arg::Str("end".into())]),
        Constraint::new(CKind::TangentLineCircleAt, vec![e(le1), e(ce1), Arg::Str("p1".into())]),
        Constraint::new(CKind::TangentLineCircleAt, vec![e(le2), e(ce2), Arg::Str("p2".into())]),
        Constraint::new(CKind::Symmetric, vec![e(pe), e(qe), e(le1)]),
        // a distance from a line, both ways it can be written: naming no side, which is the
        // magnitude form (issue #48, item 4), and naming one, which is the signed form
        Constraint::new(CKind::ParallelDistance, vec![e(le1), e(le2), Arg::Num(4.0)]),
        Constraint::new(CKind::PointLineDistance, vec![e(pe), e(le1), Arg::Num(4.0)]),
        Constraint::new(
            CKind::ParallelDistance,
            vec![e(le2), e(le1), Arg::Num(4.0), Arg::Str("right".into())],
        ),
        Constraint::new(
            CKind::PointLineDistance,
            vec![e(qe), e(le1), Arg::Num(4.0), Arg::Str("left".into())],
        ),
        Constraint::new(CKind::AnnularDistance, vec![e(ce1), e(ae), Arg::Num(1.5)]),
        Constraint::point_on_spline(&sk, pe, spe),
        Constraint::point_on_spline(&sk, qe, spe),
        Constraint::spline_tangent_line(&sk, spe, le1),
        Constraint::spline_tangent_line(&sk, spe, le2),
        Constraint::spline_curvature(&sk, spe, ce1),
        Constraint::spline_curvature(&sk, spe, ae),
        Constraint::point_on_spline(&sk, qe, spwe),
        Constraint::spline_tangent_line(&sk, spwe, le2),
        Constraint::spline_curvature(&sk, spwe, ce2),
        // a projection between two held planes, and the lift over a free plane and a held one
        Constraint::project(&sk, EntRef::point(dp), pe).expect("two images on two planes that fold"),
        Constraint::new(CKind::Lift, vec![e(EntRef::point(lc)), e(pce)]),
        Constraint::new(CKind::Lift, vec![e(EntRef::point(dp)), e(EntRef::plane(pd))]),
        // the relations in space, over the hidden points `Sketch::add` mints for them — the two
        // that read a plane once over the solved view and once over a stated one
        Constraint::new(CKind::Coincident3, vec![e(pe), e(EntRef::point(lq))]),
        Constraint::new(CKind::Distance3, vec![e(pe), e(EntRef::point(lc)), Arg::Num(3.0)]),
        Constraint::new(CKind::Distance3, vec![e(s3), e(EntRef::point(lq)), Arg::Num(3.0)]),
        Constraint::new(CKind::PointLine3, vec![e(qe), e(me1), Arg::Num(2.0)]),
        Constraint::in_space(&sk, CKind::LineLine3, &[me1, me2], Some(1.5)).unwrap(),
        Constraint::new(CKind::LineLine3, vec![e(me2), e(me1), Arg::Num(1.5), Arg::Int(-1)]),
        Constraint::new(CKind::Angle3, vec![e(me1), e(me2), Arg::Num(0.9)]),
        Constraint::two_line(CKind::Perpendicular3, me1, me2),
        Constraint::two_line(CKind::Parallel3, me1, me2),
        Constraint::new(CKind::PointOnPlane, vec![e(qe), e(pce)]),
        Constraint::new(CKind::PointOnPlane, vec![e(EntRef::point(lc)), e(pae)]),
        Constraint::new(CKind::PointOnPlane, vec![e(s3), e(pbe)]),
        // a plane tangent at a point to the cone about a line (#145)
        Constraint::new(CKind::TangentPlaneCone, vec![e(pce), e(me1), e(qe)]),
        Constraint::new(CKind::PointOnCircle3, vec![e(qe), e(EntRef::circle(kc))]),
        Constraint::new(CKind::PointOnCircle3, vec![e(pe), e(EntRef::circle(kb))]),
        // the rest of the words in space: a point on a line and true lengths, a point's
        // distance along a plane's normal and its ordinates, and a line on a plane — each over a
        // free plane and a held one
        Constraint::new(CKind::PointOnLine3, vec![e(qe), e(me1)]),
        Constraint::two_line(CKind::EqualLength3, me1, me2),
        Constraint::new(CKind::LineOnPlane, vec![e(me2), e(pce)]),
        Constraint::new(CKind::LineOnPlane, vec![e(me1), e(pae)]),
        // the midpoint and the mirror in a line, in space
        Constraint::new(CKind::Midpoint3, vec![e(qe), e(me1)]),
        Constraint::new(CKind::Symmetric3, vec![e(pe), e(qe), e(fold_line)]),
        // an axis's own two rows, a point on one, and the direction words over an axis and an axis or
        // a line, the axis handed to the line's kernels as the segment from the origin
        Constraint::new(CKind::AxisUnit, vec![e(axis_b)]),
        Constraint::new(CKind::AxisFoot, vec![e(axis_a)]),
        Constraint::new(CKind::PointOnAxis, vec![e(qe), e(axis_a)]),
        Constraint::new(CKind::Angle3, vec![e(axis_a), e(me1), Arg::Num(0.9)]),
        Constraint::two_line(CKind::Perpendicular3, axis_a, axis_b),
        Constraint::two_line(CKind::Parallel3, axis_b, me2),
        Constraint::two_line(CKind::Parallel3, me1, axis_a),
        // two axes on one line, either way round
        Constraint::new(CKind::AxisCoincident, vec![e(axis_a), e(axis_b)]),
        // a line lying on an axis
        Constraint::new(CKind::LineOnAxis, vec![e(me1), e(axis_a)]),
        // an axis and a plane: on it, along it and square to it, over a free plane and a held one;
        // and two planes a distance apart
        Constraint::new(CKind::AxisOnPlane, vec![e(axis_a), e(pce)]),
        Constraint::new(CKind::AxisParallelPlane, vec![e(axis_b), e(pce)]),
        Constraint::new(CKind::AxisPerpendicularPlane, vec![e(axis_b), e(pbe)]),
        Constraint::new(CKind::AxisPerpendicularPlane, vec![e(axis_a), e(pae)]),
        Constraint::new(CKind::PlaneDistance, vec![e(pbe), e(pce), Arg::Num(2.0)]),
        // two planes facing alike, over free planes and a held one
        Constraint::new(CKind::PlaneParallel, vec![e(pbe), e(pce)]),
        Constraint::new(CKind::PlaneParallel, vec![e(pae), e(pce)]),
        // a projection over a free plane
        Constraint::project(&sk, pe, qe).expect("two images on two planes that fold"),
        Constraint::new(CKind::ProjectSolved, vec![e(EntRef::point(dp)), e(EntRef::point(lc)), e(EntRef::plane(pd)), e(pce)]),
        // every dimension again with its number written in terms of a free variable, which is
        // an unknown of the sketch rather than a constant — one more column, and (m, c) where
        // the number was.  A different name each time, so no two of them are tied together, and
        // a scale, an offset and a sign among them so the map is not always the identity.
        fx(CKind::Distance, vec![e(pe), e(qe)], "u", 3.0),
        fx(CKind::Angle, vec![e(le1), e(le2)], "2 * v + 5", 0.7),
        fx(CKind::Radius, vec![e(ce1)], "w", 2.0),
        fx(CKind::ParallelDistance, vec![e(le1), e(le2)], "x / 2", 4.0),
        fx(CKind::PointLineDistance, vec![e(pe), e(le1)], "y", 4.0),
        // and their free twins with a side named, which is the fourth of the four kernels each
        // of these two types now has: signed or magnitude, stated or shared
        sided(fx(CKind::ParallelDistance, vec![e(le2), e(le1)], "s1", 4.0), "left"),
        sided(fx(CKind::PointLineDistance, vec![e(qe), e(le1)], "s2", 4.0), "right"),
        fx(CKind::AnnularDistance, vec![e(ce1), e(ae)], "z + 1", 1.5),
        fx(CKind::Ordinate, vec![e(pe), e(cpbe), e(pa_u)], "g", 2.5),
        fx(CKind::Ordinate, vec![e(pe), e(cpbe), e(pa_v)], "-k", -1.5),
        fx(CKind::Ordinate, vec![e(pe), e(cpbe), e(in_pa)], "2 * ol + 1", 2.0),
        {
            let mut c = fx(CKind::Ordinate, vec![e(pe), e(cpbe), e(pa_u)], "gl", 2.5);
            c.args[4] = Arg::Str("left".into());
            c
        },
        fx(CKind::Distance3, vec![e(pe), e(EntRef::point(lc))], "u3", 3.0),
        fx(CKind::PointLine3, vec![e(qe), e(me1)], "2 * v3 - 1", 2.0),
        fx(CKind::LineLine3, vec![e(me1), e(me2)], "w3 / 2", 1.5),
        {
            let mut c = fx(CKind::LineLine3, vec![e(me2), e(me1)], "x3", 1.5);
            c.args[3] = Arg::Int(-1);
            c
        },
        fx(CKind::Angle3, vec![e(me1), e(me2)], "t3 + 0.5", 0.9),
        fx(CKind::Angle3, vec![e(axis_b), e(me2)], "t3r + 0.5", 0.9),
        fx(CKind::Ordinate, vec![e(qe), e(EntRef::point(lc)), e(pce)], "n3 + 1", 1.2),
        fx(CKind::Ordinate, vec![e(s3), e(pe), e(pae)], "-2 * n4", 0.5),
        fx(CKind::Ordinate, vec![e(s3), e(qe), e(axis_b)], "o3 + 1", 1.2),
        fx(CKind::Ordinate, vec![e(qe), e(EntRef::point(lc)), e(me2)], "-2 * o4", 0.5),
        worded(fx(CKind::Ordinate, vec![e(pc_o), e(qe), e(pc_u)], "fu + 1", 1.0), "u"),
        worded(fx(CKind::Ordinate, vec![e(pc_o), e(s3), e(pc_v)], "2 * fv", 1.0), "v"),
        fx(CKind::PlaneDistance, vec![e(pae), e(pce)], "3 * pd3", 0.5),
        fx(CKind::ArcLength, vec![e(ae)], "3 * al + 1", 7.5),
        fx(CKind::SplineLength, vec![e(spwe)], "2 * sl + 3", 30.0),
    ];
    // the two intrinsic PointOnCircle constraints the arc brought with it stay in the sketch
    sk.constraints.clear();
    for c in cs {
        sk.add(c);
    }
    sk
}

#[test]
fn every_constraint_jacobian_agrees_with_finite_differences() {
    for seed in 0..5u32 {
        let sk = all_constraints(seed);
        for c in &sk.constraints {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("seed {seed}: {e}"));
            assert!(err.is_finite());
        }
    }
}

#[test]
fn every_constraint_type_has_a_kernel_and_all_are_covered() {
    let sk = all_constraints(0);
    // a kind whose kernel is built at its operands' width has no static one
    let used: std::collections::BTreeSet<usize> =
        sk.constraints.iter().filter(|c| !c.kind.built()).map(|c| c.kernel_id()).collect();
    assert_eq!(used.len(), gcs_core::kernels::N_KERNELS);
}

#[test]
fn a_param_used_twice_in_one_constraint_sums_its_contributions() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(1.0, 0.2, false, "b");
    let c = sk.point(2.0, 1.0, false, "c");
    let l1 = sk.line(a, b);
    let l2 = sk.line(b, c);
    sk.add(Constraint::two_line(CKind::Perpendicular, EntRef::line(l1), EntRef::line(l2)));
    sk.add(Constraint::two_line(CKind::EqualLength, EntRef::line(l1), EntRef::line(l2)));
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(0.3)],
    ));
    check_sketch(&sk, 1e-6, 1e-6).unwrap();
}

#[test]
fn example_sketch_jacobians() {
    for name in examples::EXAMPLES {
        check_sketch(&examples::example(name).unwrap(), 1e-6, 1e-6).unwrap();
    }
}

#[test]
fn fixed_params_are_dropped_from_the_jacobian() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let mut sys = System::new(&sk);
    // the corner's two coordinates are held, and so is everything `use std` brings
    assert_eq!(sys.n_free, sk.params.iter().filter(|p| !p.fixed).count());
    assert!(sys.n_free < sk.params.len() - 2);
    let z = sys.z0(&sk);
    let j = sys.jacobian_dense(&z);
    assert_eq!((j.rows, j.cols), (sk.n_residuals(), sys.n_free));
}

#[test]
fn system_blocks_cover_every_constraint_once() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let mut s = System::new(&sk);
    assert_eq!(s.blocks.iter().map(|b| b.count).sum::<usize>(), sk.constraints.len());
    assert_eq!(s.n_res, sk.n_residuals());
    let z = s.z0(&sk);
    let r = s.residuals(&z);
    for c in &sk.constraints {
        let off = s.row_of(c.id).expect("compiled constraint has a row");
        let expect = c.residual(&sk, &c.local_values(&sk));
        for (i, v) in expect.iter().enumerate() {
            assert!((r[off + i] - v).abs() < 1e-12);
        }
    }
}

/// Every type that states a number can have that number written in terms of a free variable, and
/// its free twin is the same kernel with one more column: the unknown, where the constant was.
/// The row count is the same on purpose — a binding sizes its buffers off the type's kernel, and
/// a free constraint is still one of that type.
#[test]
fn every_dimension_can_be_written_free() {
    use gcs_core::kernels;
    for k in gcs_core::constraints::ALL_KINDS {
        // a built kernel's free twin is built beside it, at the same width plus the unknown's
        if k.built() {
            continue;
        }
        assert_eq!(k.has_dimension(), k.free_kernel().is_some(), "{k:?}");
        let Some(free) = k.free_kernel() else { continue };
        let (stated, free) = (kernels::kernel(k.kernel()), kernels::kernel(free));
        assert_eq!(free.n_par, stated.n_par + 1, "{k:?}");
        // the affine map, m and c
        assert_eq!(free.n_const, 2, "{k:?}");
        assert_eq!(free.n_res, stated.n_res, "{k:?}");
        assert_eq!(free.degree, stated.degree, "{k:?}");
    }
}
