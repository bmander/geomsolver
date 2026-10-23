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
    // a point drawn in a spatial frame's three coordinates, the frame free (#383): its lift's
    // `ChartLift` row
    let pf = free(&mut sk, &mut rng, "pf");
    sk.spatial_chart(pf);
    let cf = sk.point3(r3(&mut rng).map(|x| 5.0 * x), false, "cf");
    sk.draw_in_chart(cf, pf);
    sk.lift_point(cf).unwrap();
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
    // and an arc drawn in the solved view, for a point at a fraction of its turn in space
    let (kas, kae) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    sk.set_plane(kas, Some(pc));
    sk.set_plane(kae, Some(pc));
    let ka = EntRef::arc(sk.arc(cpc, kas, kae, "ka"));
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
    let (pce, pbe, pae) = (EntRef::frame(pc), EntRef::frame(pb), EntRef::frame(pa));
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
    let (pa_u, pa_v) = (EntRef::new(gcs_core::model::EntKind::Axis, sk.frames[pa].u as usize),
        EntRef::new(gcs_core::model::EntKind::Axis, sk.frames[pa].v as usize));
    let cpbe = EntRef::point(cpb);
    // and the free plane's origin and axes, for an ordinate along its frame from its origin
    let pc_o = EntRef::point(sk.frames[pc].origin as usize);
    let (pc_u, pc_v) = (EntRef::new(gcs_core::model::EntKind::Axis, sk.frames[pc].u as usize),
        EntRef::new(gcs_core::model::EntKind::Axis, sk.frames[pc].v as usize));
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
        Constraint::drag_seen(s3, 1.0, 2.0, 0.3, (0.6, 0.4), 1.5),
        Constraint::one_line(CKind::Horizontal, le1),
        Constraint::one_line(CKind::Vertical, le1),
        Constraint::two_line(CKind::Parallel, le1, le2),
        Constraint::two_line(CKind::Perpendicular, le1, le2),
        // square the way it says, `angle`'s kernel at a quarter turn either way (#497)
        Constraint::new(CKind::Perpendicular, vec![e(le1), e(le2), Arg::Str("ccw".into())]),
        Constraint::new(CKind::Perpendicular, vec![e(le2), e(le1), Arg::Str("cw".into())]),
        // alike the way it says, `angle`'s kernel at no turn or a half (#535)
        Constraint::new(CKind::Parallel, vec![e(le1), e(le2), Arg::Str("same".into())]),
        Constraint::new(CKind::Parallel, vec![e(le2), e(le1), Arg::Str("opposite".into())]),
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
        // a point carried by a map (#357): the affine map over three pairs, the similarity over
        // two, keeping its turn and mirrored — the source points columns, so rational rows
        {
            let pts = [a1, b1, a2, b2, cc1, cc2, ac, as_].map(EntRef::point);
            Constraint::new(CKind::Mapped, pts.iter().map(|&x| e(x)).collect())
        },
        {
            let pts = [a1, b1, a2, b2, cc1, cc2].map(EntRef::point);
            let mut args: Vec<Arg> = pts.iter().map(|&x| e(x)).collect();
            args.push(Arg::Bool(false));
            Constraint::new(CKind::MappedSimilar, args)
        },
        {
            let pts = [b2, as_, cc2, a1, ac, b1].map(EntRef::point);
            let mut args: Vec<Arg> = pts.iter().map(|&x| e(x)).collect();
            args.push(Arg::Bool(true));
            Constraint::new(CKind::MappedSimilar, args)
        },
        // and a circle carried by one, its centre and its radius
        {
            let mut args = vec![e(ce1), e(ce2)];
            args.extend([a1, b1, a2, b2].map(|x| e(EntRef::point(x))));
            args.push(Arg::Bool(true));
            Constraint::new(CKind::MappedCircle, args)
        },
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
        // a point at a fraction of a line's run and of an arc's turn (#248), and in space —
        // across views, and a point in space of its own
        Constraint::new(CKind::PointAlongLine, vec![e(pe), e(le1), Arg::Num(0.3)]),
        Constraint::new(CKind::PointAlongLine, vec![e(qe), e(le2), Arg::Num(1.4)]),
        Constraint::new(CKind::PointAlongArc, vec![e(pe), e(ae), Arg::Num(0.4)]),
        Constraint::new(CKind::PointAlongLine3, vec![e(qe), e(me1), Arg::Num(0.6)]),
        Constraint::new(CKind::PointAlongLine3, vec![e(s3), e(me2), Arg::Num(-0.2)]),
        Constraint::new(CKind::PointAlongArc3, vec![e(qe), e(ka), Arg::Num(0.7)]),
        Constraint::new(CKind::PointAlongArc3, vec![e(s3), e(ka), Arg::Num(0.2)]),
        // a line out of a round's plane touching it at a point of it (#363 S4): at an arc's end,
        // and at a point of a circle drawn in a stated view
        Constraint::new(CKind::TangentCircleAt3, vec![e(ka), e(me1), e(EntRef::point(kas))]),
        Constraint::new(CKind::TangentCircleAt3, vec![e(EntRef::circle(kb)), e(me2), e(qe)]),
        Constraint::point_on_spline(&sk, pe, spe),
        Constraint::point_on_spline(&sk, qe, spe),
        Constraint::spline_tangent_line(&sk, spe, le1),
        Constraint::spline_tangent_line(&sk, spe, le2),
        Constraint::spline_curvature(&sk, spe, ce1),
        Constraint::spline_curvature(&sk, spe, ae),
        Constraint::point_on_spline(&sk, qe, spwe),
        Constraint::spline_tangent_line(&sk, spwe, le2),
        Constraint::spline_curvature(&sk, spwe, ce2),
        // the tangent at a point a contact holds on the curve (#251), its rate there held along
        // a line (#365): its place the contact's
        Constraint::new(
            CKind::Rate,
            vec![e(spe), e(le2), e(pe), Arg::Seed { value: 0.5, pinned: false },
                 Arg::Str("t".into()), Arg::Str("tangent".into())],
        ),
        Constraint::new(
            CKind::Rate,
            vec![e(spwe), e(le1), e(qe), Arg::Seed { value: 0.5, pinned: false },
                 Arg::Str("t".into()), Arg::Str("parallel".into())],
        ),
        Constraint::new(CKind::TangentCircleAt, vec![e(ae), e(le1), e(pe)]),
        Constraint::new(CKind::TangentCircleAt, vec![e(ce1), e(le2), e(qe)]),
        // a projection between two held planes, and the lift over a free plane and a held one
        Constraint::project(&sk, EntRef::point(dp), pe).expect("two images on two planes that fold"),
        Constraint::new(CKind::Lift, vec![e(EntRef::point(lc)), e(pce)]),
        Constraint::new(CKind::Lift, vec![e(EntRef::point(dp)), e(EntRef::frame(pd))]),
        Constraint::new(CKind::ChartLift, vec![e(EntRef::point(cf)), e(EntRef::frame(pf))]),
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
        // and each the way it says, the turn onto the second its only root (#535)
        Constraint::new(CKind::Parallel3, vec![e(me1), e(me2), Arg::Str("same".into())]),
        Constraint::new(CKind::Parallel3, vec![e(axis_b), e(me2), Arg::Str("opposite".into())]),
        Constraint::new(CKind::AxisCoincident, vec![e(axis_a), e(axis_b), Arg::Str("same".into())]),
        Constraint::new(CKind::AxisCoincident,
            vec![e(axis_b), e(axis_a), Arg::Str("opposite".into())]),
        Constraint::new(CKind::LineOnAxis, vec![e(me1), e(axis_a), Arg::Str("same".into())]),
        Constraint::new(CKind::LineOnAxis, vec![e(me2), e(axis_a), Arg::Str("opposite".into())]),
        // the turn about an axis or a line, from a line or an axis or a line drawn in a plane, and
        // the sided perpendicular, a quarter of it (#558)
        Constraint::new(CKind::AngleAbout,
            vec![e(me1), e(axis_b), e(axis_a), Arg::Num(0.7), Arg::Str(String::new())]),
        Constraint::new(CKind::AngleAbout,
            vec![e(fold_line), e(me2), e(me1), Arg::Num(2.5), Arg::Str("cw".into())]),
        Constraint::new(CKind::PerpendicularAbout,
            vec![e(axis_a), e(me1), e(axis_b), Arg::Str("ccw".into())]),
        Constraint::new(CKind::PerpendicularAbout,
            vec![e(me2), e(in_pa), e(me1), Arg::Str("cw".into())]),
        // an axis and a plane: on it, along it and square to it, over a free plane and a held one;
        // and two planes a distance apart
        Constraint::new(CKind::AxisOnPlane, vec![e(axis_a), e(pce)]),
        Constraint::new(CKind::AxisParallelPlane, vec![e(axis_b), e(pce)]),
        Constraint::new(CKind::AxisPerpendicularPlane, vec![e(axis_b), e(pbe)]),
        Constraint::new(CKind::AxisPerpendicularPlane, vec![e(axis_a), e(pae)]),
        // a drawn line square to a plane, over a free plane and a held one
        Constraint::new(CKind::LinePerpendicularPlane, vec![e(me1), e(pce)]),
        Constraint::new(CKind::LinePerpendicularPlane, vec![e(me2), e(pae)]),
        Constraint::new(CKind::PlaneDistance, vec![e(pbe), e(pce), Arg::Num(2.0)]),
        // two planes facing alike, over free planes and a held one
        Constraint::new(CKind::PlaneParallel, vec![e(pbe), e(pce)]),
        Constraint::new(CKind::PlaneParallel, vec![e(pae), e(pce)]),
        // a projection over a free plane
        Constraint::project(&sk, pe, qe).expect("two images on two planes that fold"),
        Constraint::new(CKind::ProjectSolved, vec![e(EntRef::point(dp)), e(EntRef::point(lc)), e(EntRef::frame(pd)), e(pce)]),
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
        // the turn about an axis, its sense turning the map (#558)
        {
            let mut c = fx(CKind::AngleAbout, vec![e(me1), e(axis_b), e(axis_a)], "ta + 0.5", 0.7);
            c.args.push(Arg::Str("cw".into()));
            c
        },
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
    // a kind whose kernel is built at its operands' width has no static one — but a rate of a
    // point on a spline runs the spline's tangent kernel (#365)
    let used: std::collections::BTreeSet<usize> = sk.constraints.iter()
        .filter_map(|c| match c.kernel_key(&sk) {
            gcs_core::kernels::KernelKey::Static(id) => Some(id),
            _ => None,
        })
        .collect();
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
        // the affine map, m and c — and a skew distance's pin beside them, as its stated kernel
        // carries it beside its number (#217)
        let pin = usize::from(k == gcs_core::constraints::CKind::LineLine3);
        assert_eq!(free.n_const, 2 + pin, "{k:?}");
        assert_eq!(stated.n_const, 1 + pin, "{k:?}");
        assert_eq!(free.n_res, stated.n_res, "{k:?}");
        assert_eq!(free.degree, stated.degree, "{k:?}");
    }
}

/// A row whose value is the output of a linear solve (#510 P5): the adjoint gradient of a braced
/// structure's output, in every node's coordinates and every bar's area, against central
/// differences of the value — in the plane and in space, stated and with its number an unknown,
/// and two instances in one block, as `System` evaluates a block of rows built at one width.
#[test]
fn a_linear_interior_agrees_with_finite_differences() {
    interior_agrees(crate::interior::braced, 40, 1e-6);
}

/// The same of a frame (#510 S0): beams rigid at their ends, whose rows read their ends' turns
/// and their section's four numbers — the adjoint gradient in every node's coordinates and every
/// section's area, second moments and torsion constant, its rates in the beam's axes carried.
/// A frame's normal equations square a stiffer conditioning than a truss's, so its value is
/// good to fewer figures and the difference is taken over a longer step.
#[test]
fn a_frames_interior_agrees_with_finite_differences() {
    interior_agrees(crate::interior::framed, 60, 1e-4);
}

/// The same of a frame whose sections are read over two unknowns (#510 S1, `Sizing`): each
/// beam's area, second moments and torsion constant a function of them — products, powers, a
/// constant — and its loads spread along it, its body load growing with the area the unknowns
/// size.  The adjoint gradient carried through each tape to the unknowns, against central
/// differences, in the plane and in space.
#[test]
fn a_sized_frames_interior_agrees_with_finite_differences() {
    use gcs_core::fdcheck::fd_jacobian;
    use gcs_core::interior::Sizing;
    let vars = ["u".to_string(), "w".to_string()];
    let tape = |text: &str| {
        let ast = gcs_core::expr::parse_in(text, gcs_core::units::Units::default()).unwrap();
        gcs_core::tape::Tape::compile(&ast.body, &vars).unwrap().flat
    };
    for seed in 0..4u32 {
        let mut rng = Rng::new(seed + 80);
        for dim in [2, 3] {
            let (s, x) = crate::interior::framed(dim, 5 + seed as usize, &mut rng);
            let sections = (0..s.beams.len()).map(|i| {
                let k = 1.0 + 0.1 * i as f64;
                [tape(&format!("{k} * u * w + 1")), tape(&format!("u^2 / {k} + 0.5")),
                 tape("w^3 / 4 + u"), tape(&format!("{k} + 0 * u"))]
            }).collect();
            let sizing = Sizing { unknowns: 2, sections };
            let (n_par, n_const) = s.widths_sized(false, Some(&sizing));
            let k = s.consts_sized(&[0.3], Some(&sizing));
            assert_eq!(n_const, k.len());
            let kn = gcs_core::system::build_kernel(
                &Sketch::new(), gcs_core::kernels::KernelKey::Interior { n_par, n_const });
            let direct = dim * s.nodes;
            let v = [&x[..direct], &[rng.uniform(1.0, 2.0), rng.uniform(1.0, 2.0)]].concat();
            assert_eq!(v.len(), n_par);
            let res = |w: &[f64]| {
                let mut r = vec![0.0; 1];
                (kn.res)(1, w, &k, &mut r);
                r
            };
            let jn = fd_jacobian(res, &v, 1e-4);
            let mut ja = vec![0.0; n_par];
            (kn.jac)(1, &v, &k, &mut ja);
            let scale = gcs_core::linalg::absmax(&jn.data);
            for (col, a) in ja.iter().enumerate() {
                let n = jn.data[col];
                assert!((a - n).abs() <= 1e-7 + 1e-6 * scale,
                        "seed {seed}, {dim}D: column {col}: {a} against {n}");
            }
        }
    }
}

/// **A frame's deflection, as the drawing states it** (#510 S1, `CKind::Deflection`): the row
/// over the joints' lifts and the unknowns the sections read — the library's four driving cases
/// (a section sized, a node placed, a bracket under a spread load, a tube's diameter under a
/// point and its weight), and an L frame placed by an unknown and sized by another under a force
/// spread along it and its weight — against finite differences, at the solution and knocked off
/// it.
#[test]
fn a_frames_deflection_agrees_with_finite_differences() {
    let l_frame = "\
unit mm
use std (horizontal)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
param h: Length hint(30mm)
param reach: Length hint(400mm)
in std.front {
  foot := point
  corner := point
  end := point hint((400, 600))
  fix((0mm, 0mm)) foot
  fix((0mm, 600mm)) corner
  corner horizontal end
  corner distance(reach) end
  post := line(foot, corner)
  arm := line(corner, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
frame := beam(post, arm, A: 20mm * h, Iy: h * (20mm)^3 / 12, Iz: 20mm * h^3 / 12,
              J: 0.2 * h * (20mm)^3) of steel
scenario s {
  fixed foot
  load arm 2kN along: down
  accelerate frame (0, 0, -9.81 * 1m / 1s^2)
  deflection(end, along: down) == 3mm
}
";
    let library = ["sized_cantilever", "reach_cantilever", "shelf_bracket", "portal_frame"]
        .map(|k| gcs_core::examples::source(k).unwrap());
    let mut rng = Rng::new(510);
    for src in library.into_iter().chain([l_frame]) {
        let e = crate::common::read(src);
        let mut sk = crate::common::solved(&e);
        for round in 0..3 {
            if round > 0 {
                for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                    p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
                }
            }
            let c = sk.constraints.iter().find(|c| c.kind == CKind::Deflection).unwrap();
            let err = check_constraint(&sk, c, 1e-5, 1e-6)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// **A measure of a structure** (`interior::Reads::Volume`, `Reads::Mass`): what it is made of,
/// with no solve read — bars and beams in the plane and in space, a sheet of quadratic triangles
/// and a block of quadratic tetrahedra, every node of both knocked about so their sides curve —
/// its own gradient in every column against central differences, stated and with its number an
/// unknown, two instances in one block.  A bar weighs nothing; a structure held by nothing still
/// has a measure.
#[test]
fn a_measure_agrees_with_finite_differences() {
    use gcs_core::interior::Reads;
    let mut rng = Rng::new(520);
    let mut cases = Vec::new();
    for dim in [2, 3] {
        cases.push(crate::interior::braced(dim, 7, &mut rng));
        cases.push(crate::interior::framed(dim, 6, &mut rng));
    }
    let mut knocked = |(s, mut x): (gcs_core::interior::Structure, Vec<f64>), by: f64| {
        for v in &mut x {
            *v += rng.uniform(-by, by);
        }
        (s, x)
    };
    let mut r = Rng::new(521);
    cases.push(knocked(crate::interior::sheet(3, 2, 4.0, 2.0, 0.2, false, &mut r), 0.04));
    cases.push(knocked(crate::interior::block([2, 1, 1], [2.0, 1.0, 1.0], 0.1, &mut r), 0.02));
    for (mut s, x) in cases {
        s.fixed.fill(false);
        s.beams.iter_mut().for_each(|b| b.rho = 3.0);
        s.tris.iter_mut().for_each(|t| t.rho = 2.5);
        s.tets.iter_mut().for_each(|t| t.rho = 1.5);
        for reads in [Reads::Volume, Reads::Mass] {
            s.reads = reads;
            let weighs = !(s.beams.is_empty() && s.tris.is_empty() && s.tets.is_empty());
            if reads == Reads::Mass && !weighs {
                assert_eq!(s.measure(&x), 0.0, "a bar weighs nothing");
                continue;
            }
            kernel_agrees(&s, &x, 1e-6, &mut rng, &format!("{reads:?}, {} nodes", s.nodes));
        }
    }
}

/// **A mass and a volume, as the drawing states them** (`CKind::Mass`, `CKind::Volume`): a
/// frame's row over its points' lifts and the unknowns its areas read — a section's breadth,
/// its area not affine in it, and an L's reach — and a prism's over its face's points' numbers
/// and its radii, its face's nodes moved with them (a round corner, a hole) — each stated of the
/// drawing — against finite differences, at the solution and knocked off it.
#[test]
fn a_measure_agrees_with_finite_differences_as_the_drawing_states_it() {
    let frame = "\
unit mm
use std (horizontal)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
param d: Length hint(30mm)
param reach: Length hint(400mm)
in std.front {
  foot := point
  corner := point
  end := point hint((400, 600))
  fix((0mm, 0mm)) foot
  fix((0mm, 600mm)) corner
  corner horizontal end
  corner distance(reach) end
  post := line(foot, corner)
  arm := line(corner, end)
}
tube := beam(post, arm, A: d^2 / 2, Iy: 0.05 * d^4, Iz: 0.05 * d^4, J: 0.1 * d^4) of steel
mass(3kg) tube
volume(200 * 1cm^3) arm
";
    let plate = "\
unit mm
use std (horizontal, vertical)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
param r: Length hint(9mm)
in std.front {
  a := point
  b := point hint((120, 0))
  c := point hint((120, 30))
  k := point hint((100, 30))
  d := point hint((100, 50))
  e := point
  o := point
  fix((0mm, 0mm)) a
  fix((0mm, 50mm)) e
  fix((40mm, 25mm)) o
  fix((120mm, 0mm)) b
  b vertical c
  b distance(30mm) c
  c horizontal k
  k vertical d
  low := line(a, b)
  end := line(b, c)
  round := radius(20mm) arc(center: k, start: c, end: d)
  top := line(d, e)
  back := line(e, a)
  hole := radius(r) circle(center: o) hint(r: 9)
  region := face(low, end, round, top, back, holes: hole)
}
plate := solid(region, from: -2mm, to: 2mm) of steel
mass(0.17584kg) plate
";
    let mut rng = Rng::new(522);
    for src in [frame, plate] {
        let mut sk = crate::common::solved(&crate::common::read(src));
        for round in 0..3 {
            if round > 0 {
                for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                    p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
                }
            }
            let rows: Vec<_> = sk.constraints.iter().filter(|c| c.kind.interior()).collect();
            assert!(!rows.is_empty());
            for c in rows {
                let v = c.local_values(&sk);
                let ja = c.jacobian(&sk, &v);
                let jn = gcs_core::fdcheck::fd_jacobian(|w| c.residual(&sk, w), &v, 1e-4);
                let scale = gcs_core::linalg::absmax(&jn.data);
                assert!(scale.is_finite() && scale > 0.0, "round {round}");
                for (col, (a, n)) in ja.iter().zip(&jn.data).enumerate() {
                    assert!((a - n).abs() <= 1e-6 * scale,
                            "{:?}, round {round}: column {col}: {a} against {n}", c.kind);
                }
            }
        }
    }
}

/// The interior's kernel over structure `s` at `x` — stated, and with its number an unknown, two
/// instances in one block, the second's nodes moved and its areas grown — against central
/// differences over `step`.
fn kernel_agrees(s: &gcs_core::interior::Structure, x: &[f64], step: f64, rng: &mut Rng,
                 what: &str) {
    use gcs_core::fdcheck::fd_jacobian;
    use gcs_core::kernels::KernelKey;
    for stated in [&[0.3][..], &[1.7, -0.2]] {
        let free = stated.len() == 2;
        let (n_par, n_const) = s.widths(free);
        let kn = gcs_core::system::build_kernel(
            &Sketch::new(), KernelKey::Interior { n_par, n_const });
        let mut one = x.to_vec();
        if free {
            one.push(rng.uniform(-1.0, 1.0));
        }
        let two: Vec<f64> = one.iter().map(|v| v * 1.1 + 0.05).collect();
        let v = [one, two].concat();
        let k = [s.consts(stated), s.consts(stated)].concat();
        let res = |w: &[f64]| {
            let mut r = vec![0.0; 2];
            (kn.res)(2, w, &k, &mut r);
            r
        };
        let jn = fd_jacobian(res, &v, step);
        let mut ja = vec![0.0; 2 * n_par];
        (kn.jac)(2, &v, &k, &mut ja);
        let scale = gcs_core::linalg::absmax(&jn.data);
        assert!(scale.is_finite() && scale > 0.0, "{what}");
        for (i, a) in ja.iter().enumerate() {
            let (row, col) = (i / n_par, i % n_par);
            let n = jn.data[row * 2 * n_par + row * n_par + col];
            assert!((a - n).abs() <= 1e-7 + 1e-6 * scale,
                    "{what}, free {free}: row {row} column {col}: {a} against {n}");
        }
    }
}

fn interior_agrees(
    make: fn(usize, usize, &mut Rng) -> (gcs_core::interior::Structure, Vec<f64>),
    first: u32,
    step: f64,
) {
    for seed in 0..5u32 {
        let mut rng = Rng::new(seed + first);
        for dim in [2, 3] {
            let (s, x) = make(dim, 6 + seed as usize, &mut rng);
            kernel_agrees(&s, &x, step, &mut rng, &format!("seed {seed}, {dim}D"));
        }
    }
}

/// Every contact with an envelope in space (#283) — a point on it, a line in its tangent plane, a
/// line and an axis along its normal, a plane touching it, a circle osculating its section — over
/// each source the solve reads: a cone and a cylinder set, each with its number an unknown, and a
/// revolution's line and circle turned about a line and about an axis; under a screw mesh of
/// skew shafts.  Checked at the seeds and at points knocked off them, since the rows are read
/// wherever a solve passes.
#[test]
fn envelope_contacts_agree_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((0, 0, 10))
fix((0, 0, 0)) o
fix((0, 0, 10)) z
ax := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
fix((0, 30, 0)) a
fix((10, 30, 2)) b
gear_ax := line(a, b)
cone := std.Cone(ax, half: hint(20deg))
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5, advance: 2mm)
flank := envelope(cone over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
c1 := point hint((30, 0, -5))
c2 := point hint((31, 1, 5))
cax := line(c1, c2)
cyl := std.Cylinder(cax, r: hint(5mm))
f2 := envelope(cyl over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
in std.front {
  e1 := point hint((6, 2))
  e2 := point hint((9, 7))
  e3 := point hint((12, 7))
  e4 := point hint((12, 2))
  l1 := line(e1, e2)
  l2 := line(e2, e3)
  l3 := line(e3, e4)
  l4 := line(e4, e1)
  rc := point hint((20, 4))
  rim := circle(center: rc) hint(r: 2)
  ro := point hint((0, 0))
  rq := point hint((0, 10))
  rax := line(ro, rq)
}
blade := solid(face(l1, l2, l3, l4), about: std.z)
ring := solid(face(rim), about: rax)
side := surface(blade, l1)
round := surface(ring, rim)
f3 := envelope(side over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
f4 := envelope(round over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
p := point hint((9, 3, 1))
p coincident flank
r := point hint((26, 2, 1))
r coincident f2
s := point hint((7, 1, 4))
s coincident f3
w := point hint((19, 2, 5))
w coincident f4
in std.top {
  kc := point hint((5, 2))
  k := circle(center: kc) hint(r: 10)
}
m := point hint((15, 5, 4))
nl := line(p, m)
t := axis hint(dir: (1, 2, 3))
flank tangent(at: p) nl
flank perpendicular(at: p) t
flank perpendicular(at: p) nl
flank tangent(at: p) std.front
flank curvature(at: p) k
f2 tangent(at: r) t
f2 curvature(at: r) k
f3 perpendicular(at: s) t
f3 curvature(at: s) k
f4 tangent(at: w) std.top
f4 curvature(at: w) k
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::PointOnEnvelope));
    // the envelope's frame, read by a rate of its contact (#365): every word a surface's
    use gcs_core::contact_rate::Word;
    for w in [Word::InPlane, Word::Normal, Word::Plane, Word::Curvature] {
        assert!(sk.constraints.iter().any(|c| c.kind == CKind::Rate && c.rate_word() == w),
                "{w:?} is checked");
    }
    let mut rng = Rng::new(283);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        let read = |c: &&Constraint| c.kind.envelope_contact().is_some() || c.rate_source().is_some();
        for c in sk.constraints.iter().filter(read) {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// A point on the meshing of two flanks (#283, step 3), and a line along the common normal and one
/// along the path of contact at it (#345) — a generated flank against a formed one
/// read off a cone set whose number is an unknown, against a formed revolution's round side, and
/// two generated flanks (each an envelope under a motion of its own) — under a screw mesh of skew
/// shafts.  Checked at the seeds and at points knocked off them.
#[test]
fn meshing_contacts_agree_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((0, 0, 10))
fix((0, 0, 0)) o
fix((0, 0, 10)) z
ax := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
fix((0, 30, 0)) a
fix((10, 30, 2)) b
gear_ax := line(a, b)
cone := std.Cone(ax, half: hint(20deg))
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5, advance: 2mm)
c1 := point hint((30, 0, -5))
c2 := point hint((31, 1, 5))
cax := line(c1, c2)
cyl := std.Cylinder(cax, r: hint(5mm))
pinion_flank := envelope(cyl over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
gear_flank := envelope(cone over t in (-30deg, 30deg), under: gear_turn, seen_from: spin)
in std.front {
  rc := point hint((20, 4))
  rim := circle(center: rc) hint(r: 2)
  ro := point hint((0, 0))
  rq := point hint((0, 10))
  rax := line(ro, rq)
}
ring := solid(face(rim), about: rax)
round := surface(ring, rim)
formed := meshing(pinion_flank, cone over t in (-20deg, 20deg), under: gear_turn, seen_from: spin)
rounded := meshing(pinion_flank, round over t in (-20deg, 20deg), under: gear_turn, seen_from: spin)
both := meshing(pinion_flank, gear_flank over t in (-20deg, 20deg), under: gear_turn, seen_from: spin)
p := point hint((9, 3, 1))
p coincident(t == 5deg) formed
r := point hint((20, 3, 4))
r coincident rounded
s := point hint((7, 1, 4))
s coincident(lag == 0.5) both
m := point hint((15, 5, 4))
nl := line(p, m)
t := axis hint(dir: (1, 2, 3))
formed perpendicular(at: p) nl
formed tangent(at: p) t
rounded perpendicular(at: r) t
rounded tangent(at: r) nl
both perpendicular(at: s) nl
both tangent(at: s) t
param kp: Scalar hint(0.02)
e0 := point hint((8, 2, 2))
e1 := point hint((9, 3, 5))
el := line(e0, e1)
eased := meshing(pinion_flank, cone over t in (-20deg, 20deg), under: gear_turn, seen_from: spin,
                 ease: e0, along: el, profile: kp / 1mm, lengthwise: -0.01 / 1mm)
standing := meshing(round, cone over t in (-20deg, 20deg), under: gear_turn, seen_from: spin,
                    ease: e1, along: el, profile: 0.03 / 1mm, lengthwise: kp / 1mm)
u := point hint((9, 3, 2))
u coincident(t == 3deg) eased
w := point hint((19, 3, 4))
w coincident standing
eased perpendicular(at: u) nl
eased tangent(at: u) t
standing perpendicular(at: w) t
standing tangent(at: w) nl
rate(u over t) parallel nl
rate(w over t) perpendicular t
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    assert_eq!(sk.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing).count(), 5);
    // the common normal and the path, and the rates parallel and square: one kind (#365)
    assert_eq!(sk.constraints.iter().filter(|c| c.kind == CKind::Rate).count(), 12);
    assert_eq!(sk.meshings.iter().filter(|m| m.ease.is_some()).count(), 2);
    let mut rng = Rng::new(283);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        let read = |c: &&Constraint| c.kind.meshing_contact().is_some() || c.rate_source().is_some();
        for c in sk.constraints.iter().filter(read) {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// A motion read (#326, #365, `envelope::screw`): a point it carries, and that point's rate — held
/// square to a line (`std.sv`'s `reciprocal`), along an axis and measured along it, its distance
/// free and stated (`std.Screw`) — under the mesh of two turns about skew shafts (the gear's a screw), a
/// slide seen from a turn, a turn alone, and a turn about a point of a view seen from a turn about
/// a line — every axis free, so the motion's columns are differentiated too.
/// Checked at the seeds and at points knocked off them.
#[test]
fn twist_relations_agree_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std (reciprocal)
o := point hint((0, 0, 0))
z := point hint((1, 0.5, 10))
ax := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
gear_ax := line(a, b)
s1 := point hint((2, 3, 4))
s2 := point hint((5, 1, 9))
rail := line(s1, s2)
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5, advance: 2mm)
slide := frame
std.Slide(slide, along: rail, advance: 40mm)
rest := frame
l1 := point hint((3, 20, 1))
l2 := point hint((9, 24, -4))
L := line(l1, l2)
L reciprocal(seen_from: spin) gear_turn
L reciprocal(seen_from: spin) slide
k1 := point hint((-3, 8, 2))
k2 := point hint((4, 6, 1))
K := line(k1, k2)
K reciprocal(seen_from: rest) gear_turn
isa := axis hint(dir: (0.3, -0.2, 0.9))
sa := std.Screw(isa, gear_turn, seen_from: spin)
jsa := axis hint(dir: (0.1, 0.9, 0.2))
sb := std.Screw(jsa, slide, seen_from: spin)
ksa := axis hint(dir: (0.8, 0.1, -0.5))
sc := std.Screw(ksa, gear_turn, pitch: 3mm)
x := point hint((3, -2, 5))
X := point hint((1, 1, 1))
X coincident(under: gear_turn, seen_from: spin, t == 20) x
Y := point hint((2, 1, 0))
Y coincident(under: slide, seen_from: spin) x
in std.top {
  o2 := point hint((1, 2))
  q2 := point hint((7, 3))
}
twirl := frame
std.Spin(twirl, about: o2, on: std.top, ratio: 0.7)
Z := point hint((0, 4, 1))
Z coincident(under: twirl, seen_from: spin, t == -35) q2
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    let read = |c: &&Constraint| c.kind.reads_motion() || c.rate_source().is_some();
    assert_eq!(sk.constraints.iter().filter(read).count(), 24);
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::RateDistance && c.free.is_some()),
            "a pitch left to the solve is read as a free column");
    let mut rng = Rng::new(326);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(read) {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// A point carried to a stated roll (#366, `envelope::screw`'s `At` and `AtIn`): in space under a
/// screw seen from a turn and under a screw about a line of its own, at a roll stated and at one a
/// free variable names (the twin's last column); drawn in a view under a turn in it, its two rows
/// along the view's axes — every axis free, so the motion's columns are differentiated too; and
/// under a screw about a plane's normal, a joint's own axis its point and direction (#366).
/// Checked at the seeds and at points knocked off them.
#[test]
fn carried_at_rows_agree_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((1, 0.5, 10))
ax := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
gear_ax := line(a, b)
c := point hint((0.5, -1, 0))
d := point hint((0, -1.5, 10))
lift_ax := line(c, d)
spin := frame
std.Turn(spin, about: ax, ratio: 1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5, advance: 2mm)
lift := frame
std.Turn(lift, about: lift_ax, ratio: 1.3, advance: 5mm)
param theta: Angle hint(25deg)
x := point hint((3, -2, 5))
X := point hint((1, 1, 1))
X coincident x at(under: gear_turn, seen_from: spin) 35deg
Y := point hint((2, 1, 0))
Y coincident x at(under: lift) -110deg
W := point hint((0, 2, 1))
W coincident x at(under: gear_turn, seen_from: spin) theta
in std.top {
  o2 := point hint((1, 2))
  q2 := point hint((7, 3))
  T := point hint((4, 4))
}
twirl := frame
std.Spin(twirl, about: o2, on: std.top, ratio: 0.7)
T coincident q2 at(under: twirl) 60deg
up := frame
std.Turn(up, about: std.top.normal, ratio: 0.9, advance: 3mm)
V := point hint((1, 2, 3))
V coincident x at(under: up, seen_from: spin) 50deg
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    let read = |c: &&Constraint| c.kind.reads_motion() && c.stated_roll();
    assert_eq!(sk.constraints.iter().filter(read).count(), 5);
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::CarriedPointIn));
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::CarriedPoint && c.free.is_some()),
            "a roll left to the solve is read as a free column");
    let mut rng = Rng::new(366);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(read) {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// A flank's regularity within its blank (#345 S3, `envelope::regular`): the least area factor of
/// the Formate pinion's flank where the gear's tooth cuts it in its blank — the inner cone's
/// envelope, the cone's half an unknown, the worst point on the tool's round — against finite
/// differences of the search's answer, at the solution and knocked off it.  The worst point
/// moves with the columns; the Jacobian is the Lagrangian's there, the place standing.
#[test]
fn regularity_agrees_with_finite_differences() {
    let text = crate::regular::pair(12.5, &crate::regular::regular("0.3"), true);
    let mut sk = fixtures::gear::read(&text).sketch;
    let mut rng = Rng::new(345);
    for round in 0..3 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-1e-4, 1e-4) * (1.0 + p.value.abs().min(10.0));
            }
        }
        let c = sk.constraints.iter().find(|c| c.kind == CKind::RegularWithin).unwrap();
        assert!(c.local_values(&sk).len() > 40, "the layout's columns are read");
        let err = check_constraint(&sk, c, 1e-5, 1e-6)
            .unwrap_or_else(|e| panic!("round {round}: {e}"));
        assert!(err.is_finite());
    }
}

/// **A point carried by a chain** (#383 stage 1, `Sketch::chain`): the planet's turn about a
/// skew line carried by the arm's screw about another, lowered to the relative over the arm's
/// minted inverse.  Both axes' ends are free, so the carried point's Jacobian is checked over
/// both axes' columns, at the start and knocked off it.
#[test]
fn a_point_carried_by_a_chain_agrees_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
z := point hint((1, 0.5, 10))
shaft := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
pin := line(a, b)
arm := frame
std.Turn(arm, about: shaft, ratio: 0.8, advance: 2mm)
planet := frame
std.Turn(planet, about: pin, ratio: -2.5)
x := point hint((3, -2, 5))
X := point hint((1, 1, 1))
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    let at = |name: &str| sk.motions.iter().position(|m| m.name == name).unwrap();
    let (arm, planet) = (at("arm"), at("planet"));
    let chain = sk.chain(arm, planet, "carried").unwrap();
    let (x, big) = (e.map.ent_named("x").unwrap(), e.map.ent_named("X").unwrap());
    let motion = gcs_core::model::EntRef::new(gcs_core::model::EntKind::Motion, chain);
    for (k, t) in [(0, 25.0), (1, -70.0)] {
        let c = Constraint::new(CKind::CarriedPoint, vec![Arg::Ent(big), Arg::Ent(x), Arg::Ent(motion),
            Arg::Seed { value: t, pinned: k == 0 }]);
        sk.add(c);
    }
    let mut rng = Rng::new(383);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(|c| c.kind == CKind::CarriedPoint) {
            assert!(c.params(&sk).len() >= 3 + 3 + 1 + 12, "both axes are columns");
            let err = check_constraint(&sk, c, 1e-6, 1e-7).unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// **A joint's numbers the solve finds are columns** (#383 stage 5): the spin's ratio left to the
/// solve (`std.Turn` with no `ratio:`) and the gear's advance stated nowhere (its joint held flat),
/// read by a point on an envelope, a point on a meshing, a carried point and its rate — each
/// row's Jacobian checked over the joints' columns with the rest, at the seeds and knocked off.
#[test]
fn a_joints_free_numbers_agree_with_finite_differences() {
    const SRC: &str = "\
unit mm
use std (reciprocal)
o := point hint((0, 0, 0))
z := point hint((1, 0.5, 10))
ax := line(o, z)
a := point hint((0, 30, 0))
b := point hint((10, 30, 2))
gear_ax := line(a, b)
cone := std.Cone(ax, half: 20deg)
spin := frame
std.Turn(spin, about: ax)
gear_turn := frame
gear_ax coincident gear_turn.axis
fix(ratio == -0.5) gear_turn
flank := envelope(cone over t in (-30deg, 30deg), under: spin, seen_from: gear_turn)
p := point hint((9, 3, 1))
p coincident flank
c1 := point hint((30, 0, -5))
c2 := point hint((31, 1, 5))
cax := line(c1, c2)
mated := std.Cone(cax, half: 15deg)
both := meshing(flank, mated over t in (-20deg, 20deg), under: gear_turn, seen_from: spin)
q := point hint((20, 3, 4))
q coincident both
x := point hint((3, -2, 5))
X := point hint((1, 1, 1))
X coincident(under: gear_turn, seen_from: spin, t == 15deg) x
m := point hint((15, 5, 4))
nl := line(X, m)
rate(X over t) perpendicular nl
";
    let (prog, errs) = crate::common::parse(SRC);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch;
    let joint = |name: &str| {
        let f = e.map.ent_named(name).unwrap();
        sk.frames[f.i()].joint.clone().unwrap()
    };
    let (spin, gear) = (joint("spin"), joint("gear_turn"));
    assert!(!sk.params[spin.ratio as usize].fixed && sk.params[spin.advance as usize].fixed);
    assert!(sk.params[gear.ratio as usize].fixed && !sk.params[gear.advance as usize].fixed);
    let free = [spin.ratio, gear.advance];
    let read = |c: &&Constraint| c.kind.envelope_contact().is_some()
        || c.kind.meshing_contact().is_some() || c.kind == CKind::CarriedPoint
        || c.rate_source().is_some();
    let rows: Vec<&Constraint> = sk.constraints.iter().filter(read).collect();
    assert!(rows.len() >= 4);
    for c in &rows {
        let ps = c.params(&sk);
        assert!(free.iter().all(|q| ps.contains(q)), "{:?} reads the joints' numbers", c.kind);
    }
    let mut rng = Rng::new(3835);
    for round in 0..4 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.05, 0.05) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(read) {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("round {round}: {e}"));
            assert!(err.is_finite());
        }
    }
}

/// **Which lines a row reads by direction is the kernel's** (#496): `Constraint::direction_lines`
/// names them for `solve::collapsed`, and the table is held to the kernels here.  For every line
/// slot of every row of the corpus, the line is shrunk toward each of its ends, in its view and
/// in space (its ends' hidden points, where the row reads those).  The row reads it only by
/// direction when a millionth of the line leaves the residual where it was, and the line's
/// collapse puts it at nil whatever else — at the pose and with every other column moved: no
/// nearer its root as it shrinks, and on it at a line of no length.  That is a slot
/// `direction_lines` must name, and a slot it names must be one; a row that comes to its root
/// continuously (`b − a` unnormalised), or holds at a nil line only by the sign of a zero (an
/// angle's `atan2`), is not.
#[test]
fn every_row_read_by_direction_is_declared() {
    use gcs_core::model::EntKind;
    let norm = |r: &[f64]| r.iter().map(|x| x * x).sum::<f64>().sqrt();
    for seed in 0..3u32 {
        let sk = all_constraints(seed);
        let mut rng = Rng::new(seed + 70);
        for c in &sk.constraints {
            let declared: Vec<usize> = c.direction_lines().collect();
            for a in &c.args {
                let Arg::Ent(e) = a else { continue };
                if e.kind != EntKind::Line {
                    continue;
                }
                let l = &sk.lines[e.i()];
                // the sketch with the line `s` of its length, its end `to` held, and the
                // Params of its two ends
                let shrunk = |to: u32, from: u32, s: f64| {
                    let (to, from) = (to as usize, from as usize);
                    let mut pairs: Vec<(u32, u32)> =
                        sk.point_params(to).into_iter().zip(sk.point_params(from)).collect();
                    if let (Some(a), Some(b)) = (sk.lift_of(to), sk.lift_of(from)) {
                        pairs.extend(sk.lifts[a].x.into_iter().zip(sk.lifts[b].x));
                    }
                    let mut k = sk.clone();
                    for &(t, f) in &pairs {
                        let (vt, vf) = (sk.params[t as usize].value, sk.params[f as usize].value);
                        k.params[f as usize].value = vt + s * (vf - vt);
                    }
                    let ends: Vec<u32> = pairs.iter().flat_map(|&(t, f)| [t, f]).collect();
                    (k, ends)
                };
                let at = |to: u32, from: u32, s: f64| {
                    let (k, _) = shrunk(to, from, s);
                    norm(&c.residual(&k, &c.local_values(&k)))
                };
                let mut by_direction = |to: u32, from: u32| {
                    let r = at(to, from, 1.0);
                    if !(r > 1e-6 && (at(to, from, 1e-6) - r).abs() < 1e-6 * r) {
                        return false;
                    }
                    let (k, ends) = shrunk(to, from, 0.0);
                    let cols = c.params(&k);
                    (0..16).all(|n| {
                        let mut v = c.local_values(&k);
                        for (x, p) in v.iter_mut().zip(&cols) {
                            if n > 0 && !ends.contains(p) {
                                *x += rng.uniform(-20.0, 20.0);
                            }
                        }
                        norm(&c.residual(&k, &v)) < 1e-12
                    })
                };
                let found = by_direction(l.p1, l.p2) || by_direction(l.p2, l.p1);
                assert_eq!(found, declared.contains(&e.i()),
                           "seed {seed}: {:?} {:?} reads line {} by direction: {found}",
                           c.kind, c.args, e.i());
            }
        }
    }
}

/// **A region's deflection, as the drawing states it** (#510 S2, `CKind::Deflection` over a
/// region read in its plane): the row over the region's points' numbers and its radii, its nodes
/// moved with them — the library's sized plate (lines), and a plate in plane strain with a round
/// corner and a hole (an arc, a circle) under a pressure on the arc, a force along a side and its
/// weight — against finite differences, at the solution and knocked off it.
#[test]
fn a_regions_deflection_agrees_with_finite_differences() {
    let holed = "\
unit mm
use std (horizontal, vertical)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
param r: Length hint(9mm)
in std.front {
  a := point
  b := point hint((120, 0))
  c := point hint((120, 30))
  k := point hint((100, 30))
  d := point hint((100, 50))
  e := point
  o := point
  fix((0mm, 0mm)) a
  fix((0mm, 50mm)) e
  fix((40mm, 25mm)) o
  fix((120mm, 0mm)) b
  b vertical c
  b distance(30mm) c
  c horizontal k
  k vertical d
  low := line(a, b)
  end := line(b, c)
  round := radius(20mm) arc(center: k, start: c, end: d)
  top := line(d, e)
  back := line(e, a)
  hole := radius(r) circle(center: o) hint(r: 9)
  region := face(low, end, round, top, back, holes: hole)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
plate := solid(region, from: -2mm, to: 2mm) of steel
scenario s {
  plane strain plate
  fixed plate.back
  pressure plate.round 30MPa
  load plate.low 3kN along: down
  accelerate plate (0, 0, -9.81 * 1m / 1s^2)
  deflection(b, along: down) == 0.23mm
  claim deflection(c, along: down) >= 0mm
}
";
    let mut rng = Rng::new(511);
    for src in [gcs_core::examples::source("sized_plate").unwrap(), holed] {
        let mut sk = crate::common::solved(&crate::common::read(src));
        for round in 0..3 {
            if round > 0 {
                for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                    p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
                }
            }
            // a region's normal equations are good to fewer figures than a frame's, so the
            // difference is taken over a longer step, and good to its truncation, 1e-5
            let c = sk.constraints.iter().find(|c| c.kind == CKind::Deflection).unwrap();
            let v = c.local_values(&sk);
            let ja = c.jacobian(&sk, &v);
            let jn = gcs_core::fdcheck::fd_jacobian(|w| c.residual(&sk, w), &v, 1e-4);
            let scale = gcs_core::linalg::absmax(&jn.data);
            assert!(scale.is_finite() && scale > 0.0, "round {round}");
            for (col, (a, n)) in ja.iter().zip(&jn.data).enumerate() {
                assert!((a - n).abs() <= 1e-5 * scale,
                        "round {round}: column {col}: {a} against {n}");
            }
        }
    }
}


/// **A solid's deflection, as the drawing states it** (#510 S3, `CKind::Deflection` over a solid
/// read in three dimensions): the row over its face's points' numbers and its radii, its face's
/// nodes moved with them and its own stacked over them — the library's plank (lines, a load
/// across its plane), and a plate with a round corner and a hole (an arc, a circle) under a
/// pressure on the arc, a force across it on a side and its weight — against finite
/// differences, the plank at the solution and both knocked off it.
#[test]
fn a_solids_deflection_agrees_with_finite_differences() {
    let holed = "\
unit mm
use std (horizontal, vertical)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
param r: Length hint(9mm)
in std.front {
  a := point
  b := point hint((120, 0))
  c := point hint((120, 30))
  k := point hint((100, 30))
  d := point hint((100, 50))
  e := point
  o := point
  fix((0mm, 0mm)) a
  fix((0mm, 50mm)) e
  fix((40mm, 25mm)) o
  fix((120mm, 0mm)) b
  b vertical c
  b distance(30mm) c
  c horizontal k
  k vertical d
  low := line(a, b)
  end := line(b, c)
  round := radius(20mm) arc(center: k, start: c, end: d)
  top := line(d, e)
  back := line(e, a)
  hole := radius(r) circle(center: o) hint(r: 9)
  region := face(low, end, round, top, back, holes: hole)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
plate := solid(region, from: -4mm, to: 4mm) of steel
scenario s {
  fixed plate.back
  pressure plate.round 30MPa
  load plate.low 3kN along: std.y
  accelerate plate (0, 0, -9.81 * 1m / 1s^2)
  deflection(b, along: down) == 0.14mm
  claim deflection(c, along: down) >= 0mm
}
";
    let mut rng = Rng::new(513);
    // the holed plate's mesh is the larger, and is read once, knocked off its solution: its
    // nodes moved off where they were meshed and its row off what it states
    let plank = gcs_core::examples::source("sized_plank").unwrap();
    for (src, rounds) in [(plank, 0..2), (holed, 1..2)] {
        let mut sk = crate::common::solved(&crate::common::read(src));
        for round in rounds {
            if round > 0 {
                for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                    p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
                }
            }
            // a solid's stiffness is good to fewer figures than a region's, its rounding in a
            // difference over 1e-4 of a millimetre a thousandth of the rate: the difference is
            // taken over 1e-2, where its truncation is under 1e-6 of it
            let c = sk.constraints.iter().find(|c| c.kind == CKind::Deflection).unwrap();
            let v = c.local_values(&sk);
            let ja = c.jacobian(&sk, &v);
            let jn = gcs_core::fdcheck::fd_jacobian(|w| c.residual(&sk, w), &v, 1e-2);
            let scale = gcs_core::linalg::absmax(&jn.data);
            assert!(scale.is_finite() && scale > 0.0, "round {round}");
            for (col, (a, n)) in ja.iter().zip(&jn.data).enumerate() {
                assert!((a - n).abs() <= 1e-5 * scale,
                        "round {round}: column {col}: {a} against {n}");
            }
        }
    }
}
