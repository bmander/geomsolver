//! **A section against a smooth chain, beyond the seam's normal surface** (#422): where a ring's
//! section — a chamfer's flat, a round's ball — reaches across a seam where the face it stands on
//! goes on tangent into another (a plate's far face into its rim's round) and the seam is no circle
//! about the ring's axis, the section against the chain is the one against whichever face of it is
//! nearest, and changes face where it crosses the seam's normal surface (the cylinder through a
//! plane's and a torus's seam, along their common normal).  Short of that surface the ring stands
//! against its own face, a revolution kept within it (`solid::lowering`'s `Ring` and its `Kept`);
//! beyond it, the piece here: what the section sweeps against the next face, sectioned in the
//! planes square to its edge — the face's meeting with the wall, whose square planes are a ring's
//! half-planes through its axis where that meeting is the ring's circle — and fitted, as
//! `fillet::flat_along` fits a flat along a traced run.  The two are glued on the normal surface
//! (`boolean::glued`), where both are the same set: there a point's distance from either face is
//! its distance from the seam.
//!
//! A flat's piece beyond is a lens: on the next face the strip between the seam and the leg, the
//! flat between the leg and its crossing with the normal surface, and on that surface the cap
//! between the crossing and the seam, the three meeting where the leg on the ring's own face
//! crosses the seam (`ends`).  What is only fitted is the flat beyond; the cap's and the strip's
//! surfaces are the faces' own, and the seam is the circle it is.  An edge is kept exact wherever
//! `ssi::intersect` gives the surfaces' meeting in closed form (a ring's crease its circle, a
//! cap's line down a cylinder's wall), and fitted only where the meeting must be traced.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::fillet::{column, crossing_plane, face_out};
use super::geom::{Curve, Frame, Surface, Traced, V};
use super::nurbs::{fit_curve, fit_net};
use super::topo::{Brep, EdgeCurve};
use crate::util::space::{add, cross, distance, dot, norm, scale, sub, unit};
use std::sync::Arc;

/// A chamfer's flat or a round beyond a seam's normal surface (all in the boundary's units): the
/// face it stands on (`face`), whose meeting with the wall is the edge it runs along, read in the
/// planes square to it; which way from that face the region lies (`toward`, unit); the chain's
/// next face (`chain`) and the other face (`wall`), with which way each one's signed distance runs
/// toward the piece (`sides`, ±1); the seam (`seam`); what it is across the seam (`section`); and
/// where the leg on the face it stands on crosses the seam, or where the ball touches it (`ends`).
#[derive(Clone, Debug)]
pub struct Beyond {
    pub face: Surface,
    pub toward: V,
    pub chain: Surface,
    pub wall: Surface,
    pub sides: [f64; 2],
    pub seam: Seam,
    pub section: Across<Surface>,
    pub ends: [V; 2],
}

/// What a piece beyond a seam's normal surface is across the seam (#426), its surfaces `S` (the
/// model's, or the boundary's): a chamfer's flat — its legs, on the chain from the wall and on the
/// wall from the chain, and the seam's normal surface it starts at (`split`), whose signed distance
/// runs from the edge's own piece toward it (a plane's normal turned so; the edge within a
/// cylinder) — or a round's ball of `radius`, glued to the edge's own piece where it touches the
/// seam (`round_beyond`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Across<S> {
    Flat { legs: [f64; 2], split: S },
    Round { radius: f64 },
}

impl<S> Across<S> {
    /// Its size: a flat's longer leg, a round's radius.
    pub fn size(&self) -> f64 {
        match self { Across::Flat { legs, .. } => legs[0].max(legs[1]), Across::Round { radius } => *radius }
    }
}

/// A seam a chain's face goes on across: a circle (centre, unit normal, radius) or a line (a point
/// of it, its unit direction).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seam {
    Circle(V, V, f64),
    Line(V, V),
}

/// A piece's sections between its ends: each `u` in [0, 1] the edge's point and the unit normal of
/// the section's plane through it (the way `u` runs), the ends in that order; and the edge whole
/// where `ssi::intersect` gives it in closed form (a ring's circle, a slope's ellipse).
struct Sections<'a> {
    at: Box<dyn Fn(f64) -> (V, V) + 'a>,
    ends: [V; 2],
    exact: Option<Curve>,
}

/// `b`'s sections between the two through `through` (its ends, or the ball's centres there; `ends`
/// in their order): the planes square to the face's meeting with the wall, along the stretch of it
/// between them whose middle stands nearer theirs.
fn sections<'a>(b: &'a Beyond, through: [V; 2], tol: f64) -> Result<Sections<'a>, String> {
    let face = &b.face;
    let meet = Traced { a: face.clone(), b: b.wall.clone(), pts: Vec::new(), closed: false };
    let feet = through.map(|q| meet.project(q));
    let size = distance(b.ends[0], b.ends[1]) + b.section.size();
    let (curves, exact) = match super::ssi::intersect(face, &b.wall, tol) {
        super::ssi::Ssi::Curves(cs) => (cs, true),
        super::ssi::Ssi::Same => return Err("a face and a wall on one surface".into()),
        super::ssi::Ssi::Traced => {
            let lo = [0, 1, 2].map(|k| feet[0][k].min(feet[1][k]) - 4.0 * size);
            let hi = [0, 1, 2].map(|k| feet[0][k].max(feet[1][k]) + 4.0 * size);
            (super::ssi::trace(face, &b.wall, &feet, lo, hi, tol)?, false)
        }
    };
    let curve = nearest(curves, &feet).ok_or("a face and a wall that meet nowhere")?;
    // the stretch between the ends' sections — each end's nearest point of the meeting, the plane
    // square to it there through the end — the one whose middle stands nearer the ends'
    let t = through.map(|p| curve.inverse(p));
    let mid = scale(add(through[0], through[1]), 0.5);
    let (t0, span) = match curve.period() {
        Some(period) => {
            let fwd = (t[1] - t[0]).rem_euclid(period);
            let near = |t0: f64, span: f64| distance(meet.project(curve.point(t0 + 0.5 * span)), mid);
            if near(t[0], fwd) <= near(t[1], period - fwd) { (t[0], fwd) } else { (t[1], period - fwd) }
        }
        None => (t[0].min(t[1]), (t[1] - t[0]).abs()),
    };
    if span <= 1e-9 * (1.0 + t0.abs()) {
        return Err("a piece beyond a seam whose ends stand at one section".into());
    }
    let q = if (t0 - t[0]).abs() <= 1e-12 * (1.0 + t0.abs()) { b.ends } else { [b.ends[1], b.ends[0]] };
    let guide = super::fillet::smooth_guide(&curve, t0, span).ok_or("the meeting could not be read smoothly")?;
    let (fa, wall) = (face.clone(), b.wall.clone());
    Ok(Sections {
        at: Box::new(move |u: f64| {
            let u = u.clamp(0.0, 1.0);
            let e = meet.project(guide.point(u));
            let d = cross(fa.gradient(e), wall.gradient(e));
            (e, if dot(d, guide.d2(u).1) < 0.0 { scale(unit(d), -1.0) } else { unit(d) })
        }),
        ends: q,
        exact: exact.then_some(curve),
    })
}

/// Of `curves`, the one passing nearest the points `by`.
fn nearest(curves: Vec<Curve>, by: &[V]) -> Option<Curve> {
    let off = |c: &Curve| by.iter().map(|&p| distance(c.point(c.inverse(p)), p)).sum::<f64>();
    curves.into_iter().min_by(|x, y| off(x).total_cmp(&off(y)))
}

/// The stretch of the closed-form curve `c` from `a` to `z` as an edge of `out`, from `va` to `vz`:
/// a line's segment, or a circle's or an ellipse's arc, the way round passing nearer `near`.
/// Nothing for any other curve, or where `a` or `z` stands off it by more than `tol`.
fn exact_edge(out: &mut Brep, c: &Curve, [a, z]: [V; 2], [va, vz]: [u32; 2], near: V, tol: f64)
    -> Option<usize> {
    if [a, z].iter().any(|&p| distance(c.point(c.inverse(p)), p) > tol) {
        return None;
    }
    let tau = std::f64::consts::TAU;
    // a frame turned over (its second and third axes reversed): the same conic run the other way
    let over = |f: &Frame| Frame { o: f.o, x: f.x, y: scale(f.y, -1.0), z: scale(f.z, -1.0) };
    let e = match *c {
        Curve::Line { .. } => out.edge(EdgeCurve::Curve(Curve::Line { p: a, d: unit(sub(z, a)) }),
            [0.0, distance(a, z)], [va, vz]),
        Curve::Circle(f, r) => {
            // from `a` itself: its frame's first axis through it
            let x = unit(sub(a, f.o));
            let f = Frame { o: f.o, x, y: cross(f.z, x), z: f.z };
            let at = |f: &Frame| { let w = sub(z, f.o); dot(w, f.y).datan2(dot(w, f.x)).rem_euclid(tau) };
            let off = |f: &Frame, t: f64| distance(f.at([r * t.dcos(), r * t.dsin(), 0.0]), near);
            let f = if off(&f, 0.5 * at(&f)) <= off(&over(&f), 0.5 * at(&over(&f))) { f } else { over(&f) };
            out.edge(EdgeCurve::Curve(Curve::Circle(f, r)), [0.0, at(&f)], [va, vz])
        }
        Curve::Ellipse(f, ma, mi) => {
            let run = |f: Frame| {
                let e = Curve::Ellipse(f, ma, mi);
                let t0 = e.inverse(a);
                let span = (e.inverse(z) - t0).rem_euclid(tau);
                let off = distance(e.point(t0 + 0.5 * span), near);
                (e, [t0, t0 + span], off)
            };
            let (fwd, back) = (run(f), run(over(&f)));
            let (e, t, _) = if fwd.2 <= back.2 { fwd } else { back };
            out.edge(EdgeCurve::Curve(e), t, [va, vz])
        }
        _ => return None,
    };
    Some(e as usize)
}

/// Where the surfaces `s` and `t` meet from `a` to `z`, as an edge of `out` from `va` to `vz`, the
/// way passing nearer `near`: the meeting's closed form where `ssi::intersect` gives one through
/// both, else nothing.
fn exact_meeting(out: &mut Brep, s: &Surface, t: &Surface, ends: [V; 2], vs: [u32; 2], near: V, tol: f64)
    -> Option<usize> {
    let super::ssi::Ssi::Curves(cs) = super::ssi::intersect(s, t, tol) else { return None };
    exact_edge(out, &nearest(cs, &ends)?, ends, vs, near, tol)
}

/// The seam's stretch between the ends `q` as an edge of `out`, from `vq[0]` to `vq[1]`: a circle's
/// arc (the shorter, nearer the edge `near`), or a line's.
fn seam_edge(out: &mut Brep, seam: Seam, q: [V; 2], vq: [u32; 2], near: V) -> usize {
    let curve = match seam {
        Seam::Line(p, d) => Curve::Line { p, d },
        Seam::Circle(o, n, r) => Curve::Circle(Frame::about(o, n), r),
    };
    exact_edge(out, &curve, q, vq, near, f64::INFINITY).expect("a line's or a circle's stretch")
}

/// The piece of a chamfer's flat beyond the seam's normal surface (`Beyond`), fitted within `tol`;
/// glued to the edge's own piece on that surface (`split`).
pub fn flat_beyond(b: &Beyond, tol: f64) -> Result<Brep, String> {
    let Across::Flat { legs: [lc, lw], ref split } = b.section else {
        return Err("a round's piece built as a flat's".into());
    };
    let [sc, sw] = b.sides;
    let Sections { at: section, ends: q, .. } = sections(b, b.ends, tol)?;
    let size = lc.max(lw);
    // the flat's sum of distances, nothing on it, and its gradient
    let dc = |p: V| sc * b.chain.implicit(p);
    let dw = |p: V| sw * b.wall.implicit(p);
    let sum = |p: V| dc(p) / lw + dw(p) / lc - 1.0;
    let grad = |p: V| add(scale(b.chain.gradient(p), sc / lw), scale(b.wall.gradient(p), sw / lc));
    // each leg in the section at `u`: the chain's point `lc` from the wall, the wall's `lw` from
    // the chain — each where the face meets the other's offset, crossing the section's plane, from
    // the edge's point stepped out along each face
    let leg_meetings = [
        Traced { a: b.chain.clone(), b: b.wall.offset(sw * lc).ok_or("a leg wider than the wall holds")?, pts: Vec::new(),
            closed: false },
        Traced { a: b.wall.clone(), b: b.chain.offset(sc * lw).ok_or("a leg wider than the chain holds")?, pts: Vec::new(),
            closed: false },
    ];
    let legs_at = |u: f64| -> (V, [V; 2]) {
        let (p, n) = section(u);
        let along_chain = scale(unit(b.wall.gradient(p)), sw);
        (n, [crossing_plane(&leg_meetings[0], p, n, add(p, scale(along_chain, lc))),
            crossing_plane(&leg_meetings[1], p, n, add(p, scale(b.toward, lw)))])
    };
    let frames = std::cell::RefCell::new(std::collections::BTreeMap::<u64, (V, [V; 2])>::new());
    let frame_at = |u: f64| -> (V, [V; 2]) {
        let u = u.clamp(0.0, 1.0);
        if let Some(&fr) = frames.borrow().get(&u.to_bits()) {
            return fr;
        }
        let fr = legs_at(u);
        frames.borrow_mut().insert(u.to_bits(), fr);
        fr
    };
    // the flat in each section: from the chain's leg to the wall's, pulled onto the sum's level
    // across the chord
    let flat = |u: f64, v: f64| -> V {
        let (t, [pa, pb]) = frame_at(u);
        if v <= 0.0 {
            return pa;
        }
        if v >= 1.0 {
            return pb;
        }
        let across = unit(cross(t, sub(pb, pa)));
        let mut p = crate::util::space::lerp(pa, pb, v);
        for _ in 0..32 {
            let (fv, g) = (sum(p), dot(grad(p), across));
            if g.abs() < 1e-300 {
                break;
            }
            let step = fv / g;
            p = sub(p, scale(across, step));
            if step.abs() <= 1e-15 * (1.0 + norm(p)) {
                break;
            }
        }
        p
    };
    for (k, &end) in q.iter().enumerate() {
        let leg = frame_at(k as f64).1[0];
        if distance(leg, end) > 1e-6 * (size + norm(end)) {
            return Err(format!("the leg on the chain's next face misses the seam where the flat crosses it ({:.2e} off)",
                distance(leg, end)));
        }
    }
    let bar = tol.max(1e-7 * (size + norm(q[0])));
    let (net, net_err) = fit_net(&flat, 3, 3, 64, 4, tol).ok_or("the flat beyond the seam could not be fitted")?;
    if net_err > bar {
        return Err(format!("the flat beyond the seam misses its fit by {net_err:e}"));
    }
    // where the flat crosses the normal surface, in each section: from the seam's end on, each
    // section's from the last's
    let settled = |at: &[(V, f64); 3]| at.iter().all(|(_, m)| m.abs() <= 1e-13 * (size + norm(q[0])));
    let crossing = |u: f64, guess: V| -> Option<V> {
        let (o, n) = section(u);
        crate::util::roots::newton_onto::<3>(guess, size, |p| Some([
            (split.gradient(p), split.implicit(p)), (grad(p), sum(p)), (n, dot(n, sub(p, o))),
        ]), settled)
    };
    let steps = 256;
    let mut table: Vec<V> = Vec::with_capacity(steps + 1);
    let mut last = q[0];
    for k in 0..=steps {
        let u = k as f64 / steps as f64;
        // (a step up off the seam, toward the region, so the march leaves the end it starts at)
        let guess = if k == 0 { q[0] } else { add(last, scale(b.toward, 1e-3 * size / steps as f64)) };
        let p = if k == 0 { q[0] } else if k == steps { q[1] } else {
            crossing(u, guess).ok_or("the flat beyond the seam never crosses its normal surface")?
        };
        table.push(p);
        last = p;
    }
    let cut_at = |s: f64| -> V {
        if s <= 0.0 {
            return q[0];
        }
        if s >= 1.0 {
            return q[1];
        }
        let x = s * steps as f64;
        let k = (x.floor() as usize).min(steps - 1);
        let guess = crate::util::space::lerp(table[k], table[k + 1], x - k as f64);
        crossing(s, guess).unwrap_or(guess)
    };
    let (cut_curve, cut_err) = fit_curve(&cut_at, 3, 64, tol).ok_or("the flat's crossing could not be fitted")?;
    if cut_err > bar {
        return Err(format!("the flat's crossing with the seam's normal surface misses its fit by {cut_err:e}"));
    }
    let mut out = Brep::default();
    let vq = q.map(|p| out.vertex(p));
    let seam = seam_edge(&mut out, b.seam, q, vq, section(0.5).0);
    let [[u0, u1], _] = net.domain();
    let leg = out.edge(EdgeCurve::Curve(column(&net, 0)), [u0, u1], vq) as usize;
    let cut = out.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(cut_curve))), [0.0, 1.0], vq) as usize;
    // each face's surface with its seam turned away from the piece
    let middle = flat(0.5, 0.0);
    let chain = turned_from(&b.chain, middle);
    let split_face = turned_from(split, middle);
    let sheet = Surface::BSpline(Frame::new([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Arc::new(net));
    let p = q[0];
    face_out(&mut out, chain, scale(b.chain.gradient(p), -sc), &[seam, leg], "a")?;
    face_out(&mut out, sheet, grad(p), &[leg, cut], "flat")?;
    face_out(&mut out, split_face, scale(split.gradient(p), -1.0), &[cut, seam], "split")?;
    out.measure_tolerances();
    for e in [leg, cut] {
        let t = &mut out.edges[e].tol;
        *t = t.max(net_err).max(cut_err);
    }
    out.check(10.0 * tol.max(net_err).max(cut_err)).map_err(|e| format!("the flat beyond the seam: {e}"))?;
    Ok(out)
}

/// **A round's piece beyond a seam's normal surface** (#422's rung 3): the ball of its `radius` beside
/// the wall touches the chain's next face (the torus) where its centre stands past the seam's
/// offset — between the sections through its centres where it touches the seam (over `ends`, its
/// touches there) — and the edge's own face elsewhere: a ring's stretch glued to this at both
/// ends, or along a traced edge the ball rolled along it with that stretch cut away.  Its centre runs
/// where the wall's offset meets the chain's, read in the edge's sections (the planes square to
/// it; a ring's half-planes through its axis, where at its ends the ball's arc lies, its centre's
/// path square to them there); its canal is the ball's arc between where it touches the chain and
/// the wall, fitted.  The piece: the canal; on the chain's face the lens between the seam and the
/// touch; on the edge's own face (a plane) the strip from the edge to the seam; on the wall the
/// strip from the edge to the touch; and at each end the cap in the plane of the ball's arc, square
/// to its centre's path.  The edge and each cap's line down the wall are the meetings
/// `ssi::intersect` gives in closed form (a ring's circle, a line down a cylinder), fitted where
/// it gives none.
pub fn round_beyond(b: &Beyond, tol: f64) -> Result<Brep, String> {
    let Across::Round { radius: r } = b.section else { return Err("a flat's piece built as a round's".into()) };
    let [sc, sw] = b.sides;
    // the sections through the ball's centres at the ends, over the plane the edge stands on
    let through = b.ends.map(|q| add(q, scale(b.toward, r)));
    let Sections { at: section, ends: q, exact } = sections(b, through, tol)?;
    // the ball's centre in the section at `u`: where the wall's offset meets the torus's, from the
    // edge's point stepped off both faces
    let spine = Traced {
        a: b.wall.offset(sw * r).ok_or("a ball larger than the wall holds")?,
        b: b.chain.offset(sc * r).ok_or("a ball larger than the chain's face holds")?,
        pts: Vec::new(), closed: false,
    };
    let toward = |s: &Surface, side: f64, p: V| scale(unit(s.gradient(p)), -side);
    let frames = std::cell::RefCell::new(std::collections::BTreeMap::<u64, (V, [V; 2])>::new());
    let frame_at = |u: f64| -> (V, [V; 2]) {
        let u = u.clamp(0.0, 1.0);
        if let Some(&fr) = frames.borrow().get(&u.to_bits()) {
            return fr;
        }
        let (p, n) = section(u);
        let guess = add(add(p, scale(scale(unit(b.wall.gradient(p)), sw), r)), scale(b.toward, r));
        let c = crossing_plane(&spine, p, n, guess);
        let fr = (c, [toward(&b.chain, sc, c), toward(&b.wall, sw, c)]);
        frames.borrow_mut().insert(u.to_bits(), fr);
        fr
    };
    let size = r + distance(q[0], q[1]);
    for (k, &end) in q.iter().enumerate() {
        let (c, [et, _]) = frame_at(k as f64);
        let touch = add(c, scale(et, r));
        if distance(touch, end) > 1e-6 * (size + norm(end)) {
            return Err(format!("the ball beyond the seam misses the seam at its end ({:.2e} off)", distance(touch, end)));
        }
    }
    let canal = |u: f64, v: f64| { let (c, e) = frame_at(u); add(c, scale(crate::util::space::slerp(e[0], e[1], v), r)) };
    let bar = tol.max(1e-7 * (size + norm(q[0])));
    let (net, net_err) = fit_net(&canal, 3, 3, 64, 4, tol).ok_or("the canal beyond the seam could not be fitted")?;
    if net_err > bar {
        return Err(format!("the canal beyond the seam misses its fit by {net_err:e}"));
    }
    let [[u0, u1], [v0, v1]] = net.domain();
    let last_v = net.poles[0].len() - 1;
    // each cap: the plane of the ball's arc at an end, square to its centre's path there
    // (the way the centre's path runs at `c`, where both offsets meet)
    let path = |c: V| unit(cross(spine.a.gradient(c), spine.b.gradient(c)));
    // each end's centre, where the ball touches the wall, and its cap's normal along the path
    let caps_at: [(V, V, V); 2] = [0, 1].map(|k| {
        let (c, [_, ew]) = frame_at(k as f64);
        let d = path(c);
        let ahead = sub(frame_at(if k == 0 { 1e-3 } else { 1.0 }).0, frame_at(if k == 0 { 0.0 } else { 1.0 - 1e-3 }).0);
        (c, add(c, scale(ew, r)), if dot(d, ahead) < 0.0 { scale(d, -1.0) } else { d })
    });
    // the edge's point in the plane through `c` square to `d`: where the face, the wall and that
    // plane meet, from the corner the ball stands in
    let plane_of = |p: V| dot(b.toward, sub(p, q[0]));
    let crease_in = |c: V, d: V| -> Option<V> {
        let ew = toward(&b.wall, sw, c);
        let guess = sub(add(c, scale(ew, r)), scale(b.toward, r));
        crate::util::roots::newton_onto::<3>(guess, r, |p| Some([
            (b.toward, plane_of(p)), (b.wall.gradient(p), b.wall.implicit(p)), (d, dot(d, sub(p, c))),
        ]), |at| at.iter().all(|(_, m)| m.abs() <= 1e-13 * (size + norm(c))))
    };
    let crease = [0, 1].map(|k| crease_in(caps_at[k].0, caps_at[k].2));
    let [Some(k0), Some(k1)] = crease else { return Err("the round's caps meet no edge".into()) };
    let crease = [k0, k1];
    let mut out = Brep::default();
    let vc = [0, 1].map(|k| out.vertex(crease[k]));
    let vq = [0, 1].map(|k| out.vertex(net.point([u0, u1][k], v0)));
    let vw = [0, 1].map(|k| out.vertex(net.point([u0, u1][k], v1)));
    // a circle's arc from `a` to `z_` the short way about `c`, turning about `n` or against it
    let arc = |out: &mut Brep, c: V, n: V, a: V, z_: V, va: u32, vz: u32| -> usize {
        let x = unit(sub(a, c));
        let n = if dot(cross(x, sub(z_, c)), n) < 0.0 { scale(n, -1.0) } else { n };
        let frame = Frame { o: c, x, y: cross(n, x), z: n };
        let w = sub(z_, c);
        let end = dot(w, frame.y).datan2(dot(w, frame.x));
        out.edge(EdgeCurve::Curve(Curve::Circle(frame, distance(a, c))), [0.0, end], [va, vz]) as usize
    };
    // a curve through `at(s)` from its start to its end, fitted, its ends exactly those
    let fitted = |out: &mut Brep, at: &dyn Fn(f64) -> V, va: u32, vz: u32| -> Result<(usize, f64), String> {
        let (curve, err) = fit_curve(at, 3, 16, tol).ok_or("a round's edge could not be fitted")?;
        let e = out.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(curve))), [0.0, 1.0], [va, vz]) as usize;
        out.edges[e].tol = out.edges[e].tol.max(err);
        Ok((e, err))
    };
    let mut fit_err: f64 = 0.0;
    // the edge between the caps: the meeting's closed form, else the meeting read through the
    // planes square to the centre's path along the way
    let crease_edge = match exact.and_then(|c| exact_edge(&mut out, &c, crease, vc, section(0.5).0, tol)) {
        Some(e) => e,
        None => {
            let at = |s: f64| -> V {
                if s <= 0.0 { return crease[0] }
                if s >= 1.0 { return crease[1] }
                let (c, _) = frame_at(s);
                crease_in(c, path(c)).unwrap_or(c)
            };
            let (e, err) = fitted(&mut out, &at, vc[0], vc[1])?;
            fit_err = fit_err.max(err);
            e
        }
    };
    let seam_arc = seam_edge(&mut out, b.seam, q, vq, section(0.5).0);
    let touch_chain = out.edge(EdgeCurve::Curve(column(&net, 0)), [u0, u1], vq) as usize;
    let touch_wall = out.edge(EdgeCurve::Curve(column(&net, last_v)), [u0, u1], vw) as usize;
    // each cap: along the face from the edge to the seam, the ball's arc, back down the wall
    let mut caps = Vec::new();
    for k in 0..2 {
        let (c, w, d) = caps_at[k];
        let bottom = out.edge(EdgeCurve::Curve(Curve::Line { p: crease[k], d: unit(sub(q[k], crease[k])) }),
            [0.0, distance(crease[k], q[k])], [vc[k], vq[k]]) as usize;
        // where the cap's plane cuts the wall, from the edge to the wall's touch
        let cap = Surface::Plane(Frame::about(c, d));
        let mid = crate::util::space::lerp(crease[k], w, 0.5);
        let side = match exact_meeting(&mut out, &b.wall, &cap, [crease[k], w], [vc[k], vw[k]], mid, tol) {
            Some(e) => e,
            None => {
                let on = Traced { a: b.wall.clone(), b: cap, pts: Vec::new(), closed: false };
                let at = |s: f64| on.project(crate::util::space::lerp(crease[k], w, s));
                let (e, err) = fitted(&mut out, &|s: f64| if s <= 0.0 { crease[k] } else if s >= 1.0 { w } else { at(s) },
                    vc[k], vw[k])?;
                fit_err = fit_err.max(err);
                e
            }
        };
        let ball = arc(&mut out, c, d, q[k], w, vq[k], vw[k]);
        caps.push([bottom, ball, side]);
    }
    let middle = canal(0.5, 0.0);
    let plane = Surface::Plane(Frame::about(q[0], b.toward));
    let wall = turned_from(&b.wall, middle);
    let chain = turned_from(&b.chain, middle);
    let sheet = Surface::BSpline(Frame::new([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Arc::new(net));
    // outward from the piece at each face's first vertex
    face_out(&mut out, plane, scale(b.toward, -1.0), &[crease_edge, caps[1][0], seam_arc, caps[0][0]], "a")?;
    face_out(&mut out, chain, toward(&b.chain, sc, q[0]), &[seam_arc, touch_chain], "a")?;
    face_out(&mut out, wall, toward(&b.wall, sw, crease[0]), &[crease_edge, caps[1][2], touch_wall, caps[0][2]], "b")?;
    face_out(&mut out, sheet, sub(caps_at[0].0, q[0]), &[touch_chain, caps[1][1], touch_wall, caps[0][1]], "round")?;
    for k in 0..2 {
        let (c, _, d) = caps_at[k];
        let cap = Surface::Plane(Frame::about(c, d));
        face_out(&mut out, cap, if k == 0 { scale(d, -1.0) } else { d }, &caps[k], ["start", "end"][k])?;
    }
    out.measure_tolerances();
    for e in [touch_chain, touch_wall, caps[0][1], caps[1][1]] {
        let t = &mut out.edges[e].tol;
        *t = t.max(net_err);
    }
    out.check(10.0 * tol.max(net_err).max(fit_err)).map_err(|e| format!("the round beyond the seam: {e}"))?;
    Ok(out)
}

/// A torus's or a cylinder's frame turned so its seam (first parameter nought) stands opposite
/// `p` about its axis, away from a piece about `p`; any other surface as it is.
fn turned_from(s: &Surface, p: V) -> Surface {
    let frame = |f: &Frame| {
        let w = sub(p, f.o);
        let radial = sub(w, scale(f.z, dot(w, f.z)));
        (norm(radial) > 1e-12).then(|| { let x = scale(unit(radial), -1.0); Frame { x, y: cross(f.z, x), ..*f } })
    };
    match s {
        Surface::Torus(f, big, r) => frame(f).map_or(s.clone(), |g| Surface::Torus(g, *big, *r)),
        Surface::Cylinder(f, r) => frame(f).map_or(s.clone(), |g| Surface::Cylinder(g, *r)),
        _ => s.clone(),
    }
}
