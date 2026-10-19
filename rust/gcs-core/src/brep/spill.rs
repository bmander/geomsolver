//! **A section against a smooth chain, beyond the seam's normal surface** (#422): where a ring's
//! section — a chamfer's flat, a round's ball — reaches across a seam where the face it stands on
//! goes on tangent into another (a plate's far face into its rim's round) and the seam is no circle
//! about the ring's axis, the section against the chain is the one against whichever face of it is
//! nearest, and changes face where it crosses the seam's normal surface (the cylinder through a
//! plane's and a torus's seam, along their common normal).  Short of that surface the ring stands
//! against its own face, a revolution kept within it (`solid::lowering`'s `Ring` and its `Kept`);
//! beyond it, the piece here: what the section sweeps against the next face, sectioned in the
//! half-planes through the ring's axis — the ring's own sections — and fitted, as
//! `fillet::flat_along` fits a flat along a traced run.  The two are glued on the normal surface
//! (`boolean::glued`), where both are the same set: there a point's distance from either face is
//! its distance from the seam.
//!
//! A flat's piece beyond is a lens: on the next face the strip between the seam and the leg, the
//! flat between the leg and its crossing with the normal surface, and on that surface the cap
//! between the crossing and the seam, the three meeting where the leg on the ring's own face
//! crosses the seam (`ends`).  What is only fitted is the flat beyond; the cap's and the strip's
//! surfaces are the faces' own, and the seam is the circle it is.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::fillet::{column, crossing_plane, face_out};
use super::geom::{Curve, Frame, Surface, Traced, V};
use super::nurbs::{fit_curve, fit_net};
use super::topo::{Brep, EdgeCurve};
use crate::space::{add, cross, distance, dot, norm, scale, sub, unit};
use std::sync::Arc;

/// A chamfer's flat or a round beyond a seam's normal surface (all in the boundary's units): the
/// edge it runs along (`along`, whose sections it is read in), which way from the face it stands on
/// the region lies (`toward`, unit); the chain's next face (`chain`), the other face (`wall`) and
/// the seam's normal surface (`split`), with which way each one's signed distance runs toward the
/// piece (`sides`, ±1); the seam (`seam`); a chamfer's legs — on the chain from the wall, on the
/// wall from the chain — or a round's `radius` (nothing for a chamfer, `round_beyond`); and where
/// the leg on the face it stands on crosses the seam, or where the ball touches it (`ends`).
#[derive(Clone, Debug)]
pub struct Beyond {
    pub along: Along,
    pub toward: V,
    pub chain: Surface,
    pub wall: Surface,
    pub split: Surface,
    pub sides: [f64; 3],
    pub seam: Seam,
    pub legs: [f64; 2],
    pub radius: f64,
    pub ends: [V; 2],
}

/// The edge a section beyond a seam is read along, and so its sections.
#[derive(Clone, Debug)]
pub enum Along {
    /// A ring about the line through `axis[0]` along the unit `axis[1]`, its edge the circle
    /// `crease` about it (centre, radius): its sections the half-planes through the axis.
    Ring { axis: [V; 2], crease: (V, f64) },
    /// The meeting of the face the section stands on (`face`) and the wall, which no line or
    /// coaxial circle carries: its sections the planes square to it, as `fillet::flat_along`'s.
    Meeting { face: Surface },
}

/// A seam a chain's face goes on across: a circle (centre, unit normal, radius) or a line (a point
/// of it, its unit direction).
#[derive(Clone, Copy, Debug)]
pub enum Seam {
    Circle(V, V, f64),
    Line(V, V),
}

/// A piece's sections between its ends: each `u` in [0, 1] the edge's point and the unit normal of
/// the section's plane through it (the way `u` runs), the ends in that order.
struct Sections<'a> {
    at: Box<dyn Fn(f64) -> (V, V) + 'a>,
    ends: [V; 2],
}

/// `b`'s sections between its ends, the shorter stretch of its edge between them.
fn sections<'a>(b: &'a Beyond, tol: f64) -> Result<Sections<'a>, String> {
    match &b.along {
        Along::Ring { axis: [o, z], crease: (centre, radius) } => {
            let (o, z, centre, radius) = (*o, *z, *centre, *radius);
            let flat_of = move |p: V| { let w = sub(p, o); sub(w, scale(z, dot(w, z))) };
            // the sections' angles about the axis, from the ends' middle
            let e1 = unit(flat_of(scale(add(b.ends[0], b.ends[1]), 0.5)));
            let e2 = cross(z, e1);
            let angle = |p: V| { let w = flat_of(p); dot(w, e2).datan2(dot(w, e1)) };
            let (mut q, mut f) = (b.ends, b.ends.map(angle));
            if f[0] > f[1] {
                q.swap(0, 1);
                f.swap(0, 1);
            }
            if !(f[1] - f[0] > 1e-9 && f[1] - f[0] < std::f64::consts::PI) {
                return Err("a piece beyond a seam whose ends stand at one section".into());
            }
            let radial = move |phi: f64| { let (s, c) = phi.dsin_cos(); add(scale(e1, c), scale(e2, s)) };
            Ok(Sections {
                at: Box::new(move |u: f64| {
                    let phi = f[0] + u * (f[1] - f[0]);
                    (add(centre, scale(radial(phi), radius)), cross(z, radial(phi)))
                }),
                ends: q,
            })
        }
        Along::Meeting { face } => {
            let meet = Traced { a: face.clone(), b: b.wall.clone(), pts: Vec::new(), closed: false };
            let feet = b.ends.map(|q| meet.project(q));
            let size = distance(b.ends[0], b.ends[1]) + b.legs[0].max(b.legs[1]) + b.radius;
            let curve = match super::ssi::intersect(face, &b.wall, tol) {
                super::ssi::Ssi::Curves(cs) => cs,
                super::ssi::Ssi::Same => return Err("a face and a wall on one surface".into()),
                super::ssi::Ssi::Traced => {
                    let lo = [0, 1, 2].map(|k| feet[0][k].min(feet[1][k]) - 4.0 * size);
                    let hi = [0, 1, 2].map(|k| feet[0][k].max(feet[1][k]) + 4.0 * size);
                    super::ssi::trace(face, &b.wall, &feet, lo, hi, tol)?
                }
            }.into_iter().min_by(|x, y| {
                let off = |c: &Curve| feet.iter().map(|&p| distance(c.point(c.inverse(p)), p)).sum::<f64>();
                off(x).total_cmp(&off(y))
            }).ok_or("a face and a wall that meet nowhere")?;
            // the stretch between the ends' sections — each end's nearest point of the meeting, the
            // plane square to it there through the end — the one whose middle stands nearer the
            // ends'
            let t = b.ends.map(|p| curve.inverse(p));
            let mid = scale(add(b.ends[0], b.ends[1]), 0.5);
            let (t0, span) = match curve.period() {
                Some(period) => {
                    let fwd = (t[1] - t[0]).rem_euclid(period);
                    let near = |t0: f64, span: f64| distance(meet.project(curve.point(t0 + 0.5 * span)), mid);
                    if near(t[0], fwd) <= near(t[1], period - fwd) { (t[0], fwd) } else { (t[1], period - fwd) }
                }
                None => (t[0].min(t[1]), (t[1] - t[0]).abs()),
            };
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
            })
        }
    }
}

/// The seam's stretch between the ends `q` as an edge of `out`, from `vq[0]` to `vq[1]`: a circle's
/// arc (the shorter, nearer the edge `near`), or a line's.
fn seam_edge(out: &mut Brep, seam: Seam, q: [V; 2], vq: [u32; 2], near: V) -> usize {
    match seam {
        Seam::Line(..) => out.edge(EdgeCurve::Curve(Curve::Line { p: q[0], d: unit(sub(q[1], q[0])) }),
            [0.0, distance(q[0], q[1])], vq) as usize,
        Seam::Circle(sc_o, sc_n, sc_r) => {
            let x = unit(sub(q[0], sc_o));
            let circle = Frame { o: sc_o, x, y: cross(sc_n, x), z: sc_n };
            let at = |p: V| { let w = sub(p, sc_o); dot(w, circle.y).datan2(dot(w, circle.x)).rem_euclid(std::f64::consts::TAU) };
            let end = at(q[1]);
            let off = |t: f64| distance(circle.at([sc_r * t.dcos(), sc_r * t.dsin(), 0.0]), near);
            let (frame, t) = if off(0.5 * end) <= off(0.5 * end + std::f64::consts::PI) {
                (circle, [0.0, end])
            } else {
                // the other way round: the frame turned over, so the arc still runs from the first end
                let z = scale(sc_n, -1.0);
                (Frame { o: sc_o, x, y: cross(z, x), z }, [0.0, std::f64::consts::TAU - end])
            };
            out.edge(EdgeCurve::Curve(Curve::Circle(frame, sc_r)), t, vq) as usize
        }
    }
}

/// The piece of a chamfer's flat beyond the seam's normal surface (`Beyond`), fitted within `tol`.
pub fn flat_beyond(b: &Beyond, tol: f64) -> Result<Brep, String> {
    let [sc, sw, ss] = b.sides;
    let [lc, lw] = b.legs;
    let Sections { at: section, ends: q } = sections(b, tol)?;
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
        let mut p = crate::space::lerp(pa, pb, v);
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
        crate::roots::newton_onto::<3>(guess, size, |p| Some([
            (b.split.gradient(p), b.split.implicit(p)), (grad(p), sum(p)), (n, dot(n, sub(p, o))),
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
        let guess = crate::space::lerp(table[k], table[k + 1], x - k as f64);
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
    let split = turned_from(&b.split, middle);
    let sheet = Surface::BSpline(Frame::new([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Arc::new(net));
    let p = q[0];
    face_out(&mut out, chain, scale(b.chain.gradient(p), -sc), &[seam, leg], "a")?;
    face_out(&mut out, sheet, grad(p), &[leg, cut], "flat")?;
    face_out(&mut out, split, scale(b.split.gradient(p), -ss), &[cut, seam], "split")?;
    out.measure_tolerances();
    for e in [leg, cut] {
        let t = &mut out.edges[e].tol;
        *t = t.max(net_err).max(cut_err);
    }
    out.check(10.0 * tol.max(net_err).max(cut_err)).map_err(|e| format!("the flat beyond the seam: {e}"))?;
    Ok(out)
}

/// **A round's piece beyond a seam's normal surface** (#422's rung 3): the ball of radius `r` beside
/// the wall touches the chain's next face (the torus) where its centre stands past the seam's
/// offset — between the half-planes through the ring's axis where the ball touches the seam
/// (`ends`, its touches there) — and the ring's own face elsewhere, a ring's stretch glued to this at
/// both ends.  Its centre runs where the wall's offset meets the torus's, read in each half-plane
/// (the ring's sections: at its ends the ball's arc lies in them, its centre's path square to them
/// there); its canal is the ball's arc between where it touches the torus and the wall, fitted.  The
/// piece: the canal; on the torus the lens between the seam and the touch; on the ring's own face
/// (a plane) the strip from the edge to the seam; on the wall the strip from the edge to the touch;
/// and in each end's half-plane the ring's section, a cap.
pub fn round_beyond(b: &Beyond, r: f64, tol: f64) -> Result<Brep, String> {
    let Along::Ring { axis: [o, z], crease } = b.along else {
        return Err("a round beyond a seam along an edge no circle carries".into());
    };
    let [sc, sw, _] = b.sides;
    let flat_of = |p: V| { let w = sub(p, o); sub(w, scale(z, dot(w, z))) };
    let e1 = unit(flat_of(scale(add(b.ends[0], b.ends[1]), 0.5)));
    let e2 = cross(z, e1);
    let angle = |p: V| { let w = flat_of(p); dot(w, e2).datan2(dot(w, e1)) };
    let (mut q, mut f) = (b.ends, b.ends.map(angle));
    if f[0] > f[1] {
        q.swap(0, 1);
        f.swap(0, 1);
    }
    if !(f[1] - f[0] > 1e-9 && f[1] - f[0] < std::f64::consts::PI) {
        return Err("a round beyond a seam whose ends stand at one section".into());
    }
    let radial = |phi: f64| { let (s, c) = phi.dsin_cos(); add(scale(e1, c), scale(e2, s)) };
    let phi_at = |u: f64| f[0] + u * (f[1] - f[0]);
    let (centre, radius) = crease;
    // the ball's centre in the half-plane at `phi`: where the wall's offset meets the torus's, from
    // the crease's point stepped off both faces
    let spine = Traced {
        a: b.wall.offset(sw * r).ok_or("a ball larger than the wall holds")?,
        b: b.chain.offset(sc * r).ok_or("a ball larger than the torus holds")?,
        pts: Vec::new(), closed: false,
    };
    let toward = |s: &Surface, side: f64, p: V| scale(unit(s.gradient(p)), -side);
    let frames = std::cell::RefCell::new(std::collections::BTreeMap::<u64, (V, [V; 2])>::new());
    let frame_at = |u: f64| -> (V, [V; 2]) {
        let u = u.clamp(0.0, 1.0);
        if let Some(&fr) = frames.borrow().get(&u.to_bits()) {
            return fr;
        }
        let phi = phi_at(u);
        let p = add(centre, scale(radial(phi), radius));
        let guess = add(add(p, scale(scale(unit(b.wall.gradient(p)), sw), r)), scale(b.toward, r));
        let c = crossing_plane(&spine, o, cross(z, radial(phi)), guess);
        let fr = (c, [toward(&b.chain, sc, c), toward(&b.wall, sw, c)]);
        frames.borrow_mut().insert(u.to_bits(), fr);
        fr
    };
    let size = r + radius;
    for (k, &end) in q.iter().enumerate() {
        let (c, [et, _]) = frame_at(k as f64);
        let touch = add(c, scale(et, r));
        if distance(touch, end) > 1e-6 * (size + norm(end)) {
            return Err(format!("the ball beyond the seam misses the seam at its end ({:.2e} off)", distance(touch, end)));
        }
    }
    let canal = |u: f64, v: f64| { let (c, e) = frame_at(u); add(c, scale(super::fillet::slerp(e[0], e[1], v), r)) };
    let bar = tol.max(1e-7 * (size + norm(q[0])));
    let (net, net_err) = fit_net(&canal, 3, 3, 64, 4, tol).ok_or("the canal beyond the seam could not be fitted")?;
    if net_err > bar {
        return Err(format!("the canal beyond the seam misses its fit by {net_err:e}"));
    }
    let [[u0, u1], [v0, v1]] = net.domain();
    let last_v = net.poles[0].len() - 1;
    // the points: the crease's, the seam's touches and the wall's touches at each end
    let crease = |k: usize| add(centre, scale(radial(f[k]), radius));
    let ends = [0, 1].map(|k| { let (c, e) = frame_at(k as f64); (c, add(c, scale(e[1], r))) });
    let mut out = Brep::default();
    let vc = [0, 1].map(|k| out.vertex(crease(k)));
    let vq = [0, 1].map(|k| out.vertex(net.point([u0, u1][k], v0)));
    let vw = [0, 1].map(|k| out.vertex(net.point([u0, u1][k], v1)));
    // the crease's arc and the seam's, each from the first end to the second the short way
    let arc = |out: &mut Brep, c: V, n: V, a: V, z_: V, va: u32, vz: u32| -> usize {
        let x = unit(sub(a, c));
        let n = if dot(cross(x, sub(z_, c)), n) < 0.0 { scale(n, -1.0) } else { n };
        let frame = Frame { o: c, x, y: cross(n, x), z: n };
        let w = sub(z_, c);
        let end = dot(w, frame.y).datan2(dot(w, frame.x));
        out.edge(EdgeCurve::Curve(Curve::Circle(frame, distance(a, c))), [0.0, end], [va, vz]) as usize
    };
    let crease_arc = arc(&mut out, centre, z, crease(0), crease(1), vc[0], vc[1]);
    let Seam::Circle(so, sn, _) = b.seam else { return Err("a round beyond a seam no circle".into()) };
    let seam_arc = arc(&mut out, so, sn, q[0], q[1], vq[0], vq[1]);
    let touch_chain = out.edge(EdgeCurve::Curve(column(&net, 0)), [u0, u1], vq) as usize;
    let touch_wall = out.edge(EdgeCurve::Curve(column(&net, last_v)), [u0, u1], vw) as usize;
    // each cap: along the plane from the crease to the seam, the ball's arc, down the wall
    let mut caps = Vec::new();
    for k in 0..2 {
        let (c, w) = ends[k];
        let bottom = out.edge(EdgeCurve::Curve(Curve::Line { p: crease(k), d: unit(sub(q[k], crease(k))) }),
            [0.0, distance(crease(k), q[k])], [vc[k], vq[k]]) as usize;
        let side = out.edge(EdgeCurve::Curve(Curve::Line { p: crease(k), d: unit(sub(w, crease(k))) }),
            [0.0, distance(crease(k), w)], [vc[k], vw[k]]) as usize;
        let ball = arc(&mut out, c, cross(z, radial(f[k])), q[k], w, vq[k], vw[k]);
        caps.push([bottom, ball, side]);
    }
    let middle = canal(0.5, 0.0);
    let plane = Surface::Plane(Frame::about(q[0], b.toward));
    let wall = turned_from(&b.wall, middle);
    let chain = turned_from(&b.chain, middle);
    let sheet = Surface::BSpline(Frame::new([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Arc::new(net));
    // outward from the piece at each face's first vertex
    face_out(&mut out, plane, scale(b.toward, -1.0), &[crease_arc, caps[1][0], seam_arc, caps[0][0]], "a")?;
    face_out(&mut out, chain, toward(&b.chain, sc, q[0]), &[seam_arc, touch_chain], "a")?;
    face_out(&mut out, wall, toward(&b.wall, sw, crease(0)), &[crease_arc, caps[1][2], touch_wall, caps[0][2]], "b")?;
    let (c0, _) = frame_at(0.0);
    face_out(&mut out, sheet, sub(c0, q[0]), &[touch_chain, caps[1][1], touch_wall, caps[0][1]], "round")?;
    for k in 0..2 {
        let n = cross(z, radial(f[k]));
        let cap = Surface::Plane(Frame::about(crease(k), n));
        face_out(&mut out, cap, if k == 0 { scale(n, -1.0) } else { n }, &caps[k], ["start", "end"][k])?;
    }
    out.measure_tolerances();
    for e in [touch_chain, touch_wall, caps[0][1], caps[1][1]] {
        let t = &mut out.edges[e].tol;
        *t = t.max(net_err);
    }
    out.check(10.0 * tol.max(net_err)).map_err(|e| format!("the round beyond the seam: {e}"))?;
    let _ = (v0, v1);
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
