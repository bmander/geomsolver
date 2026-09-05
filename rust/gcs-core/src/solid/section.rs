//! Validation of planar sections with holes, before any sweep is built.

use super::{face_poly, loop_poly, FacePoly};
use crate::model::{EntKind, EntRef, Sketch};

/// Read and validate an outer loop and strictly contained, mutually disjoint holes.
/// All boundaries use the same faceting as their sweeps and the solid classifier.
pub(super) fn face_polys(sk: &Sketch, fi: usize, unit: f64) -> Result<Vec<FacePoly>, String> {
    let outer = face_poly(sk, fi, unit).ok_or("self-intersecting or degenerate profile: a face must bound a simple nonzero region")?;
    let mut polys = vec![outer];
    let face = &sk.faces[fi];
    for (hi, hole) in face.holes.iter().enumerate() {
        let p = loop_poly(sk, &hole.edges, &hole.edge_names, sk.faces[fi].plane, unit)
            .ok_or("self-intersecting or degenerate hole boundary")?;
        if boundaries_touch(sk, &face.edges, &hole.edges) || loops_touch(&polys[0], &p) || !inside_loop(&polys[0], p.pts[0]) {
            return Err("a hole must lie strictly inside the outer boundary without touching it".into());
        }
        if polys[1..].iter().zip(&face.holes[..hi]).any(|(q, h)|
            boundaries_touch(sk, &h.edges, &hole.edges) || loops_touch(q, &p)
            || inside_loop(q, p.pts[0]) || inside_loop(&p, q.pts[0])) {
            return Err("holes must be disjoint: they cannot touch, overlap or contain one another".into());
        }
        polys.push(p);
    }
    Ok(polys)
}

fn inside_loop(poly: &FacePoly, (x, y): (f64, f64)) -> bool {
    let mut inside = false;
    for i in 0..poly.pts.len() {
        let (a, b) = (poly.pts[i], poly.pts[(i + 1) % poly.pts.len()]);
        if (a.1 > y) != (b.1 > y) && x < a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1) {
            inside = !inside;
        }
    }
    inside
}

fn loops_touch(a: &FacePoly, b: &FacePoly) -> bool {
    let scale = a.pts.iter().chain(&b.pts).fold(0.0f64, |m, p|
        m.max((p.0 - a.pts[0].0).abs()).max((p.1 - a.pts[0].1).abs()));
    let tol = scale * 1e-12;
    let area_tol = tol * scale;
    let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)|
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    let same_side = |x: f64, y: f64| (x > area_tol && y > area_tol) || (x < -area_tol && y < -area_tol);
    for i in 0..a.pts.len() {
        let (p, q) = (a.pts[i], a.pts[(i + 1) % a.pts.len()]);
        for j in 0..b.pts.len() {
            let (r, s) = (b.pts[j], b.pts[(j + 1) % b.pts.len()]);
            if p.0.min(q.0) > r.0.max(s.0) + tol || r.0.min(s.0) > p.0.max(q.0) + tol
                || p.1.min(q.1) > r.1.max(s.1) + tol || r.1.min(s.1) > p.1.max(q.1) + tol { continue; }
            if !same_side(cross(p, q, r), cross(p, q, s)) && !same_side(cross(r, s, p), cross(r, s, q)) {
                return true;
            }
        }
    }
    false
}

/// Faceting must not turn a tangency between source curves into a valid gap. Line/line
/// contacts are already exact in `loops_touch`; test the curved pairs analytically here.
fn boundaries_touch(sk: &Sketch, a: &[EntRef], b: &[EntRef]) -> bool {
    type P = (f64, f64);
    let sub = |a: P, b: P| (a.0 - b.0, a.1 - b.1);
    let dot = |a: P, b: P| a.0 * b.0 + a.1 * b.1;
    let round = |e: EntRef| {
        let c = sk.point_xy(sk.round_center(e));
        (c, sk.params[sk.round_radius(e)].value.abs())
    };
    let on_arc = |e: EntRef, p: P, tol: f64| {
        if e.kind == EntKind::Circle { return true; }
        let (c, r) = round(e);
        let (start, end) = sk.arc_angles(e.i());
        let angle = (p.1 - c.1).atan2(p.0 - c.0);
        let turn = std::f64::consts::TAU;
        let offset = (angle - start).rem_euclid(turn);
        offset <= end - start + tol / r || turn - offset <= tol / r
    };
    for &a in a {
        for &b in b {
            if a.kind == EntKind::Line && b.kind == EntKind::Line { continue; }
            let (a, b) = if a.kind == EntKind::Line { (b, a) } else { (a, b) };
            let (c, r) = round(a);
            if b.kind == EntKind::Line {
                let line = &sk.lines[b.i()];
                let p = sub(sk.point_xy(line.p1 as usize), c);
                let q = sub(sk.point_xy(line.p2 as usize), c);
                let d = sub(q, p);
                let len2 = dot(d, d);
                if len2 == 0.0 { continue; }
                let tol = r.max(len2.sqrt()) * 1e-12;
                let t = -dot(p, d) / len2;
                let closest = (p.0 + t * d.0, p.1 + t * d.1);
                let h2 = r * r - dot(closest, closest);
                if h2 < -tol * r { continue; }
                let dt = (h2.max(0.0) / len2).sqrt();
                for t in [t - dt, t + dt] {
                    if t >= -tol / len2.sqrt() && t <= 1.0 + tol / len2.sqrt()
                        && on_arc(a, (c.0 + p.0 + t * d.0, c.1 + p.1 + t * d.1), tol) {
                        return true;
                    }
                }
            } else {
                let (bc, br) = round(b);
                let d = sub(bc, c);
                let length = d.0.hypot(d.1);
                let tol = r.max(br).max(length) * 1e-12;
                if length > r + br + tol || length < (r - br).abs() - tol { continue; }
                if length <= tol {
                    if a.kind == EntKind::Circle || b.kind == EntKind::Circle { return true; }
                    for (e, other) in [(a, b), (b, a)] {
                        let arc = &sk.arcs[e.i()];
                        if [arc.start, arc.end].iter().any(|&p| on_arc(other, sk.point_xy(p as usize), tol)) {
                            return true;
                        }
                    }
                    continue;
                }
                let along = (r * r - br * br + length * length) / (2.0 * length);
                let across = (r * r - along * along).max(0.0).sqrt();
                let (u, v) = (d.0 / length, d.1 / length);
                for sign in [-1.0, 1.0] {
                    let p = (c.0 + along * u - sign * across * v, c.1 + along * v + sign * across * u);
                    if on_arc(a, p, tol) && on_arc(b, p, tol) { return true; }
                }
            }
        }
    }
    false
}
