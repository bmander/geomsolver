//! One transport for both sweeps and lofts. Sections are solved geometry at the guide ends;
//! omitting the second section repeats the first in the transported frame.
use super::*;
use plane::{cross, dot, norm, scaled};

use crate::space::{add, sub};

type V = [f64; 3];
fn mix(a: V, b: V, t: f64) -> V {
    add(scaled(a, 1.0 - t), scaled(b, t))
}

pub(super) fn point(sk: &Sketch, i: usize) -> V {
    sk.world_point(i)
}

struct Guide {
    start: V,
    delta: V,
    center: V,
    axis: V,
    angle: f64,
    tangent: V,
}
impl Guide {
    fn read(sk: &Sketch, e: EntRef) -> Result<Self, String> {
        let (a, b) =
            crate::model::edge_ends(sk, e).ok_or("a guide must be a line or circular arc")?;
        let start = point(sk, a as usize);
        let delta = sub(point(sk, b as usize), start);
        let mut g = Self {
            start,
            delta,
            center: [0.0; 3],
            axis: [0.0; 3],
            angle: 0.0,
            tangent: [0.0; 3],
        };
        match e.kind {
            EntKind::Line => {
                g.tangent = plane::unit(delta).ok_or("a guide needs distinct finite endpoints")?;
            }
            EntKind::Arc => {
                let arc = &sk.arcs[e.i()];
                let plane = sk.plane_of(arc.center as usize);
                if [a, b].iter().any(|&p| sk.plane_of(p as usize) != plane) {
                    return Err("an arc guide's center and endpoints must share a plane".into());
                }
                g.center = point(sk, arc.center as usize);
                g.axis = plane.map_or(Basis::page(), |i| sk.planes[i].basis).normal();
                let radius = sub(start, g.center);
                let r = norm(radius);
                let declared = sk.params[arc.radius as usize].value.abs();
                if !r.is_finite()
                    || r <= 0.0
                    || (r - declared).abs() > r * 1e-7
                    || (norm(sub(point(sk, b as usize), g.center)) - r).abs() > r * 1e-7
                {
                    return Err(
                        "an arc guide needs a finite positive radius and endpoints on its circle"
                            .into(),
                    );
                }
                let (s, e) = sk.arc_angles(e.i());
                g.angle = e - s;
                if !g.angle.is_finite() || g.angle <= 0.0 || g.angle >= std::f64::consts::TAU - 1e-9
                {
                    return Err("an arc guide must be an open circular arc".into());
                }
                g.tangent = plane::unit(cross(g.axis, radius)).ok_or("degenerate arc guide")?;
            }
            _ => return Err("a guide must be a line or circular arc".into()),
        }
        if start.iter().chain(&delta).any(|x| !x.is_finite()) {
            return Err("a guide needs finite coordinates".into());
        }
        Ok(g)
    }
    fn rotate(&self, p: V, t: f64) -> V {
        let (s, c) = (self.angle * t).sin_cos();
        add(
            add(scaled(p, c), scaled(cross(self.axis, p), s)),
            scaled(self.axis, dot(self.axis, p) * (1.0 - c)),
        )
    }
    fn at(&self, p: V, t: f64) -> V {
        if self.angle == 0.0 {
            add(p, scaled(self.delta, t))
        } else {
            add(self.center, self.rotate(sub(p, self.center), t))
        }
    }
    fn untransport(&self, p: V) -> V {
        if self.angle == 0.0 {
            sub(p, self.delta)
        } else {
            add(self.center, self.rotate(sub(p, self.center), -1.0))
        }
    }
}

/// Cache both the world placement and the defining geometry, including arc branch/radius.
pub(super) fn reads(sk: &Sketch, guide: EntRef, v: &mut Vec<f64>) {
    v.extend([guide.kind as u32 as f64, guide.idx as f64]);
    for p in sk.entity_params(guide) {
        v.push(sk.params[p as usize].value);
    }
    for e in sk.children(guide) {
        if e.kind == EntKind::Point {
            v.extend([e.idx as f64, sk.plane_of(e.i()).map_or(-1.0, |p| p as f64)]);
            v.extend(point(sk, e.i()));
            if let Some(p) = sk.plane_of(e.i()) {
                v.extend(sk.planes[p].basis.u);
                v.extend(sk.planes[p].basis.v);
            }
        }
    }
}

struct Pair {
    start: FacePoly,
    end: FacePoly,
}
pub(super) struct Loft {
    guide: Guide,
    pairs: Vec<Pair>,
    steps: usize,
    forward: bool,
}

/// Keep every source corner and tessellate each corresponding curve at the finer count.
/// Shared angular samples avoid tiny slivers from overlaying two unrelated chord grids.
fn pair(
    mut a: FacePoly,
    mut b: FacePoly,
    ac: &[Option<(f64, f64)>],
    bc: &[Option<(f64, f64)>],
) -> Result<Pair, String> {
    fn edges(p: &mut FacePoly) -> Vec<Vec<usize>> {
        let n = p.pts.len();
        let first = (0..n)
            .find(|&i| p.of[i].0 == 0 && (p.of[(i + n - 1) % n].0 != 0 || p.names.len() == 1))
            .unwrap_or(0);
        p.pts.rotate_left(first);
        p.of.rotate_left(first);
        let mut groups: Vec<Vec<usize>> = Vec::new();
        for i in 0..n {
            if i == 0 || p.of[i].0 != p.of[i - 1].0 {
                groups.push(Vec::new());
            }
            groups.last_mut().unwrap().push(i);
        }
        groups
    }
    // A whole circle has no distinguished vertex or phase. Pair radial directions, so
    // changing the end plane's drawing axes cannot twist or collapse a circular loft.
    if a.names.len() == 1 && b.names.len() == 1 {
        if let (Some(ac), Some(bc)) = (ac[0], bc[0]) {
            let angle = (a.pts[0].1 - ac.1).atan2(a.pts[0].0 - ac.0);
            let radius = (b.pts[0].0 - bc.0).hypot(b.pts[0].1 - bc.1);
            let count = b.pts.len();
            for (i, p) in b.pts.iter_mut().enumerate() {
                let (s, c) = (angle + std::f64::consts::TAU * i as f64 / count as f64).sin_cos();
                *p = (bc.0 + radius * c, bc.1 + radius * s);
            }
        }
    }
    let ag = edges(&mut a);
    let bg = edges(&mut b);
    if ag.len() != bg.len() || a.names.len() != b.names.len() {
        return Err("loft boundaries must have matching numbers of source edges (and holes in matching order)".into());
    }
    let mut ap = Vec::new();
    let mut bp = Vec::new();
    let mut provenance = Vec::new();
    for (aa, bb) in ag.iter().zip(&bg) {
        let count = aa.len().max(bb.len());
        let sampler = |p: &FacePoly, group: &[usize], centers: &[Option<(f64, f64)>]| {
            let first = p.pts[group[0]];
            let last = p.pts[(group.last().unwrap() + 1) % p.pts.len()];
            let center = centers[p.of[group[0]].0];
            let mut angle = 0.0;
            if let Some(c) = center {
                for &i in group {
                    let (a, b) = (p.pts[i], p.pts[(i + 1) % p.pts.len()]);
                    let (u, v) = ((a.0 - c.0, a.1 - c.1), (b.0 - c.0, b.1 - c.1));
                    angle += (u.0 * v.1 - u.1 * v.0).atan2(u.0 * v.0 + u.1 * v.1);
                }
            }
            move |t: f64| {
                if let Some(c) = center {
                    let (sn, cs) = (angle * t).sin_cos();
                    let (x, y) = (first.0 - c.0, first.1 - c.1);
                    (c.0 + x * cs - y * sn, c.1 + x * sn + y * cs)
                } else {
                    (
                        first.0 + (last.0 - first.0) * t,
                        first.1 + (last.1 - first.1) * t,
                    )
                }
            }
        };
        let sa = sampler(&a, aa, ac);
        let sb = sampler(&b, bb, bc);
        for i in 0..count {
            let t = i as f64 / count as f64;
            ap.push(sa(t));
            bp.push(sb(t));
            provenance.push(a.of[aa[0]]);
        }
    }
    a.pts = ap;
    b.pts = bp;
    a.of = provenance.clone();
    b.of = provenance;
    Ok(Pair { start: a, end: b })
}

pub(super) fn prepare(
    sk: &Sketch,
    face: u32,
    end: Option<u32>,
    guide: EntRef,
    unit: f64,
) -> Result<Loft, String> {
    let g = Guide::read(sk, guide)?;
    let start = face_polys(sk, face as usize, unit)?;
    let basis = start[0].basis;
    let n = basis.normal();
    let scale = start
        .iter()
        .flat_map(|p| (0..p.pts.len()).map(|i| norm(sub(p.lift(i), g.start))))
        .fold(norm(g.delta), f64::max)
        .max(if g.angle != 0.0 {
            norm(sub(g.start, g.center))
        } else {
            0.0
        });
    let tol = scale * 1e-7;
    if dot(n, g.tangent).abs() < 1.0 - 1e-9 || dot(n, sub(g.start, basis.o)).abs() > tol {
        return Err(
            "the start section must lie at the guide start, perpendicular to its tangent".into(),
        );
    }
    let ends = if let Some(f) = end {
        let mut polys = face_polys(sk, f as usize, unit)?;
        if polys.len() != start.len() {
            return Err("loft sections must have matching numbers of holes".into());
        }
        let end_basis = polys[0].basis;
        let tangent = g.rotate(g.tangent, 1.0);
        if dot(end_basis.normal(), tangent).abs() < 1.0 - 1e-9
            || dot(end_basis.normal(), sub(g.at(g.start, 1.0), end_basis.o)).abs() > tol
        {
            return Err(
                "the end section must lie at the guide end, perpendicular to its tangent".into(),
            );
        }
        for p in &mut polys {
            p.pts = (0..p.pts.len())
                .map(|i| {
                    let q = sub(g.untransport(p.lift(i)), basis.o);
                    (dot(q, basis.u), dot(q, basis.v))
                })
                .collect();
            p.basis = basis;
            *p = p.ccw();
        }
        polys
    } else {
        start.clone()
    };
    let centers = |f: u32, transport: bool| {
        sk.faces[f as usize]
            .boundaries()
            .map(|(edges, _)| {
                edges
                    .iter()
                    .map(|e| {
                        if !matches!(e.kind, EntKind::Circle | EntKind::Arc) {
                            return None;
                        }
                        let mut c = point(sk, sk.round_center(*e));
                        if transport {
                            c = g.untransport(c);
                        }
                        let q = sub(c, basis.o);
                        Some((dot(q, basis.u), dot(q, basis.v)))
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>()
    };
    let ac = centers(face, false);
    let bc = end.map_or_else(|| ac.clone(), |f| centers(f, true));
    let pairs = start
        .into_iter()
        .zip(ends)
        .enumerate()
        .map(|(i, (a, b))| pair(a, b, &ac[i], &bc[i]))
        .collect::<Result<Vec<_>, _>>()?;
    if pairs
        .iter()
        .flat_map(|p| &p.start.names)
        .any(|n| n == "start" || n == "end")
    {
        return Err(
            "edge name collides with a sweep cap (`start` or `end`); rename the source edge".into(),
        );
    }
    let mut rmax = 0.0f64;
    if g.angle != 0.0 {
        let radial = plane::unit(sub(g.start, g.center)).unwrap();
        for pair in &pairs {
            for p in [&pair.start, &pair.end] {
                for i in 0..p.pts.len() {
                    let r = dot(sub(p.lift(i), g.center), radial);
                    if r <= tol {
                        return Err("the arc guide's bend axis touches or crosses a section".into());
                    }
                    rmax = rmax.max(r);
                }
            }
        }
    }
    let flatness = crate::curve::flatness(unit);
    let step = if rmax > flatness {
        2.0 * (1.0 - flatness / rmax).acos()
    } else {
        std::f64::consts::TAU
    };
    let steps = if g.angle == 0.0 {
        1
    } else {
        ((g.angle / step.min(std::f64::consts::TAU / 64.0)).ceil() as usize).clamp(3, 2048)
    };
    if pairs
        .iter()
        .map(|p| p.start.pts.len())
        .sum::<usize>()
        .saturating_mul(steps)
        > 262144
    {
        return Err("loft exceeds the section/guide tessellation budget".into());
    }
    // Validate the interpolated sections in the transported frame, including hole clearance.
    // Extra stations on a straight guide catch collapsing or crossed correspondences too.
    if end.is_some() {
        for k in 0..=steps.max(32) {
            let t = k as f64 / steps.max(32) as f64;
            let polys: Vec<_> = pairs
                .iter()
                .map(|pair| {
                    let mut p = pair.start.clone();
                    p.pts = p
                        .pts
                        .iter()
                        .zip(&pair.end.pts)
                        .map(|(a, b)| (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t))
                        .collect();
                    p
                })
                .collect();
            for (i, p) in polys.iter().enumerate() {
                if !p.valid() || p.area() <= 0.0 {
                    return Err("loft correspondence creates a crossed or collapsed section".into());
                }
                if i > 0
                    && (section::loops_touch(&polys[0], p)
                        || !section::inside_loop(&polys[0], p.pts[0])
                        || polys[1..i].iter().any(|q| {
                            section::loops_touch(q, p)
                                || section::inside_loop(q, p.pts[0])
                                || section::inside_loop(p, q.pts[0])
                        }))
                {
                    return Err("loft correspondence makes holes touch, overlap or leave the outer boundary".into());
                }
            }
        }
    }
    let forward = dot(n, g.tangent) > 0.0;
    Ok(Loft {
        guide: g,
        pairs,
        steps,
        forward,
    })
}

/// Cut a planar region into convex strips between vertex ordinates. Within a strip no
/// boundary vertices or crossings occur, so sorted intersections pair by even/odd fill.
/// This handles any number of holes without subtracting two nearly coincident swept shells.
fn cap(polys: &[&FacePoly]) -> Vec<Vec<(f64, f64)>> {
    let mut ys: Vec<f64> = polys
        .iter()
        .flat_map(|p| p.pts.iter().map(|p| p.1))
        .collect();
    ys.sort_by(f64::total_cmp);
    let tol = (ys.last().unwrap() - ys[0]).abs() * 1e-12;
    ys.dedup_by(|a, b| (*a - *b).abs() <= tol);
    let mut out = Vec::new();
    for band in ys.windows(2) {
        let (lo, hi) = (band[0], band[1]);
        let mid = (lo + hi) * 0.5;
        let mut hits = Vec::new();
        for p in polys {
            for i in 0..p.pts.len() {
                let (a, b) = (p.pts[i], p.pts[(i + 1) % p.pts.len()]);
                if (a.1 > mid) == (b.1 > mid) {
                    continue;
                }
                let at = |y: f64| {
                    if (y - a.1).abs() <= tol {
                        a
                    } else if (y - b.1).abs() <= tol {
                        b
                    } else {
                        (a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1), y)
                    }
                };
                hits.push((at(mid).0, at(lo), at(hi)));
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        for interval in hits.chunks_exact(2) {
            let (l, r) = (interval[0], interval[1]);
            let mut quad = vec![l.1, r.1, r.2, l.2];
            quad.dedup_by(|a, b| (a.0 - b.0).hypot(a.1 - b.1) <= tol);
            if quad.len() > 2
                && (quad[0].0 - quad.last().unwrap().0).hypot(quad[0].1 - quad.last().unwrap().1)
                    <= tol
            {
                quad.pop();
            }
            if quad.len() >= 3 {
                out.push(quad);
            }
        }
    }
    out
}

impl Loft {
    pub(super) fn primitive(&self, origin: V, name: &str) -> Prim {
        let mut faces = vec!["start".into(), "end".into()];
        let mut facets = Vec::new();
        let mut push = |mut pts: Vec<V>, face, smooth, reverse| {
            if reverse {
                pts.reverse();
            }
            if let Some(n) = facet_normal(&pts) {
                facets.push(Facet {
                    pts,
                    n,
                    face,
                    smooth,
                });
            }
        };
        for (t, face, reverse) in [(0.0, 0, self.forward), (1.0, 1, !self.forward)] {
            let polys: Vec<_> = self
                .pairs
                .iter()
                .map(|p| if t == 0.0 { &p.start } else { &p.end })
                .collect();
            let basis = polys[0].basis;
            for polygon in cap(&polys) {
                push(
                    polygon
                        .iter()
                        .map(|&(x, y)| sub(self.guide.at(basis.lift(x, y), t), origin))
                        .collect(),
                    face,
                    false,
                    reverse,
                );
            }
        }
        for (hole, pair) in self.pairs.iter().enumerate() {
            let p = &pair.start;
            let n = p.pts.len();
            let base = faces.len();
            faces.extend(p.names.clone());
            let at = |i: usize, k: usize| {
                let t = k as f64 / self.steps as f64;
                sub(
                    self.guide.at(mix(p.lift(i), pair.end.lift(i), t), t),
                    origin,
                )
            };
            for k in 0..self.steps {
                for i in 0..n {
                    let j = (i + 1) % n;
                    let (edge, smooth) = p.of[i];
                    let (a, b, c, d) = (at(i, k), at(j, k), at(j, k + 1), at(i, k + 1));
                    let normal = cross(sub(b, a), sub(c, a));
                    let planar =
                        dot(normal, sub(d, a)).abs() <= norm(normal) * norm(sub(d, a)) * 1e-10;
                    let reverse = (!self.forward) != (hole > 0);
                    if planar {
                        push(
                            vec![a, b, c, d],
                            edge + base,
                            smooth || self.guide.angle != 0.0,
                            reverse,
                        );
                    } else {
                        push(
                            vec![a, b, c],
                            edge + base,
                            smooth || self.guide.angle != 0.0,
                            reverse,
                        );
                        push(
                            vec![a, c, d],
                            edge + base,
                            smooth || self.guide.angle != 0.0,
                            reverse,
                        );
                    }
                }
            }
        }
        // The strip caps introduce vertices along wall edges. Join those subdivisions before
        // seam extraction (which needs identical shared segments), not only at mesh export.
        let pieces: Vec<_> = facets
            .iter()
            .map(|f| crate::csg::Piece {
                pts: f.pts.clone(),
                n: f.n,
                path: String::new(),
                prim: 0,
                smooth: f.smooth,
            })
            .collect();
        let welded = crate::mesh::weld(&pieces);
        debug_assert_eq!(welded.len(), facets.len());
        for (facet, piece) in facets.iter_mut().zip(welded) {
            facet.pts = piece.pts;
        }
        finish(facets, faces, name)
    }
}
