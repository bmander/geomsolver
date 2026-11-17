//! Silhouettes, sections, visibility splitting and stroke joining for the polygon renderer.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::plane::{self, Basis};
use crate::solid::{LocalPoint, PageFrame};
use super::{Drawing, Renderer, Stroke, View};
use crate::bvh::{Bounds, Bvh};

/// How near two points must be, relative to the drawing, to be one point.
const SAME: f64 = 1e-7;

pub(super) fn project(renderer: &Renderer<'_>, view: View) -> Drawing {
    project_together(std::slice::from_ref(renderer), view)
}

/// **Several solids in one picture** (#940): each part's edges are drawn by its own rule, split
/// wherever any edge of any part crosses it in the picture, and a piece is hidden where its own
/// solid or any other stands between it and the eye — an assembly's elevation, each part behind
/// another dashed.  Lines the parts share are drawn once (`overlay`), the nearer part's ink over
/// the hidden one's.  Each part's geometry is worked in its own coordinates, and only a point
/// asked about another part is carried through the world to it.
pub(super) fn project_together(renderers: &[Renderer<'_>], view: View) -> Drawing {
    let edges: Vec<Vec<crate::csg::Edge>> = renderers.iter().map(|r| match view.section {
        Some(cut) => section_edges(r, view.frame, cut),
        None => Vec::new(),
    }).collect();
    Drawing::new(view_clipped(renderers, view.frame, view.section, &edges))
}

/// One edge of one part as this view draws it: its two ends in its part's coordinates, whether it
/// is a silhouette, its path, and its two ends on the page.
struct Drawn {
    part: usize,
    a: [f64; 3],
    b: [f64; 3],
    sil: bool,
    path: String,
    pa: (f64, f64),
    pb: (f64, f64),
}

fn view_clipped(renderers: &[Renderer<'_>], page: PageFrame, cut: Option<Basis>,
                section_edges: &[Vec<crate::csg::Edge>]) -> Vec<Stroke> {
    // **which edges this view draws at all**: each solid's rule (`EvaluatedSolid::drawn_edges`)
    let mut drawn: Vec<Drawn> = Vec::new();
    for (part, renderer) in renderers.iter().enumerate() {
        let solid = renderer.solid;
        let basis = solid.local_basis(page.basis());
        let cut = cut.map(|c| solid.local_basis(c));
        let eye = basis.normal();
        let eps = solid.epsilon();
        let none = Vec::new();
        for e in solid.drawn_edges(eye, section_edges.get(part).unwrap_or(&none)) {
            let sil = e.smooth;
            let (mut a, mut b) = (e.a, e.b);
            if let Some(cut) = cut {
                let height = |p: [f64; 3]| plane::dot(eye, std::array::from_fn(|k| p[k] - cut.o[k]));
                let (ha, hb) = (height(a), height(b));
                if ha > eps && hb > eps {
                    continue;
                }
                if (ha > eps) != (hb > eps) {
                    let t = (ha / (ha - hb)).clamp(0.0, 1.0);
                    let at = std::array::from_fn(|k| a[k] + t * (b[k] - a[k]));
                    if ha > eps {
                        a = at;
                    } else {
                        b = at;
                    }
                }
            }
            let flat = |p: [f64; 3]| solid.to_page(LocalPoint(p), page).0;
            drawn.push(Drawn { part, a, b, sil, path: e.path, pa: flat(a), pb: flat(b) });
        }
    }

    // **split where one edge crosses another in the picture** (Appel): visibility can only
    // change at an apparent crossing or at an end, so between two of them a whole piece is seen
    // or a whole piece is not
    let boxes: Vec<Bounds<2>> = drawn.iter().map(|d| {
        let (a, b) = (d.pa, d.pb);
        let lo = [a.0.min(b.0), a.1.min(b.1)];
        let hi = [a.0.max(b.0), a.1.max(b.1)];
        let pad: [f64; 2] = std::array::from_fn(|k| {
            (hi[k] - lo[k]) * 1e-9 + lo[k].abs().max(hi[k].abs()).max(1.0) * f64::EPSILON * 16.0
        });
        Bounds { lo: std::array::from_fn(|k| lo[k] - pad[k]), hi: std::array::from_fn(|k| hi[k] + pad[k]) }
    }).collect();
    let index = Bvh::new(boxes.iter().copied());
    // each part's place on the page, so a point is asked of only the parts it could be behind
    let reach: Vec<Option<[f64; 4]>> = renderers.iter().map(|r| r.bounds(page)).collect();
    let mut candidates = 0;
    let mut out = Vec::new();
    for (i, d) in drawn.iter().enumerate() {
        let solid = renderers[d.part].solid;
        let basis = solid.local_basis(page.basis());
        let eye = basis.normal();
        let local_cut = cut.map(|c| solid.local_basis(c));
        let (a, b) = (d.a, d.b);
        let d3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let mut cuts = vec![0.0f64, 1.0];
        index.query_box(boxes[i], |j| {
            if i == j { return; }
            candidates += 1;
            if let Some(t) = cross(d.pa, d.pb, drawn[j].pa, drawn[j].pb) { cuts.push(t); }
        });
        cuts.sort_by(|x, y| x.partial_cmp(y).expect("no NaN from a finite drawing"));
        for w in cuts.windows(2) {
            if w[1] - w[0] < 1e-9 {
                continue;
            }
            let at = |t: f64| [a[0] + t * d3[0], a[1] + t * d3[1], a[2] + t * d3[2]];
            let m = at((w[0] + w[1]) / 2.0);
            let limit = local_cut.map(|c| plane::dot(eye, std::array::from_fn(|k| c.o[k] - m[k])));
            let mut hidden = renderers[d.part].occludes(LocalPoint(m), eye, limit);
            if !hidden && renderers.len() > 1 {
                let world = solid.to_world(LocalPoint(m));
                let seen = solid.to_page(LocalPoint(m), page).0;
                hidden = renderers.iter().enumerate().any(|(k, other)| {
                    let within = reach[k].is_some_and(|[x0, y0, x1, y1]| {
                        let pad = other.solid.epsilon();
                        seen.0 >= x0 - pad && seen.0 <= x1 + pad
                            && seen.1 >= y0 - pad && seen.1 <= y1 + pad
                    });
                    k != d.part && within && other.occludes(other.solid.to_local(world),
                        other.solid.vector_to_local(page.basis().normal()), limit)
                });
            }
            let (p, q) = (at(w[0]), at(w[1]));
            out.push(Stroke {
                pts: vec![solid.to_page(LocalPoint(p), page).0, solid.to_page(LocalPoint(q), page).0],
                hidden,
                silhouette: d.sil,
                path: d.path.clone(),
                part: d.part,
            });
        }
    }
    let total: usize = drawn.len();
    let first = &renderers[0];
    let mut stats = first.stats.get();
    stats.crossing_candidates += candidates;
    stats.crossing_exhaustive += total * total.saturating_sub(1);
    first.stats.set(stats);
    join(out)
}

/// The edges of the face a cutting plane leaves on one solid.
fn section_edges(renderer: &Renderer<'_>, page: PageFrame, cut: Basis) -> Vec<crate::csg::Edge> {
    let solid = renderer.solid;
    let basis = solid.local_basis(page.basis());
    let eps = solid.epsilon();
    let n = basis.normal();
    let d = plane::dot(n, solid.local_basis(cut).o);

    // every boundary piece meets the cutting plane in a segment; a segment whose middle has
    // material on one side of it *within the plane* is an edge of the cut face
    let mut section_edges = Vec::new();
    for p in solid.boundary() {
        let Some((a, b)) = meet_plane(&p.pts, n, d) else { continue };
        let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
        // a segment lying *in* the cut and bounding material is drawn; one the cut merely
        // grazes from outside is not
        let along = plane::unit([b[0] - a[0], b[1] - a[1], b[2] - a[2]]);
        let Some(along) = along else { continue };
        let across = plane::cross(n, along);
        let one = solid.contains(LocalPoint(step(m, across, eps)));
        let two = solid.contains(LocalPoint(step(m, across, -eps)));
        if one == two {
            continue;
        }
        section_edges.push(crate::csg::Edge { a, b, na: n, nb: across, smooth: false, path: p.path.clone() });
    }
    section_edges
}

fn step(p: [f64; 3], d: [f64; 3], k: f64) -> [f64; 3] {
    [p[0] + k * d[0], p[1] + k * d[1], p[2] + k * d[2]]
}

/// Where the segment `a→b` is crossed, in the picture, by `c→d`.  `None` when they miss, or are
/// parallel — a parallel pair changes no visibility along its length.
fn cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> Option<f64> {
    let r = (b.0 - a.0, b.1 - a.1);
    let s = (d.0 - c.0, d.1 - c.1);
    let den = r.0 * s.1 - r.1 * s.0;
    if den.abs() < 1e-12 {
        return None;
    }
    let q = (c.0 - a.0, c.1 - a.1);
    let t = (q.0 * s.1 - q.1 * s.0) / den;
    let u = (q.0 * r.1 - q.1 * r.0) / den;
    (t > 1e-9 && t < 1.0 - 1e-9 && u > -1e-9 && u < 1.0 + 1e-9).then_some(t)
}

/// Where a planar convex piece meets a plane: a segment, or nothing.
fn meet_plane(pts: &[[f64; 3]], n: [f64; 3], d: f64) -> Option<([f64; 3], [f64; 3])> {
    let mut hits: Vec<[f64; 3]> = Vec::new();
    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        let (a, b) = (pts[i], pts[j]);
        let (sa, sb) = (plane::dot(n, a) - d, plane::dot(n, b) - d);
        if (sa > 0.0) != (sb > 0.0) && (sa - sb).abs() > 0.0 {
            let t = sa / (sa - sb);
            hits.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])]);
        }
    }
    (hits.len() >= 2).then(|| (hits[0], hits[hits.len() - 1]))
}

/// **Coincident lines are drawn once, and a line anything can see is not dashed.**
///
/// The draughtsman's rule, and it has to be an *interval* rule rather than a segment one.  A
/// block seen square on puts its far corners exactly behind its near ones, so the two agree
/// segment for segment and matching endpoints would do; but a cylinder's rim seen edge-on folds
/// in half onto its own image, and the visible half and the hidden half are split at different
/// places, so nothing matches end to end.  What is true either way is that the page has one line
/// there — so every stroke is laid on the line it belongs to, the visible stretches are unioned,
/// and the hidden ones are what is left over.
///
/// Drawn any other way, a solid outline gets a dashed one laid under it: the same ink twice, and
/// at a printer's resolution a line that reads as neither.
fn overlay(v: Vec<Stroke>, tol: f64) -> Vec<Stroke> {
    // a line, as the page has it: a canonical direction and the offset across it
    let key = |a: (f64, f64), b: (f64, f64)| {
        let (mut dx, mut dy) = (b.0 - a.0, b.1 - a.1);
        let n = dx.dhypot(dy);
        if n <= 0.0 {
            return None;
        }
        dx /= n;
        dy /= n;
        // one of the two directions, chosen the same way every time
        if dx < -1e-12 || (dx.abs() <= 1e-12 && dy < 0.0) {
            dx = -dx;
            dy = -dy;
        }
        let off = a.0 * dy - a.1 * dx;
        let g = tol.max(1e-12);
        Some((
            ((dx / g).round() as i64, (dy / g).round() as i64, (off / g).round() as i64),
            (dx, dy),
        ))
    };
    // per line: its direction, the foot of the perpendicular from the origin (so a `t` is a
    // coordinate along it and a point is `base + t·d`), and every stretch anyone drew on it
    let mut lines: std::collections::BTreeMap<
        (i64, i64, i64),
        ((f64, f64), (f64, f64), Vec<Span>),
    > = Default::default();
    for s in v {
        let (a, b) = (s.pts[0], s.pts[s.pts.len() - 1]);
        let Some((k, d)) = key(a, b) else { continue };
        let (mut t0, mut t1) = (a.0 * d.0 + a.1 * d.1, b.0 * d.0 + b.1 * d.1);
        if t1 < t0 {
            std::mem::swap(&mut t0, &mut t1);
        }
        let ta = a.0 * d.0 + a.1 * d.1;
        let base = (a.0 - ta * d.0, a.1 - ta * d.1);
        lines
            .entry(k)
            .or_insert_with(|| (d, base, Vec::new()))
            .2
            .push(Span { t0, t1, hidden: s.hidden, sil: s.silhouette, path: s.path, part: s.part });
    }
    let mut out = Vec::new();
    for (_, (d, base, spans)) in lines {
        let seen = union(spans.iter().filter(|s| !s.hidden));
        let dark = subtract(union(spans.iter().filter(|s| s.hidden)), &seen);
        for (hidden, runs) in [(false, seen), (true, dark)] {
            for (t0, t1) in runs {
                if t1 - t0 <= tol {
                    continue;
                }
                // what this stretch is *called*, and whether it is a corner: a corner outranks a
                // silhouette, since a silhouette is a fact about this view and a corner is one
                // about the object
                let over: Vec<&Span> =
                    spans.iter().filter(|s| s.t0 < t1 - tol && s.t1 > t0 + tol).collect();
                let sil = !over.is_empty() && over.iter().all(|s| s.sil);
                // and whose: the first part's of those drawing it, a seen line the part seen's
                let named = over.iter().filter(|s| s.hidden == hidden)
                    .map(|s| (s.part, &s.path)).min();
                let (part, path) = named.or_else(|| over.iter().map(|s| (s.part, &s.path)).min())
                    .map(|(k, p)| (k, p.clone()))
                    .unwrap_or_default();
                out.push(Stroke {
                    pts: vec![at_t(base, d, t0), at_t(base, d, t1)],
                    hidden,
                    silhouette: sil,
                    path,
                    part,
                });
            }
        }
    }
    out
}

/// One stretch of one page line.
struct Span {
    t0: f64,
    t1: f64,
    hidden: bool,
    sil: bool,
    path: String,
    part: usize,
}

fn at_t(base: (f64, f64), d: (f64, f64), t: f64) -> (f64, f64) {
    (base.0 + d.0 * t, base.1 + d.1 * t)
}

/// Overlapping stretches, merged.
fn union<'a>(it: impl Iterator<Item = &'a Span>) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = it.map(|s| (s.t0, s.t1)).collect();
    v.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("finite"));
    let mut out: Vec<(f64, f64)> = Vec::with_capacity(v.len());
    for (a, b) in v {
        match out.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// What is left of `a` once every stretch of `b` is taken out of it.
fn subtract(a: Vec<(f64, f64)>, b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for (mut lo, hi) in a {
        for &(c, d) in b {
            if d <= lo || c >= hi {
                continue;
            }
            if c > lo {
                out.push((lo, c));
            }
            lo = lo.max(d);
        }
        if hi > lo {
            out.push((lo, hi));
        }
    }
    out
}

/// Runs of collinear strokes of the same kind, joined end to end.
///
/// Splitting at every apparent crossing is what makes visibility right and what leaves one drawn
/// edge as a dozen segments; a reader sees a line, and a file that says so is a twelfth the size.
fn join(mut v: Vec<Stroke>) -> Vec<Stroke> {
    let Some(origin) = v.first().and_then(|s| s.pts.first()).copied() else { return v };
    for s in &mut v {
        for p in &mut s.pts {
            p.0 -= origin.0;
            p.1 -= origin.1;
        }
    }
    let scale =
        v.iter().flat_map(|s| s.pts.iter()).fold(1.0f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
    let tol = scale * SAME;
    // an edge seen end-on is a point, and a point is not a line
    v.retain(|s| {
        let (a, b) = (s.pts[0], s.pts[s.pts.len() - 1]);
        (a.0 - b.0).dhypot(a.1 - b.1) > tol
    });
    let v = overlay(v, tol.max(1e-12));
    let mut out: Vec<Stroke> = Vec::with_capacity(v.len());
    for s in v {
        let joined = out.iter_mut().find(|t| {
            t.hidden == s.hidden
                && t.silhouette == s.silhouette
                && t.path == s.path
                && t.part == s.part
                && collinear(t, &s, tol)
        });
        match joined {
            Some(t) => {
                let (ta, tb) = (t.pts[0], t.pts[t.pts.len() - 1]);
                let (sa, sb) = (s.pts[0], s.pts[s.pts.len() - 1]);
                let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).dhypot(p.1 - q.1) <= tol;
                if near(tb, sa) {
                    t.pts.pop();
                    t.pts.extend(s.pts);
                } else if near(ta, sb) {
                    let mut pts = s.pts.clone();
                    pts.pop();
                    pts.extend(std::mem::take(&mut t.pts));
                    t.pts = pts;
                }
            }
            None => out.push(s),
        }
    }
    for s in &mut out {
        for p in &mut s.pts {
            p.0 += origin.0;
            p.1 += origin.1;
        }
    }
    out
}

fn collinear(t: &Stroke, s: &Stroke, tol: f64) -> bool {
    let (ta, tb) = (t.pts[0], t.pts[t.pts.len() - 1]);
    let (sa, sb) = (s.pts[0], s.pts[s.pts.len() - 1]);
    let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).dhypot(p.1 - q.1) <= tol;
    if !(near(tb, sa) || near(ta, sb)) {
        return false;
    }
    let d1 = (tb.0 - ta.0, tb.1 - ta.1);
    let d2 = (sb.0 - sa.0, sb.1 - sa.1);
    let n1 = d1.0.dhypot(d1.1).max(1e-300);
    let n2 = d2.0.dhypot(d2.1).max(1e-300);
    ((d1.0 * d2.1 - d1.1 * d2.0) / (n1 * n2)).abs() < 1e-6
}
