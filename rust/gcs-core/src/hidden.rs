//! **A picture of a solid, in a view** (Solvent §6.11) — the draughtsman's three answers: which
//! edges are there, which of them the eye can see, and where they land on the page.
//!
//! What a part sheet used to be written as, and what item 9 of issue #48 is about: the V-twin's
//! cylinder drew its body three times, once as the design and twice as page-aligned rectangles
//! re-tied by `project`, with every depth ordinate related to the section by nothing at all.  A
//! solid knows its own depths, so a view is a *question* asked of it.
//!
//! Three rules, each of which is the whole of a draughtsman's convention:
//!
//! * **A corner is drawn and a tessellation seam is not.**  The flats of a bore's wall meet at
//!   angles no one designed, so `csg::edges` marks them `smooth` and they are drawn only where
//!   they are a **silhouette** — where the surface turns away from the eye, which is the two
//!   lines a cylinder is drawn as and not the sixty-four its facets would give.
//! * **What the material covers is dashed, not dropped.**  A hidden line is a line: the eye's
//!   ray from the middle of each piece is classified against the term, and a piece the solid
//!   stands in front of carries the class `.hidden`, which is the name every sheet already
//!   styles.
//! * **Where a view lands is the view's own business.**  Everything here comes back in *page*
//!   coordinates, through `plane::on_page`, so a derived view sits on the sheet at its plane's
//!   own origin and rotor — over the geometry drawn there, for the face it was swept from.
//!
//! The core projects and the front end strokes, the seam `callout.rs` sits on: an SVG export and
//! a canvas get the same polylines and neither owns a line of 3D arithmetic.

use crate::model::Sketch;
use crate::plane::{self, Basis};
use crate::solid;

/// One stroke of a derived picture, in page coordinates.
#[derive(Clone, Debug)]
pub struct Stroke {
    pub pts: Vec<(f64, f64)>,
    /// True where the solid stands between this edge and the eye.
    pub hidden: bool,
    /// A silhouette rather than a corner — a cylinder's side, drawn where the surface turns
    /// away.  Kept apart because it is *not* a fact about the object, only about this view.
    pub silhouette: bool,
    /// `body.bore.wall` — which face of which solid it bounds.
    pub path: String,
}

/// How near two points must be, relative to the drawing, to be one point.
const SAME: f64 = 1e-7;

/// **The edges of `si` as `plane` sees them.**
///
/// `unit` is the world length of one screen pixel, exactly as a callout and a curve use it: the
/// faceting of every round surface follows it, so a view drawn at one zoom is refined for that
/// zoom and no finer.
pub fn view(sk: &Sketch, si: usize, plane_i: Option<usize>, unit: f64) -> Vec<Stroke> {
    let (basis, pose) = view_frame(sk, plane_i);
    let eye = basis.normal();
    let edges = sk.solid_edges(si, unit);
    let csg = solid::resolve(sk, si, unit);
    let eps = sk.extent() * solid::EPS;

    // **which edges this view draws at all**: every corner, and a smooth seam only where the
    // surface turns away from the eye across it
    let mut drawn: Vec<(([f64; 3], [f64; 3]), bool, String)> = Vec::new();
    for e in &edges {
        let sil = e.smooth;
        if sil {
            let (a, b) = (plane::dot(e.na, eye), plane::dot(e.nb, eye));
            // a silhouette is where the sign changes; a seam whose two facets both face the eye
            // (or both face away) is the tessellation and is not drawn
            if a * b > 0.0 || (a.abs() < 1e-12 && b.abs() < 1e-12) {
                continue;
            }
        }
        drawn.push(((e.a, e.b), sil, e.path.clone()));
    }

    // **split where one edge crosses another in the picture** (Appel): visibility can only
    // change at an apparent crossing or at an end, so between two of them a whole piece is seen
    // or a whole piece is not
    let flat = |p: [f64; 3]| basis.view_coords(p);
    let mut out = Vec::new();
    for (i, ((a, b), sil, path)) in drawn.iter().enumerate() {
        let (pa, pb) = (flat(*a), flat(*b));
        let d3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let mut cuts = vec![0.0f64, 1.0];
        for (j, ((c, d), _, _)) in drawn.iter().enumerate() {
            if i == j {
                continue;
            }
            if let Some(t) = cross(pa, pb, flat(*c), flat(*d)) {
                cuts.push(t);
            }
        }
        cuts.sort_by(|x, y| x.partial_cmp(y).expect("no NaN from a finite drawing"));
        for w in cuts.windows(2) {
            if w[1] - w[0] < 1e-9 {
                continue;
            }
            let at = |t: f64| [a[0] + t * d3[0], a[1] + t * d3[1], a[2] + t * d3[2]];
            let m = at((w[0] + w[1]) / 2.0);
            let hidden = behind(&csg, m, eye, eps);
            let (p, q) = (at(w[0]), at(w[1]));
            out.push(Stroke {
                pts: vec![on_page(&basis, pose, p), on_page(&basis, pose, q)],
                hidden,
                silhouette: *sil,
                path: path.clone(),
            });
        }
    }
    join(out)
}

/// **A section**: the solid cut at `at`, drawn in `plane`.  The cut face is the loops the
/// material makes in the cutting plane; what stands beyond it is drawn as a view would.
pub fn section(
    sk: &Sketch,
    si: usize,
    at: Option<usize>,
    plane_i: Option<usize>,
    unit: f64,
) -> Vec<Stroke> {
    let (cut, _) = view_frame(sk, at);
    let (basis, pose) = view_frame(sk, plane_i);
    let csg = solid::resolve(sk, si, unit);
    let eps = sk.extent() * solid::EPS;
    let n = cut.normal();
    let d = plane::dot(n, cut.o);

    // every boundary piece meets the cutting plane in a segment; a segment whose middle has
    // material on one side of it *within the plane* is an edge of the cut face
    let mut segs: Vec<([f64; 3], [f64; 3], String)> = Vec::new();
    for p in sk.solid_boundary(si, unit) {
        if let Some((a, b)) = meet_plane(&p.pts, n, d) {
            segs.push((a, b, p.path.clone()));
        }
    }
    let mut out = Vec::new();
    for (a, b, path) in segs {
        let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
        // a segment lying *in* the cut and bounding material is drawn; one the cut merely
        // grazes from outside is not
        let along = plane::unit([b[0] - a[0], b[1] - a[1], b[2] - a[2]]);
        let Some(along) = along else { continue };
        let across = plane::cross(n, along);
        let one = csg.inside(step(m, across, eps));
        let two = csg.inside(step(m, across, -eps));
        if one == two {
            continue;
        }
        out.push(Stroke {
            pts: vec![on_page(&basis, pose, a), on_page(&basis, pose, b)],
            hidden: false,
            silhouette: false,
            path,
        });
    }
    // and what stands beyond the cut, as a view of it
    for s in view(sk, si, plane_i, unit) {
        out.push(s);
    }
    join(out)
}

/// The plane a picture is drawn in, and the pose that puts it on the page.  `None` is the page
/// itself, which is what a document with no `plane` statement draws in.
fn view_frame(sk: &Sketch, plane_i: Option<usize>) -> (Basis, (f64, f64, (f64, f64))) {
    match plane_i.and_then(|i| sk.planes.get(i)) {
        Some(p) => (
            p.basis,
            (
                sk.params[p.frame.c as usize].value,
                sk.params[p.frame.s as usize].value,
                sk.point_xy(p.frame.origin as usize),
            ),
        ),
        None => (Basis::page(), (1.0, 0.0, (0.0, 0.0))),
    }
}

fn on_page(b: &Basis, pose: (f64, f64, (f64, f64)), p: [f64; 3]) -> (f64, f64) {
    plane::on_page(pose.0, pose.1, pose.2, b.view_coords(p))
}

fn step(p: [f64; 3], d: [f64; 3], k: f64) -> [f64; 3] {
    [p[0] + k * d[0], p[1] + k * d[1], p[2] + k * d[2]]
}

/// Does the solid stand between `m` and the eye?  Orthographic, so the eye's ray is a line along
/// the view's own normal — the whole of what "hidden" means here.
fn behind(csg: &solid::Csg, m: [f64; 3], eye: [f64; 3], eps: f64) -> bool {
    let bb = csg.bbox();
    if bb.is_empty() {
        return false;
    }
    let reach = (0..3).fold(0.0f64, |a, i| a.max(bb.hi[i] - bb.lo[i])) * 2.0 + 1.0;
    // walked out along the ray, so a thin wall between the edge and the eye is not stepped over
    let n = 64;
    for k in 1..=n {
        let t = eps * 4.0 + reach * k as f64 / n as f64;
        if csg.inside(step(m, eye, t)) {
            return true;
        }
    }
    false
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
        let n = dx.hypot(dy);
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
            .push(Span { t0, t1, hidden: s.hidden, sil: s.silhouette, path: s.path });
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
                let path = over
                    .iter()
                    .map(|s| &s.path)
                    .min()
                    .cloned()
                    .unwrap_or_default();
                out.push(Stroke {
                    pts: vec![at_t(base, d, t0), at_t(base, d, t1)],
                    hidden,
                    silhouette: sil,
                    path,
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
    let scale = v
        .iter()
        .flat_map(|s| s.pts.iter())
        .fold(1.0f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
    let tol = scale * SAME;
    // an edge seen end-on is a point, and a point is not a line
    v.retain(|s| {
        let (a, b) = (s.pts[0], s.pts[s.pts.len() - 1]);
        (a.0 - b.0).hypot(a.1 - b.1) > tol
    });
    let v = overlay(v, tol.max(1e-12));
    let mut out: Vec<Stroke> = Vec::with_capacity(v.len());
    for s in v {
        let joined = out.iter_mut().find(|t| {
            t.hidden == s.hidden
                && t.silhouette == s.silhouette
                && t.path == s.path
                && collinear(t, &s, tol)
        });
        match joined {
            Some(t) => {
                let (ta, tb) = (t.pts[0], t.pts[t.pts.len() - 1]);
                let (sa, sb) = (s.pts[0], s.pts[s.pts.len() - 1]);
                let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1) <= tol;
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
    out
}

fn collinear(t: &Stroke, s: &Stroke, tol: f64) -> bool {
    let (ta, tb) = (t.pts[0], t.pts[t.pts.len() - 1]);
    let (sa, sb) = (s.pts[0], s.pts[s.pts.len() - 1]);
    let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1) <= tol;
    if !(near(tb, sa) || near(ta, sb)) {
        return false;
    }
    let d1 = (tb.0 - ta.0, tb.1 - ta.1);
    let d2 = (sb.0 - sa.0, sb.1 - sa.1);
    let n1 = d1.0.hypot(d1.1).max(1e-300);
    let n2 = d2.0.hypot(d2.1).max(1e-300);
    ((d1.0 * d2.1 - d1.1 * d2.0) / (n1 * n2)).abs() < 1e-6
}

/// **Every picture the document asked for, laid out.**  The one entry both front ends read, so
/// the SVG export and the canvas are one picture of one drawing and not two — `callout::layout`'s
/// bargain, and the reason `paint.ts` owns no 3D arithmetic.
///
/// The classes are the core's: `.visible`, `.hidden`, `.section` under whatever the statement
/// itself carries, so a sheet says what a hidden line looks like the way it says what a
/// dimension does, and a document that already writes `style .hidden` gets it for free.
pub fn layout(sk: &Sketch, unit: f64) -> Vec<Drawn> {
    let mut out = Vec::new();
    for (i, d) in sk.derived.iter().enumerate() {
        let strokes = match d.at {
            None => view(sk, d.solid as usize, d.plane.map(|p| p as usize), unit),
            Some(at) => section(
                sk,
                d.solid as usize,
                Some(at as usize),
                d.plane.map(|p| p as usize),
                unit,
            ),
        };
        for s in strokes {
            let mut class = crate::style::Classes(vec![
                if s.hidden { "hidden".to_string() } else { "visible".to_string() },
            ]);
            if d.at.is_some() && !s.hidden {
                class.0.push("section".to_string());
            }
            class.0.extend(d.class.0.iter().cloned());
            out.push(Drawn {
                of: i,
                solid: sk.solids.get(d.solid as usize).map(|x| x.name.clone()).unwrap_or_default(),
                path: s.path,
                hidden: s.hidden,
                silhouette: s.silhouette,
                style: crate::style::resolve(&sk.sheet, &class),
                pts: s.pts,
            });
        }
    }
    out
}

/// One polyline of a derived picture, resolved: page coordinates and the ink to stroke it in.
#[derive(Clone, Debug)]
pub struct Drawn {
    /// Which `view`/`section` statement asked for it.
    pub of: usize,
    pub solid: String,
    pub path: String,
    pub hidden: bool,
    pub silhouette: bool,
    pub style: crate::style::Style,
    pub pts: Vec<(f64, f64)>,
}
