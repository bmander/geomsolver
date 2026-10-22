//! **Where a load scenario falls on the drawing** (#510): a mark for every line of it — its
//! supports, its loads, its requirements and its readings in the plane — read off the solved
//! drawing, no analysis made.
//!
//! A mark is the places it is drawn at (its anchors), and at each the direction it acts along —
//! a force's, a pressure's into its face, the one direction a support holds where it says
//! `along:` — and, on a face, the face's outward normal there; the faces it tints; its size among
//! the scenario's loads of its kind; and its words, the value as a reader reads it (`600N`,
//! `2MPa`, `pinned along z`).  Every coordinate is here: a host chooses only what a mark looks
//! like (a cone, an arrow, a plate) and how large it is on screen.
//!
//! A face a path names is read off the mesh the workspace already draws (`EvaluatedSolid::mesh`,
//! asked at the object's own scale, as the box asks), its groups named by their primitive and
//! face, `lug.root` or `pin.bore` — so a mark on a face builds no exact boundary.  **A mark adds
//! nothing to the drawing**: no entity, unknown or row.

use crate::model::{
    Applied, EntKind, EntRef, FaceRef, Hold, LoadKind, LoadValue, Output, Sketch,
};
use crate::program::Stated;
use crate::solid::{ApproximationPolicy, LocalPoint};
use crate::space::{add, cross, distance, lerp, norm, normalised, scale, sub};
use crate::units::Dim;

type V = [f64; 3];

/// How many places a mark along a line or an edge is drawn at, its ends among them.
pub const ALONG: usize = 7;
/// The most places a mark on faces is drawn at, spread over them.
pub const OVER: usize = 12;

/// What a mark is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Fixed,
    Pinned,
    Load,
    Pressure,
    Accelerate,
    Requirement,
    Plane,
}

impl Kind {
    /// The word a host draws it by.
    pub fn word(self) -> &'static str {
        match self {
            Kind::Fixed => "fixed",
            Kind::Pinned => "pinned",
            Kind::Load => "load",
            Kind::Pressure => "pressure",
            Kind::Accelerate => "accelerate",
            Kind::Requirement => "requirement",
            Kind::Plane => "plane",
        }
    }
}

/// **One line of a scenario, drawn**: what it is, which line (its index among the scenario's
/// supports, loads, requirements or readings, by `kind`), its anchors and at each the direction
/// it acts along (empty where it acts along none) and a face's outward normal (empty off a face),
/// the faces it tints as triangles, its size among the scenario's loads of its kind (0 to 1; 1
/// for the rest), its words, and where its words are drawn.
#[derive(Clone, Debug, PartialEq)]
pub struct Mark {
    pub kind: Kind,
    pub index: usize,
    pub at: Vec<V>,
    pub dir: Vec<V>,
    pub normal: Vec<V>,
    pub tint: Vec<[V; 3]>,
    pub size: f64,
    pub text: String,
    pub label: V,
    /// A requirement read at a place — a deflection — which a host marks there, as against one of
    /// a whole body (its mass, its safety), which has no place of its own.
    pub probe: bool,
}

/// **Every mark of scenario `s`**, at the drawing `sk` stands at, in the order its lines are
/// written: supports, loads, requirements, readings.  A line whose target has no place to be
/// drawn (a face path naming no face) has no mark.
pub fn marks(sk: &Sketch, s: &Stated) -> Vec<Mark> {
    let sc = &s.scenario;
    let reach = Reach::of(sk);
    let mut out = Vec::new();
    for (k, sup) in sc.supports.iter().enumerate() {
        let Some(on) = reach.anchors(sk, &sup.at) else { continue };
        let held = sup.along.and_then(|d| crate::diagnose::analysis::direction(sk, d).ok())
            .and_then(normalised);
        let kind = if sup.hold == Hold::Fixed { Kind::Fixed } else { Kind::Pinned };
        let text = match (sup.along, held) {
            (Some(_), Some(d)) => format!("{} along {}", sup.hold.word(), direction_text(d)),
            _ => sup.hold.word().to_string(),
        };
        let dir = held.map_or(Vec::new(), |d| vec![d; on.at.len()]);
        out.push(on.mark(kind, k, dir, text));
    }
    for (k, load) in sc.loads.iter().enumerate() {
        let Some(on) = reach.anchors(sk, &load.at) else { continue };
        let (kind, dir, magnitude) = match (load.kind, &load.value) {
            (LoadKind::Pressure, LoadValue::Scalar(p)) => {
                // a pressure pushes into the material: against the face's outward normal
                let sign = if *p < 0.0 { 1.0 } else { -1.0 };
                (Kind::Pressure, on.normal.iter().map(|n| scale(*n, sign)).collect(), p.abs())
            }
            (kind, value) => {
                let v = match value {
                    LoadValue::Vector(v) => *v,
                    LoadValue::Along { magnitude, along } => {
                        match crate::diagnose::analysis::direction(sk, *along) {
                            Ok(d) => scale(normalised(d).unwrap_or([0.0; 3]), *magnitude),
                            Err(_) => [0.0; 3],
                        }
                    }
                    LoadValue::Scalar(v) => [0.0, 0.0, *v],
                };
                let kind =
                    if kind == LoadKind::Acceleration { Kind::Accelerate } else { Kind::Load };
                let d = normalised(v).unwrap_or([0.0; 3]);
                (kind, vec![d; on.at.len()], norm(v))
            }
        };
        let text = quantity(sk, load.kind.dim(), magnitude);
        let mut m = on.mark(kind, k, dir, text);
        m.size = magnitude;
        out.push(m);
    }
    for (k, r) in sc.requirements.iter().enumerate() {
        let Some(on) = reach.anchors(sk, &r.of) else { continue };
        // a requirement is drawn once, where its words are
        let at = on.label;
        let text = s.heads.get(k).cloned().unwrap_or_else(|| r.output.word().to_string());
        let mut m = Anchors { at: vec![at], normal: Vec::new(), tint: Vec::new(), label: at }
            .mark(Kind::Requirement, k, Vec::new(), text);
        if r.output == Output::Deflection {
            m.probe = true;
            m.dir = r.along.and_then(|d| crate::diagnose::analysis::direction(sk, d).ok())
                .and_then(normalised).map_or(Vec::new(), |d| vec![d]);
        }
        out.push(m);
    }
    for (k, r) in sc.readings.iter().enumerate() {
        let Some(on) = reach.anchors(sk, &r.of) else { continue };
        let at = on.label;
        out.push(Anchors { at: vec![at], normal: Vec::new(), tint: Vec::new(), label: at }
            .mark(Kind::Plane, k, Vec::new(), format!("plane {}", r.plane.word())));
    }
    sized(&mut out, Kind::Load);
    sized(&mut out, Kind::Pressure);
    sized(&mut out, Kind::Accelerate);
    out
}

/// Each mark of `kind`'s size as a fraction of the largest of them.
fn sized(marks: &mut [Mark], kind: Kind) {
    let most = marks.iter().filter(|m| m.kind == kind).map(|m| m.size).fold(0.0, f64::max);
    for m in marks.iter_mut().filter(|m| m.kind == kind) {
        m.size = if most > 0.0 { m.size / most } else { 1.0 };
    }
}

/// What a target is drawn at: its anchors, a face's outward normal at each (empty off a face),
/// the faces it tints, and where its words go — the anchor nearest the anchors' middle.
struct Anchors {
    at: Vec<V>,
    normal: Vec<V>,
    tint: Vec<[V; 3]>,
    label: V,
}

impl Anchors {
    fn of(at: Vec<V>, normal: Vec<V>, tint: Vec<[V; 3]>) -> Option<Anchors> {
        if at.is_empty() {
            return None;
        }
        let mid = scale(at.iter().fold([0.0; 3], |a, &p| add(a, p)), 1.0 / at.len() as f64);
        let label = *at.iter().min_by(|a, b| distance(**a, mid).total_cmp(&distance(**b, mid)))?;
        Some(Anchors { at, normal, tint, label })
    }

    fn mark(self, kind: Kind, index: usize, dir: Vec<V>, text: String) -> Mark {
        Mark { kind, index, at: self.at, dir, normal: self.normal, tint: self.tint, size: 1.0,
               text, label: self.label, probe: false }
    }
}

/// The scale of the drawing, for how finely an edge is followed.
struct Reach {
    unit: f64,
}

impl Reach {
    fn of(sk: &Sketch) -> Reach {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for i in 0..sk.points.len() {
            let p = sk.world_point(i);
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
        Reach { unit: 1e-3 * if extent.is_finite() && extent > 0.0 { extent } else { 1.0 } }
    }

    /// Where `a` is drawn: a point; `ALONG` places along a line, an edge or each line of a beam;
    /// a solid at its middle; faces a path names at up to `OVER` places spread over them, with
    /// their normals, tinted.
    fn anchors(&self, sk: &Sketch, a: &Applied) -> Option<Anchors> {
        match a {
            Applied::Ent(e) => match e.kind {
                EntKind::Point => Anchors::of(vec![sk.world_point(e.i())], Vec::new(), Vec::new()),
                EntKind::Solid => {
                    let tris = triangles(sk, e.i(), |_| true);
                    Anchors::of(vec![middle(&tris)?], Vec::new(), Vec::new())
                }
                _ => Anchors::of(self.along(sk, *e), Vec::new(), Vec::new()),
            },
            Applied::Beam(b) => {
                let at = sk.beams.get(*b)?.lines.iter()
                    .flat_map(|&l| self.along(sk, EntRef { kind: EntKind::Line, idx: l as u32 }))
                    .collect();
                Anchors::of(at, Vec::new(), Vec::new())
            }
            Applied::Face(f) => {
                let name = sk.solids.get(f.solid as usize)?.name.clone();
                let tris = triangles(sk, f.solid as usize, |path| names_group(&name, f, path));
                let (at, normal) = spread(&tris);
                Anchors::of(at, normal, tris)
            }
        }
    }

    /// `ALONG` places along entity `e` as the drawing draws it, evenly by length, in space.
    fn along(&self, sk: &Sketch, e: EntRef) -> Vec<V> {
        let poly: Vec<V> = match e.kind {
            EntKind::Line => {
                let [a, b] = sk.line_ends(e.i());
                vec![sk.world_point(a), sk.world_point(b)]
            }
            EntKind::Point => vec![sk.world_point(e.i())],
            _ if e.kind.bears_points() || e.kind == EntKind::Curve => {
                let view = match e.kind {
                    EntKind::Curve => sk.curve_view(e.i()),
                    EntKind::Arc | EntKind::Circle => sk.plane_of(sk.round_center(e)),
                    _ => crate::program::plane_of_entity(sk, e),
                };
                let drawn = super::drawable(sk, e, self.unit);
                if drawn.is_empty() {
                    // a face, say: its edges, each
                    return sk.children(e).into_iter().flat_map(|c| self.along(sk, c)).collect();
                }
                drawn.into_iter().flatten().map(|p| sk.world_in(view, p)).collect()
            }
            _ => Vec::new(),
        };
        evenly(&poly, ALONG)
    }
}

/// `n` places along a polyline, evenly by length, its ends among them.
fn evenly(poly: &[V], n: usize) -> Vec<V> {
    let total: f64 = poly.windows(2).map(|w| distance(w[0], w[1])).sum();
    if poly.len() < 2 || !(total > 0.0) {
        return poly.first().copied().into_iter().collect();
    }
    let mut out = Vec::with_capacity(n);
    let (mut seg, mut done) = (0, 0.0);
    for k in 0..n {
        let want = total * k as f64 / (n - 1) as f64;
        while seg + 2 < poly.len() && done + distance(poly[seg], poly[seg + 1]) < want {
            done += distance(poly[seg], poly[seg + 1]);
            seg += 1;
        }
        let l = distance(poly[seg], poly[seg + 1]);
        let t = if l > 0.0 { ((want - done) / l).clamp(0.0, 1.0) } else { 0.0 };
        out.push(lerp(poly[seg], poly[seg + 1], t));
    }
    out
}

/// The triangles of solid `solid`'s mesh, as the workspace draws it, in the world, of the faces
/// whose group `keep` takes.  None where the solid does not evaluate.
fn triangles(sk: &Sketch, solid: usize, keep: impl Fn(&str) -> bool) -> Vec<[V; 3]> {
    let Ok(ev) = sk.evaluated_solid(solid, ApproximationPolicy::from_unit(0.0)) else {
        return Vec::new();
    };
    let m = ev.mesh();
    let world = |k: usize| -> V {
        let p = &m.positions[3 * k..3 * k + 3];
        ev.to_world(LocalPoint([p[0], p[1], p[2]])).0
    };
    let mut out = Vec::new();
    for g in m.groups.iter().filter(|g| keep(&g.path)) {
        for t in g.start..g.start + g.count {
            out.push([world(3 * t), world(3 * t + 1), world(3 * t + 2)]);
        }
    }
    out
}

/// Whether the mesh's group `group` (`lug.root`, `pin.bore`: its primitive's name and its face)
/// is a face that path `f` past solid `solid` names: its segments from the first are the path's,
/// with or without the solid's own name first — `lug.root` is `lug`'s `root`, and `pin.bore` its
/// `pin.bore`.
fn names_group(solid: &str, f: &FaceRef, group: &str) -> bool {
    let segs: Vec<&str> = group.split('.').collect();
    let starts = |segs: &[&str]| {
        f.path.len() <= segs.len() && f.path.iter().zip(segs).all(|(a, b)| same_step(a, b))
    };
    let own: Vec<&str> = solid.split('.').collect();
    starts(&segs) || (segs.starts_with(&own) && starts(&segs[own.len()..]))
}

/// Whether segment `written` names segment `mine`: the same, or a copy of it (`drill` names
/// `drill[1]`).
fn same_step(written: &str, mine: &str) -> bool {
    written == mine || mine.strip_prefix(written).and_then(|r| r.strip_prefix('['))
        .and_then(|r| r.strip_suffix(']'))
        .is_some_and(|k| !k.is_empty() && k.bytes().all(|b| b.is_ascii_digit()))
}

/// The middle of triangles' corners' box.
fn middle(tris: &[[V; 3]]) -> Option<V> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in tris.iter().flatten() {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (!tris.is_empty()).then(|| lerp(lo, hi, 0.5))
}

/// **Up to `OVER` places spread over triangles**, and each one's outward normal: their
/// centroids, the first the one nearest their middle weighted by area, each next the farthest
/// from those taken.
fn spread(tris: &[[V; 3]]) -> (Vec<V>, Vec<V>) {
    let mut c = Vec::new();
    let mut n = Vec::new();
    let mut mid = [0.0; 3];
    let mut area = 0.0;
    let reach = middle(tris).map_or(0.0, |_| {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in tris.iter().flatten() {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        distance(lo, hi)
    });
    for t in tris {
        let (ab, ac) = (sub(t[1], t[0]), sub(t[2], t[0]));
        let x = cross(ab, ac);
        let Some(normal) = normalised(x) else { continue };
        // a flat face is a few large triangles: each cut in a grid no coarser than a sixth of
        // the faces' reach, its pieces' middles the places to choose among
        let longest = distance(t[0], t[1]).max(distance(t[1], t[2])).max(distance(t[2], t[0]));
        let cuts = ((6.0 * longest / reach.max(f64::MIN_POSITIVE)).ceil() as usize).clamp(1, 12);
        let piece = norm(x) / (cuts * cuts) as f64;
        for i in 0..cuts {
            for j in 0..cuts - i {
                let (a, b) = ((i as f64 + 1.0 / 3.0) / cuts as f64,
                              (j as f64 + 1.0 / 3.0) / cuts as f64);
                let at = add(t[0], add(scale(ab, a), scale(ac, b)));
                mid = add(mid, scale(at, piece));
                area += piece;
                c.push(at);
                n.push(normal);
            }
        }
    }
    if c.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let mid = scale(mid, 1.0 / area);
    let first = (0..c.len()).min_by(|&a, &b| distance(c[a], mid).total_cmp(&distance(c[b], mid)))
        .unwrap();
    let mut taken = vec![first];
    let mut near: Vec<f64> = c.iter().map(|&p| distance(p, c[first])).collect();
    while taken.len() < OVER.min(c.len()) {
        let (next, far) = near.iter().enumerate().fold((0, -1.0), |best, (i, &d)| {
            if d > best.1 { (i, d) } else { best }
        });
        // places that are one place (a face fanned about its middle) are one anchor
        if !(far > 1e-9 * near.iter().fold(0.0f64, |m, &d| m.max(d)).max(f64::MIN_POSITIVE)) {
            break;
        }
        taken.push(next);
        for (i, d) in near.iter_mut().enumerate() {
            *d = d.min(distance(c[i], c[next]));
        }
    }
    (taken.iter().map(|&i| c[i]).collect(), taken.iter().map(|&i| n[i]).collect())
}

/// A direction as a reader reads it: `x`, `-z`, else its components to three figures.
fn direction_text(d: V) -> String {
    for (k, name) in ["x", "y", "z"].iter().enumerate() {
        if (d[k].abs() - 1.0).abs() <= 1e-9 {
            return if d[k] > 0.0 { name.to_string() } else { format!("-{name}") };
        }
    }
    let parts: Vec<String> = d.iter().map(|v| crate::json::fmt_g(*v, 3)).collect();
    format!("({})", parts.join(", "))
}

/// **A load's size as a reader reads it**, in SI with a prefix that keeps it between 1 and 1000:
/// `600N`, `2.5kN`, `100kPa`, `9.81m/s²`.
pub fn quantity(sk: &Sketch, dim: Dim, v: f64) -> String {
    let si = sk.units.to_si(v, dim);
    let unit = if dim == LoadKind::Force.dim() {
        "N"
    } else if dim == LoadKind::Pressure.dim() {
        "Pa"
    } else {
        return format!("{}m/s²", crate::json::fmt_g(si, 4));
    };
    let prefixes = [(1e9, "G"), (1e6, "M"), (1e3, "k"), (1.0, ""), (1e-3, "m")];
    let (k, p) = prefixes.iter().find(|(k, _)| si.abs() >= *k).copied().unwrap_or((1.0, ""));
    format!("{}{p}{unit}", crate::json::fmt_g(si / k, 4))
}
