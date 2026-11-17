//! **What a document's objects are cut from** (#940, `solventc --parts`): each part — the objects
//! of one binding being one part placed where each instance stands, as `--step-assembly` writes
//! them — with how many there are and the box it is cut from, its three sides longest first
//! (a board's length, width and thickness), in the document's units.  Most of a bill of
//! materials, and a lumber build's cut list.
//!
//! The box is the part's own, whichever way it stands in the assembly: the smallest of the boxes
//! square to its faces (each pair of the largest flat faces' normals, and its own axes), so a
//! rafter pitched at 25° is its board's length, not the box round it in the world.

use crate::model::Sketch;
use crate::solid::ApproximationPolicy;
use crate::space::{cross, dot, norm};
use crate::json::{object, Json};

/// One part, and the objects that are it.
#[derive(Clone, Debug, PartialEq)]
pub struct Stock {
    /// What the part is called: an instance's solid by its name in the component (`post`), any
    /// other object by the name the document gives it.
    pub name: String,
    /// The box's sides, longest first, in the document's units.
    pub size: [f64; 3],
    /// Each object that is this part, by the name a reader writes (`bent_a.post`).
    pub objects: Vec<String>,
}

/// **The document's parts and what each is cut from**, longest first, or the first object that
/// would not evaluate, said why.
pub fn cut_list(sk: &Sketch) -> Result<Vec<Stock>, String> {
    // by binding first, each part with its volume
    let mut parts: Vec<(String, Stock, f64)> = Vec::new();
    for i in crate::overview::objects(sk) {
        let key = crate::colours::part(sk, i);
        let called = sk.solid_name(i);
        if let Some((_, s, _)) = parts.iter_mut().find(|(k, _, _)| *k == key) {
            s.objects.push(called);
            continue;
        }
        let solid = sk.evaluated_solid(i, ApproximationPolicy::Report)
            .map_err(|why| format!("`{called}`: {why}"))?;
        let points: Vec<[f64; 3]> =
            solid.boundary().iter().flat_map(|p| p.pts.iter().copied()).collect();
        let normals = solid.boundary().iter().map(|p| (p.n, area(&p.pts)));
        let name = match &sk.solids[i].binding {
            Some(b) => crate::vocab::public_path(
                sk.solids[i].name.strip_prefix(b.at.as_str()).unwrap_or(&sk.solids[i].name)),
            None => called.clone(),
        };
        let size = oriented(&points, normals);
        parts.push((key, Stock { name, size, objects: vec![called] }, solid.volume()));
    }
    // then the parts no binding made one that are one all the same: called alike in their
    // components (`one.body`, `two.body`), and of one box and one volume — two boards of one cut
    let mut out: Vec<(Stock, f64)> = Vec::new();
    for (_, s, volume) in parts {
        let tol = 1e-6 * s.size[0].max(1e-300);
        let alike = |o: &(Stock, f64)| leaf(&o.0.name) == leaf(&s.name)
            && (0..3).all(|k| (o.0.size[k] - s.size[k]).abs() <= tol)
            && (o.1 - volume).abs() <= 1e-6 * volume.abs().max(1e-300);
        match out.iter_mut().find(|o| alike(o)) {
            Some((o, _)) => {
                if o.name != s.name { o.name = leaf(&s.name).to_string(); }
                o.objects.extend(s.objects);
            }
            None => out.push((s, volume)),
        }
    }
    let mut out: Vec<Stock> = out.into_iter().map(|(s, _)| s).collect();
    out.sort_by(|a, b| b.size[0].total_cmp(&a.size[0]).then(a.name.cmp(&b.name)));
    Ok(out)
}

/// What a part is called within whatever holds it: `body` of `one.body`, `post[1]` of
/// `bent.post[1]`.
fn leaf(name: &str) -> &str {
    name.rsplit('.').next().unwrap_or(name)
}

/// A planar polygon's area.
fn area(pts: &[[f64; 3]]) -> f64 {
    let mut s = [0.0; 3];
    for k in 1..pts.len().saturating_sub(1) {
        let c = cross(sub(pts[k], pts[0]), sub(pts[k + 1], pts[0]));
        s = [s[0] + c[0], s[1] + c[1], s[2] + c[2]];
    }
    0.5 * norm(s)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > 1e-12).then(|| [a[0] / n, a[1] / n, a[2] / n])
}

/// How many of the largest face directions the box is tried square to.
const DIRECTIONS: usize = 24;

/// **The smallest box square to a solid's faces** round `points`: its sides, longest first.  Tried
/// square to each pair of the `DIRECTIONS` directions with the most flat face facing them, and to
/// the solid's own axes; a body with no flat face (a turned one) is boxed square to its axes.
pub fn oriented(points: &[[f64; 3]], faces: impl Iterator<Item = ([f64; 3], f64)>) -> [f64; 3] {
    // each direction, either way, with the area facing it
    let mut dirs: Vec<([f64; 3], f64)> = Vec::new();
    for (n, a) in faces {
        let Some(n) = unit(n) else { continue };
        match dirs.iter_mut().find(|(d, _)| dot(*d, n).abs() > 1.0 - 1e-9) {
            Some((_, total)) => *total += a,
            None => dirs.push((n, a)),
        }
    }
    dirs.sort_by(|a, b| b.1.total_cmp(&a.1));
    dirs.truncate(DIRECTIONS);
    let mut frames: Vec<[[f64; 3]; 3]> = vec![[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]];
    for (i, &(a, _)) in dirs.iter().enumerate() {
        for &(b, _) in &dirs[i + 1..] {
            let along = dot(a, b);
            if along.abs() > 1.0 - 1e-6 { continue }
            let Some(v) = unit([b[0] - along * a[0], b[1] - along * a[1], b[2] - along * a[2]])
            else { continue };
            frames.push([a, v, cross(a, v)]);
        }
    }
    let mut best: Option<(f64, [f64; 3])> = None;
    for f in frames {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in points {
            for k in 0..3 {
                let t = dot(*p, f[k]);
                lo[k] = lo[k].min(t);
                hi[k] = hi[k].max(t);
            }
        }
        let mut size = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        if size.iter().any(|s| !s.is_finite()) { continue }
        size.sort_by(|a, b| b.total_cmp(a));
        let volume = size[0] * size[1] * size[2];
        // a smaller box by more than rounding, else the shorter longest side
        let better = best.is_none_or(|(v, s)| volume < v * (1.0 - 1e-9)
            || (volume <= v * (1.0 + 1e-9) && size[0] < s[0] * (1.0 - 1e-9)));
        if better { best = Some((volume, size)); }
    }
    best.map_or([0.0; 3], |(_, s)| s)
}

/// **The cut list as a reader is told it**: a heading, then a line a part — how many, what it
/// is, its sides longest first in the document's unit, and the objects that are it.
pub fn text(sk: &Sketch, parts: &[Stock]) -> Vec<String> {
    let count: usize = parts.iter().map(|p| p.objects.len()).sum();
    let unit = sk.units.name();
    let mut out = vec![format!("cut list: {} part{}, {count} object{} (length × width × thickness, \
        {unit})", parts.len(), if parts.len() == 1 { "" } else { "s" },
        if count == 1 { "" } else { "s" })];
    let fig = |v: f64| crate::expr::reading(crate::constraints::SpecKind::Length, v);
    for p in parts {
        out.push(format!("  {} × {}: {} × {} × {} — {}", p.objects.len(), p.name, fig(p.size[0]),
            fig(p.size[1]), fig(p.size[2]), p.objects.join(", ")));
    }
    out
}

/// **The cut list as JSON**: per part, its `name`, `count`, `length`, `width` and `thickness` in
/// the document's `unit`, and its `objects`.
pub fn json(sk: &Sketch, parts: &[Stock]) -> Json {
    Json::Arr(parts.iter().map(|p| object([
        ("name", p.name.as_str().into()),
        ("count", Json::Int(p.objects.len() as i64)),
        ("length", Json::Num(p.size[0])),
        ("width", Json::Num(p.size[1])),
        ("thickness", Json::Num(p.size[2])),
        ("unit", sk.units.name().into()),
        ("objects", Json::Arr(p.objects.iter().map(|o| o.as_str().into()).collect())),
    ])).collect())
}
