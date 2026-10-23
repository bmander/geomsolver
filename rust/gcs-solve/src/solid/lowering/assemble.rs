//! **Pieces as one construction** (#194): the regions a body takes in one set are one union of
//! pieces, each distinct piece once, a run cut short at a corner's section and joined to its cell
//! there — and that construction as a recipe node (`Assembly::node`) and this kernel's B-rep.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::recipe::Writer;
use crate::json::{object, Json};
use super::{Across, Beyond, Corner, Piece, Ring, Rolled, Run, Seam, Stroke, Surf, V};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};

impl Surf {
    /// As a recipe reads it (written by `w`; `recipe::Lowered`'s faces).
    pub(super) fn json(&self, w: &Writer) -> Json {
        let (by, vector) = (w.scale, Writer::triple);
        match *self {
            Surf::Plane { origin, normal } => object([("kind", "plane".into()),
                ("origin", w.at(scale(origin, by))), ("normal", vector(normal))]),
            Surf::Cylinder { origin, axis, radius } => object([("kind", "cylinder".into()),
                ("origin", w.at(scale(origin, by))), ("axis", vector(axis)), ("radius", (radius * by).into())]),
            Surf::Cone { origin, axis, radius, half } => object([("kind", "cone".into()),
                ("origin", w.at(scale(origin, by))), ("axis", vector(axis)), ("radius", (radius * by).into()),
                ("half", half.into())]),
            Surf::Sphere { centre, radius } => object([("kind", "sphere".into()),
                ("origin", w.at(scale(centre, by))), ("radius", (radius * by).into())]),
            Surf::Torus { origin, axis, big, tube } => object([("kind", "torus".into()),
                ("origin", w.at(scale(origin, by))), ("axis", vector(axis)), ("big", (big * by).into()),
                ("tube", (tube * by).into())]),
        }
    }
}

/// What a piece is kept within at its ends, as a recipe reads it (written by `w`): each plane's
/// half-space, `within` (`at` on it, its `normal` out of it), and each curved face's solid, `kept`
/// (its surface, and whether `inside`).
fn ends_json(r: &Run, w: &Writer) -> [(&'static str, Json); 2] {
    let within = r.within.iter().map(|&(n, d)| object([("at", w.at(scale(n, d * w.scale))), ("normal", Writer::triple(n))]))
        .collect();
    let kept = r.kept.iter().map(|k| { let mut o = k.face.json(w); o.set("inside", k.inside.into()); o }).collect();
    [("within", Json::Arr(within)), ("kept", Json::Arr(kept))]
}

/// A run's section moved by `m` (a turn about a ring's axis): its strokes, its origin and way; the
/// faces it is kept within stand where they stood.
pub(super) fn turned(r: &Run, m: &crate::brep::geom::Rigid) -> Run {
    let stroke = |s: &Stroke| match *s {
        Stroke::Line { from, to } => Stroke::Line { from: m.point(from), to: m.point(to) },
        Stroke::Arc { centre, radius, x, angles } => Stroke::Arc { centre: m.point(centre), radius, x: m.vector(x), angles },
        Stroke::Conic { a, m: mid, b, w } => Stroke::Conic { a: m.point(a), m: m.point(mid), b: m.point(b), w },
    };
    Run {
        origin: m.point(r.origin), along: m.vector(r.along), span: r.span,
        strokes: r.strokes.iter().map(|(s, name)| (stroke(s), *name)).collect(),
        within: r.within.clone(),
        kept: r.kept.clone(),
    }
}

/// A run's section as a profile `brep::build` reads (model units): a conic's arc the rational
/// quadratic B-spline it is.
fn profile(r: &Run) -> Result<crate::brep::build::Profile, String> {
    use crate::brep::build::ProfileEdge;
    use crate::brep::geom::{BSpline, Frame};
    let (edges, names) = r.strokes.iter().map(|(stroke, name)| Ok((match *stroke {
        Stroke::Line { from, to } => ProfileEdge::Line { a: from, b: to },
        Stroke::Arc { centre, radius, x, angles } => ProfileEdge::Arc {
            frame: Frame::new(centre, r.along, x), r: radius, span: Some(angles),
        },
        Stroke::Conic { a, m, b, w } => ProfileEdge::Spline(std::sync::Arc::new(
            BSpline::rational(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![a, m, b], vec![1.0, w, 1.0])?)),
    }, name.to_string()))).collect::<Result<Vec<_>, String>>()?.into_iter().unzip();
    let (loops, names) = (vec![edges], vec![names]);
    Ok(crate::brep::build::Profile { origin: r.origin, normal: r.along, loops, names })
}

impl Stroke {
    /// Whether it is `b`, each point within a billionth of `size` of its own.
    fn same(&self, b: &Stroke, size: f64) -> bool {
        let near = |p: V, q: V| distance(p, q) <= 1e-9 * size;
        match (self, b) {
            (Stroke::Line { from: a0, to: a1 }, Stroke::Line { from: b0, to: b1 }) => near(*a0, *b0) && near(*a1, *b1),
            (Stroke::Arc { centre: c, radius: r, .. }, Stroke::Arc { centre: d, radius: s, .. }) => {
                near(*c, *d) && (r - s).abs() <= 1e-9 * size
            }
            (Stroke::Conic { a: a0, m: m0, b: b0, w: w0 }, Stroke::Conic { a: a1, m: m1, b: b1, w: w1 }) => {
                near(*a0, *a1) && near(*m0, *m1) && near(*b0, *b1) && (w0 - w1).abs() <= 1e-9 * w0.max(1.0)
            }
            _ => false,
        }
    }
}

impl Run {
    /// Whether it is `b`, within a billionth of `size`: the same section over the same stretch,
    /// kept within the same faces.
    fn same(&self, b: &Run, size: f64) -> bool {
        let a = self;
        let near = |p: V, q: V| distance(p, q) <= 1e-9 * size;
        near(a.origin, b.origin) && near(a.along, b.along)
            && a.span.iter().zip(b.span).all(|(s, t)| (s - t).abs() <= 1e-9 * size)
            && a.strokes.len() == b.strokes.len()
            && a.strokes.iter().zip(&b.strokes).all(|((x, _), (y, _))| x.same(y, size))
            && a.within.len() == b.within.len()
            && a.within.iter().zip(&b.within).all(|(x, y)| near(x.0, y.0) && (x.1 - y.1).abs() <= 1e-9 * size)
            && a.kept == b.kept
    }
}

/// Pieces as one construction: each distinct piece once, each run with the stretch of its edge it
/// is built over (cut short at the section of a corner standing at either end), and which run is
/// joined to which corner there (runs counted first, then rings, then corners, then rolled runs,
/// then the pieces beyond a seam's normal surface, each joined to the ring or rolled run it was
/// made with).
#[derive(Clone, Debug, Default)]
pub struct Assembly {
    pub runs: Vec<Run>,
    pub rings: Vec<Ring>,
    pub corners: Vec<Corner>,
    pub joins: Vec<[usize; 2]>,
    pub rolled: Vec<Rolled>,
    pub beyond: Vec<Beyond>,
}

/// What a piece beyond a seam's normal surface is glued to: the ring, or the rolled run, it was
/// made with (counted among its kind).
enum Partner {
    Ring(usize),
    Rolled(usize),
}

/// The union of `pieces` as one construction (`Assembly`).
pub fn assemble<'a>(pieces: impl IntoIterator<Item = &'a Piece>) -> Assembly {
    let pieces: Vec<&Piece> = pieces.into_iter().collect();
    let beyond_size = |b: &Option<Beyond>| b.as_ref().map_or(0.0, |b| norm(b.ends[0]) + b.section.size());
    let size = pieces.iter().map(|p| match p {
        Piece::Run(r) => norm(r.origin) + r.span[0].abs().max(r.span[1].abs()),
        Piece::Ring(g) => (norm(g.section.origin) + norm(g.axis_origin)).max(beyond_size(&g.beyond)),
        Piece::Corner(c) => norm(c.vertex) + c.r,
        Piece::Rolled(k) => (norm(k.at) + k.r + k.flat.map_or(0.0, |([la, lb], _)| la.max(lb))).max(beyond_size(&k.beyond)),
    }).fold(1.0, f64::max);
    let near = |p: V, q: V| distance(p, q) <= 1e-9 * size;
    let mut out = Assembly::default();
    // each piece beyond a seam's normal surface goes with the piece that holds it: kept with it,
    // joined to it, and dropped with it where another region named the same edge
    let mut partners = Vec::new();
    for p in pieces {
        match p {
            Piece::Run(r) if !out.runs.iter().any(|q| q.same(r, size)) => out.runs.push(r.clone()),
            Piece::Ring(g) if !out.rings.iter().any(|h| h.section.same(&g.section, size)
                && (h.angle - g.angle).abs() <= 1e-12) => {
                let mut g = g.clone();
                if let Some(b) = g.beyond.take() {
                    out.beyond.push(b);
                    partners.push(Partner::Ring(out.rings.len()));
                }
                out.rings.push(g);
            }
            Piece::Corner(c) if !out.corners.iter().any(|d| near(c.vertex, d.vertex) && near(c.centre, d.centre)) => {
                out.corners.push(c.clone())
            }
            Piece::Rolled(k) if !out.rolled.iter().any(|j| near(j.at, k.at) && (j.r - k.r).abs() <= 1e-9 * size
                && j.flat.is_some() == k.flat.is_some()) => {
                let mut k = k.clone();
                if let Some(b) = k.beyond.take() {
                    out.beyond.push(b);
                    partners.push(Partner::Rolled(out.rolled.len()));
                }
                out.rolled.push(k);
            }
            _ => {}
        }
    }
    // a round's run, at an end of its edge where a corner stands whose ball its own rolls into,
    // runs from the corner's section on: what lies between is the corner's cell (and a face the
    // run was carried on past and cut off by at a slant there no longer reaches it)
    let n = out.runs.len() + out.rings.len();
    for (i, r) in out.runs.iter_mut().enumerate() {
        let Some(o) = r.centre() else { continue };
        for (k, c) in out.corners.iter().enumerate() {
            let off = sub(c.centre, o);
            let at = dot(sub(c.vertex, r.origin), r.along);
            if norm(cross(off, r.along)) > 1e-9 * size || !near(c.vertex, add(r.origin, scale(r.along, at))) {
                continue;
            }
            // the run's origin is its edge's first end
            let s = dot(off, r.along);
            if at.abs() <= 1e-9 * size {
                r.span[0] = s;
            } else if at > 0.0 && at <= r.span[1] + 1e-9 * size {
                r.span[1] = s;
            } else {
                continue;
            }
            r.release(c.vertex, size);
            out.joins.push([i, n + k]);
        }
    }
    // a ring's stretch, at an end where a corner stands whose ball its own rolls round into, turns
    // only as far as the corner's section: the meridian through its centre (#218's 4)
    let (n, rings) = (out.runs.len(), out.rings.len());
    for (i, g) in out.rings.iter_mut().enumerate() {
        let Some(o) = g.section.centre() else { continue };
        if !g.stretch() {
            continue;
        }
        let radial = |p: V| { let w = sub(p, g.axis_origin); sub(w, scale(g.axis, dot(w, g.axis))) };
        let height = |p: V| dot(sub(p, g.axis_origin), g.axis);
        for (k, c) in out.corners.iter().enumerate() {
            // the angle a point stands at round the axis from the stretch's start, the way it turns
            let from = radial(g.section.origin);
            let angle_of = |p: V| {
                let w = radial(p);
                dot(cross(from, w), g.axis).datan2(dot(from, w)).rem_euclid(std::f64::consts::TAU)
            };
            let on_balls = (norm(radial(c.centre)) - norm(radial(o))).abs() <= 1e-9 * size
                && (height(c.centre) - height(o)).abs() <= 1e-9 * size;
            let on_circle = (norm(radial(c.vertex)) - norm(from)).abs() <= 1e-9 * size
                && (height(c.vertex) - height(g.section.origin)).abs() <= 1e-9 * size;
            if !on_balls || !on_circle {
                continue;
            }
            let (at, cut) = (angle_of(c.vertex), angle_of(c.centre));
            // a corner at one end of this stretch (or within what it was carried on past it), not
            // one standing on another stretch of the same circle
            let at = if at > std::f64::consts::TAU - 1e-9 { at - std::f64::consts::TAU } else { at };
            if at > g.angle + 1e-9 {
                continue;
            }
            if at <= 0.5 * g.angle {
                // at its start: turned on to the corner's section
                let turn = crate::brep::geom::Rigid::turn(g.axis_origin, g.axis, cut);
                g.section = turned(&g.section, &turn);
                g.angle -= cut;
            } else {
                g.angle = cut;
            }
            g.section.release(c.vertex, size);
            out.joins.push([n + i, n + rings + k]);
        }
    }
    // a rolled run at an end of which a corner stands is cut at that corner's section: the plane of
    // the corner's ball's arc between the two faces the run lies between (through its centre and
    // where it touches each), its cell taking what lies between there and the vertex, and glued
    // to it there (rolled pieces counted after the corners)
    let (pieces_before, corners) = (out.runs.len() + out.rings.len(), out.corners.len());
    for (i, k) in out.rolled.iter_mut().enumerate() {
        for (j, c) in out.corners.iter().enumerate() {
            if !k.ends.iter().any(|&v| near(v, c.vertex)) {
                continue;
            }
            let on: Vec<&Surf> = c.faces.iter().filter(|f| f.side(k.at).abs() <= 1e-7 * size).collect();
            let &[fa, fb] = &on[..] else { continue };
            let touch = |f: &Surf| { let mut q = c.centre; for _ in 0..8 { q = sub(q, scale(f.gradient(q), f.side(q))); } q };
            let Some(n) = normalised(cross(sub(touch(fa), c.centre), sub(touch(fb), c.centre))) else { continue };
            let away = if dot(n, sub(c.vertex, c.centre)) > 0.0 { scale(n, -1.0) } else { n };
            // (only near the corner: a run round a hole comes back past the section's plane)
            k.cuts.push((c.centre, away, distance(c.vertex, c.centre) + 2.0 * c.r));
            out.joins.push([pieces_before + corners + i, pieces_before + j]);
        }
    }
    // a piece beyond a seam's normal surface, glued to its edge's own piece there: the one it was
    // made with, the ring or the rolled run kept short of that surface
    let rolled_from = out.runs.len() + out.rings.len() + out.corners.len();
    let first = rolled_from + out.rolled.len();
    for (k, partner) in partners.iter().enumerate() {
        let at = match *partner { Partner::Ring(i) => out.runs.len() + i, Partner::Rolled(i) => rolled_from + i };
        out.joins.push([at, first + k]);
    }
    // a run and a stretch of a ring going on from one another (a round along an obround's flat
    // side, then round its half circle): one section, where the ring's turn starts or ends
    for (i, r) in out.runs.iter().enumerate() {
        let ends = [r.end(0), r.end(1)];
        for (k, g) in out.rings.iter().enumerate().filter(|(_, g)| g.stretch()) {
            let (end, way) = g.end();
            let meets = |p: V, d: V| ends.iter().any(|&q| near(p, q)) && norm(cross(d, r.along)) <= 1e-9;
            if meets(g.section.origin, g.section.along) || meets(end, way) {
                out.joins.push([i, out.runs.len() + k]);
            }
        }
    }
    // two stretches of rings going on from one another (a round across a tangent seam between two
    // arcs, #574): one section, where one's turn ends and the other's starts — glued there, as a
    // run to a ring, never unioned (two tori tangent along that section's circle meet in nothing
    // a trace can follow)
    let hulls: Vec<Option<[Vec<V>; 2]>> = out.rings.iter().map(|g| g.stretch().then(|| {
        let turn = crate::brep::geom::Rigid::turn(g.axis_origin, g.axis, g.angle);
        [g.section.section_hull(), turned(&g.section, &turn).section_hull()]
    })).collect();
    for (i, g) in hulls.iter().enumerate().filter_map(|(i, g)| Some((i, g.as_ref()?))) {
        for (k, h) in hulls.iter().enumerate().skip(i + 1).filter_map(|(k, h)| Some((k, h.as_ref()?))) {
            if same_section(g, h, size) {
                out.joins.push([out.runs.len() + i, out.runs.len() + k]);
            }
        }
    }
    out
}

/// Whether two stretches of rings, each by its section's hull where its turn starts and where it
/// ends, meet in one section: at an end of each, every point of either hull within a billionth of
/// `size` of one of the other's (the pieces' caps there one face, `boolean::glued`'s to pair).
fn same_section(g: &[Vec<V>; 2], h: &[Vec<V>; 2], size: f64) -> bool {
    let within = |a: &[V], b: &[V]| a.iter().all(|&p| b.iter().any(|&q| distance(p, q) <= 1e-9 * size));
    g.iter().any(|x| h.iter().any(|y| within(x, y) && within(y, x)))
}

impl Assembly {
    /// As a recipe node (written by `w`): each run a prism over its stretch (within the
    /// half-spaces it is kept in), each ring a revolution, each corner where it stands, and the
    /// joins.
    pub fn node(&self, w: &Writer) -> Result<Json, String> {
        let (by, vector) = (w.scale, Writer::triple);
        let mut pieces = self.runs.iter().map(|r| {
            let [from, to] = r.span;
            let mut o = object([
                ("kind", "prism".into()), ("profile", profile(r)?.to_json(w)?),
                ("from", (from * by).into()), ("to", (to * by).into()),
            ]);
            for (k, v) in ends_json(r, w) {
                o.set(k, v);
            }
            Ok(o)
        }).collect::<Result<Vec<_>, String>>()?;
        for g in &self.rings {
            let mut o = object([
                ("kind", "revolve".into()), ("profile", profile(&g.section)?.to_json(w)?),
                ("origin", w.line_origin(scale(g.axis_origin, by), g.axis)), ("axis", vector(g.axis)), ("angle", g.angle.into()),
            ]);
            for (k, v) in ends_json(&g.section, w) {
                o.set(k, v);
            }
            pieces.push(o);
        }
        let corners = self.corners.iter().map(|c| object([
            ("vertex", w.at(scale(c.vertex, by))), ("centre", w.at(scale(c.centre, by))),
            ("radius", (c.r * by).into()), ("faces", Json::Arr(c.faces.iter().map(|f| f.json(w)).collect())),
        ])).collect();
        let joins = self.joins.iter().map(|j| Json::Arr(j.iter().map(|&k| k.into()).collect())).collect();
        let rolled = self.rolled.iter().map(|k| {
            let mut o = object([("at", w.at(scale(k.at, by))), ("radius", (k.r * by).into())]);
            if let Some(([la, lb], on)) = k.flat {
                o.set("legs", Json::Arr(vec![(la * by).into(), (lb * by).into()]));
                o.set("on", w.at(scale(on, by)));
            }
            if !k.cuts.is_empty() {
                o.set("cuts", Json::Arr(k.cuts.iter().map(|&(at, n, reach)| object([("at", w.at(scale(at, by))),
                    ("normal", vector(n)), ("reach", (reach * by).into())])).collect()));
            }
            if !k.wedge.is_empty() {
                o.set("wedge", Json::Arr(k.wedge.iter().map(|&(at, n)| object([("at", w.at(scale(at, by))),
                    ("normal", vector(n))])).collect()));
            }
            if !k.kept.is_empty() {
                o.set("kept", Json::Arr(k.kept.iter().map(|kept| { let mut s = kept.face.json(w); s.set("inside", kept.inside.into()); s })
                    .collect()));
            }
            o
        }).collect();
        let beyond = self.beyond.iter().map(|b| {
            let mut o = object([
                ("toward", vector(b.toward)), ("chain", b.chain.json(w)), ("wall", b.wall.json(w)),
                ("sides", Json::Arr(b.sides.iter().map(|&x| x.into()).collect())),
                ("seam", match b.seam {
                    Seam::Circle(c, n, r) => object([("centre", w.at(scale(c, by))), ("normal", vector(n)), ("radius", (r * by).into())]),
                    Seam::Line(p, d) => object([("point", w.line_origin(scale(p, by), d)), ("direction", vector(d))]),
                }),
                ("ends", Json::Arr(b.ends.iter().map(|&p| w.at(scale(p, by))).collect())),
                ("along", object([("face", b.face.json(w))])),
            ]);
            // a flat's legs and the seam's normal surface it starts at, or a round's radius
            match b.section {
                Across::Flat { legs, split } => {
                    o.set("legs", Json::Arr(legs.iter().map(|&l| (l * by).into()).collect()));
                    o.set("split", split.json(w));
                }
                Across::Round { radius } => o.set("radius", (radius * by).into()),
            }
            o
        }).collect();
        let mut node = object([("kind", "lowered".into()), ("pieces", Json::Arr(pieces)), ("corners", Json::Arr(corners)),
            ("joins", Json::Arr(joins)), ("rolled", Json::Arr(rolled))]);
        if !self.beyond.is_empty() {
            node.set("beyond", Json::Arr(beyond));
        }
        Ok(node)
    }

    /// Built by this kernel (lengths times `by`).
    pub fn brep(&self, by: f64) -> Result<crate::brep::topo::Brep, String> {
        crate::brep::recipe::node_named(&self.node(&Writer::world(by))?, &Default::default(), &Default::default(), 0.0)
    }
}
