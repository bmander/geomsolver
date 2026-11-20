//! **Pieces as one construction** (#194): the regions a body takes in one set are one union of
//! pieces, each distinct piece once, a run cut short at a corner's section and joined to its cell
//! there — and that construction as a recipe node (`Assembly::node`) and this kernel's B-rep.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::recipe::Writer;
use crate::brep::geom::{Frame, Surface};
use crate::brep::recipe;
use super::{Across, Beyond, Corner, Piece, Ring, Rolled, Run, Seam, Stroke, Surf, V};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};

impl Surf {
    /// As a recipe holds it (written by `w`; `recipe::Lowered`'s faces).
    pub(super) fn surface(&self, w: &Writer) -> Surface {
        let by = w.scale;
        match *self {
            Surf::Plane { origin, normal } => Surface::Plane(Frame::about(w.moved(scale(origin, by)), normal)),
            Surf::Cylinder { origin, axis, radius } =>
                Surface::Cylinder(Frame::about(w.moved(scale(origin, by)), axis), radius * by),
            Surf::Cone { origin, axis, radius, half } =>
                Surface::Cone(Frame::about(w.moved(scale(origin, by)), axis), radius * by, half),
            Surf::Sphere { centre, radius } =>
                Surface::Sphere(Frame::about(w.moved(scale(centre, by)), [0., 0., 1.]), radius * by),
            Surf::Torus { origin, axis, big, tube } =>
                Surface::Torus(Frame::about(w.moved(scale(origin, by)), axis), big * by, tube * by),
        }
    }
}

/// What a piece is kept within at its ends, as a recipe holds it (written by `w`): each plane's
/// half-space, `within` (a point on it, its normal out of it), and each curved face's solid, `kept`
/// (its surface, and whether inside).
fn ends(node: crate::brep::recipe::Node, r: &Run, w: &Writer) -> recipe::Run {
    let within = r.within.iter().map(|&(n, d)| (w.moved(scale(n, d * w.scale)), n)).collect();
    let kept = r.kept.iter().map(|k| (k.face.surface(w), k.inside)).collect();
    recipe::Run { node, within, kept }
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
        origin: m.point(r.origin), along: m.vector(r.along), edge: r.edge, span: r.span,
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
    mitred(&mut out.runs, &out.corners, size, &mut out.joins);
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

/// **Two runs mitred at a corner of their edges** (#952): where two runs' edges end at one vertex
/// no corner's ball stands at, each section reaching over the other's edge (a convex corner: a rim
/// along two sides of a plate), and each run is the other's image in the plane halving the turn
/// between them (a round of one radius, or a flat of the same legs, between faces alike), the two
/// meet in that plane — their rounds' cylinders cross in it — so each is cut there and glued to
/// the other (`joins`), not carried past the corner onto the other's face and unioned.  Carried,
/// a run goes on along the other's face until its whole section has crossed it: past a turn of a
/// degree, a hundred times its width, its round all but on the other's all the way and inside the
/// other's piece, the two caps where that one ends one plane, a tongue of a face between them.
fn mitred(runs: &mut [Run], corners: &[Corner], size: f64, joins: &mut Vec<[usize; 2]>) {
    let near = |p: V, q: V| distance(p, q) <= 1e-9 * size;
    let end = |r: &Run, k: usize| add(r.origin, scale(r.along, if k == 0 { 0.0 } else { r.edge }));
    // the way from an end into the run, and from the edge into its section (toward what it takes)
    let into = |r: &Run, k: usize| if k == 0 { r.along } else { scale(r.along, -1.0) };
    let inward = |r: &Run| {
        let hull = r.section_hull();
        let middle = scale(hull.iter().fold([0.0; 3], |s, &p| add(s, p)), 1.0 / hull.len().max(1) as f64);
        let w = sub(middle, r.origin);
        sub(w, scale(r.along, dot(w, r.along)))
    };
    // each run's end cut by the half-space `m·x <= m·v` it is kept in
    let mut cuts: Vec<(usize, usize, V, V)> = Vec::new();
    for i in 0..runs.len() {
        for j in i + 1..runs.len() {
            let (a, b) = (&runs[i], &runs[j]);
            if a.edge <= 0.0 || b.edge <= 0.0 {
                continue;
            }
            for (ka, kb) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                let v = end(a, ka);
                if !near(v, end(b, kb)) || corners.iter().any(|c| near(c.vertex, v)) {
                    continue;
                }
                // (two runs alone ending there: three, at a block's corner, meet in no one plane)
                let ending = |r: &&Run| r.edge > 0.0 && (near(end(r, 0), v) || near(end(r, 1), v));
                if runs.iter().filter(ending).count() != 2 {
                    continue;
                }
                let (ua, ub) = (into(a, ka), into(b, kb));
                let Some(n) = normalised(sub(ua, ub)) else { continue };
                if dot(inward(a), ub) <= 0.0 || dot(inward(b), ua) <= 0.0 {
                    continue;
                }
                // `a`'s section in the plane halving the turn, carried along `b` onto `b`'s
                let mirror = |p: V| sub(p, scale(n, 2.0 * dot(n, sub(p, v))));
                let onto = |p: V| sub(p, scale(b.along, dot(sub(p, b.origin), b.along)));
                if norm(cross(sub(a.along, scale(n, 2.0 * dot(n, a.along))), b.along)) > 1e-9 {
                    continue;
                }
                let (ha, hb) = (a.section_hull(), b.section_hull());
                let image = |h: &[V], g: &[V]| h.iter().all(|&p| g.iter().any(|&q| near(onto(mirror(p)), q)));
                if ha.len() != hb.len() || !image(&ha, &hb) {
                    continue;
                }
                // and what else each is kept within leaves the two the same where they meet and
                // takes nothing of either's that the other's carried past the corner took: each
                // half-space the other's too, or holding the section in that plane and all the
                // other was carried over (a face a run's other end stands on, at a slant, reaching
                // back across its section there leaves the two unalike, and they stay unioned)
                let ((mut wa, ca), (mut wb, cb)) = (kept_but(a, v, size), kept_but(b, v, size));
                // (and each prism's own end at its other end)
                wa.push(other_end(a, ka));
                wb.push(other_end(b, kb));
                // (a curved face's solid only where both are kept by it)
                if ca.iter().any(|k| !cb.contains(k)) || cb.iter().any(|k| !ca.contains(k)) {
                    continue;
                }
                let (ma, mb) = (scale(n, -1.0), n);
                let (cap, over_a, over_b) =
                    (carried(a, ma, v), carried_over(a, ka, ma, v, size), carried_over(b, kb, mb, v, size));
                let alike = |w: &(V, f64), x: &[(V, f64)]| {
                    x.iter().any(|y| near(w.0, y.0) && (w.1 - y.1).abs() <= 1e-9 * size)
                };
                let holds = |w: &(V, f64), pts: &[V]| pts.iter().all(|&p| dot(w.0, p) <= w.1 + 1e-9 * size);
                let fits = |own: &[(V, f64)], other: &[(V, f64)], over: &[V]| own.iter()
                    .all(|w| alike(w, other) || (holds(w, &cap) && holds(w, over)));
                if !fits(&wa, &wb, &over_b) || !fits(&wb, &wa, &over_a) {
                    continue;
                }
                // `a` on the side `ua` is, `b` on the other
                cuts.push((i, ka, ma, v));
                cuts.push((j, kb, mb, v));
                joins.push([i, j]);
            }
        }
    }
    for (i, k, m, v) in cuts {
        let r = &mut runs[i];
        r.release(v, size);
        // carried as far as the plane, its whole section past it there
        let slope = dot(m, r.along);
        let reach = r.section_hull().into_iter().map(|q| dot(m, sub(v, q)) / slope);
        r.span[k] = if k == 0 { reach.fold(f64::INFINITY, f64::min) }
            else { reach.fold(f64::NEG_INFINITY, f64::max) };
        // (a face it was carried on past no longer reaching it)
        let hull = r.hull();
        r.within.retain(|&(n, d)| hull.iter().any(|&p| dot(n, p) > d + 1e-9 * size));
        r.within.push((m, dot(m, v)));
    }
}

/// What run `r` is kept within but the faces standing at `v`: the half-spaces, and the curved
/// faces' solids.
fn kept_but(r: &Run, v: V, size: f64) -> (Vec<(V, f64)>, Vec<super::Kept>) {
    let mut s = r.clone();
    s.release(v, size);
    (s.within, s.kept)
}

/// Where run `r`'s prism ends at the end other than `k`, as the half-space it holds.
fn other_end(r: &Run, k: usize) -> (V, f64) {
    let at = add(r.origin, scale(r.along, r.span[1 - k]));
    if k == 0 { (r.along, dot(r.along, at)) } else { (scale(r.along, -1.0), -dot(r.along, at)) }
}

/// Run `r`'s section's hull carried to the plane `m·x = m·v`.
fn carried(r: &Run, m: V, v: V) -> Vec<V> {
    let slope = dot(m, r.along);
    r.section_hull().into_iter().map(|q| add(q, scale(r.along, dot(m, sub(v, q)) / slope))).collect()
}

/// The corners of what run `r` takes past the plane `m·x = m·v`, on the side `m` points to: its
/// prism from there to where it was carried past its end `k`, within every half-space it is kept
/// in — each point three of those faces meet in that stands within all the rest.
fn carried_over(r: &Run, k: usize, m: V, v: V, size: f64) -> Vec<V> {
    let tol = 1e-9 * size;
    // the prism's sides: the lines through two points of the section's hull with all the rest on
    // one side, each the half-space along the run on that side
    let hull = r.section_hull();
    let mut faces: Vec<(V, f64)> = Vec::new();
    for (i, &p) in hull.iter().enumerate() {
        for &q in &hull[i + 1..] {
            let Some(n) = normalised(cross(r.along, sub(q, p))) else { continue };
            let sides: Vec<f64> = hull.iter().map(|&x| dot(n, sub(x, p))).collect();
            let face = if sides.iter().all(|&s| s <= tol) { (n, dot(n, p)) }
                else if sides.iter().all(|&s| s >= -tol) { (scale(n, -1.0), -dot(n, p)) } else { continue };
            if !faces.iter().any(|f| distance(f.0, face.0) <= 1e-9 && (f.1 - face.1).abs() <= tol) {
                faces.push(face);
            }
        }
    }
    // its ends: the plane, and the section where it was carried to
    let far = dot(r.along, add(r.origin, scale(r.along, r.span[k])));
    faces.push((scale(m, -1.0), -dot(m, v)));
    faces.push(if k == 1 { (r.along, far) } else { (scale(r.along, -1.0), -far) });
    faces.extend(r.within.iter().copied());
    let inside = |x: V| faces.iter().all(|&(n, d)| dot(n, x) <= d + tol);
    let mut corners = Vec::new();
    for i in 0..faces.len() {
        for j in i + 1..faces.len() {
            for l in j + 1..faces.len() {
                let rows = [faces[i].0, faces[j].0, faces[l].0];
                if dot(rows[0], cross(rows[1], rows[2])).abs() <= 1e-12 {
                    continue;
                }
                let x = crate::brep::ssi::solve3(rows, [faces[i].1, faces[j].1, faces[l].1]);
                if inside(x) {
                    corners.push(x);
                }
            }
        }
    }
    corners
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
    pub fn node(&self, w: &Writer) -> Result<recipe::Node, String> {
        use crate::brep::spill;
        let by = w.scale;
        let mut pieces = self.runs.iter().map(|r| {
            let [from, to] = r.span;
            let prism = recipe::Node::Prism { profile: profile(r)?.section(w)?, from: from * by, to: to * by, draft: None };
            Ok(ends(prism, r, w))
        }).collect::<Result<Vec<_>, String>>()?;
        for g in &self.rings {
            let ring = recipe::Node::Revolve { profile: profile(&g.section)?.section(w)?,
                origin: w.line_origin(scale(g.axis_origin, by), g.axis), axis: g.axis, angle: g.angle };
            pieces.push(ends(ring, &g.section, w));
        }
        let corners = self.corners.iter().map(|c| recipe::Corner {
            vertex: w.moved(scale(c.vertex, by)), centre: w.moved(scale(c.centre, by)), radius: c.r * by,
            faces: c.faces.iter().map(|f| f.surface(w)).collect(),
        }).collect();
        let rolled = self.rolled.iter().map(|k| recipe::Rolled {
            at: w.moved(scale(k.at, by)), radius: k.r * by,
            flat: k.flat.map(|([la, lb], on)| ([la * by, lb * by], w.moved(scale(on, by)))),
            cuts: k.cuts.iter().map(|&(at, n, reach)| (w.moved(scale(at, by)), n, reach * by)).collect(),
            kept: k.kept.iter().map(|kept| (kept.face.surface(w), kept.inside)).collect(),
            wedge: k.wedge.iter().map(|&(at, n)| (w.moved(scale(at, by)), n)).collect(),
        }).collect();
        let beyond = self.beyond.iter().map(|b| spill::Beyond {
            face: b.face.surface(w), toward: b.toward, chain: b.chain.surface(w), wall: b.wall.surface(w),
            sides: b.sides,
            seam: match b.seam {
                Seam::Circle(c, n, r) => spill::Seam::Circle(w.moved(scale(c, by)), n, r * by),
                Seam::Line(p, d) => spill::Seam::Line(w.line_origin(scale(p, by), d), d),
            },
            // a flat's legs and the seam's normal surface it starts at, or a round's radius
            section: match b.section {
                Across::Flat { legs, split } => spill::Across::Flat { legs: legs.map(|l| l * by), split: split.surface(w) },
                Across::Round { radius } => spill::Across::Round { radius: radius * by },
            },
            ends: b.ends.map(|p| w.moved(scale(p, by))),
        }).collect();
        Ok(recipe::Node::Lowered(recipe::Lowered { pieces, corners, rolled, joins: self.joins.clone(), stratum: None, beyond }))
    }

    /// Built by this kernel (lengths times `by`).
    pub fn brep(&self, by: f64) -> Result<crate::brep::topo::Brep, String> {
        crate::brep::recipe::node_named(&self.node(&Writer::world(by))?, &Default::default(), &Default::default(), 0.0)
    }
}
