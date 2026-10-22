//! **A scenario read over a prism in its plane** (#510 S2): plane stress or plane strain over the
//! face it is swept from, as claims.
//!
//! `plane stress plate` in a scenario reads the solid `plate` — a prism, its face swept square to
//! its plane — over that face, its depth the thickness (in plane strain, the length it is held
//! along).  The face is meshed in quadratic triangles (`delaunay::plane`), its boundary nodes on
//! the drawing's own curves and its elements curved as its sides are, and each is the interior's
//! plane element (`interior::Tri6`).  A point the scenario names in the region is a node.
//!
//! What the scenario says falls on the region where it names it: a side of the prism (a face path
//! that names its wall), the line it was swept from, the face, a cap, the solid, a point.  A
//! support holds the nodes it names along the plane's axes (along `along:` one of them, held
//! across the thickness it holds nothing the plane moves); a force at a point is a load at its
//! node, one on a side spread evenly along it, one on the solid or a cap spread over the area; a
//! pressure on a side pushes in through its depth; `accelerate` is a body load of the material's
//! density.  The value is in the document's units, the material read back into them.  Where there
//! is no value it says why: a support leaving the region free (in the solver's words), a material
//! that does not state what is read of it, a load across the plane, a factor of safety where the
//! stress is unbounded at a sharp re-entrant edge (`diagnose::analysis::reentrant`), a buckling
//! load (a plate buckles out of its plane, which it has no move for).

use crate::beams::Read;
use crate::delaunay::plane::{self as mesher, Criteria, Mesh, Side};
use crate::diagnose::analysis::{self, direction, Held};
use crate::eigen::{self, Pairs};
use crate::interior::{Failure, Laid, State, Structure, Tri6};
use crate::model::{
    Applied, EntKind, EntRef, InPlane, LoadKind, LoadValue, Output, Property, Requirement,
    Scenario, Sketch, SolidDef,
};
use crate::plane::Basis;
use crate::solid::faces::BodyFaces;
use crate::space::{dot, norm, scale, sub};
use std::cell::OnceCell;

type V = [f64; 3];
type P = [f64; 2];

/// The most a piece of a curved side may turn, in radians: the boundary's own fineness, where a
/// stress concentrates at a hole.
pub const TURN: f64 = std::f64::consts::PI / 72.0;
/// The most an element's circumradius may be, as a fraction of the region's extent.
pub const SIZE: f64 = 1.0 / 24.0;
/// The most nodes a region's mesh may take.
pub const MOST: usize = 200_000;
/// Two places within this of the region's extent are one; a point within it of a side lies on
/// the side.
pub const ON: f64 = 1e-6;

/// **A side of the region as the drawing has it**: a line between two points, an arc about its
/// centre run from one end to the other, a closed circle, or a stretch of a spline or a curve
/// between two of its parameters — each read in the plane's own coordinates at a parameter on
/// `[0, 1]`, in the order the face's loop walks it.
#[derive(Clone, Debug, PartialEq)]
pub enum Bound {
    Line { a: usize, b: usize },
    Arc { c: usize, r: usize, from: usize, to: usize, ccw: bool },
    Circle { c: usize, r: usize },
    Curved { e: EntRef, t: [f64; 2] },
}

impl Bound {
    /// Where it is at `s` on `[0, 1]`, in the plane.
    pub fn at(&self, sk: &Sketch, s: f64) -> P {
        let xy = |i: usize| -> P {
            let (x, y) = sk.point_xy(i);
            [x, y]
        };
        match *self {
            Bound::Line { a, b } => {
                let (p, q) = (xy(a), xy(b));
                [p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])]
            }
            Bound::Arc { c, r, from, to, ccw } => {
                let (o, rad) = (xy(c), sk.params[r].value.abs());
                let bearing = |i: usize| {
                    let p = xy(i);
                    (p[1] - o[1]).atan2(p[0] - o[0])
                };
                let (a0, a1) = (bearing(from), bearing(to));
                let mut sweep = if ccw { a1 - a0 } else { a0 - a1 };
                while sweep <= 0.0 {
                    sweep += std::f64::consts::TAU;
                }
                let a = if ccw { a0 + s * sweep } else { a0 - s * sweep };
                [o[0] + rad * a.cos(), o[1] + rad * a.sin()]
            }
            Bound::Circle { c, r } => {
                let (o, rad) = (xy(c), sk.params[r].value.abs());
                let a = std::f64::consts::TAU * s;
                [o[0] + rad * a.cos(), o[1] + rad * a.sin()]
            }
            Bound::Curved { e, t } => {
                let (x, y) = crate::solid::profile::curved_point(sk, e, t[0] + s * (t[1] - t[0]));
                [x, y]
            }
        }
    }

    /// Whether it is straight.
    pub fn straight(&self) -> bool {
        matches!(self, Bound::Line { .. })
    }
}

/// **A prism's region**: the solid, the plane its face is drawn in, the ordinate of its middle
/// along the plane's normal and its depth, its sides loop by loop (the outer first, `loops` how
/// many each loop has) and its extent.
#[derive(Clone, Debug)]
pub struct Region {
    pub solid: usize,
    pub basis: Basis,
    pub mid: f64,
    pub depth: f64,
    pub sides: Vec<Bound>,
    pub loops: Vec<usize>,
    pub extent: f64,
}

impl Region {
    /// Solid `solid`'s region, or why it has none: no prism swept square to its face, or a face
    /// whose loop the walk cannot follow.
    pub fn of(sk: &Sketch, solid: usize) -> Result<Region, String> {
        let name = &sk.solids[solid].name;
        let SolidDef::Prism { face, from, to, draft: None } = &sk.solids[solid].def else {
            return Err(format!("`{name}` is no prism swept square to its face"));
        };
        let f = &sk.faces[*face as usize];
        let basis = match f.plane {
            Some(p) => sk.basis(p as usize),
            None => Basis::page(),
        };
        let (from, to) = (from.value, to.value);
        let mut sides = Vec::new();
        let mut loops = Vec::new();
        for (edges, _) in f.boundaries() {
            let before = sides.len();
            if edges.len() == 1 && edges[0].kind == EntKind::Circle {
                let c = &sk.circles[edges[0].i()];
                sides.push(Bound::Circle { c: c.center as usize, r: c.radius as usize });
            } else {
                let walk = crate::solid::profile::walk_loop(sk, edges)
                    .ok_or_else(|| format!("`{name}`'s face is no loop its sides close"))?;
                for (e, a, b) in walk {
                    sides.push(match e.kind {
                        EntKind::Line => Bound::Line { a: a as usize, b: b as usize },
                        EntKind::Arc => {
                            let arc = &sk.arcs[e.i()];
                            Bound::Arc { c: arc.center as usize, r: arc.radius as usize,
                                         from: a as usize, to: b as usize, ccw: a == arc.start }
                        }
                        EntKind::Spline | EntKind::Curve => {
                            let (_, t) = crate::solid::profile::curved_span(sk, e, a)
                                .ok_or_else(|| format!("`{name}`'s face has a curve of no span"))?;
                            Bound::Curved { e, t }
                        }
                        kind => return Err(format!("`{name}`'s face is bounded by {}, which a \
                                                    reading in its plane does not read yet",
                                                   kind.a())),
                    });
                }
            }
            loops.push(sides.len() - before);
        }
        let mut region = Region { solid, basis, mid: 0.5 * (from + to), depth: (to - from).abs(),
                                  sides, loops, extent: 0.0 };
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for b in &region.sides {
            for k in 0..=16 {
                let p = b.at(sk, k as f64 / 16.0);
                for c in 0..2 {
                    lo[c] = lo[c].min(p[c]);
                    hi[c] = hi[c].max(p[c]);
                }
            }
        }
        region.extent = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        if !(region.extent > 0.0 && region.depth > 0.0) {
            return Err(format!("`{name}` has no extent or no depth"));
        }
        Ok(region)
    }

    /// Where a place in space is in the plane, and how far off it along its normal.
    pub fn local(&self, p: V) -> (P, f64) {
        let d = sub(p, self.basis.o);
        ([dot(d, self.basis.u), dot(d, self.basis.v)], dot(d, self.basis.normal()))
    }

    /// Where a point of the plane stands in space at the prism's middle.
    pub fn lift(&self, p: P) -> V {
        let n = self.basis.normal();
        let q = self.basis.lift(p[0], p[1]);
        [q[0] + self.mid * n[0], q[1] + self.mid * n[1], q[2] + self.mid * n[2]]
    }

    /// A direction in space as its components along the plane's axes, and its part across it.
    pub fn along(&self, d: V) -> (P, f64) {
        ([dot(d, self.basis.u), dot(d, self.basis.v)], dot(d, self.basis.normal()))
    }

    /// The side `p` lies on and its parameter there, if it lies on one: the nearest of 64 samples
    /// along each side, then a golden section about it.
    pub fn on_side(&self, sk: &Sketch, p: P) -> Option<(usize, f64)> {
        let tol = ON * self.extent;
        let dist = |g: usize, s: f64| {
            let q = self.sides[g].at(sk, s);
            (q[0] - p[0]).hypot(q[1] - p[1])
        };
        let mut best: Option<(f64, usize, f64)> = None;
        for g in 0..self.sides.len() {
            let n: usize = 64;
            let k = (0..=n).min_by(|&a, &b| {
                dist(g, a as f64 / n as f64).total_cmp(&dist(g, b as f64 / n as f64))
            }).unwrap();
            let (mut a, mut b) = (k.saturating_sub(1) as f64 / n as f64,
                                  (k + 1).min(n) as f64 / n as f64);
            let r = (5f64.sqrt() - 1.0) / 2.0;
            for _ in 0..80 {
                let (c, d) = (b - r * (b - a), a + r * (b - a));
                if dist(g, c) < dist(g, d) { b = d } else { a = c }
            }
            let s = 0.5 * (a + b);
            let d = dist(g, s);
            if d <= tol && best.is_none_or(|(e, _, _)| d < e) {
                best = Some((d, g, s));
            }
        }
        best.map(|(_, g, s)| (g, s))
    }

    /// **The region meshed**, cut at the places `named` (each on a side, or inside), to the
    /// reading's criteria.
    pub fn mesh(&self, sk: &Sketch, named: &[P]) -> Result<Mesh, String> {
        let mut cuts: Vec<Vec<f64>> = vec![Vec::new(); self.sides.len()];
        let mut inside: Vec<P> = Vec::new();
        for &p in named {
            match self.on_side(sk, p) {
                Some((g, s)) if s > 1e-9 && s < 1.0 - 1e-9 => cuts[g].push(s),
                Some(_) => {}
                None => inside.push(p),
            }
        }
        let mut loops = Vec::new();
        let mut g = 0;
        for &n in &self.loops {
            let mut lp = Vec::new();
            for _ in 0..n {
                let b = self.sides[g].clone();
                lp.push(Side { at: Box::new(move |s| b.at(sk, s)), straight: self.sides[g].straight(),
                               cuts: std::mem::take(&mut cuts[g]) });
                g += 1;
            }
            loops.push(lp);
        }
        let criteria = Criteria { ratio: 2f64.sqrt(), size: SIZE * self.extent, turn: TURN,
                                  most: MOST };
        mesher::mesh(&mesher::Region { loops, points: inside }, &criteria)
    }
}

/// A material's property for solid `solid`, read back into the document's units, or why there
/// is none.
fn property(sk: &Sketch, solid: usize, p: Property) -> Result<f64, String> {
    let s = &sk.solids[solid];
    let name = s.material.as_ref().ok_or_else(|| {
        format!("`{}` is made of no material it can be read by: say what, `solid(…) of steel`",
                s.name)
    })?;
    let si = sk.materials[name].get(p).ok_or_else(|| {
        format!("`{}` is made of `{name}`, which states no `{}`", s.name, p.name())
    })?;
    Ok(sk.units.from_si(si, p.dim()))
}

/// What a line of the scenario names on the region: nodes, the sides they are on, or the whole.
enum On {
    Nothing,
    Nodes(Vec<usize>),
    Sides(Vec<usize>),
    Whole,
}

/// The region meshed under a scenario, as a structure, and where the structure stands; `name`
/// the solid's, for what is said.
pub struct Plated {
    pub region: Region,
    pub mesh: Mesh,
    pub s: Structure,
    pub x: Vec<f64>,
    name: String,
}

/// Why a region cannot be read: the supports leave it free (in the solver's words), or anything
/// else, said.
enum Unread {
    Free(String),
    Why(String),
}

impl From<String> for Unread {
    fn from(m: String) -> Unread {
        Unread::Why(m)
    }
}

impl Unread {
    fn text(&self, name: &str) -> String {
        match self {
            Unread::Free(free) => format!("`{name}` is {free}"),
            Unread::Why(m) => m.clone(),
        }
    }
}

impl Plated {
    /// Solid `solid`'s region read in `plane` under scenario `sc`, meshed and loaded, or why it
    /// cannot be.
    fn new(sk: &Sketch, sc: &Scenario, solid: usize, plane: InPlane) -> Result<Plated, Unread> {
        let region = Region::of(sk, solid)?;
        let name = sk.solids[solid].name.clone();
        let young = property(sk, solid, Property::E)?;
        let nu = property(sk, solid, Property::Nu).map_err(|m| {
            format!("{m}: a reading in the plane reads Poisson's ratio")
        })?;
        let rho = property(sk, solid, Property::Rho).unwrap_or(f64::NAN);
        // the points the scenario names in the region are nodes
        let named: Vec<P> = sc.applied().filter_map(|a| point_of(sk, &region, a)).collect();
        let mesh = region.mesh(sk, &named)?;
        let strain = plane == InPlane::Strain;
        let tris: Vec<Tri6> = mesh.tris.iter()
            .map(|&n| Tri6 { n, e: young, nu, rho, depth: region.depth, strain }).collect();
        let n = mesh.points.len();
        let x: Vec<f64> = mesh.points.iter().flatten().copied().collect();
        let mut s = Structure { dim: 2, nodes: n, bars: Vec::new(), beams: Vec::new(),
                                fixed: vec![false; 2 * n], load: vec![0.0; 2 * n],
                                weight: vec![0.0; 2 * n], spread: Vec::new(), tris,
                                laid: Vec::new() };
        let elements: Vec<usize> = (0..s.tris.len()).collect();
        let mut p = Plated { region, mesh, s: Structure { tris: Vec::new(), ..s.clone() }, x, name };
        // a support holds its nodes' moves along the plane's axes: a plane element's node has
        // no turn, so `fixed` holds what `pinned` does
        for sup in &sc.supports {
            let on = p.on(sk, &sup.at)?;
            let nodes = p.nodes(&on);
            if nodes.is_empty() {
                continue;
            }
            let axes: Vec<usize> = match sup.along {
                None => vec![0, 1],
                Some(d) => {
                    let (inp, across) = p.region.along(direction(sk, d)?);
                    if norm([inp[0], inp[1], 0.0]) <= 1e-9 && across.abs() > 0.0 {
                        continue; // across the thickness: nothing the plane moves
                    }
                    match (0..2).find(|&k| (inp[k].abs() - 1.0).abs() <= 1e-9) {
                        Some(k) => vec![k],
                        None => return Err(Unread::Why(format!(
                            "a support holds `{}` along a direction off its plane's axes, which a \
                             reading in the plane is not read under yet", p.name))),
                    }
                }
            };
            for node in nodes {
                for &k in &axes {
                    s.fixed[2 * node + k] = true;
                }
            }
        }
        for load in &sc.loads {
            let on = p.on(sk, &load.at)?;
            if matches!(on, On::Nothing) {
                continue;
            }
            let v = match &load.value {
                LoadValue::Vector(v) => *v,
                LoadValue::Along { magnitude, along } => scale(direction(sk, *along)?, *magnitude),
                LoadValue::Scalar(_) => [0.0; 3],
            };
            let (f, across) = p.region.along(v);
            if across.abs() > 1e-9 * norm(v) {
                return Err(Unread::Why(format!(
                    "a {} across the plane of `{}`, which a reading in its plane has no move for",
                    load.kind.word(), p.name)));
            }
            match (load.kind, &on) {
                (LoadKind::Force, On::Nodes(nodes)) => {
                    for &node in nodes {
                        s.load[2 * node] += f[0];
                        s.load[2 * node + 1] += f[1];
                    }
                }
                (LoadKind::Force, On::Sides(g)) => {
                    s.laid.push(Laid::Along { sides: p.sides(g), force: f });
                }
                (LoadKind::Force, On::Whole) => {
                    s.laid.push(Laid::Over { elements: elements.clone(), force: f });
                }
                (LoadKind::Pressure, On::Sides(g)) => {
                    let LoadValue::Scalar(pr) = load.value else { continue };
                    s.laid.push(Laid::Pressure { sides: p.sides(g), p: pr, depth: p.region.depth });
                }
                (LoadKind::Pressure, _) => {
                    return Err(Unread::Why(format!(
                        "a pressure on a cap of `{}` pushes across its plane, which a reading in \
                         its plane has no move for", p.name)));
                }
                (LoadKind::Acceleration, _) => {
                    let rho = property(sk, solid, Property::Rho)?;
                    s.laid.push(Laid::Body { elements: elements.clone(),
                                             force: [rho * f[0], rho * f[1]] });
                }
                (LoadKind::Force, On::Nothing) => {}
            }
        }
        p.s = s;
        Ok(p)
    }

    /// What `a` names on the region: a point's node, a side (a face path naming its wall, or the
    /// line it was swept from), the whole (the solid, its face, a cap) — or nothing of it.
    fn on(&self, sk: &Sketch, a: &Applied) -> Result<On, String> {
        let tol = ON * self.region.extent;
        Ok(match a {
            Applied::Ent(e) if e.kind == EntKind::Point => {
                match point_of(sk, &self.region, a).and_then(|p| self.node_at(p, tol)) {
                    Some(n) => On::Nodes(vec![n]),
                    None => On::Nothing,
                }
            }
            Applied::Ent(e) if e.kind == EntKind::Solid => {
                if e.i() == self.region.solid { On::Whole } else { On::Nothing }
            }
            Applied::Ent(e) if e.kind == EntKind::Face => {
                match &sk.solids[self.region.solid].def {
                    SolidDef::Prism { face, .. } if *face as usize == e.i() => On::Whole,
                    _ => On::Nothing,
                }
            }
            Applied::Ent(e) => {
                // an edge of the drawing: the sides it is, by the places along it
                let sides = self.sides_where(sk, |q| self.region.on_side(sk, q).is_some()
                    && edge_holds(sk, *e, &self.region, q, tol));
                if sides.is_empty() { On::Nothing } else { On::Sides(sides) }
            }
            Applied::Face(f) if f.solid as usize == self.region.solid => {
                let body = BodyFaces::whole(sk, self.region.solid)?;
                let faces = body.faces(f);
                if faces.is_empty() {
                    let name = &sk.solids[self.region.solid].name;
                    return Err(format!("`{name}.{}` names no face of `{name}`",
                                       f.path.join(".")));
                }
                let n = self.region.basis.normal();
                let cap = faces.iter().any(|&fi| {
                    body.plane(fi).is_some_and(|(m, _)| (dot(m, n).abs() - 1.0).abs() <= 1e-9)
                });
                if cap {
                    On::Whole
                } else {
                    let near = 1e-6 * self.region.extent + body.tolerance();
                    On::Sides(self.sides_where(sk, |q| {
                        let p = self.region.lift(q);
                        faces.iter().any(|&fi| body.patch_distance(fi, p) <= near)
                    }))
                }
            }
            _ => On::Nothing,
        })
    }

    /// The region's sides whose three quarter points all pass `test`.
    fn sides_where(&self, sk: &Sketch, test: impl Fn(P) -> bool) -> Vec<usize> {
        (0..self.region.sides.len()).filter(|&g| {
            [0.25, 0.5, 0.75].iter().all(|&s| test(self.region.sides[g].at(sk, s)))
        }).collect()
    }

    /// The node at `p`, within `tol`.
    fn node_at(&self, p: P, tol: f64) -> Option<usize> {
        (0..self.mesh.points.len()).map(|i| {
            let q = self.mesh.points[i];
            ((q[0] - p[0]).hypot(q[1] - p[1]), i)
        }).filter(|&(d, _)| d <= tol).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, i)| i)
    }

    /// The segments of sides `g`, each `[first, middle, last]` with the material on its left.
    fn sides(&self, g: &[usize]) -> Vec<[usize; 3]> {
        g.iter().flat_map(|&g| self.mesh.sides[g].iter().copied()).collect()
    }

    /// The nodes `on` names.
    fn nodes(&self, on: &On) -> Vec<usize> {
        let mut out: Vec<usize> = match on {
            On::Nothing => Vec::new(),
            On::Nodes(n) => n.clone(),
            On::Sides(g) => self.sides(g).into_iter().flatten().collect(),
            On::Whole => (0..self.mesh.points.len()).collect(),
        };
        out.sort();
        out.dedup();
        out
    }
}

/// Where a point a line names is in the region's plane, if it is a point standing in it (within
/// the prism's depth of its middle).
fn point_of(sk: &Sketch, region: &Region, a: &Applied) -> Option<P> {
    let Applied::Ent(e) = a else { return None };
    if e.kind != EntKind::Point {
        return None;
    }
    let (p, off) = region.local(sk.world_point(e.i()));
    let within = (off - region.mid).abs() <= 0.5 * region.depth + ON * region.extent;
    // a named point is a node only where it is in the region: on a side, or inside
    (within && (region.on_side(sk, p).is_some() || inside(sk, region, p))).then_some(p)
}

/// Whether `p` is inside the region: the winding of its sides about it, sampled.
fn inside(sk: &Sketch, region: &Region, p: P) -> bool {
    let mut turn = 0.0;
    for b in &region.sides {
        let n = 64;
        for k in 0..n {
            let (a, c) = (b.at(sk, k as f64 / n as f64), b.at(sk, (k + 1) as f64 / n as f64));
            let (u, v) = ([a[0] - p[0], a[1] - p[1]], [c[0] - p[0], c[1] - p[1]]);
            turn += (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]);
        }
    }
    // the outer loop winds once, a hole round the point back again
    turn.abs() > std::f64::consts::PI
}

/// Whether the drawing's edge `e` passes through `q` (in the region's plane), within `tol`.
fn edge_holds(sk: &Sketch, e: EntRef, region: &Region, q: P, tol: f64) -> bool {
    match e.kind {
        EntKind::Line => {
            let [a, b] = sk.line_ends(e.i()).map(|i| region.local(sk.world_point(i)).0);
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            if !(l2 > 0.0) {
                return false;
            }
            let t = ((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / l2;
            let f = [a[0] + t * d[0], a[1] + t * d[1]];
            (-1e-9..=1.0 + 1e-9).contains(&t) && (f[0] - q[0]).hypot(f[1] - q[1]) <= tol
        }
        EntKind::Arc | EntKind::Circle => {
            let c = region.local(sk.world_point(sk.round_center(e))).0;
            ((q[0] - c[0]).hypot(q[1] - c[1]) - sk.radius_value(e).abs()).abs() <= tol
        }
        _ => {
            // a spline or a curve: a side of the region whose curve it is
            region.sides.iter().any(|b| matches!(b, Bound::Curved { e: f, .. } if *f == e))
        }
    }
}

/// One region of the scenario, analysed: the region and its structure, the solved
/// displacements, and what the requirements read of it, worked out once.
struct Analysed {
    p: Plated,
    st: State,
    modes: usize,
    pairs: OnceCell<Result<Pairs, String>>,
    stresses: OnceCell<Vec<f64>>,
}

impl Analysed {
    fn new(sk: &Sketch, sc: &Scenario, solid: usize, plane: InPlane, modes: usize)
        -> Result<Analysed, Unread>
    {
        let p = Plated::new(sk, sc, solid, plane)?;
        let st = p.s.solve(&p.x).map_err(|why| unsolved(&p, why))?;
        Ok(Analysed { p, st, modes, pairs: OnceCell::new(), stresses: OnceCell::new() })
    }

    /// Node `i`'s move in the plane.
    fn moved(&self, i: usize) -> P {
        [self.st.u[2 * i], self.st.u[2 * i + 1]]
    }

    /// The modes the requirements ask for, solved once for the most of them.
    fn pairs(&self, sk: &Sketch) -> Result<&Pairs, String> {
        self.pairs.get_or_init(|| {
            property(sk, self.p.region.solid, Property::Rho)?;
            let k = self.p.s.stiffness(&self.p.x).map_err(|_| "no stiffness".to_string())?;
            let m = self.p.s.mass(&self.p.x).map_err(|_| "no mass".to_string())?;
            eigen::lowest(&k, &m, self.modes, &eigen::Opts::default())
                .map_err(|why| format!("its modes are not found: {why:?}"))
        }).as_ref().map_err(Clone::clone)
    }

    /// Each node's von Mises stress: the elements' stresses at it, averaged, then read.
    fn stresses(&self) -> &[f64] {
        self.stresses.get_or_init(|| {
            let s = &self.p.s;
            let n = s.nodes;
            let mut sum = vec![[0.0; 3]; n];
            let mut count = vec![0u32; n];
            for (i, t) in s.tris.iter().enumerate() {
                let u: [f64; 12] = std::array::from_fn(|k| self.st.u[2 * t.n[k / 2] + k % 2]);
                let Some(at) = t.stresses(&s.tri_places(i, &self.p.x), &u) else { continue };
                for (k, &node) in t.n.iter().enumerate() {
                    for c in 0..3 {
                        sum[node][c] += at[k][c];
                    }
                    count[node] += 1;
                }
            }
            let t = s.tris[0];
            (0..n).map(|i| match count[i] {
                0 => 0.0,
                c => t.von_mises(sum[i].map(|v| v / c as f64)),
            }).collect()
        })
    }
}

/// Why a region's structure gave no displacement, in the solver's words: the rigid motions its
/// supports leave, read in space with every node held across the plane.
fn unsolved(p: &Plated, why: Failure) -> Unread {
    match why {
        Failure::Free => {
            let r = &p.region;
            let nodes: Vec<V> = p.mesh.points.iter().map(|&q| r.lift(q)).collect();
            let axes = [r.basis.u, r.basis.v];
            let mut holds: Vec<Held> = (0..p.s.fixed.len()).filter(|&i| p.s.fixed[i])
                .map(|i| Held { at: vec![nodes[i / 2]], along: Some(axes[i % 2]), turn: false })
                .collect();
            holds.push(Held { at: nodes.clone(), along: Some(r.basis.normal()), turn: false });
            let fs = analysis::freedoms(3, &nodes, &holds);
            Unread::Free(analysis::freedoms_text(&fs, 3).unwrap_or_else(|| "free".to_string()))
        }
        Failure::Collapsed => Unread::Why("an element of its mesh is turned over".to_string()),
        Failure::Unstiff => {
            Unread::Why("its material is no material: E > 0 and -1 < nu < 0.5".to_string())
        }
    }
}

/// **A scenario read over the prisms it reads in their plane**, at the solved drawing: per
/// requirement its value or why none (`None` for one about no region read so), the words its
/// regions say (one the supports leave free, once), and which supports hold a region.
pub fn read(sk: &Sketch, sc: &Scenario) -> Read {
    let mut out = Read {
        words: Vec::new(),
        answers: vec![None; sc.requirements.len()],
        holds: vec![false; sc.supports.len()],
    };
    if sc.readings.is_empty() {
        return out;
    }
    // each reading's region, for which lines fall on it
    let regions: Vec<(usize, InPlane, Option<Region>)> = sc.readings.iter().filter_map(|r| {
        let solid = r.of.solid()?;
        Some((solid, r.plane, Region::of(sk, solid).ok()))
    }).collect();
    let of = |a: &Applied| -> Option<usize> {
        if let Some(s) = a.solid() {
            return regions.iter().position(|(solid, _, _)| *solid == s);
        }
        regions.iter().position(|(_, _, r)| r.as_ref().is_some_and(|r| point_of(sk, r, a).is_some()
            || matches!(a, Applied::Ent(e) if e.kind.runs_along() && r.sides.iter().any(|b|
                [0.25, 0.5, 0.75].iter().all(|&s| edge_holds(sk, *e, r, b.at(sk, s),
                                                              ON * r.extent))))))
    };
    for (k, s) in sc.supports.iter().enumerate() {
        out.holds[k] = of(&s.at).is_some();
    }
    let mut modes = vec![0usize; regions.len()];
    for r in &sc.requirements {
        if let (Output::Mode, Some(i)) = (r.output, of(&r.of)) {
            modes[i] = modes[i].max(r.count.unwrap_or(1) as usize);
        }
    }
    let mut read: Vec<Option<Result<Analysed, Unread>>> = (0..regions.len()).map(|_| None).collect();
    for (k, r) in sc.requirements.iter().enumerate() {
        let Some(i) = of(&r.of) else { continue };
        let (solid, plane, _) = regions[i];
        let a = read[i].get_or_insert_with(|| Analysed::new(sk, sc, solid, plane, modes[i]));
        out.answers[k] = Some(match a {
            Ok(a) => answer(sk, a, r),
            Err(m) => Err(m.text(&sk.solids[solid].name)),
        });
    }
    for (i, a) in read.iter().enumerate() {
        if let Some(Err(Unread::Free(free))) = a {
            out.words.push(format!("scenario `{}` leaves `{}` {free}", sc.name,
                                   sk.solids[regions[i].0].name));
        }
    }
    out
}

/// A requirement's value over its region `a`, or why it has none.
fn answer(sk: &Sketch, a: &Analysed, r: &Requirement) -> Result<f64, String> {
    let p = &a.p;
    let name = &sk.solids[p.region.solid].name;
    let nodes = p.nodes(&p.on(sk, &r.of)?);
    match r.output {
        Output::Deflection => {
            let along = match r.along {
                Some(d) => {
                    let (inp, _) = p.region.along(direction(sk, d)?);
                    if inp[0].hypot(inp[1]) <= 1e-9 {
                        return Err(format!("`{name}` is read in its plane, and moves nothing \
                                            across it"));
                    }
                    Some(inp)
                }
                None => None,
            };
            let mut worst: Option<f64> = None;
            for i in nodes {
                let u = a.moved(i);
                let v = match along {
                    Some(d) => u[0] * d[0] + u[1] * d[1],
                    None => u[0].hypot(u[1]),
                };
                if worst.is_none_or(|w| v.abs() > w.abs()) {
                    worst = Some(v);
                }
            }
            worst.ok_or_else(|| format!("it names no node of `{name}`'s mesh"))
        }
        Output::Compliance => {
            Ok(p.s.forces(&p.x).iter().zip(&a.st.u).map(|(f, u)| f * u).sum())
        }
        Output::Mass => {
            let rho = property(sk, p.region.solid, Property::Rho)?;
            let area: f64 = (0..p.s.tris.len()).map(|i| p.s.tris[i].area(&p.s.tri_places(i, &p.x)))
                .sum();
            Ok(rho * p.region.depth * area)
        }
        Output::Mode => {
            let n = r.count.unwrap_or(1) as usize;
            let v = *a.pairs(sk)?.values.get(n - 1).ok_or("it has fewer modes than that")?;
            Ok(v.max(0.0).sqrt() / (2.0 * std::f64::consts::PI))
        }
        Output::Buckling => Err(format!("`{name}` is read in its plane, and a plate buckles out of \
                                         its plane, which it has no move for")),
        Output::Safety => {
            let body = BodyFaces::whole(sk, p.region.solid)?;
            if let Some(edge) = analysis::reentrant(&body).first() {
                return Err(edge.text());
            }
            let yields = property(sk, p.region.solid, Property::Yield)?;
            let stresses = a.stresses();
            let worst = nodes.iter().map(|&i| stresses[i]).fold(0.0, f64::max);
            if !(worst > 0.0) {
                return Err(format!("the loads stress `{name}` nowhere"));
            }
            Ok(yields / worst)
        }
    }
}

/* -- a driving row (#510 S2) ------------------------------------------------------------------ */

/// **What a driving deflection's row reads over a region** (#510 S2): the region meshed once,
/// where the drawing stands when the row is compiled, its structure under the scenario with the
/// deflection's weights on its point's node, and how its nodes move with the drawing (`Moved`):
/// each node at a point of the drawing that point, each node along a side the side's point at
/// its parameter, the nodes inside a harmonic map of the boundary.  Its columns are each point's
/// two numbers in the plane, then each arc's or circle's radius.
#[derive(Clone, Debug, PartialEq)]
pub struct Row {
    pub structure: Structure,
    pub moved: crate::interior::Moved,
    pub columns: Vec<u32>,
    /// Where the structure stands: its nodes' places at the drawing.
    pub x: Vec<f64>,
}

/// The reading among `sc`'s whose region holds the drawing's point `at`, and the region.
fn region_of(sk: &Sketch, sc: &Scenario, at: usize) -> Option<(usize, InPlane, Region)> {
    sc.readings.iter().find_map(|r| {
        let solid = r.of.solid()?;
        let region = Region::of(sk, solid).ok()?;
        point_of(sk, &region, &Applied::Ent(EntRef::point(at))).map(|_| (solid, r.plane, region))
    })
}

/// **Whether what a requirement names is a point of a region the scenario reads in its plane**
/// (#510 S2): what a driving row over a region may be compiled at.
pub fn of_region(sk: &Sketch, sc: &Scenario, of: &Applied) -> bool {
    matches!(of, Applied::Ent(e) if e.kind == EntKind::Point) && match of {
        Applied::Ent(e) => region_of(sk, sc, e.i()).is_some(),
        _ => false,
    }
}

impl Row {
    /// **The row of `deflection(at, along: along)` under scenario `sc`** over the region whose
    /// point `at` is, or why the solve cannot read it: a direction the solve moves (its own, a
    /// load's) or a plane it turns, a side the row does not move (a spline's, a curve's), or a
    /// region that cannot be read at all.
    pub fn of(sk: &Sketch, sc: &Scenario, at: usize, along: EntRef) -> Result<Row, String> {
        for d in [Some(along)].into_iter().chain(sc.loads.iter().map(|l| match l.value {
            LoadValue::Along { along, .. } => Some(along),
            _ => None,
        })).flatten() {
            if !sk.direction_held(d) {
                return Err("its direction moves in the solve, and a driving row reads every \
                            direction as it stands: name an axis the drawing holds, or a line \
                            whose ends it holds".to_string());
            }
        }
        let (solid, plane, _) = region_of(sk, sc, at)
            .ok_or("it is of a point of no region the scenario reads in its plane")?;
        let SolidDef::Prism { face, .. } = sk.solids[solid].def else {
            return Err("it is read over no prism".to_string());
        };
        let face_plane = sk.faces[face as usize].plane;
        if face_plane.is_some_and(|p| !sk.plane_fixed(p as usize)) {
            return Err("the plane its region is drawn in moves in the solve, and a driving row \
                        reads its axes as they stand".to_string());
        }
        let p = Plated::new(sk, sc, solid, plane).map_err(|u| u.text(&sk.solids[solid].name))?;
        let Plated { region, mesh, mut s, x, .. } = p;
        let tol = ON * region.extent;
        let node_at = |q: P| -> Option<usize> {
            (0..mesh.points.len()).map(|i| {
                let m = mesh.points[i];
                ((m[0] - q[0]).hypot(m[1] - q[1]), i)
            }).filter(|&(d, _)| d <= tol).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, i)| i)
        };
        let (here, _) = region.local(sk.world_point(at));
        let node = node_at(here).ok_or("its point is no node of its region's mesh")?;
        let (d, _) = region.along(direction(sk, along)?);
        s.weight[2 * node] = d[0];
        s.weight[2 * node + 1] = d[1];
        // the columns: each point drawn in the region's plane, its two numbers, then each radius
        let mut columns: Vec<u32> = Vec::new();
        let mut point_col: std::collections::BTreeMap<usize, usize> = Default::default();
        let mut col_of = |q: usize, columns: &mut Vec<u32>| -> usize {
            *point_col.entry(q).or_insert_with(|| {
                let pt = &sk.points[q];
                columns.extend([pt.x, pt.y]);
                columns.len() - 2
            })
        };
        let mut edges = Vec::new();
        for b in &region.sides {
            edges.push(match *b {
                Bound::Line { a, b } => {
                    crate::interior::Edge::Line { a: col_of(a, &mut columns), b: col_of(b, &mut columns) }
                }
                Bound::Arc { c, r, from, to, ccw } => {
                    let (c, from, to) =
                        (col_of(c, &mut columns), col_of(from, &mut columns), col_of(to, &mut columns));
                    columns.push(r as u32);
                    crate::interior::Edge::Arc { c, from, to, r: columns.len() - 1, ccw }
                }
                Bound::Circle { c, r } => {
                    let c = col_of(c, &mut columns);
                    columns.push(r as u32);
                    crate::interior::Edge::Circle { c, r: columns.len() - 1 }
                }
                Bound::Curved { .. } => return Err("its region is bounded by a spline or a curve, \
                                                    which a driving row does not move yet"
                    .to_string()),
            });
        }
        // the points standing at nodes: each side's start, and every point the scenario names
        // drawn in the region's plane
        let mut at_point: Vec<Option<usize>> = vec![None; mesh.points.len()];
        let stands = |q: usize, at_point: &mut Vec<Option<usize>>| {
            if sk.plane_of(q).map(|p| p as u32) != face_plane {
                return;
            }
            if let Some(i) = node_at(region.local(sk.world_point(q)).0) {
                at_point[i].get_or_insert(q);
            }
        };
        for b in &region.sides {
            match *b {
                Bound::Line { a, .. } => stands(a, &mut at_point),
                Bound::Arc { from, .. } => stands(from, &mut at_point),
                _ => {}
            }
        }
        for a in sc.applied().chain([&Applied::Ent(EntRef::point(at))]) {
            if let Applied::Ent(e) = a {
                if e.kind == EntKind::Point && point_of(sk, &region, a).is_some() {
                    stands(e.i(), &mut at_point);
                }
            }
        }
        let nodes: Vec<crate::interior::Node> = (0..mesh.points.len()).map(|i| {
            match (at_point[i], mesh.on[i]) {
                (Some(q), _) => crate::interior::Node::At(col_of(q, &mut columns)),
                (None, Some((g, t))) => crate::interior::Node::On(g, t),
                (None, None) => crate::interior::Node::Inner,
            }
        }).collect();
        let laplace = crate::interior::Moved::laplacian(&mesh.tris, &x);
        let moved = crate::interior::Moved { columns: columns.len(), edges, nodes, x0: x.clone(),
                                             laplace };
        Ok(Row { structure: s, moved, columns, x })
    }

    /// Whether the structure gives a displacement where the drawing stands, or why not — what
    /// the row is refused for where it is stated (a region the supports leave free).
    pub fn check(&self) -> Result<(), String> {
        self.structure.solve(&self.x).map(|_| ()).map_err(|why| match why {
            Failure::Free => "its region is free: its supports leave it a motion".to_string(),
            _ => "its region gives no displacement".to_string(),
        })
    }

    /// Its constants, stating the deflection `d`.
    pub fn consts(&self, d: f64) -> Vec<f64> {
        self.structure.consts_with(&[d], None, Some(&self.moved))
    }

    /// Its widths, `(n_par, n_const)`.
    pub fn widths(&self) -> (usize, usize) {
        self.structure.widths_with(false, None, Some(&self.moved))
    }
}
