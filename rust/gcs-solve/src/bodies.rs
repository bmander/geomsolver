//! **A scenario read over a solid in three dimensions** (#510 S3), as claims.
//!
//! A solid a requirement names, and no `plane stress`/`plane strain` line reads in its plane, is
//! read as the solid it is: its volume meshed in quadratic tetrahedra (`interior::Tet10`), each a
//! TET10 of the interior.  A prism — a face swept square to its plane — is meshed as its face's
//! mesh stacked through its depth (`Mesh::stacked`): the face meshed in quadratic triangles
//! (`prisms::Region::mesh`, its boundary nodes on the drawing's curves), the depth cut in layers,
//! each triangle of a layer a prism cut into three tetrahedra, and every middle node a node of the
//! face at a middle ordinate — so a side curved in the face is curved through the depth, and a
//! point the scenario names is a node (its place in the face a node of the face's mesh, its
//! ordinate a level of the stack).
//!
//! What the scenario says falls on the solid where it names it: a side (a face path naming its
//! wall, or the line it was swept from), a cap, the solid, its face, a point.  A support holds the
//! nodes it names along the world's axes (`along:` one of them; off them no answer yet); a force
//! at a point is a load at its node, one on faces spread evenly over them, one on the solid
//! spread through it; a pressure on faces pushes into them; `accelerate` is a body load of the
//! material's density.  The value is in the document's units, the material read back into them.
//! Where there is no value it says why: a support leaving the solid free (in the solver's words),
//! a material that does not state what is read of it, a factor of safety where the stress is
//! unbounded at a sharp re-entrant edge (`diagnose::analysis::reentrant`), a buckling load under
//! loads that press nothing.

use crate::beams::Read;
use crate::deformed::Body;
use crate::delaunay::plane::Mesh as Plane;
use crate::delaunay::tetra::{self, EDGES};
use crate::diagnose::analysis::{self, direction, Held};
use crate::eigen::{self, Pairs};
use crate::interior::{Borne, Failure, Reads, State, Structure, Tet10};
use crate::model::{
    Applied, EntKind, LoadKind, LoadValue, Output, Property, Requirement, Scenario, Sketch,
};
use crate::prisms::{self, property, Region, Unread, ON};
use crate::solid::faces::BodyFaces;
use crate::space::{dot, norm, scale, sub};
use crate::sparse::Sym;
use std::cell::OnceCell;
use std::collections::BTreeMap;

type V = [f64; 3];
type P = [f64; 2];

/// The most an element of a prism's face may be across, as a fraction of the face's breadth
/// (`Region::breadth`), beside `prisms::SIZE` of its extent: a slender face has elements enough
/// across it to bend (the corpus's cantilever, 50mm deep and a metre long, reads its tip's sag
/// 1.1% stiff at the extent's size alone, 0.13% at this).
pub const BREADTH: f64 = 1.0 / 3.0;

/// The most a layer of a prism's stack may be through its depth, as a multiple of the size its
/// face is meshed to: the depth of a prism is mostly where nothing varies (a plate's breadth, a
/// tube's length), and a layer twice as deep as its elements are wide keeps them fair.
pub const LAYER: f64 = 2.0;

/// The most an element of a solid that is no prism may be (`Mesh::filled`), as a fraction of
/// its extent.
pub const FILLED: f64 = 1.0 / 16.0;

/// How many pieces a feature curve of a body's boundary is read in, for its protection.
const CURVE: usize = 64;

/// The fewest layers a prism's depth is cut into: one prism of tetrahedra through a plate reads
/// its bending stiff (the corpus's plate, its sixth mode 1.05% high in one layer, 0.28% in two).
pub const LAYERS: usize = 2;

/// **What a facet of a solid's boundary lies on**: a side of a prism (its region's side), one of
/// its caps (the far one where `true`, at the greater ordinate along its plane's normal), or a
/// face of the body's exact boundary (its index among `BodyFaces`'s).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wall {
    Side(usize),
    Cap(bool),
    Face(usize),
}

/// **A solid's volume meshed**: its nodes, its quadratic tetrahedra (`delaunay::tetra::EDGES`'
/// numbering, positively turned), its boundary's facets — each a quadratic triangle `[a, b, c,
/// ab, bc, ca]` turned out of the material, and what it lies on — and where it is a prism's
/// stack, how.
#[derive(Clone, Debug)]
pub struct Mesh {
    pub nodes: Vec<V>,
    pub tets: Vec<[usize; 10]>,
    pub facets: Vec<([usize; 6], Wall)>,
    pub stack: Option<Stacked>,
    /// How near a point a node must be to be the point's: a stack's named points are its nodes,
    /// to `prisms::ON` of its extent; a filled body's point names the node nearest it, within an
    /// element's size.
    pub near: f64,
}

/// **A prism's mesh, its face's stacked**: the region, the face's mesh, and per node its node of
/// the face and its ordinate along the plane's normal.
#[derive(Clone, Debug)]
pub struct Stacked {
    pub region: Region,
    pub plane: Plane,
    pub of: Vec<(usize, f64)>,
}

/// The key of a pair of nodes, either way round.
fn key(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

impl Mesh {
    /// **A prism meshed as its face's mesh stacked** (#510 S3), the points `named` (each its
    /// place in the face and its ordinate) nodes, `coarser` times the reading's size: the face
    /// meshed in quadratic triangles, no larger than `prisms::SIZE` of its extent nor `BREADTH`
    /// of its breadth; the depth cut at the named ordinates, then into layers no deeper than
    /// `LAYER` of the face's size, and `LAYERS` at least; each triangle of a layer a prism of three
    /// tetrahedra, cut by the order of its corners' nodes so that two prisms sharing a wall cut it
    /// on one diagonal; every middle node a node of the face — a corner at a middle level, or a
    /// side's middle at its corners' level or between.
    pub fn stacked(sk: &Sketch, region: Region, named: &[(P, f64)], coarser: f64)
        -> Result<Mesh, String>
    {
        let at: Vec<P> = named.iter().map(|n| n.0).collect();
        let size = coarser * (prisms::SIZE * region.extent).min(BREADTH * region.breadth(sk));
        let plane = region.mesh_sized(sk, &at, size, coarser * prisms::TURN)?;
        let (lo, hi) = (region.mid - 0.5 * region.depth, region.mid + 0.5 * region.depth);
        let tol = ON * region.extent;
        let mut cuts = vec![lo, hi];
        cuts.extend(named.iter().map(|n| n.1).filter(|&h| h > lo + tol && h < hi - tol));
        cuts.sort_by(f64::total_cmp);
        cuts.dedup_by(|a, b| (*a - *b).abs() <= tol);
        let thick = LAYER * size;
        // every level a node stands at: the corners' and, between them, the middles'
        let mut levels = vec![lo];
        for w in cuts.windows(2) {
            // a depth no named point cuts is cut into `LAYERS` at least
            let least = if cuts.len() == 2 { LAYERS } else { 1 };
            let n = (((w[1] - w[0]) / thick).ceil() as usize).max(least);
            let at = |k: usize| w[0] + (w[1] - w[0]) * k as f64 / n as f64;
            for k in 0..n {
                levels.extend([0.5 * (at(k) + at(k + 1)), at(k + 1)]);
            }
        }
        let (np, nl) = (plane.points.len(), levels.len());
        let id = |(p, h): (usize, usize)| h * np + p;
        let n = region.basis.normal();
        let mut nodes = Vec::with_capacity(np * nl);
        let mut of = Vec::with_capacity(np * nl);
        for &z in &levels {
            for (p, q) in plane.points.iter().enumerate() {
                let b = region.basis.lift(q[0], q[1]);
                nodes.push([0, 1, 2].map(|c| b[c] + z * n[c]));
                of.push((p, z));
            }
        }
        // each side of the face's mesh its middle node, by its corners
        let mut middle: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        for t in &plane.tris {
            for (m, (a, b)) in [(3, (0, 1)), (4, (1, 2)), (5, (2, 0))] {
                middle.insert(key(t[a], t[b]), t[m]);
            }
        }
        let mid = |(pa, ha): (usize, usize), (pb, hb): (usize, usize)| {
            let p = if pa == pb { pa } else { middle[&key(pa, pb)] };
            id((p, (ha + hb) / 2))
        };
        let mut tets = Vec::with_capacity(3 * plane.tris.len() * (nl / 2));
        for t in &plane.tris {
            let mut c = [t[0], t[1], t[2]];
            c.sort();
            for k in 0..nl / 2 {
                let (b, u) = (2 * k, 2 * k + 2);
                for mut corners in [[(c[0], b), (c[1], b), (c[2], b), (c[0], u)],
                                    [(c[1], b), (c[2], b), (c[0], u), (c[1], u)],
                                    [(c[2], b), (c[0], u), (c[1], u), (c[2], u)]] {
                    if tetra::volume6(corners.map(|q| nodes[id(q)])) < 0.0 {
                        corners.swap(1, 2);
                    }
                    let mut e = [0usize; 10];
                    for m in 0..4 {
                        e[m] = id(corners[m]);
                    }
                    for (m, &(i, j)) in EDGES.iter().enumerate() {
                        e[4 + m] = mid(corners[i], corners[j]);
                    }
                    tets.push(e);
                }
            }
        }
        // each segment of the face's boundary its side
        let mut side_of: BTreeMap<(usize, usize), usize> = BTreeMap::new();
        for (g, segs) in plane.sides.iter().enumerate() {
            for s in segs {
                side_of.insert(key(s[0], s[2]), g);
            }
        }
        let facets = boundary(&tets, &nodes).into_iter().filter_map(|f| {
            let at: Vec<(usize, usize)> = f[..3].iter().map(|&i| (i % np, i / np)).collect();
            let wall = if at.iter().all(|q| q.1 == 0) {
                Wall::Cap(false)
            } else if at.iter().all(|q| q.1 == nl - 1) {
                Wall::Cap(true)
            } else {
                let mut ps: Vec<usize> = at.iter().map(|q| q.0).collect();
                ps.sort();
                ps.dedup();
                Wall::Side(*side_of.get(&key(ps[0], *ps.last()?))?)
            };
            Some((f, wall))
        }).collect();
        Ok(Mesh { nodes, tets, facets, stack: Some(Stacked { region, plane, of }), near: tol })
    }

    /// **A solid that is no prism, meshed by refining its exact boundary** (#510 S3, P6's
    /// `delaunay::refine::volume`): the domain the boundary's signed distance
    /// (`BodyFaces::signed_distance`), the boundary's sharp edges — each where two faces meet at
    /// a corner — protected as feature curves, the cells held to a radius–edge ratio of 2, a
    /// least dihedral angle of 12° and a size of `FILLED` of the body's extent or `BREADTH` of
    /// its breadth (three times its volume over its area), whichever is less, `coarser` times
    /// it; each cell a TET10 whose boundary nodes lie on the surface and the curves.  Each facet
    /// of its boundary lies on the face its middle is nearest.
    pub fn filled(sk: &Sketch, solid: usize, coarser: f64) -> Result<Mesh, String> {
        use crate::delaunay::refine::{volume, Cells, Criteria};
        let body = BodyFaces::whole(sk, solid)?;
        let (lo, hi) = body.reach();
        let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
        if !(extent > 0.0) {
            return Err(format!("`{}` has no extent", sk.solids[solid].name));
        }
        let centre: V = std::array::from_fn(|k| 0.5 * (lo[k] + hi[k]));
        let radius = 0.6 * norm(sub(hi, lo));
        // its breadth, three times its volume over its area: a slab's thickness and a half, a
        // rod's radius and a half
        let (b, mm, _) = body.brep();
        let area: f64 = b.faces.iter().map(|f| crate::brep::props::area(b, f)).sum();
        let breadth = 3.0 * crate::brep::props::volume(b) / area / mm;
        let size = coarser * (FILLED * extent).min(BREADTH * breadth);
        let edges = body.brep().0.edges.len() as u32;
        let curves: Vec<Vec<V>> = (0..edges).filter(|&e| body.convex(e).is_some()).map(|e| {
            (0..=CURVE).map(|k| body.along(e, k as f64 / CURVE as f64)).collect()
        }).collect();
        let criteria = Criteria { facet_size: size, facet_distance: size / 20.0, facet_angle: 25.0,
                                  edge_size: size, bisection: 1e-9 * extent,
                                  max_points: 200_000, normal_angle: 0.0 };
        let cells = Cells { radius_edge: 2.0, size, dihedral: 12.0 };
        let domain = |p: V| body.signed_distance(p).0;
        let v = volume(domain, centre, radius, &curves, &criteria, &cells)
            .map_err(|m| format!("`{}` is not meshed: {m}", sk.solids[solid].name))?;
        let tets: Vec<[usize; 10]> = v.elements.iter().map(|e| e.map(|n| n as usize)).collect();
        let facets = boundary(&tets, &v.nodes).into_iter().map(|f| {
            let middle: V = std::array::from_fn(|c| {
                f[..3].iter().map(|&n| v.nodes[n][c]).sum::<f64>() / 3.0
            });
            (f, Wall::Face(body.signed_distance(middle).1))
        }).collect();
        Ok(Mesh { nodes: v.nodes, tets, facets, stack: None, near: size })
    }

    /// The node nearest `p`, within `tol`.
    pub fn node_at(&self, p: V, tol: f64) -> Option<usize> {
        (0..self.nodes.len()).map(|i| (norm(sub(self.nodes[i], p)), i)).filter(|&(d, _)| d <= tol)
            .min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, i)| i)
    }

    /// Its extent: the largest side of the box about its nodes.
    pub fn extent(&self) -> f64 {
        (0..3).map(|c| {
            let (lo, hi) = self.nodes.iter().fold((f64::INFINITY, f64::NEG_INFINITY),
                                                  |(l, h), p| (l.min(p[c]), h.max(p[c])));
            hi - lo
        }).fold(0.0, f64::max)
    }
}

/// **The boundary of a mesh of quadratic tetrahedra**: every face on one element, as a facet
/// `[a, b, c, ab, bc, ca]` turned so that `(b − a) × (c − a)` points away from the element's
/// fourth corner — out of the material.
pub fn boundary(tets: &[[usize; 10]], nodes: &[V]) -> Vec<[usize; 6]> {
    let mut faces: BTreeMap<[usize; 3], (usize, usize, u32)> = BTreeMap::new();
    for (t, e) in tets.iter().enumerate() {
        for far in 0..4 {
            let mut k = [0, 1, 2, 3].map(|c| e[c]);
            k.swap(far, 3);
            let mut f = [k[0], k[1], k[2]];
            f.sort();
            faces.entry(f).or_insert((t, far, 0)).2 += 1;
        }
    }
    let middle = |e: &[usize; 10], i: usize, j: usize| {
        let m = EDGES.iter().position(|&q| q == (i, j) || q == (j, i)).unwrap();
        e[4 + m]
    };
    faces.into_values().filter(|f| f.2 == 1).map(|(t, far, _)| {
        let e = &tets[t];
        let mut c: Vec<usize> = (0..4).filter(|&k| k != far).collect();
        if tetra::volume6([c[0], c[1], c[2], far].map(|k| nodes[e[k]])) > 0.0 {
            c.swap(1, 2);
        }
        [e[c[0]], e[c[1]], e[c[2]], middle(e, c[0], c[1]), middle(e, c[1], c[2]),
         middle(e, c[2], c[0])]
    }).collect()
}

/// What a line of the scenario names on the solid: nodes, facets of its boundary (by index), or
/// the whole.
enum On {
    Nothing,
    Nodes(Vec<usize>),
    Facets(Vec<usize>),
    Whole,
}

/// **The solid meshed under a scenario, as a structure**, and where the structure stands;
/// `name` the solid's, for what is said.
pub struct Filled {
    pub solid: usize,
    pub mesh: Mesh,
    pub s: Structure,
    pub x: Vec<f64>,
    name: String,
}

impl Filled {
    /// Solid `solid`, meshed (`mesh`), read under scenario `sc` and loaded, or why it cannot be.
    fn new(sk: &Sketch, sc: &Scenario, solid: usize, mesh: Mesh) -> Result<Filled, Unread> {
        let name = sk.solids[solid].name.clone();
        let young = property(sk, solid, Property::E)?;
        let nu = property(sk, solid, Property::Nu).map_err(|m| {
            format!("{m}: a reading in three dimensions reads Poisson's ratio")
        })?;
        let rho = property(sk, solid, Property::Rho);
        let n = mesh.nodes.len();
        let tets: Vec<Tet10> = mesh.tets.iter()
            .map(|&n| Tet10 { n, e: young, nu, rho: *rho.as_ref().unwrap_or(&f64::NAN) })
            .collect();
        let x: Vec<f64> = mesh.nodes.iter().flatten().copied().collect();
        let elements: Vec<usize> = (0..tets.len()).collect();
        let s = Structure { dim: 3, nodes: n, fixed: vec![false; 3 * n], load: vec![0.0; 3 * n],
                            weight: vec![0.0; 3 * n], tets, ..Structure::default() };
        // a line naming a part of the body (a solid united into it, a cutter) is about the body,
        // and a part's faces are named from it
        let parts = crate::solid::document::operand_paths(sk, solid);
        for a in sc.supports.iter().map(|s| &s.at).chain(sc.loads.iter().map(|l| &l.at)) {
            let Some(part) = a.solid().filter(|&p| p != solid) else { continue };
            if let Some(route) = parts.get(&sk.solids[part].name) {
                let route = if route.is_empty() { String::new() } else { format!("{route}.") };
                return Err(Unread::Why(format!(
                    "`{part}` is a part of `{name}`: a line about it names it from the body, \
                     `{name}.{route}…`", part = sk.solids[part].name)));
            }
        }
        let mut f = Filled { solid, mesh, s, x, name };
        // a support holds its nodes' moves along the world's axes: a solid element's node has no
        // turn, so `fixed` holds what `pinned` does
        for sup in &sc.supports {
            let on = f.on(sk, &sup.at)?;
            let nodes = f.nodes(&on);
            if nodes.is_empty() {
                continue;
            }
            let axes: Vec<usize> = match sup.along {
                None => vec![0, 1, 2],
                Some(d) => {
                    let d = direction(sk, d)?;
                    match (0..3).find(|&k| (d[k].abs() - 1.0).abs() <= 1e-9) {
                        Some(k) => vec![k],
                        None => return Err(Unread::Why(format!(
                            "a support holds `{}` along a direction off the world's axes, which a \
                             reading in three dimensions is not read under yet", f.name))),
                    }
                }
            };
            for node in nodes {
                for &k in &axes {
                    f.s.fixed[3 * node + k] = true;
                }
            }
        }
        for load in &sc.loads {
            let on = f.on(sk, &load.at)?;
            if matches!(on, On::Nothing) {
                continue;
            }
            let v = match &load.value {
                LoadValue::Vector(v) => *v,
                LoadValue::Along { magnitude, along } => scale(direction(sk, *along)?, *magnitude),
                LoadValue::Scalar(_) => [0.0; 3],
            };
            match (load.kind, &on) {
                (LoadKind::Force, On::Nodes(nodes)) => {
                    for &node in nodes {
                        for c in 0..3 {
                            f.s.load[3 * node + c] += v[c];
                        }
                    }
                }
                (LoadKind::Force, On::Facets(fs)) => {
                    let facets = f.facets(fs);
                    f.s.borne.push(Borne::Across { facets, force: v });
                }
                (LoadKind::Force, On::Whole) => {
                    f.s.borne.push(Borne::Through { elements: elements.clone(), force: v });
                }
                (LoadKind::Pressure, On::Facets(fs)) => {
                    let LoadValue::Scalar(p) = load.value else { continue };
                    let facets = f.facets(fs);
                    f.s.borne.push(Borne::Pressure { facets, p });
                }
                (LoadKind::Pressure, _) => {
                    return Err(Unread::Why(format!(
                        "a pressure pushes on faces of `{}`, and this names none", f.name)));
                }
                (LoadKind::Acceleration, _) => {
                    let rho = rho.clone()?;
                    f.s.borne.push(Borne::Body { elements: elements.clone(),
                                                 force: v.map(|a| rho * a) });
                }
                (LoadKind::Force, On::Nothing) => {}
            }
        }
        Ok(f)
    }

    /// What `a` names on the solid: a point's node, facets (a face path's faces, the line a
    /// prism's side was swept from), the whole (the solid, the face a prism is swept from) — or
    /// nothing of it.
    fn on(&self, sk: &Sketch, a: &Applied) -> Result<On, String> {
        let stack = self.mesh.stack.as_ref();
        let walls = |test: &dyn Fn(Wall) -> bool| -> On {
            let fs: Vec<usize> =
                (0..self.mesh.facets.len()).filter(|&i| test(self.mesh.facets[i].1)).collect();
            if fs.is_empty() { On::Nothing } else { On::Facets(fs) }
        };
        Ok(match a {
            Applied::Ent(e) if e.kind == EntKind::Point => {
                match self.mesh.node_at(sk.world_point(e.i()), self.mesh.near) {
                    Some(n) => On::Nodes(vec![n]),
                    None => On::Nothing,
                }
            }
            Applied::Ent(e) if e.kind == EntKind::Solid => {
                if e.i() == self.solid { On::Whole } else { On::Nothing }
            }
            Applied::Ent(e) if e.kind == EntKind::Face => {
                match stack {
                    Some(st) if st.region.is_face(sk, e.i()) => On::Whole,
                    _ => On::Nothing,
                }
            }
            Applied::Ent(e) => {
                // an edge of the drawing: the sides of a prism it is, by the places along it
                let Some(st) = stack else { return Ok(On::Nothing) };
                let r = &st.region;
                let tol = ON * r.extent;
                let sides = r.sides_where(sk, |q| prisms::edge_holds(sk, *e, r, q, tol));
                walls(&|w| matches!(w, Wall::Side(g) if sides.contains(&g)))
            }
            Applied::Face(f) if f.solid as usize == self.solid => {
                let body = BodyFaces::whole(sk, self.solid)?;
                let faces = body.faces(f);
                if faces.is_empty() {
                    return Err(format!("`{}.{}` names no face of `{}`", self.name,
                                       f.path.join("."), self.name));
                }
                match stack {
                    None => walls(&|w| matches!(w, Wall::Face(i) if faces.contains(&i))),
                    Some(st) => {
                        let r = &st.region;
                        let n = r.basis.normal();
                        // a cap: a plane square to the face's normal, out of the far side or the
                        // near one
                        let caps: Vec<bool> = faces.iter().filter_map(|&fi| {
                            let (m, _) = body.plane(fi)?;
                            let c = dot(m, n);
                            ((c.abs() - 1.0).abs() <= 1e-9).then_some(c > 0.0)
                        }).collect();
                        let near = 1e-6 * r.extent + body.tolerance();
                        let sides = r.sides_where(sk, |q| {
                            let p = r.lift(q);
                            faces.iter().any(|&fi| body.patch_distance(fi, p) <= near)
                        });
                        walls(&|w| match w {
                            Wall::Cap(far) => caps.contains(&far),
                            Wall::Side(g) => sides.contains(&g),
                            Wall::Face(_) => false,
                        })
                    }
                }
            }
            _ => On::Nothing,
        })
    }

    /// The facets `fs` (by index), each `[a, b, c, ab, bc, ca]` turned out of the material.
    fn facets(&self, fs: &[usize]) -> Vec<[usize; 6]> {
        fs.iter().map(|&i| self.mesh.facets[i].0).collect()
    }

    /// The nodes `on` names.
    fn nodes(&self, on: &On) -> Vec<usize> {
        let mut out: Vec<usize> = match on {
            On::Nothing => Vec::new(),
            On::Nodes(n) => n.clone(),
            On::Facets(fs) => self.facets(fs).into_iter().flatten().collect(),
            On::Whole => (0..self.mesh.nodes.len()).collect(),
        };
        out.sort();
        out.dedup();
        out
    }
}

/// One solid of the scenario, analysed: the solid and its structure, and what the requirements
/// read of it, each worked out once and only where one is asked for — the displacements under
/// the loads among them (a modal scenario reads none).
struct Analysed {
    f: Filled,
    st: OnceCell<Result<State, String>>,
    k: OnceCell<Result<Sym, String>>,
    modes: usize,
    pairs: OnceCell<Result<Pairs, String>>,
    buckled: OnceCell<Result<f64, String>>,
    stresses: OnceCell<Vec<f64>>,
}

impl Analysed {
    fn new(sk: &Sketch, sc: &Scenario, solid: usize, mesh: Mesh, modes: usize)
        -> Result<Analysed, Unread>
    {
        let f = Filled::new(sk, sc, solid, mesh)?;
        // a motion the supports leave free is named from them, before anything is factored
        if let Some(free) = f.free() {
            return Err(Unread::Free(free));
        }
        Ok(Analysed { f, st: OnceCell::new(), k: OnceCell::new(), modes,
                      pairs: OnceCell::new(), buckled: OnceCell::new(), stresses: OnceCell::new() })
    }

    /// The displacements under the scenario's loads, solved once.
    fn state(&self) -> Result<&State, String> {
        self.st.get_or_init(|| {
            self.f.s.solve(&self.f.x).map_err(|why| unsolved(&self.f, why).text(&self.f.name))
        }).as_ref().map_err(Clone::clone)
    }

    /// **The solid drawn** (`deformed::Body`): its boundary's facets, each six-node triangle cut
    /// in four, over the nodes they use, each moving as the solve moves it — or why it does not
    /// move (its structure gave no displacement).
    fn body(&self, name: &str) -> Result<Body, String> {
        let u = &self.state()?.u;
        let mesh = &self.f.mesh;
        let mut body = Body { kind: "solid", name: name.to_string(), solid: Some(self.f.solid),
                              ..Body::default() };
        let mut at: BTreeMap<usize, u32> = BTreeMap::new();
        let mut facets = Vec::new();
        for (n, _) in &mesh.facets {
            let n = n.map(|i| *at.entry(i).or_insert_with(|| {
                body.vertex(mesh.nodes[i], [u[3 * i], u[3 * i + 1], u[3 * i + 2]])
            }));
            body.quadratic(n);
            facets.push(n);
        }
        body.side_edges(facets.iter().flat_map(|&[a, b, c, ab, bc, ca]| {
            [[a, ab, b], [b, bc, c], [c, ca, a]]
        }));
        Ok(body)
    }

    /// Node `i`'s move.
    fn moved(&self, i: usize) -> Result<V, String> {
        let u = &self.state()?.u;
        Ok([u[3 * i], u[3 * i + 1], u[3 * i + 2]])
    }

    /// Its stiffness, assembled once for its modes and its buckling.
    fn stiffness(&self) -> Result<&Sym, String> {
        let k = || self.f.s.stiffness(&self.f.x).map_err(|_| "no stiffness".to_string());
        self.k.get_or_init(k).as_ref().map_err(Clone::clone)
    }

    /// The modes the requirements ask for, solved once for the most of them.
    fn pairs(&self, sk: &Sketch) -> Result<&Pairs, String> {
        self.pairs.get_or_init(|| {
            property(sk, self.f.solid, Property::Rho)?;
            prisms::modes(&self.f.s, &self.f.x, self.stiffness()?, self.modes)
        }).as_ref().map_err(Clone::clone)
    }

    /// The least factor on the loads at which the solid buckles: the stiffness against the
    /// geometric stiffness of the stress the loads make.
    fn buckled(&self) -> Result<f64, String> {
        self.buckled.get_or_init(|| {
            let kg = self.f.s.stressed(&self.f.x, &self.state()?.u)
                .map_err(|_| "an element of its mesh is turned over".to_string())?;
            let pairs = eigen::buckling(self.stiffness()?, &kg, 1, &eigen::Opts::default())
                .map_err(|why| format!("its buckling is not found: {why:?}"))?;
            pairs.values.first().copied()
                .ok_or_else(|| "the loads press nothing, so nothing buckles".to_string())
        }).clone()
    }

    /// Each node's von Mises stress: the elements' stresses at it, averaged, then read.
    fn stresses(&self) -> Result<&[f64], String> {
        let u = &self.state()?.u;
        Ok(self.stresses.get_or_init(|| {
            let s = &self.f.s;
            let n = s.nodes;
            let mut sum = vec![[0.0; 6]; n];
            let mut count = vec![0u32; n];
            for (i, t) in s.tets.iter().enumerate() {
                let u: [f64; 30] = std::array::from_fn(|k| u[3 * t.n[k / 3] + k % 3]);
                let Some(at) = t.stresses(&s.tet_places(i, &self.f.x), &u) else { continue };
                for (k, &node) in t.n.iter().enumerate() {
                    for c in 0..6 {
                        sum[node][c] += at[k][c];
                    }
                    count[node] += 1;
                }
            }
            (0..n).map(|i| match count[i] {
                0 => 0.0,
                c => Tet10::von_mises(sum[i].map(|v| v / c as f64)),
            }).collect()
        }))
    }
}

impl Filled {
    /// The rigid motions its supports leave, in the solver's words (`analysis::freedoms`, every
    /// held degree of freedom holding its node along its axis) — `None` where they hold it.  A
    /// mesh of solid elements is one body, so a motion of its nodes no element strains is a
    /// rigid one.
    fn free(&self) -> Option<String> {
        let axes = [0, 1, 2].map(analysis::axis);
        let holds: Vec<Held> = (0..self.s.fixed.len()).filter(|&i| self.s.fixed[i])
            .map(|i| Held { at: vec![self.mesh.nodes[i / 3]], along: Some(axes[i % 3]),
                            turn: false })
            .collect();
        analysis::freedoms_text(&analysis::freedoms(3, &self.mesh.nodes, &holds), 3)
    }
}

/// Why a solid's structure gave no displacement, in the solver's words: the rigid motions its
/// supports leave.
fn unsolved(f: &Filled, why: Failure) -> Unread {
    Unread::of(why, || f.free().unwrap_or_else(|| "free".to_string()))
}

/// **How solid `solid` is meshed in three dimensions**, the points `named` its nodes, `coarser`
/// times the reading's size — or why it cannot be: a prism is its face's mesh stacked, the
/// points it holds nodes; any other solid its exact boundary refined and filled
/// (`Mesh::filled`), a point naming the node nearest it.
pub fn mesh(sk: &Sketch, solid: usize, named: &[usize], coarser: f64) -> Result<Mesh, String> {
    let Ok(region) = Region::of(sk, solid) else {
        return Mesh::filled(sk, solid, coarser);
    };
    let points: Vec<(P, f64)> = named.iter().filter_map(|&i| {
        let a = Applied::Ent(crate::model::EntRef::point(i));
        let p = prisms::point_of(sk, &region, &a)?;
        Some((p, region.local(sk.world_point(i)).1))
    }).collect();
    Mesh::stacked(sk, region, &points, coarser)
}

/// **Solid `solid` read under scenario `sc` as a structure**, `coarser` times the reading's size,
/// and where it stands — or why it cannot be read.
pub fn structure(sk: &Sketch, sc: &Scenario, solid: usize, coarser: f64)
    -> Result<(Structure, Vec<f64>), String>
{
    let mesh = mesh(sk, solid, &named_points(sc), coarser)?;
    let f = Filled::new(sk, sc, solid, mesh).map_err(|u| u.text(&sk.solids[solid].name))?;
    Ok((f.s, f.x))
}

/// What scenario `sc` names, and the point `at`: what a row's mesh stands a node at.
fn named_applied(sc: &Scenario, at: usize) -> Vec<Applied> {
    sc.applied().cloned().chain([Applied::Ent(crate::model::EntRef::point(at))]).collect()
}

/// The points a scenario names: what a stacked mesh makes nodes of.
fn named_points(sc: &Scenario) -> Vec<usize> {
    let mut out: Vec<usize> = sc.applied().filter_map(|a| match a {
        Applied::Ent(e) if e.kind == EntKind::Point => Some(e.i()),
        _ => None,
    }).collect();
    out.sort();
    out.dedup();
    out
}

/// The solid among the drawing's that what a line names falls on, not a solid's own (a point in
/// it, its face, an edge one of its sides runs along) — made of a material, read in no plane; a
/// prism by its region, any other by its exact boundary (a point within it); `None` where there
/// is none, or where it is a joint of a frame.
fn solid_of(sk: &Sketch, sc: &Scenario, a: &Applied) -> Option<usize> {
    let reads = |s: usize| sk.solids[s].material.is_some() && sc.plane_of(s).is_none();
    if let Some(s) = a.solid() {
        return reads(s).then_some(s);
    }
    if crate::beams::of_frame(sk, sc, a) {
        return None;
    }
    (0..sk.solids.len()).find(|&s| {
        reads(s) && match Region::of(sk, s) {
            Ok(r) => prisms::falls_on(sk, &r, a),
            Err(_) => match a {
                Applied::Ent(e) if e.kind == EntKind::Point => BodyFaces::whole(sk, s)
                    .is_ok_and(|b| b.signed_distance(sk.world_point(e.i())).0 <= b.tolerance()),
                _ => false,
            },
        }
    })
}

/// **A scenario read over the solids it reads in three dimensions**, at the solved drawing:
/// per requirement its value or why none (`None` for one about no solid read so), the words its
/// solids say (one the supports leave free, once), which supports hold such a solid, and which
/// solids are read.
pub fn read(sk: &Sketch, sc: &Scenario) -> Read {
    read_at(sk, sc, 1.0)
}

/// `read`, every mesh `coarser` times the reading's size.
pub fn read_at(sk: &Sketch, sc: &Scenario, coarser: f64) -> Read {
    read_pictured(sk, sc, coarser, false).0
}

/// **`read_at`, and where `pictured`, the picture of every solid read in three dimensions that a
/// line of the scenario falls on** (`deformed::Body`: its tetrahedra's boundary), or why one is
/// not drawn: each solid meshed and solved once for both.
pub fn read_pictured(sk: &Sketch, sc: &Scenario, coarser: f64, pictured: bool)
    -> (Read, Vec<Result<Body, String>>)
{
    let mut drawn = Vec::new();
    let mut out = Read {
        words: Vec::new(),
        answers: vec![None; sc.requirements.len()],
        holds: vec![false; sc.supports.len()],
        ..Read::default()
    };
    let of: Vec<Option<usize>> = sc.requirements.iter().map(|r| solid_of(sk, sc, &r.of)).collect();
    let mut solids: Vec<usize> = of.iter().flatten().copied().collect();
    solids.sort();
    solids.dedup();
    // what is drawn: every solid a line falls on, a requirement's or not
    let mut touched: Vec<usize> = if pictured {
        sc.applied().filter_map(|a| solid_of(sk, sc, a)).collect()
    } else {
        Vec::new()
    };
    touched.sort();
    touched.dedup();
    if solids.is_empty() && touched.is_empty() {
        return (out, drawn);
    }
    for (k, s) in sc.supports.iter().enumerate() {
        out.holds[k] = solid_of(sk, sc, &s.at).is_some_and(|i| solids.contains(&i));
    }
    let mut modes: BTreeMap<usize, usize> = BTreeMap::new();
    for (r, s) in sc.requirements.iter().zip(&of) {
        if let (Output::Mode, Some(s)) = (r.output, s) {
            let n = modes.entry(*s).or_default();
            *n = (*n).max(r.count.unwrap_or(1) as usize);
        }
    }
    let named = named_points(sc);
    let mut read: BTreeMap<usize, Result<Analysed, Unread>> = BTreeMap::new();
    for (k, (r, s)) in sc.requirements.iter().zip(&of).enumerate() {
        let Some(s) = *s else { continue };
        let a = read.entry(s).or_insert_with(|| {
            let mesh = mesh(sk, s, &named, coarser)?;
            Analysed::new(sk, sc, s, mesh, modes.get(&s).copied().unwrap_or(0))
        });
        out.answers[k] = Some(match a {
            Ok(a) => answer(sk, a, r),
            Err(m) => Err(m.text(&sk.solids[s].name)),
        });
    }
    for (s, a) in &read {
        if let Err(Unread::Free(free)) = a {
            out.words.push(format!("scenario `{}` leaves `{}` {free}", sc.name,
                                   sk.solids[*s].name));
        }
    }
    for s in touched {
        let a = read.entry(s).or_insert_with(|| {
            let mesh = mesh(sk, s, &named, coarser)?;
            Analysed::new(sk, sc, s, mesh, 0)
        });
        let name = &sk.solids[s].name;
        drawn.push(match a {
            Ok(a) => a.body(name).map_err(|m| format!("scenario `{}`: {m}", sc.name)),
            Err(m) => Err(format!("scenario `{}`: {}", sc.name, m.text(name))),
        });
    }
    out.bodies = solids;
    (out, drawn)
}

/// A requirement's value over its solid `a`, or why it has none.
fn answer(sk: &Sketch, a: &Analysed, r: &Requirement) -> Result<f64, String> {
    let f = &a.f;
    let name = &f.name;
    let nodes = f.nodes(&f.on(sk, &r.of)?);
    match r.output {
        Output::Deflection => {
            let along = r.along.map(|d| direction(sk, d)).transpose()?;
            let mut worst: Option<f64> = None;
            for i in nodes {
                let u = a.moved(i)?;
                let v = match along {
                    Some(d) => dot(u, d),
                    None => norm(u),
                };
                if worst.is_none_or(|w| v.abs() > w.abs()) {
                    worst = Some(v);
                }
            }
            worst.ok_or_else(|| format!("it names no node of `{name}`'s mesh"))
        }
        Output::Compliance => {
            Ok(f.s.forces(&f.x).iter().zip(&a.state()?.u).map(|(p, u)| p * u).sum())
        }
        Output::Mode => {
            let n = r.count.unwrap_or(1) as usize;
            let v = *a.pairs(sk)?.values.get(n - 1).ok_or("it has fewer modes than that")?;
            Ok(v.max(0.0).sqrt() / (2.0 * std::f64::consts::PI))
        }
        Output::Buckling => a.buckled(),
        Output::Safety => {
            let body = BodyFaces::whole(sk, f.solid)?;
            if let Some(edge) = analysis::reentrant(&body).first() {
                return Err(edge.text());
            }
            let yields = property(sk, f.solid, Property::Yield)?;
            let stresses = a.stresses()?;
            let worst = nodes.iter().map(|&i| stresses[i]).fold(0.0, f64::max);
            if !(worst > 0.0) {
                return Err(format!("the loads stress `{name}` nowhere"));
            }
            Ok(yields / worst)
        }
    }
}

/// **A mass or a volume of solid `solid`** (`reads`), meshed at `coarser` times a reading's size:
/// a prism's as its driving row reads it, its face through its depth (`prisms::measure`); any
/// other solid's over its own tetrahedra — what it is made of, read with no solve, so a solid its
/// supports leave free has one.
pub(crate) fn measure(sk: &Sketch, solid: usize, reads: Reads, coarser: f64)
    -> Result<f64, String>
{
    if Region::of(sk, solid).is_ok() {
        return prisms::measure(sk, solid, reads, coarser);
    }
    let rho = if reads == Reads::Mass { property(sk, solid, Property::Rho)? } else { 0.0 };
    let mesh = mesh(sk, solid, &[], coarser)?;
    let tets = mesh.tets.iter().map(|&n| Tet10 { n, e: 0.0, nu: 0.0, rho }).collect();
    let s = Structure { dim: 3, nodes: mesh.nodes.len(), tets, reads, ..Structure::default() };
    let x: Vec<f64> = mesh.nodes.iter().flatten().copied().collect();
    Ok(s.measure(&x))
}

/* -- a driving row (#510 S3) ------------------------------------------------------------------ */

/// **Whether what a requirement names is a point of a solid read in three dimensions** (#510
/// S3), a prism: what a driving row over a solid may be compiled at.
pub fn of_body(sk: &Sketch, sc: &Scenario, of: &Applied) -> bool {
    matches!(of, Applied::Ent(e) if e.kind == EntKind::Point)
        && solid_of(sk, sc, of).is_some_and(|s| Region::of(sk, s).is_ok())
}

/// **The row of `deflection(at, along: along)` under scenario `sc`** over the solid whose point
/// `at` is (#510 S3), or why the solve cannot read it: the row a region's is (`prisms::Row`) —
/// the solid meshed once, where the drawing stands, its structure under the scenario with the
/// deflection's weights on its point's node — its nodes the face's moved with the drawing
/// (`prisms::moved_with`) and stacked through the depth (`interior::Stack`), so each node of the
/// solid moves as the face's node it stands over does.  Refused as a region's is: a direction
/// the solve moves (its own, a load's), a plane it turns, a side it does not move (a spline's,
/// a curve's).
pub fn row(sk: &Sketch, sc: &Scenario, at: usize, along: crate::model::EntRef)
    -> Result<prisms::Row, String>
{
    crate::beams::directions_held(sk, sc, along)?;
    let of = Applied::Ent(crate::model::EntRef::point(at));
    let solid = solid_of(sk, sc, &of).ok_or("it is of a point of no solid the scenario reads")?;
    let face_plane = prisms::held_plane(sk, solid)?;
    let mut named = named_points(sc);
    named.push(at);
    let f = Filled::new(sk, sc, solid, mesh(sk, solid, &named, 1.0)?)
        .map_err(|u| u.text(&sk.solids[solid].name))?;
    let node = f.mesh.node_at(sk.world_point(at), f.mesh.near)
        .ok_or("its point is no node of its solid's mesh")?;
    let Filled { mesh, mut s, x, .. } = f;
    let d = direction(sk, along)?;
    for c in 0..3 {
        s.weight[3 * node + c] = d[c];
    }
    let st = mesh.stack.ok_or("it is read over no prism")?;
    let plane_x: Vec<f64> = st.plane.points.iter().flatten().copied().collect();
    let (mut moved, columns) =
        prisms::moved_with(sk, &named_applied(sc, at), &st.region, &st.plane, &plane_x,
                           face_plane)?;
    let b = &st.region.basis;
    moved.stack = Some(crate::interior::Stack { basis: [b.o, b.u, b.v, b.normal()], of: st.of });
    Ok(prisms::Row { structure: s, moved, columns, x })
}
