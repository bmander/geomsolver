//! **A filled body's hull, read off the drawing** (#567): what a driving row over a solid that
//! is no prism moves its mesh by (`interior::Hull`).
//!
//! Every face of the body's exact boundary is named by its path (`FacePath`): the route through
//! the body's operands to the primitive it is a piece of, and its name there — a prism's cap
//! (`near`, `far`) or the side its profile's edge sweeps, a turn's face its profile's edge turns.
//! A boolean trims a face and never changes the surface it lies on, so each face is a piece of
//! one **sheet**, a closed form of the drawing's numbers: the profile's points and radii, the
//! plane it is drawn in (held, or read off its own numbers where the solve may move it), the
//! axis it is turned about.  Each node on the boundary is tagged with the sheets of the facets it
//! is a node of — independent at the node, three at most (a face's one, an edge's two, a
//! vertex's three), and only those it lies on.
//!
//! What it does not move yet is said, and the row compiles nothing: a face of a placed copy, of a
//! sweep, a round or a region, of a drafted prism or a partial turn, a side swept from a spline or
//! a curve.

use crate::bodies::{Mesh, Wall};
use crate::interior::{Axis, Direction, Frame, Hull, Place, Profile, Sheet};
use crate::model::{EntKind, EntRef, Sketch, SolidDef};
use crate::solid::faces::BodyFaces;
use std::collections::BTreeMap;

/// Two sheets meet at a node, rather than touch, where their normals there are this far from
/// parallel (a sine): a side's line run tangent into a round lies on one sheet, not on two.
const MEET: f64 = 1e-2;

/// A sheet is one a node lies on where its level there is within this of the mesh's element size:
/// a node of an edge left straight (or put back straight) stands off its face by its chord's sag.
const NEAR: f64 = 0.25;

/// What the sheets are made of: the columns read so far, each point's (its x, its y the next),
/// each plane's and each sheet's.
#[derive(Default)]
struct Builder {
    columns: Vec<u32>,
    points: BTreeMap<usize, usize>,
    frames: Vec<Frame>,
    frame_of: BTreeMap<Option<u32>, usize>,
    sheets: Vec<Sheet>,
    sheet_of: BTreeMap<(usize, String), usize>,
}

impl Builder {
    /// Point `q`'s column (its x; its y the next).
    fn point(&mut self, sk: &Sketch, q: usize) -> usize {
        *self.points.entry(q).or_insert_with(|| {
            let pt = &sk.points[q];
            self.columns.extend([pt.x, pt.y]);
            self.columns.len() - 2
        })
    }

    /// **Point `q` as a sheet reads it** (#953): held where nothing the solve moves reads it, a
    /// point in space by its three columns, a point of a plane or a spatial frame by its own
    /// columns lifted by that plane.
    fn place(&mut self, sk: &Sketch, q: usize) -> Place {
        let pt = &sk.points[q];
        let own: Vec<u32> = [Some(pt.x), Some(pt.y), pt.z].into_iter().flatten().collect();
        let held = |sk: &Sketch| own.iter().all(|&c| sk.params[c as usize].fixed)
            && pt.chart.or(pt.plane).is_none_or(|f| sk.plane_fixed(f as usize));
        if held(sk) {
            return Place::Held(sk.world_point(q));
        }
        match (pt.z, pt.chart) {
            (Some(z), Some(f)) => {
                let frame = self.frame(sk, Some(f));
                let c = self.columns.len();
                self.columns.extend([pt.x, pt.y, z]);
                Place::Chart { frame, c }
            }
            (Some(z), None) => {
                let c = self.columns.len();
                self.columns.extend([pt.x, pt.y, z]);
                Place::Space(c)
            }
            (None, _) => {
                let frame = self.frame(sk, sk.plane_of(q).map(|p| p as u32));
                Place::Plane { frame, c: self.point(sk, q) }
            }
        }
    }

    /// **A half-space's direction as a sheet reads it** (#953): a plane's normal, an axis's
    /// direction (held where the solve moves no number of it) — or why it is not moved yet.
    fn direction(&mut self, sk: &Sketch, along: EntRef) -> Result<Direction, String> {
        Ok(match along.kind {
            EntKind::Frame => Direction::Normal(self.frame(sk, Some(along.idx))),
            EntKind::Axis => {
                let d = sk.axes[along.i()].d;
                if d.iter().all(|&q| sk.params[q as usize].fixed) {
                    Direction::Held(d.map(|q| sk.params[q as usize].value))
                } else {
                    let c = self.columns.len();
                    self.columns.extend(d);
                    Direction::Read(c)
                }
            }
            EntKind::Line => {
                let l = &sk.lines[along.i()];
                let held = |q: u32| {
                    let pt = &sk.points[q as usize];
                    [Some(pt.x), Some(pt.y), pt.z].into_iter().flatten()
                        .all(|c| sk.params[c as usize].fixed)
                        && pt.chart.or(pt.plane).is_none_or(|f| sk.plane_fixed(f as usize))
                };
                if !(held(l.p1) && held(l.p2)) {
                    return Err("a half-space along a line the solve moves, which a driving row \
                                does not move yet".into());
                }
                let d = crate::space::sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize));
                Direction::Held(d)
            }
            kind => return Err(format!("a half-space along {}", kind.a())),
        })
    }

    /// A parameter's column, as a new one.
    fn param(&mut self, p: u32) -> usize {
        self.columns.push(p);
        self.columns.len() - 1
    }

    /// The plane a profile is drawn in (`None` the page): held where nothing the solve moves
    /// reads it, else read off its origin's and its axes' numbers.
    fn frame(&mut self, sk: &Sketch, plane: Option<u32>) -> usize {
        if let Some(&f) = self.frame_of.get(&plane) {
            return f;
        }
        let held = |b: crate::plane::Basis| {
            let mut k = [0.0; 9];
            k[..3].copy_from_slice(&b.o);
            k[3..6].copy_from_slice(&b.u);
            k[6..].copy_from_slice(&b.v);
            Frame::Held(k)
        };
        let frame = match plane {
            None => held(crate::plane::Basis::page()),
            Some(p) => match &sk.frames[p as usize].chart {
                Some(c) if !sk.plane_fixed(p as usize) => {
                    let at = self.columns.len();
                    self.columns.extend(c.o);
                    self.columns.extend(sk.axes[c.u as usize].d);
                    self.columns.extend(sk.axes[c.v as usize].d);
                    Frame::Read(at)
                }
                _ => held(sk.basis(p as usize)),
            },
        };
        self.frames.push(frame);
        self.frame_of.insert(plane, self.frames.len() - 1);
        self.frames.len() - 1
    }

    /// A profile's side `e` as the sheets read it: a line, or a circle or an arc's circle.
    fn curve(&mut self, sk: &Sketch, e: EntRef) -> Result<Profile, String> {
        Ok(match e.kind {
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                Profile::Line { a: self.point(sk, l.p1 as usize), b: self.point(sk, l.p2 as usize) }
            }
            EntKind::Arc => {
                let a = &sk.arcs[e.i()];
                Profile::Circle { c: self.point(sk, a.center as usize), r: self.param(a.radius) }
            }
            EntKind::Circle => {
                let c = &sk.circles[e.i()];
                Profile::Circle { c: self.point(sk, c.center as usize), r: self.param(c.radius) }
            }
            kind => return Err(format!("a side swept from {}, which a driving row does not \
                                        move yet", kind.a())),
        })
    }

    /// **The sheet face `face` of primitive `p` lies on**, made once: a prism's cap at its
    /// ordinate, a prism's side or a full turn's face by the profile's edge of that name.
    fn sheet(&mut self, sk: &Sketch, p: usize, face: &str) -> Result<usize, String> {
        if let Some(&s) = self.sheet_of.get(&(p, face.to_string())) {
            return Ok(s);
        }
        let solid = &sk.solids[p];
        let edge = |f: u32| -> Result<EntRef, String> {
            sk.faces[f as usize].boundaries()
                .find_map(|(es, ns)| es.iter().zip(ns).find(|(_, n)| *n == face).map(|(e, _)| *e))
                .ok_or_else(|| format!("`{}` has no side `{face}`", solid.name))
        };
        let sheet = match &solid.def {
            SolidDef::Prism { face: f, from, to, draft: None } => {
                let frame = self.frame(sk, sk.faces[*f as usize].plane);
                let (lo, hi) = (from.value.min(to.value), from.value.max(to.value));
                match face {
                    "near" => Sheet::Cap { frame, h: hi },
                    "far" => Sheet::Cap { frame, h: lo },
                    _ => Sheet::Extruded { frame, curve: self.curve(sk, edge(*f)?)? },
                }
            }
            SolidDef::Revolve { face: f, axis, sweep, .. }
                if sweep.value.abs() >= std::f64::consts::TAU - 1e-9 =>
            {
                let frame = self.frame(sk, sk.faces[*f as usize].plane);
                let curve = self.curve(sk, edge(*f)?)?;
                let axis = match axis.kind {
                    EntKind::Line => {
                        let l = &sk.lines[axis.i()];
                        Axis::Line { a: self.point(sk, l.p1 as usize),
                                     b: self.point(sk, l.p2 as usize) }
                    }
                    EntKind::Axis => {
                        let x = &sk.axes[axis.i()];
                        let held = |q: &[u32; 3]| q.iter().all(|&q| sk.params[q as usize].fixed);
                        if held(&x.a) && held(&x.d) {
                            let value = |q: [u32; 3]| q.map(|q| sk.params[q as usize].value);
                            Axis::Held { a: value(x.a), d: value(x.d) }
                        } else {
                            let a = self.columns.len();
                            self.columns.extend(x.a);
                            self.columns.extend(x.d);
                            Axis::Read { a, d: a + 3 }
                        }
                    }
                    _ => return Err(format!("`{}` is turned about no line and no axis", solid.name)),
                };
                Sheet::Turned { frame, axis, curve }
            }
            // a half-space kept with a body (#953): the plane of the term its face is named by
            SolidDef::Region { terms, .. } if crate::solid::defined::kept_half(sk, p) => {
                let mut named = terms.iter().filter(|t| t.name == face);
                let (Some(t), None) = (named.next(), named.next()) else {
                    return Err(format!("`{}` names no one half-space `{face}`", solid.name));
                };
                let crate::model::RegionShape::Plane { along, from, sign } = t.shape else {
                    unreachable!("a kept half-space's every term is one")
                };
                if t.cmp == crate::vocab::Cmp::In {
                    return Err(format!("`{face}` is an interval of a half-space's ordinate, whose \
                                        two planes a driving row does not move yet"));
                }
                let at = self.place(sk, from as usize);
                let along = self.direction(sk, along)?;
                // `sign · n · (x − o) = c + m · t`: the plane where the ordinate along `sign · n`
                // is the term's number, held to `n` by turning the number's sign
                let c = sign * t.lo.c;
                let (m, t) = match &t.lo.free {
                    Some((name, m)) => {
                        let q = sk.free_vars.get(name).copied().filter(|&q| !sk.params[q as usize].fixed)
                            .ok_or_else(|| format!("`{name}` is an unknown nothing determines"))?;
                        (*m, Some(self.param(q)))
                    }
                    None => (0.0, None),
                };
                Sheet::Ordinate { at, along, c, m: sign * m, t }
            }
            SolidDef::Prism { .. } => {
                return Err(format!("`{}` is drafted, which a driving row does not move yet",
                                   solid.name));
            }
            SolidDef::Revolve { .. } => {
                return Err(format!("`{}` is a partial turn, whose ends a driving row does not \
                                    move yet", solid.name));
            }
            _ => {
                return Err(format!("`{}` is no prism and no turn, whose faces a driving row does \
                                    not move yet", solid.name));
            }
        };
        self.sheets.push(sheet);
        self.sheet_of.insert((p, face.to_string()), self.sheets.len() - 1);
        Ok(self.sheets.len() - 1)
    }
}

/// Sheet `s`'s normal at `p` where the columns stand at `v`, made unit.
fn unit_level(hull: &Hull, s: usize, v: &[f64], p: [f64; 3]) -> [f64; 3] {
    crate::space::unit(hull.level_at(s, v, p).1)
}

/// Whether sheets whose unit normals at a place are `normals` (one, two or three) meet there
/// rather than touch: the second no nearer parallel to the first than `MEET`, the third no nearer
/// the plane of the first two.
fn meet(normals: &[[f64; 3]]) -> bool {
    use crate::space::{cross, dot, norm};
    let finite = normals.iter().flatten().all(|c| c.is_finite());
    finite && match normals {
        [_] => true,
        [a, b] => norm(cross(*a, *b)) >= MEET,
        [a, b, c] => {
            let ab = cross(*a, *b);
            norm(ab) >= MEET && dot(ab, *c).abs() >= MEET * norm(ab)
        }
        _ => false,
    }
}

/// **Solid `solid`'s mesh as its hull moves it** (#567): its boundary's faces each a sheet, every
/// node of a facet tagged with the sheets it lies on, the places and the feet at the drawing as it
/// stands — and the columns the sheets read.  Or why a driving row cannot move it.
pub fn hull_of(sk: &Sketch, solid: usize, mesh: &Mesh) -> Result<(Hull, Vec<u32>), String> {
    hull_aimed(sk, solid, mesh, None, None)
}

/// **`hull_of`, and what a measure besides the whole reads off its columns** (#953): the point
/// `at` a centroid is compared with, and the line or axis `about` a second moment is taken about
/// — each held where nothing the solve moves reads it, else read as a sheet's point is.  Refused
/// for a line whose ends the solve moves, whose direction a row does not read yet.
pub fn hull_aimed(sk: &Sketch, solid: usize, mesh: &Mesh, at: Option<usize>,
                  about: Option<EntRef>) -> Result<(Hull, Vec<u32>), String> {
    hull_of_bodies(sk, &[(solid, mesh)], at, about)
}

/// **`hull_aimed` over several bodies** (#953): each body's mesh after the one before, its nodes
/// numbered on from theirs, every body's sheets read over one set of columns — what a centre of
/// mass of an instance's bodies is read over.
pub fn hull_of_bodies(sk: &Sketch, parts: &[(usize, &Mesh)], at: Option<usize>,
                      about: Option<EntRef>) -> Result<(Hull, Vec<u32>), String> {
    let mut b = Builder::default();
    let target = at.map(|q| b.place(sk, q));
    let axis = match about {
        None => None,
        // a line: its first point, and its direction held, or read toward its second point
        Some(e) if e.kind == EntKind::Line => {
            let l = &sk.lines[e.i()];
            let a = b.place(sk, l.p1 as usize);
            let d = match b.direction(sk, e) {
                Ok(d) => d,
                Err(_) => Direction::Toward(b.place(sk, l.p2 as usize)),
            };
            Some((a, d))
        }
        Some(e) if e.kind == EntKind::Axis => {
            let x = &sk.axes[e.i()];
            let held = x.a.iter().all(|&q| sk.params[q as usize].fixed);
            let a = if held {
                Place::Held(x.a.map(|q| sk.params[q as usize].value))
            } else {
                let c = b.columns.len();
                b.columns.extend(x.a);
                Place::Space(c)
            };
            Some((a, b.direction(sk, e)?))
        }
        Some(e) => return Err(format!("a second moment is about a line or an axis, and this is {}",
                                      e.kind.a())),
    };
    let (mut hull, columns) = hull_with(sk, parts, b)?;
    hull.target = target;
    hull.axis = axis;
    Ok((hull, columns))
}

/// `hull_of` of the bodies `parts`, each its mesh, over the columns `b` has read already.
fn hull_with(sk: &Sketch, parts: &[(usize, &Mesh)], mut b: Builder)
    -> Result<(Hull, Vec<u32>), String>
{
    // each body's faces' sheets, and where its nodes start among all of them
    let mut sheets_of = Vec::new();
    let mut first = Vec::new();
    let mut n = 0;
    for &(solid, mesh) in parts {
        let body = BodyFaces::whole(sk, solid)?;
        let face_sheet: Vec<usize> = body.paths().iter().map(|path| {
            crate::solid::faces::face_primitive(sk, solid, &path.route)
                .and_then(|p| b.sheet(sk, p, &path.face))
                .map_err(|why| format!("its face `{}` is not moved: {why}", path.text()))
        }).collect::<Result<_, _>>()?;
        sheets_of.push(face_sheet);
        first.push(n);
        n += mesh.nodes.len();
    }
    let extent = parts.iter().map(|(_, m)| m.extent()).fold(0.0, f64::max);
    let mut hull = Hull { columns: b.columns.len(), frames: b.frames, sheets: b.sheets,
                          nodes: vec![Vec::new(); n],
                          x0: parts.iter().flat_map(|(_, m)| m.nodes.iter().flatten().copied())
                              .collect(),
                          feet0: vec![0.0; 3 * n], extent, target: None, axis: None };
    let v0: Vec<f64> = b.columns.iter().map(|&c| sk.params[c as usize].value).collect();
    for (k, &(_, mesh)) in parts.iter().enumerate() {
        tag(&mut hull, mesh, &sheets_of[k], first[k], &v0);
    }
    Ok((hull, b.columns))
}

/// **Each node of body `mesh`'s boundary tagged with the sheets it lies on** (`hull_of`), its
/// faces' sheets `face_sheet`, its nodes `first` on among the hull's, the columns at `v0`.
fn tag(hull: &mut Hull, mesh: &Mesh, face_sheet: &[usize], first: usize, v0: &[f64]) {
    // each node of a facet, the sheets of the facets it is a node of and how many
    let mut candidates: BTreeMap<usize, BTreeMap<usize, usize>> = BTreeMap::new();
    for (f, wall) in &mesh.facets {
        let Wall::Face(fi) = *wall else { continue };
        let Some(&s) = face_sheet.get(fi) else { continue };
        for &node in f {
            *candidates.entry(node).or_default().entry(s).or_default() += 1;
        }
    }
    let near = NEAR * mesh.near;
    for (local, sheets) in candidates {
        let (node, p) = (first + local, mesh.nodes[local]);
        // nearest first, the sheet more of its facets lie on first among equals
        let mut on: Vec<(f64, usize, usize, [f64; 3])> = sheets.into_iter().filter_map(|(s, count)| {
            let (phi, g) = hull.level_at(s, &v0, p);
            (phi.abs() <= near && crate::space::norm(g) > 0.0)
                .then(|| (phi.abs(), usize::MAX - count, s, crate::space::unit(g)))
        }).collect();
        on.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut chosen: Vec<usize> = Vec::new();
        let mut normals: Vec<[f64; 3]> = Vec::new();
        for (_, _, s, g) in on {
            normals.push(g);
            if chosen.len() < 3 && meet(&normals) {
                chosen.push(s);
            } else {
                normals.pop();
            }
        }
        // the sheets it lies on are those whose foot it reaches from where it was meshed, near
        // it, and where they meet there rather than touch (a round's torus run tangent into a
        // cylinder touches it): else the last is let go, and the foot sought again
        loop {
            hull.nodes[node] = chosen.clone();
            if chosen.is_empty() {
                break;
            }
            let reached = hull.foot_at(node, &v0, p).filter(|&f| {
                let normals: Vec<[f64; 3]> =
                    chosen.iter().map(|&c| unit_level(&hull, c, &v0, f)).collect();
                crate::space::distance(f, p) <= near && meet(&normals)
            });
            if let Some(f) = reached {
                hull.feet0[3 * node..3 * node + 3].copy_from_slice(&f);
                break;
            }
            chosen.pop();
        }
    }
}
