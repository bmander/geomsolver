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
use crate::interior::{Axis, Frame, Hull, Profile, Sheet};
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
    let (_, g) = hull.level_at(s, v, p);
    crate::space::scale(g, 1.0 / crate::space::norm(g))
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
    let body = BodyFaces::whole(sk, solid)?;
    let mut b = Builder::default();
    let paths = body.paths();
    let mut face_sheet: Vec<Option<usize>> = vec![None; paths.len()];
    for (fi, path) in paths.iter().enumerate() {
        let p = crate::solid::faces::face_primitive(sk, solid, &path.route)
            .map_err(|why| format!("its face `{}` is not moved: {why}", path.text()))?;
        face_sheet[fi] = Some(b.sheet(sk, p, &path.face)
            .map_err(|why| format!("its face `{}` is not moved: {why}", path.text()))?);
    }
    let n = mesh.nodes.len();
    let x0: Vec<f64> = mesh.nodes.iter().flatten().copied().collect();
    let (lo, hi) = mesh.nodes.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(l, h), p| {
        (std::array::from_fn(|k| l[k].min(p[k])), std::array::from_fn(|k| h[k].max(p[k])))
    });
    let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    let mut hull = Hull { columns: b.columns.len(), frames: b.frames, sheets: b.sheets,
                          nodes: vec![Vec::new(); n], x0: x0.clone(), feet0: vec![0.0; 3 * n],
                          extent };
    // each node of a facet, the sheets of the facets it is a node of and how many
    let mut candidates: BTreeMap<usize, BTreeMap<usize, usize>> = BTreeMap::new();
    for (f, wall) in &mesh.facets {
        let Wall::Face(fi) = *wall else { continue };
        let Some(s) = face_sheet.get(fi).copied().flatten() else { continue };
        for &node in f {
            *candidates.entry(node).or_default().entry(s).or_default() += 1;
        }
    }
    let v0: Vec<f64> = b.columns.iter().map(|&c| sk.params[c as usize].value).collect();
    let near = NEAR * mesh.near;
    for (node, sheets) in candidates {
        let p = [x0[3 * node], x0[3 * node + 1], x0[3 * node + 2]];
        // nearest first, the sheet more of its facets lie on first among equals
        let mut on: Vec<(f64, usize, usize, [f64; 3])> = sheets.into_iter().filter_map(|(s, count)| {
            let (phi, g) = hull.level_at(s, &v0, p);
            let len = crate::space::norm(g);
            (phi.abs() <= near && len > 0.0)
                .then(|| (phi.abs(), usize::MAX - count, s, crate::space::scale(g, 1.0 / len)))
        }).collect();
        on.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut chosen: Vec<usize> = Vec::new();
        for (_, _, s, g) in on {
            let mut normals: Vec<[f64; 3]> = chosen.iter().map(|&c| unit_level(&hull, c, &v0, p))
                .collect();
            normals.push(g);
            if chosen.len() < 3 && meet(&normals) {
                chosen.push(s);
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
    Ok((hull, b.columns))
}
