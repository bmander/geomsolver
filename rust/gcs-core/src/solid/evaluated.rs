//! Validated, pose-specific solid geometry. Kernel arrays are local to this value; public
//! coordinate conversions require frame-specific point types. No consumer can construct one.
use super::*;
use crate::{
    csg::{self, Edge, Piece},
    mesh,
};
use std::{cell::OnceCell, collections::BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalPoint(pub [f64; 3]);
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorldPoint(pub [f64; 3]);
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PagePoint(pub (f64, f64));

/// Absolute report accuracy, object-relative mesh accuracy, or a view's pixel length.
/// Pixel length affects tessellation only; page extent/translation never sets boolean epsilon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ApproximationPolicy {
    Report,
    Mesh,
    View { unit: f64 },
}
impl ApproximationPolicy {
    pub fn from_unit(unit: f64) -> Self {
        if unit <= 0.0 {
            Self::Mesh
        } else if unit == REPORT_UNIT {
            Self::Report
        } else {
            Self::View { unit }
        }
    }
    pub(crate) fn cache_key(self) -> (u8, u64) {
        match self {
            Self::Report => (0, 0),
            Self::Mesh => (1, 0),
            Self::View { unit } => (2, unit.to_bits()),
        }
    }
}

/// A projection plus its page placement. `unproject` takes a signed distance from the plane.
#[derive(Clone, Copy, Debug)]
pub struct PageFrame {
    basis: Basis,
    pose: (f64, f64, (f64, f64)),
}
impl PageFrame {
    pub fn new(basis: Basis, pose: (f64, f64, (f64, f64))) -> Self {
        Self { basis, pose }
    }
    pub fn project(self, p: WorldPoint) -> PagePoint {
        PagePoint(plane::on_page(
            self.pose.0,
            self.pose.1,
            self.pose.2,
            self.basis.view_coords(p.0),
        ))
    }
    pub fn unproject(self, p: PagePoint, depth: f64) -> WorldPoint {
        let p = plane::in_view(self.pose.0, self.pose.1, self.pose.2, p.0);
        let x = self.basis.lift(p.0, p.1);
        WorldPoint(std::array::from_fn(|k| {
            x[k] + depth * self.basis.normal()[k]
        }))
    }
}

/// Analytic dimension metadata is provenance, admitted only for a surviving curved wall.
#[derive(Clone, Debug)]
pub struct RoundFeature {
    pub of: String,
    pub center: LocalPoint,
    pub normal: [f64; 3],
    pub radius: f64,
}

#[derive(Clone, Debug)]
pub struct EvaluatedSolid {
    name: String,
    origin: WorldPoint,
    policy: ApproximationPolicy,
    unit: f64,
    epsilon: f64,
    csg: Csg,
    boundary: Vec<Piece>,
    bounds: Box3,
    paths: BTreeMap<String, String>,
    surviving: BTreeSet<String>,
    round: Vec<RoundFeature>,
    edges: OnceCell<Vec<Edge>>,
    mesh: OnceCell<mesh::Mesh>,
}

impl EvaluatedSolid {
    pub(crate) fn evaluate(
        sk: &Sketch,
        si: usize,
        policy: ApproximationPolicy,
    ) -> Result<Self, String> {
        // Check the graph before resolving: an invalid operand must never become empty material.
        let mut pending = vec![(si, false)];
        let (mut active, mut done) = (BTreeSet::new(), BTreeSet::new());
        while let Some((i, ready)) = pending.pop() {
            if done.contains(&i) {
                continue;
            }
            let sol = sk
                .solids
                .get(i)
                .ok_or_else(|| format!("no solid at index {i}"))?;
            if ready {
                active.remove(&i);
                done.insert(i);
            } else {
                if !active.insert(i) {
                    return Err(format!("`{}`: cyclic solid operands", sol.name));
                }
                pending.push((i, true));
                pending.extend(
                    sol.operands()
                        .into_iter()
                        .rev()
                        .map(|o| (o as usize, false)),
                );
            }
        }
        let unit = match policy {
            ApproximationPolicy::Report => REPORT_UNIT,
            ApproximationPolicy::Mesh => mesh_unit(sk, si),
            ApproximationPolicy::View { unit } if unit.is_finite() && unit > 0.0 => unit,
            _ => return Err("solid approximation requires a finite positive pixel length".into()),
        };
        validate_at(sk, si, unit)?;
        let origin = WorldPoint(frame_origin(sk, si, unit));
        let csg = resolve_at(sk, si, unit, origin.0);
        let mut terms = vec![&csg.term];
        while let Some(term) = terms.pop() {
            match term {
                Term::Empty => {
                    return Err(format!(
                        "`{}`: an operand could not be evaluated at this approximation",
                        sk.solid_name(si)
                    ))
                }
                Term::Prim(_) => {}
                Term::Union(a, b) | Term::Diff(a, b) => {
                    terms.push(a);
                    terms.push(b);
                }
            }
        }
        if csg.prims.is_empty()
            || csg.prims.iter().any(|p| {
                p.facets.is_empty()
                    || p.facets.iter().any(|f| {
                        f.pts.len() < 3
                            || f.pts.iter().flatten().chain(&f.n).any(|x| !x.is_finite())
                    })
            })
        {
            return Err(format!(
                "`{}`: cannot evaluate finite solid geometry at this approximation",
                sk.solid_name(si)
            ));
        }
        // Preserve f64 input precision, then perform all CSG arithmetic near the solid.

        let epsilon = csg.epsilon();
        if !epsilon.is_finite() || epsilon <= 0.0 {
            return Err("solid scale is not representable".into());
        }
        let boundary = csg::boundary(&csg, epsilon);
        if boundary
            .iter()
            .any(|p| !p.area().is_finite() || p.pts.iter().flatten().any(|x| !x.is_finite()))
        {
            return Err("solid boundary exceeds numerical precision".into());
        }
        let bounds = mesh::bounds(&boundary);
        let surviving = boundary
            .iter()
            .filter(|p| p.area() > 0.0)
            .map(|p| p.path.clone())
            .collect();
        let curved: BTreeSet<_> = boundary
            .iter()
            .filter(|p| p.smooth && p.area() > 0.0)
            .map(|p| csg.prims[p.prim].of.as_str())
            .collect();
        let mut round = Vec::new();
        for i in done {
            let sol = &sk.solids[i];
            // Only a circular prism has the diameter of its source circle. A revolution of
            // a circular profile is a torus, not a bore of that diameter.
            let SolidDef::Prism { face, .. } = sol.def else {
                continue;
            };
            if !curved.contains(sol.name.as_str()) {
                continue;
            }
            let face = &sk.faces[face as usize];
            if face.edges.len() != 1 || face.edges[0].kind != EntKind::Circle {
                continue;
            }
            let circle = &sk.circles[face.edges[0].i()];
            let (basis, pose) = match face.plane {
                Some(p) => {
                    let p = &sk.planes[p as usize];
                    (
                        p.basis,
                        (
                            sk.params[p.frame.c as usize].value,
                            sk.params[p.frame.s as usize].value,
                            sk.point_xy(p.frame.origin as usize),
                        ),
                    )
                }
                None => (Basis::page(), (1.0, 0.0, (0.0, 0.0))),
            };
            let uv = plane::in_view(pose.0, pose.1, pose.2, sk.point_xy(circle.center as usize));
            let local_basis = Basis {
                o: std::array::from_fn(|k| basis.o[k] - origin.0[k]),
                ..basis
            };
            let center = LocalPoint(local_basis.lift(uv.0, uv.1));
            round.push(RoundFeature {
                of: sol.name.clone(),
                center,
                normal: basis.normal(),
                radius: sk.params[circle.radius as usize].value.abs(),
            });
        }
        Ok(Self {
            name: sk.solid_name(si),
            origin,
            policy,
            unit,
            epsilon,
            csg,
            boundary,
            bounds,
            paths: operand_paths(sk, si),
            surviving,
            round,
            edges: OnceCell::new(),
            mesh: OnceCell::new(),
        })
    }
    pub fn policy(&self) -> ApproximationPolicy {
        self.policy
    }
    pub fn unit(&self) -> f64 {
        self.unit
    }
    pub fn epsilon(&self) -> f64 {
        self.epsilon
    }
    pub fn sagitta(&self) -> f64 {
        crate::curve::flatness(self.unit)
    }
    pub fn origin(&self) -> WorldPoint {
        self.origin
    }
    pub fn to_world(&self, p: LocalPoint) -> WorldPoint {
        WorldPoint(std::array::from_fn(|k| p.0[k] + self.origin.0[k]))
    }
    pub fn to_local(&self, p: WorldPoint) -> LocalPoint {
        LocalPoint(std::array::from_fn(|k| p.0[k] - self.origin.0[k]))
    }
    pub fn to_page(&self, p: LocalPoint, frame: PageFrame) -> PagePoint {
        // Subtract origins before adding local geometry, retaining small projected features.
        let basis = self.local_basis(frame.basis);
        PagePoint(plane::on_page(
            frame.pose.0,
            frame.pose.1,
            frame.pose.2,
            basis.view_coords(p.0),
        ))
    }
    pub fn from_page(&self, p: PagePoint, depth: f64, frame: PageFrame) -> LocalPoint {
        let p = plane::in_view(frame.pose.0, frame.pose.1, frame.pose.2, p.0);
        let basis = self.local_basis(frame.basis);
        let x = basis.lift(p.0, p.1);
        LocalPoint(std::array::from_fn(|k| x[k] + depth * basis.normal()[k]))
    }
    pub(crate) fn local_basis(&self, mut basis: Basis) -> Basis {
        basis.o = self.to_local(WorldPoint(basis.o)).0;
        basis
    }
    /// Boundary/bounds/edges/mesh are solid-local. Explicit world adapters are for legacy ABI output.
    pub fn boundary(&self) -> &[Piece] {
        &self.boundary
    }
    pub fn bounds(&self) -> Box3 {
        self.bounds
    }
    pub fn world_bounds(&self) -> Box3 {
        if self.bounds.is_empty() {
            self.bounds
        } else {
            Box3 {
                lo: self.to_world(LocalPoint(self.bounds.lo)).0,
                hi: self.to_world(LocalPoint(self.bounds.hi)).0,
            }
        }
    }
    pub fn contains(&self, p: LocalPoint) -> bool {
        self.csg.inside(p.0)
    }
    pub fn contains_world(&self, p: WorldPoint) -> bool {
        self.contains(self.to_local(p))
    }
    pub fn surviving_faces(&self) -> &BTreeSet<String> {
        &self.surviving
    }
    pub fn round_features(&self) -> &[RoundFeature] {
        &self.round
    }
    pub fn provenance_paths(&self) -> &BTreeMap<String, String> {
        &self.paths
    }
    pub fn volume(&self) -> f64 {
        mesh::volume(&self.boundary)
    }
    pub fn area(&self) -> f64 {
        mesh::area(&self.boundary)
    }
    pub fn edges(&self) -> &[Edge] {
        self.edges
            .get_or_init(|| csg::edges(&self.csg, self.epsilon))
    }
    pub fn mesh(&self) -> &mesh::Mesh {
        self.mesh.get_or_init(|| mesh::grouped(&self.boundary))
    }
    pub fn world_boundary(&self) -> Vec<Piece> {
        translate_pieces(&self.boundary, self.origin.0)
    }
    pub fn world_edges(&self) -> Vec<Edge> {
        self.edges()
            .iter()
            .map(|e| Edge {
                a: self.to_world(LocalPoint(e.a)).0,
                b: self.to_world(LocalPoint(e.b)).0,
                ..e.clone()
            })
            .collect()
    }
    pub fn world_mesh(&self) -> mesh::Mesh {
        let mut m = self.mesh().clone();
        for (i, v) in m.positions.iter_mut().enumerate() {
            *v += self.origin.0[i % 3];
        }
        m
    }
    pub fn stl(&self) -> Result<Vec<u8>, String> {
        mesh::checked_stl(&self.world_boundary(), &self.name)
    }
    pub(crate) fn classifier(&self) -> &Csg {
        &self.csg
    }
    /// Rebase another validated solid into this solid's frame for pairwise kernel queries.
    pub(crate) fn relative(&self, other: &Self) -> (Csg, Vec<Piece>) {
        let delta = std::array::from_fn(|k| other.origin.0[k] - self.origin.0[k]);
        let mut csg = other.csg.clone();
        translate_csg(&mut csg, delta);
        (csg, translate_pieces(&other.boundary, delta))
    }
    /// Orthographic occlusion uses retained boundary crossings and the same material classifier.
    /// A section's limit excludes all material on the discarded side.
    pub fn occludes(&self, m: LocalPoint, eye: [f64; 3], limit: Option<f64>) -> bool {
        let eps = self.epsilon;
        if self.bounds.is_empty() {
            return false;
        }
        let m = m.0;
        let far = (0..3)
            .map(|k| {
                eye[k]
                    * (if eye[k] >= 0.0 {
                        self.bounds.hi[k]
                    } else {
                        self.bounds.lo[k]
                    } - m[k])
            })
            .sum::<f64>();
        let end = limit.unwrap_or(far + eps).min(far + eps);
        if end <= eps {
            return false;
        }
        let step = |t: f64| std::array::from_fn(|k| m[k] + t * eye[k]);
        let mut cuts = vec![eps, end];
        for f in &self.boundary {
            let denominator = plane::dot(f.n, eye);
            if denominator.abs() <= 1e-12 {
                continue;
            }
            let t = plane::dot(f.n, std::array::from_fn(|k| f.pts[0][k] - m[k])) / denominator;
            if t <= eps || t >= end {
                continue;
            }
            let x = step(t);
            if f.pts.iter().enumerate().all(|(i, a)| {
                let b = f.pts[(i + 1) % f.pts.len()];
                let edge = std::array::from_fn(|k| b[k] - a[k]);
                plane::dot(
                    f.n,
                    plane::cross(edge, std::array::from_fn(|k| x[k] - a[k])),
                ) >= -eps * plane::norm(edge)
            }) {
                cuts.push(t);
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).any(|w| {
            w[1] - w[0] > eps * 1e-3 && self.contains(LocalPoint(step((w[0] + w[1]) * 0.5)))
        })
    }
}
fn translate_csg(csg: &mut Csg, delta: [f64; 3]) {
    for p in &mut csg.prims {
        p.bbox = Box3::empty();
        for f in &mut p.facets {
            for x in &mut f.pts {
                for k in 0..3 {
                    x[k] += delta[k];
                }
                p.bbox.add(*x);
            }
        }
    }
}
fn translate_pieces(pieces: &[Piece], delta: [f64; 3]) -> Vec<Piece> {
    pieces
        .iter()
        .map(|p| {
            let mut p = p.clone();
            for x in &mut p.pts {
                for k in 0..3 {
                    x[k] += delta[k];
                }
            }
            p
        })
        .collect()
}
