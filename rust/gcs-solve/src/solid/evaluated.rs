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

/// A projection into a plane's own coordinates. `unproject` takes a signed distance from the
/// plane.
#[derive(Clone, Copy, Debug)]
pub struct PageFrame {
    basis: Basis,
}
impl PageFrame {
    pub fn new(basis: Basis) -> Self {
        Self { basis }
    }
    /// The world-space view plane.
    pub fn basis(self) -> Basis {
        self.basis
    }
    pub fn project(self, p: WorldPoint) -> PagePoint {
        PagePoint(self.basis.view_coords(p.0))
    }
    pub fn unproject(self, p: PagePoint, depth: f64) -> WorldPoint {
        let p = p.0;
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

/// Where an evaluated solid's boundary came from.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Surface {
    /// The facet term's boundary evaluation.
    Csg,
    /// A material field's surface (`from_surface`), `provisional` while the refinement that made
    /// it is still going, when it may be open.
    Field { provisional: bool },
    /// An exact B-rep's mesh (`from_brep`), and the B-rep's own volume.
    Brep { volume: f64 },
}

/// **An evaluated solid where it stands** (#412): its geometry (`Shape`), shared, and the rigid
/// placement that stands it in the world — `to_world(p) = R·p + origin`.  A fresh evaluation has
/// no rotation, its geometry the world's turned nowhere; a solid that has only moved rigidly since
/// it was evaluated (`Sketch::evaluated_solid`, a hit on its shape) is the same `Shape` under the
/// placement the move makes.  Everything this hands out in its own coordinates (`boundary`,
/// `bounds`, `edges`, `mesh`, `face_edges`, `round_features`) is in the shape's; every world
/// output (`world_*`, `stl`, a glTF's node) applies the placement.
#[derive(Clone, Debug)]
pub struct EvaluatedSolid {
    origin: WorldPoint,
    /// The placement's rotation, by rows; `None` for none, so a solid that never moved reads
    /// bit for bit as it did before there was a rotation.
    rotation: Option<[[f64; 3]; 3]>,
    /// The world box, worked out once where the solid is turned (the shape's box turned is not
    /// the tight box of what it bounds).
    world_box: OnceCell<Box3>,
    shape: std::rc::Rc<Shape>,
}

/// What an evaluation built, in its own coordinates: everything of an `EvaluatedSolid` but where
/// it stands, so a solid moved rigidly shares it rather than building it again.
#[derive(Debug)]
struct Shape {
    /// Which evaluation this is: a number no other shape in this process carries, so a host keeps
    /// what it built from one (a page's geometry) exactly while it is the same shape (#412).
    serial: u64,
    name: String,
    policy: ApproximationPolicy,
    unit: f64,
    epsilon: f64,
    csg: Csg,
    /// The boundary's pieces. A field's surface derives them from its one primitive's facets when
    /// first asked (`Surface::Field`), rather than holding every triangle twice.
    boundary: OnceCell<Vec<Piece>>,
    bounds: Box3,
    paths: BTreeMap<String, String>,
    surviving: BTreeSet<String>,
    round: Vec<RoundFeature>,
    surface: Surface,
    /// An exact solid's curved facets' outward normals at their corners, by facet (none for a flat
    /// one, and empty for any other surface): what its silhouettes are traced from.
    corner_normals: Vec<Option<[[f64; 3]; 3]>>,
    /// An exact solid's B-rep, and the B-rep face each facet is of: where a traced silhouette's
    /// points are put back on their surface.
    exact: Option<(std::rc::Rc<super::Exact>, Vec<u32>)>,
    edges: OnceCell<Vec<Edge>>,
    mesh: OnceCell<mesh::Mesh>,
    ray_indices: OnceCell<Vec<RayIndex>>,
    /// Where its faces meet, read off its mesh (`mesh::face_edges`).
    face_edges: OnceCell<Vec<mesh::FaceEdge>>,
}

const IDENTITY: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// An evaluation's shape renamed for another instance of its binding (#566,
/// `EvaluatedSolid::renamed`): the shape it was renamed from, by serial, and the renamed one,
/// which keeps its own serial however often it is placed.
#[derive(Clone, Debug)]
pub struct Renamed {
    of: u64,
    shape: std::rc::Rc<Shape>,
}

/// A name made under one instance read as the same name under another (#566): `from` at its
/// start rewritten `to`, and any other name left as it is (a face's own `near`, a route).
pub(crate) struct Renaming<'n> {
    pub from: &'n str,
    pub to: &'n str,
}

impl Renaming<'_> {
    pub(crate) fn name(&self, s: &str) -> String {
        match s.strip_prefix(self.from) {
            Some(rest) => format!("{}{rest}", self.to),
            None => s.to_string(),
        }
    }
}

impl Shape {
    /// The same shape under names `re` rewrites, a serial of its own: what a host keeps by
    /// serial reads names too.  What it has worked out already is kept, renamed; what it has not
    /// is left to be worked out from what is (the ray indices, which read no name, among them).
    fn renamed(&self, re: &Renaming<'_>) -> Shape {
        let pieces = |p: &[Piece]| p.iter().map(|q| Piece { path: re.name(&q.path), ..q.clone() })
            .collect::<Vec<_>>();
        let csg = Csg {
            prims: self.csg.prims.iter().map(|p| Prim {
                faces: p.faces.iter().map(|f| re.name(f)).collect(),
                of: re.name(&p.of),
                ..p.clone()
            }).collect(),
            term: self.csg.term.clone(),
        };
        fn kept<T>(v: Option<T>) -> OnceCell<T> {
            v.map_or_else(OnceCell::new, OnceCell::from)
        }
        Shape {
            serial: next_serial(),
            name: re.name(&self.name),
            policy: self.policy,
            unit: self.unit,
            epsilon: self.epsilon,
            csg,
            boundary: kept(self.boundary.get().map(|b| pieces(b))),
            bounds: self.bounds,
            paths: self.paths.iter().map(|(k, v)| (re.name(k), re.name(v))).collect(),
            surviving: self.surviving.iter().map(|f| re.name(f)).collect(),
            round: self.round.iter().map(|r| RoundFeature { of: re.name(&r.of), ..r.clone() })
                .collect(),
            surface: self.surface,
            corner_normals: self.corner_normals.clone(),
            exact: self.exact.as_ref().map(|(x, of)| (std::rc::Rc::new(x.renamed(re)), of.clone())),
            edges: kept(self.edges.get().map(|e| e.iter()
                .map(|e| Edge { path: re.name(&e.path), ..e.clone() }).collect())),
            mesh: kept(self.mesh.get().map(|m| mesh::Mesh {
                positions: m.positions.clone(),
                normals: m.normals.clone(),
                groups: m.groups.iter().map(|g| mesh::Group { path: re.name(&g.path), ..*g })
                    .collect(),
            })),
            ray_indices: OnceCell::new(),
            face_edges: kept(self.face_edges.get().map(|f| f.iter().map(|e| mesh::FaceEdge {
                faces: (re.name(&e.faces.0), re.name(&e.faces.1)),
                segs: e.segs.clone(),
            }).collect())),
        }
    }
}

/// A serial no evaluation in this process has had (`Shape::serial`).
fn next_serial() -> u64 {
    static SERIAL: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    SERIAL.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

impl EvaluatedSolid {
    pub(crate) fn evaluate(
        sk: &Sketch,
        si: usize,
        policy: ApproximationPolicy,
    ) -> Result<Self, String> {
        let unit = match policy {
            ApproximationPolicy::Report => REPORT_UNIT,
            ApproximationPolicy::Mesh => mesh_unit(sk, si),
            ApproximationPolicy::View { unit } if unit.is_finite() && unit > 0.0 => unit,
            _ => return Err("solid approximation requires a finite positive pixel length".into()),
        };
        let operands = validate_at(sk, si, unit)?;
        if sweeps_among(sk, &operands) {
            // a swept solid's exact B-rep where a host supplied one, else its field's surface: one
            // built with no faces is a body with no material, its volume 0 (#618)
            if let Some(exact) = sk.supplied_exact_solid(si) {
                return Self::from_brep(sk, si, policy, unit, operands, &exact);
            }
            return Self::of_field(sk, si, policy, unit);
        }
        // the exact B-rep where this kernel builds and meshes one, the facet term where it does
        // not, an empty solid among them, whose B-rep has no faces (`SOLVENT_SOLIDS=facets` asks
        // for it throughout)
        if !facets_only() {
            let exact = sk.exact_preimage(si).ok().filter(|x| !x.brep.faces.is_empty())
                .map(|x| Self::from_brep(sk, si, policy, unit, operands.clone(), &x));
            if let Some(Ok(solid)) = exact {
                return Ok(solid);
            }
        }
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
                Term::Union(a, b) | Term::Diff(a, b) | Term::Inter(a, b) => {
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
            .map(|p| p.path.as_str())
            .collect();
        let round = round_features(sk, operands, &curved, origin)?;
        Ok(Self::made(origin, Shape {
            serial: next_serial(),
            name: sk.solid_name(si),
            policy,
            unit,
            epsilon,
            csg,
            boundary: OnceCell::from(boundary),
            bounds,
            paths: operand_paths(sk, si),
            surviving,
            round,
            surface: Surface::Csg,
            corner_normals: Vec::new(),
            exact: None,
            edges: OnceCell::new(),
            mesh: OnceCell::new(),
            ray_indices: OnceCell::new(),
            face_edges: OnceCell::new(),
        }))
    }
    /// A swept solid's boundary is its material field's surface, whatever approximation is asked
    /// for: one a host meshed elsewhere and supplied (`Sketch::supply_field`), none while the
    /// sketch leaves meshing to the host (`FieldMeshing::Deferred`), or meshed here and now to the
    /// end (`FieldMeshing::Now`) — seconds of refinement inside the call that asked.
    fn of_field(sk: &Sketch, si: usize, policy: ApproximationPolicy, unit: f64) -> Result<Self, String> {
        if let Some(surface) = sk.supplied_field(si) {
            return Self::from_surface(sk, si, policy, unit, &surface);
        }
        match sk.field_meshing.get() {
            FieldMeshing::Deferred => Err(format!("`{}`: its surface is still being meshed", sk.solid_name(si))),
            FieldMeshing::Now => Self::from_surface(sk, si, policy, unit, &FieldMesher::new(sk, si)?.finish()?),
        }
    }
    /// A static solid from its exact B-rep (`solid::Exact`): the B-rep meshed within the
    /// approximation's sagitta stands in as one polyhedral primitive whose facets carry their
    /// faces' paths, so classification, edges and views read it as they read any other — and a
    /// face's facets meet another face's at a crease, the B-rep's edge. Its volume is the B-rep's.
    /// A B-rep with no faces is a solid with no material, its volume 0 (#618).
    fn from_brep(
        sk: &Sketch,
        si: usize,
        policy: ApproximationPolicy,
        unit: f64,
        operands: BTreeSet<usize>,
        exact: &std::rc::Rc<super::Exact>,
    ) -> Result<Self, String> {
        let name = sk.solid_name(si);
        let origin = WorldPoint(exact.origin);
        let mm = exact.mm;
        let b = &exact.brep;
        let bar = (crate::curve::flatness(unit) * mm).min(BREP_RELATIVE_SAG * b.size());
        let m = crate::brep::mesh::mesh(b, bar, BREP_ANGULAR)
            .map_err(|e| format!("`{name}`: {e}"))?;
        // one face index a path, so the pieces of one face split by a boolean or a seam are one;
        // the leading faces' first, so an edge is named by the face the facet term named it by
        let mut faces: Vec<String> = Vec::new();
        for f in b.faces.iter().filter(|f| exact.leading.contains(&f.name)) {
            if !faces.contains(&f.name) { faces.push(f.name.clone()); }
        }
        let index: Vec<usize> = b.faces.iter()
            .map(|f| match faces.iter().position(|n| *n == f.name) {
                Some(k) => k,
                None => { faces.push(f.name.clone()); faces.len() - 1 }
            })
            .collect();
        // the B-rep stands about the solid's origin already
        let local = |p: [f64; 3]| -> [f64; 3] { p.map(|x| x / mm) };
        let mut facets = Vec::with_capacity(m.tris.len());
        let mut corner_normals = Vec::with_capacity(m.tris.len());
        let mut brep_faces = Vec::with_capacity(m.tris.len());
        let mut order: Vec<usize> = (0..m.tris.len()).collect();
        order.sort_by_key(|&t| index[m.of[t] as usize]);
        let mut bbox = Box3::empty();
        for (t, &fi) in order.iter().map(|&t| (&m.tris[t], &m.of[t])) {
            let [a, b3, c] = t.map(|i| local(m.pts[i as usize]));
            let (u, v) = (crate::space::sub(b3, a), crate::space::sub(c, a));
            // the scale is the solid's own, whatever it is: a facet is dropped only with no area
            let n = crate::space::cross(u, v);
            let l = crate::space::norm(n);
            if !(l > 0.0 && l.is_finite()) { continue }
            let n = n.map(|x| x / l);
            for p in [a, b3, c] { bbox.add(p); }
            let face = &b.faces[fi as usize];
            let smooth = !matches!(face.surface, crate::brep::geom::Surface::Plane(_));
            // the surface's own outward normal at each corner (the facet's own at a pole)
            corner_normals.push(smooth.then(|| t.map(|i| {
                let p = m.pts[i as usize];
                match face.surface.normal(face.surface.inverse(p)) {
                    Some(k) if face.reversed => k.map(|x| -x),
                    Some(k) => k,
                    None => n,
                }
            })));
            brep_faces.push(fi);
            facets.push(Facet { pts: vec![a, b3, c], n, face: index[fi as usize], smooth });
        }
        // a B-rep with no faces is a body with no material (#618), and one whose faces mesh to
        // nothing a failure
        if facets.is_empty() && !b.faces.is_empty() {
            return Err(format!("`{name}`: the exact solid meshed to nothing"));
        }
        let prim = Prim { facets, bbox, faces, of: String::new(), exact: true };
        let csg = Csg { prims: vec![prim], term: Term::Prim(0) };
        let epsilon = csg.epsilon();
        if !epsilon.is_finite() || epsilon <= 0.0 {
            return Err("solid scale is not representable".into());
        }
        let prim = &csg.prims[0];
        let boundary: Vec<Piece> = prim.facets.iter().map(|f| Piece {
            pts: f.pts.clone(), n: f.n, path: prim.faces[f.face].clone(), prim: 0, smooth: f.smooth,
        }).collect();
        let surviving: BTreeSet<String> = prim.faces.iter().cloned().collect();
        let curved: BTreeSet<&str> =
            boundary.iter().filter(|p| p.smooth).map(|p| p.path.as_str()).collect();
        let round = round_features(sk, operands, &curved, origin)?;
        let volume =
            if b.faces.is_empty() { 0.0 } else { crate::brep::props::volume(b) / (mm * mm * mm) };
        Ok(Self::made(origin, Shape {
            serial: next_serial(),
            name,
            policy,
            unit,
            epsilon,
            bounds: bbox,
            surviving,
            boundary: OnceCell::from(boundary),
            csg,
            paths: operand_paths(sk, si),
            round,
            surface: Surface::Brep { volume },
            corner_normals,
            exact: Some((exact.clone(), brep_faces)),
            edges: OnceCell::new(),
            mesh: OnceCell::new(),
            ray_indices: OnceCell::new(),
            face_edges: OnceCell::new(),
        }))
    }
    /// A swept solid from its field's surface (`field_mesh.rs`): the mesh stands in as one
    /// polyhedral primitive, so classification, edges and views read it as they read any other.
    /// A provisional surface may be open: what it classifies is a preview's, and it exports
    /// nothing.
    fn from_surface(sk: &Sketch, si: usize, policy: ApproximationPolicy, unit: f64,
        surface: &FieldSurface) -> Result<Self, String> {
        if let Some(exact) = &surface.exact {
            return Self::from_exact_surface(sk, si, policy, unit, surface, exact);
        }
        let name = sk.solid_name(si);
        let mut world = Box3::empty();
        for &v in &surface.vertices { world.add(v); }
        if world.is_empty() { return Err(format!("`{name}`: the material field has no boundary yet")); }
        let centre: [f64; 3] = std::array::from_fn(|k| 0.5 * (world.lo[k] + world.hi[k]));
        let origin = WorldPoint(centre);
        let local = |v: [f64; 3]| -> [f64; 3] { std::array::from_fn(|k| v[k] - centre[k]) };
        let mut facets = Vec::with_capacity(surface.triangles.len());
        let mut bbox = Box3::empty();
        for t in &surface.triangles {
            let Some(corners) = t.iter().map(|&i| surface.vertices.get(i as usize).copied().map(local))
                .collect::<Option<Vec<_>>>() else { return Err(format!("`{name}`: a surface triangle names no vertex")) };
            let [a, b, c] = [corners[0], corners[1], corners[2]];
            let (u, v) = (std::array::from_fn::<f64, 3, _>(|k| b[k] - a[k]), std::array::from_fn::<f64, 3, _>(|k| c[k] - a[k]));
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if !(l > 0.0) { continue; }
            for p in [a, b, c] { bbox.add(p); }
            facets.push(Facet { pts: vec![a, b, c], n: n.map(|x| x / l), face: 0, smooth: true });
        }
        if facets.is_empty() { return Err(format!("`{name}`: the material field has no boundary yet")); }
        let of = sk.solids[si].name.clone();
        let path = format!("{of}.surface");
        let prim = Prim { facets, bbox, faces: vec!["surface".into()], of, exact: false };
        let csg = Csg { prims: vec![prim], term: Term::Prim(0) };
        let epsilon = csg.epsilon();
        if !epsilon.is_finite() || epsilon <= 0.0 {
            return Err("solid scale is not representable".into());
        }
        Ok(Self::made(origin, Shape {
            serial: next_serial(),
            name,
            policy,
            unit,
            epsilon,
            // every piece is a facet, so the facets' box is the pieces' (`mesh::bounds`)
            bounds: bbox,
            surviving: [path].into_iter().collect(),
            csg,
            boundary: OnceCell::new(),
            paths: operand_paths(sk, si),
            round: Vec::new(),
            surface: Surface::Field { provisional: surface.provisional },
            corner_normals: Vec::new(),
            exact: None,
            edges: OnceCell::new(),
            mesh: OnceCell::new(),
            ray_indices: OnceCell::new(),
            face_edges: OnceCell::new(),
        }))
    }
    /// A swept solid from its exact B-rep's mesh, built elsewhere and supplied
    /// (`brep::export::Builder`): `from_brep`'s solid without the B-rep — one polyhedral primitive
    /// whose facets keep their faces, so the faces meet at creases and a curved one's seams are a
    /// tessellation's; its silhouettes traced across corner normals averaged over each face's own
    /// facets; its volume the B-rep's. Every face is the path `<solid>.surface`: the construction
    /// names none of them.
    fn from_exact_surface(sk: &Sketch, si: usize, policy: ApproximationPolicy, unit: f64,
        surface: &FieldSurface, exact: &super::ExactFaces) -> Result<Self, String> {
        let name = sk.solid_name(si);
        let mut world = Box3::empty();
        for &v in &surface.vertices { world.add(v); }
        if world.is_empty() { return Err(format!("`{name}`: its exact surface is empty")); }
        let centre: [f64; 3] = std::array::from_fn(|k| 0.5 * (world.lo[k] + world.hi[k]));
        let origin = WorldPoint(centre);
        let local = |v: [f64; 3]| -> [f64; 3] { std::array::from_fn(|k| v[k] - centre[k]) };
        // each vertex's normal on each face it is of: its facets' area-weighted sum
        let mut normals: BTreeMap<(u32, u32), [f64; 3]> = BTreeMap::new();
        let mut kept = Vec::with_capacity(surface.triangles.len());
        for (t, &face) in surface.triangles.iter().zip(&exact.of) {
            let [a, b, c] = t.map(|i| local(surface.vertices[i as usize]));
            let n = crate::space::cross(crate::space::sub(b, a), crate::space::sub(c, a));
            let l = crate::space::norm(n);
            if !(l > 0.0 && l.is_finite()) { continue }
            for &i in t {
                let m = normals.entry((i, face)).or_insert([0.0; 3]);
                for k in 0..3 { m[k] += n[k]; }
            }
            kept.push((*t, face, [a, b, c], n.map(|x| x / l)));
        }
        if kept.is_empty() { return Err(format!("`{name}`: its exact surface has no area")); }
        let mut facets = Vec::with_capacity(kept.len());
        let mut corner_normals = Vec::with_capacity(kept.len());
        let mut bbox = Box3::empty();
        for (t, face, pts, n) in kept {
            let smooth = exact.smooth[face as usize];
            corner_normals.push(smooth.then(|| t.map(|i| {
                let m = normals[&(i, face)];
                let l = crate::space::norm(m);
                if l > 0.0 { m.map(|x| x / l) } else { n }
            })));
            for p in pts { bbox.add(p); }
            facets.push(Facet { pts: pts.to_vec(), n, face: face as usize, smooth });
        }
        let path = format!("{}.surface", sk.solids[si].name);
        let faces = vec![path.clone(); exact.smooth.len()];
        let prim = Prim { facets, bbox, faces, of: String::new(), exact: true };
        let csg = Csg { prims: vec![prim], term: Term::Prim(0) };
        let epsilon = csg.epsilon();
        if !epsilon.is_finite() || epsilon <= 0.0 {
            return Err("solid scale is not representable".into());
        }
        let prim = &csg.prims[0];
        let boundary: Vec<Piece> = prim.facets.iter().map(|f| Piece {
            pts: f.pts.clone(), n: f.n, path: path.clone(), prim: 0, smooth: f.smooth,
        }).collect();
        Ok(Self::made(origin, Shape {
            serial: next_serial(),
            name,
            policy,
            unit,
            epsilon,
            bounds: bbox,
            surviving: [path].into_iter().collect(),
            boundary: OnceCell::from(boundary),
            csg,
            paths: operand_paths(sk, si),
            round: Vec::new(),
            surface: Surface::Brep { volume: exact.volume },
            corner_normals,
            exact: None,
            edges: OnceCell::new(),
            mesh: OnceCell::new(),
            ray_indices: OnceCell::new(),
            face_edges: OnceCell::new(),
        }))
    }
    /// A preview of a surface still being refined, which may be open.
    pub fn provisional(&self) -> bool {
        self.shape.surface == Surface::Field { provisional: true }
    }
    pub fn policy(&self) -> ApproximationPolicy {
        self.shape.policy
    }
    pub fn unit(&self) -> f64 {
        self.shape.unit
    }
    pub fn epsilon(&self) -> f64 {
        self.shape.epsilon
    }
    pub fn sagitta(&self) -> f64 {
        crate::curve::flatness(self.shape.unit)
    }
    /// Where the solid's own origin stands in the world: the placement's translation.
    pub fn origin(&self) -> WorldPoint {
        self.origin
    }
    /// The placement's rotation, by rows (the identity for a solid that never moved).
    pub fn rotation(&self) -> [[f64; 3]; 3] {
        self.rotation.unwrap_or(IDENTITY)
    }
    /// Whether the placement reflects (#578): a mirrored instance's solid, its rotation's
    /// determinant −1.  Every world output written as triangles or polygons then turns their
    /// winding, so a face's corners still run counter-clockwise about its outward normal.
    pub fn improper(&self) -> bool {
        self.rotation.is_some_and(|r| det3(&r) < 0.0)
    }
    /// The placement as a 4×4 matrix, column-major (a renderer's): local to world.
    pub fn matrix(&self) -> [f64; 16] {
        let (r, o) = (self.rotation(), self.origin.0);
        let mut m = [0.0; 16];
        for c in 0..3 {
            for k in 0..3 { m[4 * c + k] = r[k][c]; }
        }
        m[12..15].copy_from_slice(&o);
        m[15] = 1.0;
        m
    }
    /// Which evaluation this is (`Shape::serial`): the same number exactly while the solid is the
    /// same shape, however it has moved since — what a host keeps its own geometry against.
    pub fn shape_serial(&self) -> u64 {
        self.shape.serial
    }
    /// Whether two solids are one evaluation, wherever each stands: one moved rigidly since the
    /// other was built shares its shape rather than building it again (#412).
    pub fn same_shape(&self, other: &Self) -> bool {
        std::rc::Rc::ptr_eq(&self.shape, &other.shape)
    }
    pub fn to_world(&self, p: LocalPoint) -> WorldPoint {
        let q = self.vector_to_world(p.0);
        WorldPoint(std::array::from_fn(|k| q[k] + self.origin.0[k]))
    }
    pub fn to_local(&self, p: WorldPoint) -> LocalPoint {
        LocalPoint(self.vector_to_local(std::array::from_fn(|k| p.0[k] - self.origin.0[k])))
    }
    /// A direction in the solid's own coordinates, in the world's: `R·v`.
    pub fn vector_to_world(&self, v: [f64; 3]) -> [f64; 3] {
        match &self.rotation {
            None => v,
            Some(r) => r.map(|row| plane::dot(row, v)),
        }
    }
    /// A direction in the world, in the solid's own coordinates: `Rᵀ·v`.
    pub fn vector_to_local(&self, v: [f64; 3]) -> [f64; 3] {
        match &self.rotation {
            None => v,
            Some(r) => std::array::from_fn(|j| (0..3).map(|i| r[i][j] * v[i]).sum()),
        }
    }
    /// **The same evaluation moved rigidly** by `motion` (#412): its shape shared, its placement
    /// the motion's after its own.  What a solid that has only moved since it was evaluated is.
    pub(crate) fn moved(&self, motion: &Rigid) -> Self {
        let r = motion.r;
        let own = self.rotation();
        // a translation alone turns nothing: the placement stays exactly what it was turned
        let rotation = (r != IDENTITY).then(|| std::array::from_fn(|i| std::array::from_fn(|j| {
            (0..3).map(|k| r[i][k] * own[k][j]).sum()
        }))).or(self.rotation);
        Self {
            origin: WorldPoint(motion.point(self.origin.0)),
            rotation,
            world_box: OnceCell::new(),
            shape: self.shape.clone(),
        }
    }
    /// **This evaluation's shape under another instance's names** (#566): every name it carries
    /// that starts with `from` — the instance it was made in — rewritten to start with `to`.
    /// Nothing is evaluated again: the geometry is this one's, a shape of its own only for its
    /// names, kept to place wherever this evaluation stands (`placed`).
    pub(crate) fn renamed(&self, from: &str, to: &str) -> Renamed {
        // a solid read as its own, reflected (#578), keeps its names and shares its shape
        let shape = match from == to {
            true => self.shape.clone(),
            false => std::rc::Rc::new(self.shape.renamed(&Renaming { from, to })),
        };
        Renamed { of: self.shape.serial, shape }
    }
    /// **Another instance's solid of the same binding** (#566): this evaluation moved by
    /// `motion`, under the names `renamed` gives its shape — renamed again from `from` to `to`
    /// where it was renamed from another shape.
    pub(crate) fn placed(&self, motion: &Rigid, renamed: &mut Renamed, from: &str, to: &str)
        -> Self {
        if renamed.of != self.shape.serial {
            *renamed = self.renamed(from, to);
        }
        Self { shape: renamed.shape.clone(), ..self.moved(motion) }
    }
    /// A fresh evaluation's placement: at `origin`, turned nowhere.
    fn made(origin: WorldPoint, shape: Shape) -> Self {
        Self { origin, rotation: None, world_box: OnceCell::new(), shape: std::rc::Rc::new(shape) }
    }
    pub fn to_page(&self, p: LocalPoint, frame: PageFrame) -> PagePoint {
        // Subtract origins before adding local geometry, retaining small projected features.
        let basis = self.local_basis(frame.basis);
        PagePoint(basis.view_coords(p.0))
    }
    pub fn from_page(&self, p: PagePoint, depth: f64, frame: PageFrame) -> LocalPoint {
        let p = p.0;
        let basis = self.local_basis(frame.basis);
        let x = basis.lift(p.0, p.1);
        LocalPoint(std::array::from_fn(|k| x[k] + depth * basis.normal()[k]))
    }
    /// A plane in space, in the solid's own coordinates: where a view or a cut of it is worked.
    pub fn local_basis(&self, mut basis: Basis) -> Basis {
        basis.o = self.to_local(WorldPoint(basis.o)).0;
        basis.u = self.vector_to_local(basis.u);
        basis.v = self.vector_to_local(basis.v);
        basis
    }
    /// Boundary/bounds/edges/mesh are in the solid's own coordinates; the `world_*` adapters place
    /// them.
    pub fn boundary(&self) -> &[Piece] {
        self.shape.boundary.get_or_init(|| self.field_pieces())
    }
    /// A field's surface as boundary pieces: one per facet of its one primitive, each its own
    /// smooth piece of the path `<solid>.surface`.
    fn field_pieces(&self) -> Vec<Piece> {
        let prim = &self.shape.csg.prims[0];
        let path = format!("{}.surface", prim.of);
        prim.facets.iter().map(|f| Piece { pts: f.pts.clone(), n: f.n, path: path.clone(), prim: 0, smooth: true }).collect()
    }
    pub fn bounds(&self) -> Box3 {
        self.shape.bounds
    }
    pub fn world_bounds(&self) -> Box3 {
        if self.shape.bounds.is_empty() {
            self.shape.bounds
        } else if self.rotation.is_some() {
            // the box of what it bounds, turned: tight, as a fresh evaluation's is
            *self.world_box.get_or_init(|| {
                let mut b = Box3::empty();
                for p in self.boundary().iter().flat_map(|p| &p.pts) {
                    b.add(self.to_world(LocalPoint(*p)).0);
                }
                b
            })
        } else {
            Box3 {
                lo: self.to_world(LocalPoint(self.shape.bounds.lo)).0,
                hi: self.to_world(LocalPoint(self.shape.bounds.hi)).0,
            }
        }
    }
    pub fn contains(&self, p: LocalPoint) -> bool {
        self.shape.csg.inside_indexed(p.0, self.ray_indices())
    }
    fn ray_indices(&self) -> &[RayIndex] {
        let shape = &self.shape;
        shape.ray_indices.get_or_init(|| shape.csg.prims.iter().map(RayIndex::new).collect())
    }
    pub fn contains_world(&self, p: WorldPoint) -> bool {
        self.contains(self.to_local(p))
    }
    /// `contains_world` by every ray of the solid's own term, no index: what the index must agree
    /// with.
    pub fn contains_exhaustive(&self, p: WorldPoint) -> bool {
        self.shape.csg.inside(self.to_local(p).0)
    }
    pub fn surviving_faces(&self) -> &BTreeSet<String> {
        &self.shape.surviving
    }
    pub fn round_features(&self) -> &[RoundFeature] {
        &self.shape.round
    }
    /// A face's path from this solid, from the path its boundary names it by (the primitive's own
    /// name first, `plate.near`): the route to that primitive, then the face — `plate.near` on a
    /// body made of `plate`, `near` on the primitive itself.  `None` for a face no primitive's
    /// name begins.
    pub fn face_route(&self, path: &str) -> Option<String> {
        let (primitive, route) = self.shape.paths.iter()
            .filter(|(p, _)| path.strip_prefix(p.as_str()).is_some_and(|r| r.starts_with('.')))
            .max_by_key(|(p, _)| p.len())?;
        let face = &path[primitive.len() + 1..];
        Some(if route.is_empty() { face.to_string() } else { format!("{route}.{face}") })
    }
    pub fn provenance_paths(&self) -> &BTreeMap<String, String> {
        &self.shape.paths
    }
    pub fn volume(&self) -> f64 {
        match self.shape.surface {
            Surface::Brep { volume } => volume,
            _ => mesh::volume(self.boundary()),
        }
    }
    pub fn area(&self) -> f64 {
        mesh::area(self.boundary())
    }
    /// **The edges a view draws, looking along `-eye`** (the solid's own coordinates): every
    /// corner, and a smooth seam only where the surface turns away from the eye across it — or,
    /// on an exact solid, the silhouettes traced on its surfaces in place of its mesh's seams.
    /// `more` (a section's edges) is judged by the same rule.  The one rule a drawing's views and
    /// `solventc --png` both draw by.
    pub fn drawn_edges(&self, eye: [f64; 3], more: &[Edge]) -> Vec<Edge> {
        let traced = self.silhouettes(eye);
        let seen = |e: &&Edge| {
            if !e.smooth { return true; }
            let (a, b) = (plane::dot(e.na, eye), plane::dot(e.nb, eye));
            // a silhouette is where the sign changes; a seam whose two facets both face the eye
            // (or both face away) is the tessellation and is not drawn
            traced.is_none() && a * b <= 0.0 && (a.abs() >= 1e-12 || b.abs() >= 1e-12)
        };
        let mut drawn: Vec<Edge> = self.edges().iter().chain(more).filter(seen).cloned().collect();
        drawn.extend(traced.into_iter().flatten());
        drawn
    }
    /// A view's silhouettes on an exact solid's curved faces, looking along `-eye` (`None` for any
    /// other solid, whose smooth seams are its silhouettes): where the surface's own outward normal
    /// turns from facing the eye to facing away, interpolated across each facet from the exact
    /// normals at its corners — the curve the surface draws, within its sag, and not a zigzag of
    /// the mesh's seams, which the view then leaves out.
    pub fn silhouettes(&self, eye: [f64; 3]) -> Option<Vec<Edge>> {
        if !matches!(self.shape.surface, Surface::Brep { .. }) { return None; }
        let prim = &self.shape.csg.prims[0];
        // a point interpolated on a chord stands inside the surface by up to the sag: put back on
        // it (where the B-rep is in hand), it is on the silhouette to second order, and a straight
        // one comes out straight
        let on = |k: usize, p: [f64; 3]| -> [f64; 3] {
            let Some((exact, faces)) = &self.shape.exact else { return p };
            let mm = exact.mm;
            let surface = &exact.brep.faces[faces[k] as usize].surface;
            let q = surface.point(surface.inverse(p.map(|x| x * mm))).map(|x| x / mm);
            if q.iter().all(|x| x.is_finite()) { q } else { p }
        };
        let mut out = Vec::new();
        for (fi, (f, normals)) in prim.facets.iter().zip(&self.shape.corner_normals).enumerate() {
            let Some(normals) = normals else { continue };
            let g = normals.map(|n| plane::dot(n, eye));
            let mut at = Vec::with_capacity(2);
            for k in 0..3 {
                let (i, j) = (k, (k + 1) % 3);
                // a corner square to the eye counts as facing it, so a crossing is counted once
                if (g[i] >= 0.0) == (g[j] >= 0.0) { continue; }
                let t = g[i] / (g[i] - g[j]);
                let (p, q) = (f.pts[i], f.pts[j]);
                let lerp = |a: [f64; 3], b: [f64; 3]| -> [f64; 3] {
                    std::array::from_fn(|c| a[c] + t * (b[c] - a[c]))
                };
                at.push((on(fi, lerp(p, q)), lerp(normals[i], normals[j])));
            }
            if let [(a, na), (b, nb)] = at[..] {
                out.push(Edge { a, b, na, nb, smooth: true, path: prim.faces[f.face].clone() });
            }
        }
        Some(out)
    }
    pub fn edges(&self) -> &[Edge] {
        let shape = &self.shape;
        shape.edges.get_or_init(|| csg::edges_indexed(&shape.csg, shape.epsilon, self.ray_indices()))
    }
    pub fn mesh(&self) -> &mesh::Mesh {
        // a field's surface came indexed, its corners shared exactly: nothing for a weld to do
        self.shape.mesh.get_or_init(|| match self.shape.surface {
            Surface::Field { .. } => mesh::grouped_welded(
                self.shape.boundary.get().cloned().unwrap_or_else(|| self.field_pieces())),
            // the B-rep's mesh shares its points exactly too
            Surface::Brep { .. } => mesh::grouped_welded(self.boundary().to_vec()),
            Surface::Csg => mesh::grouped(self.boundary()),
        })
    }
    /// Where two of its faces meet, by their paths, read off its mesh once (`mesh::face_edges`):
    /// what a round or a chamfer is written along.  In the solid's own coordinates.
    pub fn face_edges(&self) -> &[mesh::FaceEdge] {
        self.shape.face_edges.get_or_init(|| mesh::face_edges(self.mesh()))
    }
    pub fn world_boundary(&self) -> Vec<Piece> {
        if self.rotation.is_none() {
            return translate_pieces(self.boundary(), self.origin.0);
        }
        let flip = self.improper();
        self.boundary().iter().map(|p| {
            let mut pts: Vec<[f64; 3]> =
                p.pts.iter().map(|&x| self.to_world(LocalPoint(x)).0).collect();
            if flip {
                pts.reverse();
            }
            Piece { pts, n: self.vector_to_world(p.n), ..p.clone() }
        }).collect()
    }
    pub fn world_edges(&self) -> Vec<Edge> {
        self.edges()
            .iter()
            .map(|e| Edge {
                a: self.to_world(LocalPoint(e.a)).0,
                b: self.to_world(LocalPoint(e.b)).0,
                na: self.vector_to_world(e.na),
                nb: self.vector_to_world(e.nb),
                ..e.clone()
            })
            .collect()
    }
    pub fn world_mesh(&self) -> mesh::Mesh {
        let mut m = self.oriented_mesh().into_owned();
        for (i, v) in m.positions.iter_mut().enumerate() {
            *v += self.origin.0[i % 3];
        }
        m
    }
    /// The mesh turned as the solid stands but not yet moved to its origin: the world's mesh less
    /// the translation, which a file adds as it writes (`placed_stl`, a glTF node).  The mesh
    /// itself for a solid that never turned.
    pub fn oriented_mesh(&self) -> std::borrow::Cow<'_, mesh::Mesh> {
        let m = self.mesh();
        if self.rotation.is_none() {
            return std::borrow::Cow::Borrowed(m);
        }
        let mut m = m.clone();
        for v in m.positions.chunks_exact_mut(3).chain(m.normals.chunks_exact_mut(3)) {
            let w = self.vector_to_world([v[0], v[1], v[2]]);
            v.copy_from_slice(&w);
        }
        // reflected, each triangle is wound the other way round, so it still faces out (#578)
        if self.improper() {
            for t in m.positions.chunks_exact_mut(9).chain(m.normals.chunks_exact_mut(9)) {
                for k in 0..3 {
                    t.swap(3 + k, 6 + k);
                }
            }
        }
        std::borrow::Cow::Owned(m)
    }
    /// The surface as it stands even while it is still being refined (`mesh::preview_stl`): what a
    /// host offers when asked for a file before the refinement has finished.
    pub fn preview_stl(&self) -> Result<Vec<u8>, String> {
        mesh::preview_stl(&self.oriented_mesh(), self.origin.0, &self.shape.name)
    }
    pub fn stl(&self) -> Result<Vec<u8>, String> {
        self.finished()?;
        mesh::placed_stl(&self.oriented_mesh(), self.origin.0, &self.shape.name)
    }
    /// Refuse a surface still being refined: it may be open, and a file is a finished part.
    pub fn finished(&self) -> Result<(), String> {
        if !self.provisional() { return Ok(()) }
        Err(format!("`{}`: its surface is still being refined", self.shape.name))
    }
    pub(crate) fn classifier(&self) -> &Csg {
        &self.shape.csg
    }
    /// Rebase another validated solid into this solid's frame for pairwise kernel queries, moved
    /// rigidly in space by `pose` first where one is given (a claim's roll, evaluated once).
    pub(crate) fn relative(&self, other: &Self, pose: Option<crate::envelope::Motion>) -> (Csg, Vec<Piece>) {
        let mut csg = other.shape.csg.clone();
        if pose.is_none() && self.rotation.is_none() && other.rotation.is_none() {
            let delta = std::array::from_fn(|k| other.origin.0[k] - self.origin.0[k]);
            translate_csg(&mut csg, delta);
            return (csg, translate_pieces(other.boundary(), delta));
        }
        // other's own coordinates to the world, moved there, and into this solid's own
        let point = |p: [f64; 3]| {
            let w = other.to_world(LocalPoint(p)).0;
            let w = pose.as_ref().map_or(w, |m| m.point(w));
            self.to_local(WorldPoint(w)).0
        };
        let vector = |n: [f64; 3]| {
            let w = other.vector_to_world(n);
            self.vector_to_local(pose.as_ref().map_or(w, |m| m.vector(w)))
        };
        // one reflected and the other not, the motion between them reflects: wound back (#578)
        let flip = self.improper() != other.improper();
        for p in &mut csg.prims {
            p.bbox = Box3::empty();
            for f in &mut p.facets {
                for x in &mut f.pts { *x = point(*x); p.bbox.add(*x); }
                if flip {
                    f.pts.reverse();
                }
                f.n = vector(f.n);
            }
        }
        let pieces = other.boundary().iter().map(|q| {
            let mut pts: Vec<[f64; 3]> = q.pts.iter().map(|&x| point(x)).collect();
            if flip {
                pts.reverse();
            }
            Piece { pts, n: vector(q.n), ..q.clone() }
        }).collect();
        (csg, pieces)
    }

}

/// A rigid motion of space, `x ↦ r·x + t`, `r` by rows: what carries a solid that has only moved
/// since it was evaluated from where it was built to where it stands (#412).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rigid {
    pub r: [[f64; 3]; 3],
    pub t: [f64; 3],
}

impl Rigid {
    /// No motion at all.
    pub fn identity() -> Rigid {
        Rigid { r: IDENTITY, t: [0.0; 3] }
    }
    /// The motion carrying plane `from` onto plane `to`: its origin onto the other's, and its
    /// axes and normal onto the other's.
    pub fn between(from: &Basis, to: &Basis) -> Rigid {
        // a plane moved without turning is moved by its translation, exactly
        if (from.u, from.v) == (to.u, to.v) {
            return Rigid { r: IDENTITY, t: std::array::from_fn(|k| to.o[k] - from.o[k]) };
        }
        let (n0, n1) = (from.normal(), to.normal());
        let r: [[f64; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| {
            to.u[i] * from.u[j] + to.v[i] * from.v[j] + n1[i] * n0[j]
        }));
        let moved = r.map(|row| plane::dot(row, from.o));
        Rigid { r, t: std::array::from_fn(|k| to.o[k] - moved[k]) }
    }
    /// **The reflection in a plane of `at`** (#578): the plane through its origin square to its
    /// axis `axis` — 0 its `u` (its side), 1 its `v` (its front), 2 its normal (its top) — what
    /// a mirrored instance standing in `at` is published by.  `x ↦ x − 2m(m·(x − o))`, an
    /// improper motion: its determinant is −1.
    pub fn reflection(at: &Basis, axis: u8) -> Rigid {
        let m = match axis {
            0 => at.u,
            1 => at.v,
            _ => at.normal(),
        };
        let m = crate::space::normalised(m).unwrap_or(m);
        let r = std::array::from_fn(|i| std::array::from_fn(|j| {
            (if i == j { 1.0 } else { 0.0 }) - 2.0 * m[i] * m[j]
        }));
        let mo = 2.0 * plane::dot(m, at.o);
        Rigid { r, t: m.map(|x| x * mo) }
    }
    /// `self` after `first`: `x ↦ self(first(x))`.
    pub fn after(&self, first: &Rigid) -> Rigid {
        let r = std::array::from_fn(|i| std::array::from_fn(|j| {
            (0..3).map(|k| self.r[i][k] * first.r[k][j]).sum()
        }));
        Rigid { r, t: self.point(first.t) }
    }
    /// Whether it reflects: its rotation's determinant is negative.
    pub fn improper(&self) -> bool {
        det3(&self.r) < 0.0
    }
    pub fn point(&self, x: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|k| plane::dot(self.r[k], x) + self.t[k])
    }
    pub fn vector(&self, v: [f64; 3]) -> [f64; 3] {
        self.r.map(|row| plane::dot(row, v))
    }
}

/// The determinant of a 3×3 matrix by rows: its rows' triple product.
fn det3(r: &[[f64; 3]; 3]) -> f64 {
    plane::dot(crate::space::cross(r[0], r[1]), r[2])
}

/// The angle an exact solid's edge's chords may turn (radians): the sagitta decides on a large
/// curve, and a small circle is cut in 64 steps a turn at least, as the facet term cuts one.
const BREP_ANGULAR: f64 = std::f64::consts::TAU / 64.0;
/// The most an exact solid's mesh may sag, as a fraction of the solid's size, whatever absolute
/// length the approximation names: a solid a micron across is meshed as finely, for its size, as
/// one a metre across — the facet term's rule too, which never cuts a turn coarser than 64 steps.
const BREP_RELATIVE_SAG: f64 = 5e-4;

/// Whether `SOLVENT_SOLIDS=facets` asks every static solid of the facet term (for comparison and
/// measurement against the exact B-rep).
fn facets_only() -> bool {
    static ONLY: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ONLY.get_or_init(|| std::env::var("SOLVENT_SOLIDS").is_ok_and(|v| v == "facets"))
}

/// The round features a static solid's operands give it: a circular prism's wall, where it
/// survives in the boundary (`curved`, the surviving smooth paths), at its circle's centre (local
/// to `origin`) and radius. A revolution of a circular profile is a torus, not a bore of that
/// diameter.
fn round_features(
    sk: &Sketch, operands: BTreeSet<usize>, curved: &BTreeSet<&str>, origin: WorldPoint,
) -> Result<Vec<RoundFeature>, String> {
    let mut round = Vec::new();
    for i in operands {
        let sol = &sk.solids[i];
        // Only a circular prism has the diameter of its source circle. A revolution of
        // a circular profile is a torus, not a bore of that diameter.
        // a drafted one's is the diameter at its parting plane only, and a cone's elsewhere
        let (SolidDef::Prism { face, draft: None, .. } | SolidDef::Through { face, .. }) = sol.def else {
            continue;
        };
        let face = &sk.faces[face as usize];
        for (edges, names) in face.boundaries() {
            if edges.len() != 1 || edges[0].kind != EntKind::Circle
                || !curved.contains(format!("{}.{}", sol.name, names[0]).as_str()) {
                continue;
            }
            let circle = &sk.circles[edges[0].i()];
            let basis = match face.plane {
                Some(i) => sk.basis(i as usize),
                None => Basis::page(),
            };
            let uv = sk.point_xy(circle.center as usize);
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
    }
    Ok(round)
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
