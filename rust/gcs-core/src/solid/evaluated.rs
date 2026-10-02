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
    /// The world-space view plane, before its placement on the sheet.
    pub fn basis(self) -> Basis {
        self.basis
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

#[derive(Clone, Debug)]
pub struct EvaluatedSolid {
    name: String,
    origin: WorldPoint,
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
            return Self::of_field(sk, si, policy, unit);
        }
        // the exact B-rep where this kernel builds and meshes one, the facet term where it does
        // not, an empty solid among them (`SOLVENT_SOLIDS=facets` asks for it throughout)
        if !facets_only() {
            let exact = sk.exact_solid(si)
                .and_then(|x| Self::from_brep(sk, si, policy, unit, operands.clone(), &x));
            if let Ok(solid) = exact {
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
        Ok(Self {
            name: sk.solid_name(si),
            origin,
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
        })
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
        if facets.is_empty() { return Err(format!("`{name}`: the exact solid meshed to nothing")); }
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
        let volume = crate::brep::props::volume(b) / (mm * mm * mm);
        Ok(Self {
            name,
            origin,
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
        })
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
        Ok(Self {
            name,
            origin,
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
        })
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
        Ok(Self {
            name,
            origin,
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
        })
    }
    /// A preview of a surface still being refined, which may be open.
    pub fn provisional(&self) -> bool {
        self.surface == Surface::Field { provisional: true }
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
        self.boundary.get_or_init(|| self.field_pieces())
    }
    /// A field's surface as boundary pieces: one per facet of its one primitive, each its own
    /// smooth piece of the path `<solid>.surface`.
    fn field_pieces(&self) -> Vec<Piece> {
        let prim = &self.csg.prims[0];
        let path = format!("{}.surface", prim.of);
        prim.facets.iter().map(|f| Piece { pts: f.pts.clone(), n: f.n, path: path.clone(), prim: 0, smooth: true }).collect()
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
        self.csg.inside_indexed(p.0, self.ray_indices())
    }
    fn ray_indices(&self) -> &[RayIndex] {
        self.ray_indices.get_or_init(|| self.csg.prims.iter().map(RayIndex::new).collect())
    }
    pub fn contains_world(&self, p: WorldPoint) -> bool {
        self.contains(self.to_local(p))
    }
    /// `contains_world` by every ray of the solid's own term, no index: what the index must agree
    /// with.
    pub fn contains_exhaustive(&self, p: WorldPoint) -> bool {
        self.csg.inside(self.to_local(p).0)
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
        match self.surface {
            Surface::Brep { volume } => volume,
            _ => mesh::volume(self.boundary()),
        }
    }
    pub fn area(&self) -> f64 {
        mesh::area(self.boundary())
    }
    /// A view's silhouettes on an exact solid's curved faces, looking along `-eye` (`None` for any
    /// other solid, whose smooth seams are its silhouettes): where the surface's own outward normal
    /// turns from facing the eye to facing away, interpolated across each facet from the exact
    /// normals at its corners — the curve the surface draws, within its sag, and not a zigzag of
    /// the mesh's seams, which the view then leaves out.
    pub fn silhouettes(&self, eye: [f64; 3]) -> Option<Vec<Edge>> {
        if !matches!(self.surface, Surface::Brep { .. }) { return None; }
        let prim = &self.csg.prims[0];
        // a point interpolated on a chord stands inside the surface by up to the sag: put back on
        // it (where the B-rep is in hand), it is on the silhouette to second order, and a straight
        // one comes out straight
        let on = |k: usize, p: [f64; 3]| -> [f64; 3] {
            let Some((exact, faces)) = &self.exact else { return p };
            let mm = exact.mm;
            let surface = &exact.brep.faces[faces[k] as usize].surface;
            let q = surface.point(surface.inverse(p.map(|x| x * mm))).map(|x| x / mm);
            if q.iter().all(|x| x.is_finite()) { q } else { p }
        };
        let mut out = Vec::new();
        for (fi, (f, normals)) in prim.facets.iter().zip(&self.corner_normals).enumerate() {
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
        self.edges
            .get_or_init(|| csg::edges_indexed(&self.csg, self.epsilon, self.ray_indices()))
    }
    pub fn mesh(&self) -> &mesh::Mesh {
        // a field's surface came indexed, its corners shared exactly: nothing for a weld to do
        self.mesh.get_or_init(|| match self.surface {
            Surface::Field { .. } => mesh::grouped_welded(self.boundary.get().cloned().unwrap_or_else(|| self.field_pieces())),
            // the B-rep's mesh shares its points exactly too
            Surface::Brep { .. } => mesh::grouped_welded(self.boundary().to_vec()),
            Surface::Csg => mesh::grouped(self.boundary()),
        })
    }
    pub fn world_boundary(&self) -> Vec<Piece> {
        translate_pieces(self.boundary(), self.origin.0)
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
    /// The surface as it stands even while it is still being refined (`mesh::preview_stl`): what a
    /// host offers when asked for a file before the refinement has finished.
    pub fn preview_stl(&self) -> Result<Vec<u8>, String> {
        mesh::preview_stl(self.mesh(), self.origin.0, &self.name)
    }
    pub fn stl(&self) -> Result<Vec<u8>, String> {
        self.finished()?;
        mesh::placed_stl(self.mesh(), self.origin.0, &self.name)
    }
    /// Refuse a surface still being refined: it may be open, and a file is a finished part.
    pub fn finished(&self) -> Result<(), String> {
        if self.provisional() { Err(format!("`{}`: its surface is still being refined", self.name)) } else { Ok(()) }
    }
    pub(crate) fn classifier(&self) -> &Csg {
        &self.csg
    }
    /// Rebase another validated solid into this solid's frame for pairwise kernel queries.
    pub(crate) fn relative(&self, other: &Self) -> (Csg, Vec<Piece>) {
        let delta = std::array::from_fn(|k| other.origin.0[k] - self.origin.0[k]);
        let mut csg = other.csg.clone();
        translate_csg(&mut csg, delta);
        (csg, translate_pieces(other.boundary(), delta))
    }

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
        let (SolidDef::Prism { face, .. } | SolidDef::Through { face, .. }) = sol.def else {
            continue;
        };
        let face = &sk.faces[face as usize];
        for (edges, names) in face.boundaries() {
            if edges.len() != 1 || edges[0].kind != EntKind::Circle
                || !curved.contains(format!("{}.{}", sol.name, names[0]).as_str()) {
                continue;
            }
            let circle = &sk.circles[edges[0].i()];
            let (basis, pose) = match face.plane()? {
                Some(i) => {
                    let p = &sk.planes[i as usize];
                    (
                        sk.basis(i as usize),
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
