//! A static solid's sharp edges and trimmed faces, as the core's feature reader asks for them
//! (`gcs_core::solid::blank_features`). Nothing here judges material; it reports the kernel's
//! own topology of the static blank, which is the reliable half of a body with swept cuts.
use super::*;
use gcs_core::solid::blank_features::{self,BlankTopology};

extern "C" {
    fn solvent_cad_boundary(cad: *mut c_void,source: c_int,rows: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_boundary_point(cad: *mut c_void,row: *const c_int,fraction: f64,output: *mut f64) -> c_int;
}

/// One topological edge of a solid: the two faces it bounds and whether it is a seam of one
/// face or a collapsed pole edge, neither of which is a feature.
#[derive(Clone,Copy)]
pub(crate) struct BoundaryEdge { pub row: [c_int;4] }
impl BoundaryEdge {
    pub fn seam_or_pole(&self) -> bool { self.row[3] != 0 }
}

/// A point on a boundary edge (native millimetres): its position, the two faces' outward
/// normals and the signed dihedral there (negative convex, positive concave, zero smooth).
pub(crate) struct EdgePoint { pub position: [f64;3],pub normals: [[f64;3];2],pub dihedral: f64 }

impl Session {
    pub(crate) fn construct_recipe(&self,recipe: &Json) -> Result<c_int,String> { self.construct(recipe) }

    pub(crate) fn boundary(&self,solid: c_int) -> Result<Vec<BoundaryEdge>,String> {
        let count = self.result(unsafe { solvent_cad_boundary(self.ptr,solid,std::ptr::null_mut(),0) })?;
        let mut rows = vec![0;4*count as usize];
        self.result(unsafe { solvent_cad_boundary(self.ptr,solid,rows.as_mut_ptr(),count) })?;
        Ok(rows.chunks(4).map(|r| BoundaryEdge {row:[r[0],r[1],r[2],r[3]]}).collect())
    }

    pub(crate) fn boundary_point(&self,edge: &BoundaryEdge,fraction: f64) -> Result<EdgePoint,String> {
        let mut data = [0.;15];
        self.result(unsafe { solvent_cad_boundary_point(self.ptr,edge.row.as_ptr(),fraction,data.as_mut_ptr()) })?;
        Ok(EdgePoint {position:[data[0],data[1],data[2]],normals:[[data[6],data[7],data[8]],[data[9],data[10],data[11]]],
            dihedral:data[14]})
    }
}

/// A native blank, its feature edges (seams and poles left out) and, once asked, its faces.
pub(crate) struct NativeBlank<'a> { session: &'a Session,edges: Vec<BoundaryEdge>,solid: c_int,
    faces: std::cell::OnceCell<Vec<c_int>> }

impl<'a> NativeBlank<'a> {
    pub(crate) fn new(session: &'a Session,solid: c_int) -> Result<Self,String> {
        let edges = session.boundary(solid)?.into_iter().filter(|e| !e.seam_or_pole()).collect();
        Ok(NativeBlank {session,edges,solid,faces:Default::default()})
    }
    fn face(&self,face: usize) -> Result<c_int,String> {
        if self.faces.get().is_none() { let _ = self.faces.set(self.session.faces(self.solid)?); }
        Ok(self.faces.get().unwrap()[face])
    }
}

impl BlankTopology for NativeBlank<'_> {
    fn edges(&self) -> Result<usize,String> { Ok(self.edges.len()) }
    fn edge_point(&self,edge: usize,fraction: f64) -> Result<blank_features::EdgePoint,String> {
        let e = self.session.boundary_point(&self.edges[edge],fraction)?;
        Ok(blank_features::EdgePoint {position:e.position,normals:e.normals,dihedral:e.dihedral})
    }
    fn faces(&self) -> Result<usize,String> {
        if self.faces.get().is_none() { let _ = self.faces.set(self.session.faces(self.solid)?); }
        Ok(self.faces.get().unwrap().len())
    }
    fn face_seams(&self,face: usize) -> Result<[bool;2],String> { self.session.face_seams(self.face(face)?) }
    fn face_point(&self,face: usize,u: f64,v: f64,tolerance: f64) -> Result<Option<blank_features::FacePoint>,String> {
        Ok(self.session.face_point(self.face(face)?,u,v,tolerance)?
            .map(|p| blank_features::FacePoint {position:p.position,normal:p.normal,on_trim:p.on_trim}))
    }
}
