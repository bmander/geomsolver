//! A static solid's sharp edges and trimmed faces, as the 1D features and 2D patches a field
//! mesher protects (`cad::delpsc`). Nothing here judges material; it reports the kernel's own
//! topology of the static blank, which is the reliable half of a body with swept cuts.
use super::*;

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
        let count = self.result(unsafe { solvent_cad_boundary(self.0,solid,std::ptr::null_mut(),0) })?;
        let mut rows = vec![0;4*count as usize];
        self.result(unsafe { solvent_cad_boundary(self.0,solid,rows.as_mut_ptr(),count) })?;
        Ok(rows.chunks(4).map(|r| BoundaryEdge {row:[r[0],r[1],r[2],r[3]]}).collect())
    }

    pub(crate) fn boundary_point(&self,edge: &BoundaryEdge,fraction: f64) -> Result<EdgePoint,String> {
        let mut data = [0.;15];
        self.result(unsafe { solvent_cad_boundary_point(self.0,edge.row.as_ptr(),fraction,data.as_mut_ptr()) })?;
        Ok(EdgePoint {position:[data[0],data[1],data[2]],normals:[[data[6],data[7],data[8]],[data[9],data[10],data[11]]],
            dihedral:data[14]})
    }
}
