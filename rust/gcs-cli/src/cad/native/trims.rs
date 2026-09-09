//! Incidence and trimming curves in native face coordinates.
use super::*;
extern "C" {
    fn solvent_cad_curve_point(cad: *mut c_void,edge: c_int,t: f64,output: *mut f64) -> c_int;
    fn solvent_cad_face_parameters(cad: *mut c_void,face: c_int,point: *const f64,distance: f64,output: *mut f64) -> c_int;
    fn solvent_cad_pcurve(cad: *mut c_void,face: c_int,points: *const f64,count: c_int,closed: c_int,distance: f64) -> c_int;
    fn solvent_cad_split_pcurves(cad: *mut c_void,face: c_int,edges: *const c_int,count: c_int) -> c_int;
}
impl Session {
    pub(crate) fn edge_point(&self,edge: c_int,t: f64) -> Result<[f64;3],String> {
        let mut p = [0.;3];
        self.result(unsafe { solvent_cad_curve_point(self.0,edge,t,p.as_mut_ptr()) })?;
        Ok(p)
    }
    /// Normalized finite-face UV coordinates and measured incidence distance in
    /// mm. None means no on-trim projection within tolerance. A projection alone
    /// does not establish continuity across periodic seams or poles.
    pub(crate) fn face_parameters(&self,face: c_int,point: [f64;3],distance: f64)
        -> Result<Option<([f64;2],f64)>,String> {
        let mut data = [0.;3];
        match self.result(unsafe { solvent_cad_face_parameters(self.0,face,point.as_ptr(),distance,data.as_mut_ptr()) })? {
            0 => Ok(None),
            1 => Ok(Some(([data[0],data[1]],data[2]))),
            _ => Err("invalid native projection result".into()),
        }
    }
    /// Fit a UV curve and attach its spatial edge to this face. Closed points
    /// omit a duplicate final point; open endpoints must lie on face trims.
    /// `distance` controls spatial edge construction, not contact fitting error.
    pub(crate) fn pcurve(&self,face: c_int,points: &[[f64;2]],closed: bool,distance: f64) -> Result<c_int,String> {
        if points.len() > 4096 { return Err("too many contact curve samples".into()); }
        self.result(unsafe { solvent_cad_pcurve(self.0,face,points.as_ptr().cast(),points.len() as c_int,
            i32::from(closed),distance) })
    }
    pub(crate) fn split_pcurves(&self,face: c_int,edges: &[c_int]) -> Result<c_int,String> {
        if edges.len() > 1024 { return Err("too many contact edges".into()); }
        self.result(unsafe { solvent_cad_split_pcurves(self.0,face,edges.as_ptr(),edges.len() as c_int) })
    }
}
