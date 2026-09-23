//! Incidence and trimming curves in native face coordinates.
use super::*;
extern "C" {
    fn solvent_cad_curve_point(cad: *mut c_void,edge: c_int,t: f64,output: *mut f64) -> c_int;
    fn solvent_cad_face_parameters(cad: *mut c_void,face: c_int,point: *const f64,distance: f64,output: *mut f64) -> c_int;
    fn solvent_cad_surface_feet(cad: *mut c_void,face: c_int,points: *const f64,count: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_face_parameters_many(cad: *mut c_void,face: c_int,points: *const f64,count: c_int,distance: f64,
        output: *mut f64,found: *mut c_int) -> c_int;
    fn solvent_cad_pcurve(cad: *mut c_void,face: c_int,points: *const f64,count: c_int,closed: c_int,distance: f64) -> c_int;
    fn solvent_cad_split_pcurves(cad: *mut c_void,face: c_int,edges: *const c_int,count: c_int) -> c_int;
}
impl Session {
    /// Attach sufficiently sampled spatial contact points to a native face.
    /// Endpoint seam aliases are reconciled only after spatial incidence checks.
    /// Interior UV jumps over half a face span are refused: callers must refine
    /// or split those curves. This does not bound the fit against the source law.
    pub(crate) fn contact_edge(&self,face: c_int,points: &[[f64;3]],closed: bool,distance: f64) -> Result<c_int,String> {
        if points.len() < 2 || points.len() > 4096 { return Err("contact edge requires 2..4096 samples".into()); }
        let mut uv: Vec<_> = points.iter().map(|&p| self.face_parameters(face,p,distance)?
            .map(|(uv,_)| uv).ok_or("contact sample does not lie on the native face".to_string()))
            .collect::<Result<_,_>>()?;
        if !closed {
            let last = uv.len()-1;
            for (index,neighbor) in [(0,1),(last,last-1)] { for axis in 0..2 {
                let value = uv[index][axis];
                if (value-uv[neighbor][axis]).abs() <= 0.5 { continue; }
                let alias = if value < 1e-8 { 1. } else if value > 1.-1e-8 { 0. } else { continue; };
                let mut alternative = uv[index]; alternative[axis] = alias;
                if let Some(p) = self.face_point(face,alternative[0],alternative[1],1e-9)? {
                    let q = points[index];
                    if (p.position[0]-q[0]).hypot(p.position[1]-q[1]).hypot(p.position[2]-q[2]) <= distance {
                        uv[index] = alternative;
                    }
                }
            } }
        }
        let jump = |a: [f64;2],b: [f64;2]| (0..2).any(|k| (a[k]-b[k]).abs() > 0.5);
        if uv.windows(2).any(|p| jump(p[0],p[1])) || (closed && jump(uv[0],uv[uv.len()-1])) {
            return Err("contact curve crosses a face seam or needs more samples".into());
        }
        self.pcurve(face,&uv,closed,distance)
    }

    pub(crate) fn edge_point(&self,edge: c_int,t: f64) -> Result<[f64;3],String> {
        let mut p = [0.;3];
        self.result(unsafe { solvent_cad_curve_point(self.0,edge,t,p.as_mut_ptr()) })?;
        Ok(p)
    }
    /// Normalized finite-face UV coordinates and measured incidence distance in
    /// mm. None means no on-trim projection within tolerance. A projection alone
    /// does not establish continuity across periodic seams or poles.
    /// For each point, the face's support normal (unoriented) at its nearest foot and the
    /// distance to it; None where no foot is found.
    pub(crate) fn surface_feet(&self,face: c_int,points: &[[f64;3]]) -> Result<Vec<Option<([f64;3],f64)>>,String> {
        let mut data = vec![0.;4*points.len()];
        self.result(unsafe { solvent_cad_surface_feet(self.0,face,points.as_ptr().cast(),points.len() as c_int,data.as_mut_ptr()) })?;
        Ok((0..points.len()).map(|i| data[4*i+3].is_finite().then(|| ([data[4*i],data[4*i+1],data[4*i+2]],data[4*i+3]))).collect())
    }
    /// `face_parameters` for many points, the projector initialised once.
    pub(crate) fn face_parameters_many(&self,face: c_int,points: &[[f64;3]],distance: f64)
        -> Result<Vec<Option<([f64;2],f64)>>,String> {
        let mut data = vec![0.;3*points.len()];
        let mut found = vec![0 as c_int;points.len()];
        self.result(unsafe { solvent_cad_face_parameters_many(self.0,face,points.as_ptr().cast(),points.len() as c_int,
            distance,data.as_mut_ptr(),found.as_mut_ptr()) })?;
        Ok((0..points.len()).map(|i| (found[i] == 1).then(|| ([data[3*i],data[3*i+1]],data[3*i+2]))).collect())
    }
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
