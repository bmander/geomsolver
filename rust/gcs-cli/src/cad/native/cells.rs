//! Arrangement of a stock solid by candidate sheets, with material chosen per cell.
use super::*;

extern "C" {
    fn solvent_cad_split_solid(cad: *mut c_void,solid: c_int,tools: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_split_solid_fuzzy(cad: *mut c_void,solid: c_int,tools: *const c_int,count: c_int,fuzzy: f64) -> c_int;
    fn solvent_cad_solids(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_solid_sample(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_solid_samples(cad: *mut c_void,id: c_int,output: *mut f64,capacity: c_int,measure: c_int) -> c_int;
    fn solvent_cad_fuse(cad: *mut c_void,ids: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_solid_contains(cad: *mut c_void,id: c_int,points: *const f64,count: c_int,tolerance: f64,
        output: *mut c_int) -> c_int;
    fn solvent_cad_volume(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_common(cad: *mut c_void,a: c_int,b: c_int) -> c_int;
    fn solvent_cad_common_volume(cad: *mut c_void,a: c_int,b: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_place(cad: *mut c_void,source: c_int,matrix: *const f64) -> c_int;
    fn solvent_cad_bspline_face_with(cad: *mut c_void,points: *const f64,nu: c_int,nv: c_int,parametrization: c_int) -> c_int;
    fn solvent_cad_section(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,side: *const f64,
        rows: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_face_normal(cad: *mut c_void,face: c_int,point: *const f64,output: *mut f64) -> c_int;
    fn solvent_cad_tolerance(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
}

/// One cell of a partition with a point strictly inside it. `margin` is the
/// classifier tolerance at which the point still reads inside, so a ball of that
/// radius about the point lies in the cell; it is not a guarantee about the cell's
/// shape elsewhere.
#[derive(Clone,Copy,Debug)]
pub(crate) struct Cell {
    pub solid: c_int,
    pub point: [f64;3],
    pub margin: f64,
    pub volume: f64,
}

impl Session {
    pub(crate) fn split_solid(&self,solid: c_int,tools: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_split_solid(self.0,solid,tools.as_ptr(),tools.len() as c_int) })
    }
    pub(crate) fn split_solid_fuzzy(&self,solid: c_int,tools: &[c_int],fuzzy: f64) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_split_solid_fuzzy(self.0,solid,tools.as_ptr(),tools.len() as c_int,fuzzy) })
    }
    pub(crate) fn solids(&self,source: c_int) -> Result<Vec<c_int>,String> {
        let count = self.result(unsafe { solvent_cad_solids(self.0,source,std::ptr::null_mut(),0) })?;
        let mut ids = vec![-1;count as usize];
        let actual = self.result(unsafe { solvent_cad_solids(self.0,source,ids.as_mut_ptr(),count) })?;
        if actual != count { return Err("native solid count changed during enumeration".into()); }
        Ok(ids)
    }
    pub(crate) fn cell(&self,solid: c_int) -> Result<Cell,String> {
        let mut data = [0.;5];
        self.result(unsafe { solvent_cad_solid_sample(self.0,solid,data.as_mut_ptr()) })?;
        Ok(Cell {solid,point:[data[0],data[1],data[2]],margin:data[3],volume:data[4]})
    }
    pub(crate) fn cells(&self,partition: c_int) -> Result<Vec<Cell>,String> {
        self.solids(partition)?.into_iter().map(|s| self.cell(s)).collect()
    }
    /// Up to `capacity` interior points of the cell with their measured distance
    /// to its boundary, farthest first, measured on at most `measure` spread
    /// candidates.
    pub(crate) fn samples(&self,solid: c_int,capacity: usize,measure: usize) -> Result<Vec<([f64;3],f64)>,String> {
        let mut data = vec![0.;4*capacity];
        let count = self.result(unsafe { solvent_cad_solid_samples(self.0,solid,data.as_mut_ptr(),capacity as c_int,measure as c_int) })?;
        Ok((0..count as usize).map(|i| ([data[4*i],data[4*i+1],data[4*i+2]],data[4*i+3])).collect())
    }
    pub(crate) fn fuse(&self,ids: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_fuse(self.0,ids.as_ptr(),ids.len() as c_int) })
    }
    /// Per point: 0 outside, 1 inside, 2 within `tolerance` of the boundary.
    pub(crate) fn solid_contains(&self,solid: c_int,points: &[[f64;3]],tolerance: f64) -> Result<Vec<c_int>,String> {
        let mut states = vec![-1;points.len()];
        self.result(unsafe { solvent_cad_solid_contains(self.0,solid,points.as_ptr().cast(),
            points.len() as c_int,tolerance,states.as_mut_ptr()) })?;
        Ok(states)
    }
    pub(crate) fn volume(&self,solid: c_int) -> Result<f64,String> {
        let mut v = [0.];
        self.result(unsafe { solvent_cad_volume(self.0,solid,v.as_mut_ptr()) })?;
        Ok(v[0])
    }
    pub(crate) fn common(&self,a: c_int,b: c_int) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_common(self.0,a,b) })
    }
    pub(crate) fn cut(&self,a: c_int,b: c_int) -> Result<c_int,String> { self.boolean(a,b,"cut") }
    pub(crate) fn common_volume(&self,a: c_int,b: c_int) -> Result<f64,String> {
        let mut v = [0.];
        self.result(unsafe { solvent_cad_common_volume(self.0,a,b,v.as_mut_ptr()) })?;
        Ok(v[0])
    }
    /// Place a copy of any shape at a model-unit pose.
    pub(crate) fn place(&self,source: c_int,pose: gcs_core::envelope::Motion,scale: f64) -> Result<c_int,String> {
        let matrix = gcs_core::solid::cad::placement_matrix(pose,scale);
        self.result(unsafe { solvent_cad_place(self.0,source,matrix.as_ptr()) })
    }
    /// Interpolate a row-major grid of points as a B-spline face with chord-length
    /// parameters, which keep uneven row spacing from overshooting.
    pub(crate) fn fit_sheet(&self,points: &[[f64;3]],rows: usize,columns: usize) -> Result<c_int,String> {
        if points.len() != rows*columns { return Err("sheet grid size mismatch".into()); }
        self.result(unsafe { solvent_cad_bspline_face_with(self.0,points.as_ptr().cast(),rows as c_int,columns as c_int,1) })
    }
    /// Section a solid by the half-plane through `origin` containing `axis` on the
    /// `side` direction: (edge handle, 1-based face index in `faces(solid)`)
    /// pairs, unordered.
    pub(crate) fn section(&self,solid: c_int,origin: [f64;3],axis: [f64;3],side: [f64;3]) -> Result<Vec<(c_int,c_int)>,String> {
        let count = self.result(unsafe { solvent_cad_section(self.0,solid,origin.as_ptr(),axis.as_ptr(),side.as_ptr(),std::ptr::null_mut(),0) })?;
        let mut rows = vec![-1;2*count as usize];
        let actual = self.result(unsafe { solvent_cad_section(self.0,solid,origin.as_ptr(),axis.as_ptr(),side.as_ptr(),rows.as_mut_ptr(),count) })?;
        if actual != count { return Err("section changed between count and retrieval".into()); }
        Ok((0..count as usize).map(|i| (rows[2*i],rows[2*i+1])).collect())
    }
    /// Outward unit normal of a face at a point's projection onto its support,
    /// and the projection distance; no trim test.
    pub(crate) fn face_normal(&self,face: c_int,point: [f64;3]) -> Result<([f64;3],f64),String> {
        let mut data = [0.;4];
        self.result(unsafe { solvent_cad_face_normal(self.0,face,point.as_ptr(),data.as_mut_ptr()) })?;
        Ok(([data[0],data[1],data[2]],data[3]))
    }
    /// Maximum vertex, edge and face tolerances the kernel carries on the shape.
    pub(crate) fn tolerances(&self,solid: c_int) -> Result<[f64;3],String> {
        let mut t = [0.;3];
        self.result(unsafe { solvent_cad_tolerance(self.0,solid,t.as_mut_ptr()) })?;
        Ok(t)
    }
    pub(crate) fn step(&self,solid: c_int,path: &str) -> Result<(),String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        self.result(unsafe { solvent_cad_step(self.0,solid,name.as_ptr()) })?;
        Ok(())
    }
    pub(crate) fn stl(&self,solid: c_int,path: &str) -> Result<(),String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        self.result(unsafe { solvent_cad_stl(self.0,solid,name.as_ptr()) })?;
        Ok(())
    }
}
