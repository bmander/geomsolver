//! The native kernel's operations and queries a swept export uses: a stock split by sheets
//! into cells, each cell sampled and measured, the material cells fused; sections of a
//! cutter, faces and their points, and the files a solid is written as.
use super::*;

extern "C" {
    fn solvent_cad_split_solid(cad: *mut c_void,solid: c_int,tools: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_solids(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_solid_samples(cad: *mut c_void,id: c_int,output: *mut f64,capacity: c_int,measure: c_int) -> c_int;
    fn solvent_cad_fuse(cad: *mut c_void,ids: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_revolved(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,seam: *const f64) -> c_int;
    fn solvent_cad_pattern(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,angles: *const f64,count: c_int,
        sides: *const c_int,fuzzy: f64) -> c_int;
    fn solvent_cad_solid_contains(cad: *mut c_void,id: c_int,points: *const f64,count: c_int,tolerance: f64,
        output: *mut c_int) -> c_int;
    fn solvent_cad_volume(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_common_volume(cad: *mut c_void,a: c_int,b: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_place(cad: *mut c_void,source: c_int,matrix: *const f64) -> c_int;
    fn solvent_cad_bspline_face_with(cad: *mut c_void,points: *const f64,nu: c_int,nv: c_int,parametrization: c_int) -> c_int;
    fn solvent_cad_section(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,side: *const f64,
        rows: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_face_normal(cad: *mut c_void,face: c_int,point: *const f64,output: *mut f64) -> c_int;
    fn solvent_cad_tolerance(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_faces(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_face_point(cad: *mut c_void,face: c_int,u: f64,v: f64,tolerance: f64,output: *mut f64) -> c_int;
    fn solvent_cad_face_seams(cad: *mut c_void,face: c_int,axes: *mut c_int) -> c_int;
    fn solvent_cad_curve_point(cad: *mut c_void,edge: c_int,t: f64,output: *mut f64) -> c_int;
    fn solvent_cad_surface_feet(cad: *mut c_void,face: c_int,points: *const f64,count: c_int,output: *mut f64) -> c_int;
    fn solvent_cad_surface_feet_near(cad: *mut c_void,face: c_int,points: *const f64,guesses: *const f64,count: c_int,
        trust: f64,output: *mut f64) -> c_int;
    fn solvent_cad_read_step(cad: *mut c_void,path: *const c_char) -> c_int;
    fn solvent_cad_face_kind(cad: *mut c_void,face: c_int) -> c_int;
    fn solvent_cad_surface_grid(cad: *mut c_void,face: c_int,nu: c_int,nv: c_int,output: *mut f64) -> c_int;
}

/// One cell of a partition, its volume, and its deepest interior sample.
#[derive(Clone,Copy,Debug)]
pub(crate) struct Cell {
    pub solid: c_int,
    pub point: [f64;3],
    pub volume: f64,
}

/// A point of a face: native millimetres, with the face's oriented unit normal.
pub(crate) struct FacePoint {
    pub position: [f64;3],
    pub normal: [f64;3],
    pub on_trim: bool,
}

impl Session {
    pub(crate) fn split_solid(&self,solid: c_int,tools: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_split_solid(self.0,solid,tools.as_ptr(),tools.len() as c_int) })
    }
    pub(crate) fn solids(&self,source: c_int) -> Result<Vec<c_int>,String> {
        let count = self.result(unsafe { solvent_cad_solids(self.0,source,std::ptr::null_mut(),0) })?;
        let mut ids = vec![-1;count as usize];
        let actual = self.result(unsafe { solvent_cad_solids(self.0,source,ids.as_mut_ptr(),count) })?;
        if actual != count { return Err("native solid count changed during enumeration".into()); }
        Ok(ids)
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
    /// A solid and its copies turned by `angles` about the line through `origin` (mm) along `axis`,
    /// united: its faces on `sides` left out and the rest of every copy sewn to `fuzzy` (mm), no face
    /// intersected.
    pub(crate) fn pattern(&self,solid: c_int,origin: [f64;3],axis: [f64;3],angles: &[f64],sides: [c_int;2],fuzzy: f64)
        -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_pattern(self.0,solid,origin.as_ptr(),axis.as_ptr(),angles.as_ptr(),angles.len() as c_int,
            sides.as_ptr(),fuzzy) })
    }
    /// A solid of revolution about the line through `origin` (mm) along `axis`, made again by
    /// turning its meridian section about it, so that every face's frame is on that line, its
    /// parameters starting on the half-plane towards `seam`.
    pub(crate) fn revolved(&self,solid: c_int,origin: [f64;3],axis: [f64;3],seam: [f64;3]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_revolved(self.0,solid,origin.as_ptr(),axis.as_ptr(),seam.as_ptr()) })
    }
    /// The box about some shapes, lower corner then upper (mm).
    pub(crate) fn bounds(&self,ids: &[c_int]) -> Result<[[f64;3];2],String> {
        let mut b = [0.;6];
        self.result(unsafe { solvent_cad_bounds(self.0,ids.as_ptr(),ids.len() as c_int,b.as_mut_ptr()) })?;
        Ok([[b[0],b[1],b[2]],[b[3],b[4],b[5]]])
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
    /// parameters, which keep uneven row spacing from overshooting where the columns space
    /// their rows alike: each row's parameter is averaged over the columns. A sheet held to a
    /// tolerance is also fitted with centripetal parameters (`fit_sheet_with`, 2).
    pub(crate) fn fit_sheet(&self,points: &[[f64;3]],rows: usize,columns: usize) -> Result<c_int,String> {
        self.fit_sheet_with(points,rows,columns,1)
    }
    /// The same with OCCT's parametrization by number: 0 even, 1 chord length, 2 centripetal.
    pub(crate) fn fit_sheet_with(&self,points: &[[f64;3]],rows: usize,columns: usize,parametrization: c_int) -> Result<c_int,String> {
        if points.len() != rows*columns { return Err("sheet grid size mismatch".into()); }
        self.result(unsafe { solvent_cad_bspline_face_with(self.0,points.as_ptr().cast(),rows as c_int,columns as c_int,parametrization) })
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
    /// The mesh an export writes without a stated tolerance: 0.01 mm absolute deflection and
    /// 0.2 rad angular.
    pub(crate) fn stl(&self,solid: c_int,path: &str) -> Result<(),String> { self.stl_with(solid,path,0.01,0.2) }
    /// Mesh a shape afresh at an absolute chordal `deflection` (mm) and an `angular` one (radians).
    pub(crate) fn remesh(&self,solid: c_int,deflection: f64,angular: f64) -> Result<(),String> {
        self.result(unsafe { solvent_cad_remesh(self.0,solid,deflection,angular) }).map(|_| ())
    }
    /// The chordal sag a meshed shape has (mm), read at its triangles' centroids and edge midpoints
    /// against each face's surface at the same parameters, and where it is largest.
    pub(crate) fn mesh_sag(&self,solid: c_int) -> Result<(f64,[f64;3]),String> {
        let mut data = [0.;4];
        self.result(unsafe { solvent_cad_mesh_sag(self.0,solid,data.as_mut_ptr()) })?;
        Ok((data[0],[data[1],data[2],data[3]]))
    }
    /// An STL meshed at an absolute chordal `deflection` (mm) and an `angular` one (radians); a
    /// shape already meshed at least as finely keeps its mesh.
    pub(crate) fn stl_with(&self,solid: c_int,path: &str,deflection: f64,angular: f64) -> Result<(),String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        self.result(unsafe { solvent_cad_stl(self.0,solid,name.as_ptr(),deflection,angular) })?;
        Ok(())
    }
    pub(crate) fn faces(&self,source: c_int) -> Result<Vec<c_int>,String> {
        let count = self.result(unsafe { solvent_cad_faces(self.0,source,std::ptr::null_mut(),0) })?;
        let mut faces = vec![-1;count as usize];
        let actual = self.result(unsafe { solvent_cad_faces(self.0,source,faces.as_mut_ptr(),count) })?;
        if actual != count { return Err("native face count changed during enumeration".into()); }
        Ok(faces)
    }
    /// Coordinates span this face's finite UV trim box, unlike supporting-surface
    /// queries. None means outside its trims (including holes), not a surface
    /// construction failure. Singular evaluations and unresolved classifiers
    /// remain errors.
    pub(crate) fn face_point(&self,face: c_int,u: f64,v: f64,tolerance: f64)
        -> Result<Option<FacePoint>,String> {
        let mut data = [0.;6];
        match self.result(unsafe { solvent_cad_face_point(self.0,face,u,v,tolerance,data.as_mut_ptr()) })? {
            0 => Ok(None),
            state @ (1 | 2) => Ok(Some(FacePoint {position:[data[0],data[1],data[2]],
                normal:[data[3],data[4],data[5]],on_trim:state == 2})),
            _ => Err("invalid native face membership".into()),
        }
    }
    pub(crate) fn face_seams(&self,face: c_int) -> Result<[bool;2],String> {
        let mut axes = [0;2];
        self.result(unsafe { solvent_cad_face_seams(self.0,face,axes.as_mut_ptr()) })?;
        Ok(axes.map(|v| v != 0))
    }
    pub(crate) fn edge_point(&self,edge: c_int,t: f64) -> Result<[f64;3],String> {
        let mut p = [0.;3];
        self.result(unsafe { solvent_cad_curve_point(self.0,edge,t,p.as_mut_ptr()) })?;
        Ok(p)
    }
    /// A STEP file's shape, read as it was written: nothing validated or repaired.
    pub(crate) fn read_step(&self,path: &str) -> Result<c_int,String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        self.result(unsafe { solvent_cad_read_step(self.0,name.as_ptr()) })
    }
    /// The kind of a face's supporting surface, in OCCT's `GeomAbs_SurfaceType` order.
    pub(crate) fn face_kind(&self,face: c_int) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_face_kind(self.0,face) })
    }
    /// A face's supporting surface on an even `nu` x `nv` grid over its UV box (u first): each
    /// point's position and oriented unit normal, without a trim test.
    pub(crate) fn surface_grid(&self,face: c_int,nu: usize,nv: usize) -> Result<Vec<([f64;3],[f64;3])>,String> {
        let mut data = vec![0.;6*nu*nv];
        self.result(unsafe { solvent_cad_surface_grid(self.0,face,nu as c_int,nv as c_int,data.as_mut_ptr()) })?;
        Ok(data.chunks(6).map(|d| ([d[0],d[1],d[2]],[d[3],d[4],d[5]])).collect())
    }
    /// For each point, the face's support normal (unoriented) at its nearest foot and the
    /// distance to it; None where no foot is found.
    pub(crate) fn surface_feet(&self,face: c_int,points: &[[f64;3]]) -> Result<Vec<Option<([f64;3],f64)>>,String> {
        let mut data = vec![0.;4*points.len()];
        self.result(unsafe { solvent_cad_surface_feet(self.0,face,points.as_ptr().cast(),points.len() as c_int,data.as_mut_ptr()) })?;
        Ok(feet(&data))
    }
    /// The same, each point's foot searched from a guess (`u`, `v` as fractions of the face's UV
    /// box, NaN for none), falling back to the global search where the local one does not improve
    /// on the guess or lands farther than `trust` (mm).
    pub(crate) fn surface_feet_near(&self,face: c_int,points: &[[f64;3]],guesses: &[[f64;2]],trust: f64)
        -> Result<Vec<Option<([f64;3],f64)>>,String> {
        if guesses.len() != points.len() { return Err("one guess a point".into()); }
        let mut data = vec![0.;4*points.len()];
        self.result(unsafe { solvent_cad_surface_feet_near(self.0,face,points.as_ptr().cast(),guesses.as_ptr().cast(),
            points.len() as c_int,trust,data.as_mut_ptr()) })?;
        Ok(feet(&data))
    }
}

/// The feet the kernel wrote, four doubles a point: the normal and the distance, NaN for none.
fn feet(data: &[f64]) -> Vec<Option<([f64;3],f64)>> {
    data.chunks(4).map(|d| d[3].is_finite().then(|| ([d[0],d[1],d[2]],d[3]))).collect()
}
