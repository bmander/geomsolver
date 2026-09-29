//! The native kernel's operations and queries a swept export uses: a stock split by sheets
//! into cells, each cell sampled and measured, the material cells fused; sections of a
//! cutter, faces and their points, and the files a solid is written as.
use super::*;

extern "C" {
    fn solvent_cad_split_solid(cad: *mut c_void,solid: c_int,tools: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_solids(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_fused_tools(cad: *mut c_void,tools: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_solids_samples(cad: *mut c_void,ids: *const c_int,count: c_int,output: *mut f64,capacity: c_int,measure: c_int,
        written: *mut c_int) -> c_int;
    fn solvent_cad_fuse(cad: *mut c_void,ids: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_revolved(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,seam: *const f64) -> c_int;
    fn solvent_cad_revolve_region(cad: *mut c_void,region: c_int,origin: *const f64,axis: *const f64) -> c_int;
    fn solvent_cad_pattern(cad: *mut c_void,solid: c_int,origin: *const f64,axis: *const f64,angles: *const f64,count: c_int,
        sides: *const c_int,fuzzy: f64) -> c_int;
    fn solvent_cad_pattern_check(cad: *mut c_void,id: c_int) -> c_int;
    fn solvent_cad_sector_mesh(cad: *mut c_void,piece: c_int,sides: *const c_int,fuzzy: f64,deflection: f64,interior: f64,angular: f64,
        output: *mut f64) -> c_int;
    fn solvent_cad_sector_stl(cad: *mut c_void,piece: c_int,sides: *const c_int,fuzzy: f64,origin: *const f64,axis: *const f64,
        count: c_int,pitch: f64,reach: f64,path: *const c_char,output: *mut f64) -> c_int;
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
    fn solvent_cad_surface_feet_near(cad: *mut c_void,face: c_int,points: *const f64,guesses: *const f64,count: c_int,
        trust: f64,output: *mut f64) -> c_int;
    fn solvent_cad_read_step(cad: *mut c_void,path: *const c_char) -> c_int;
    fn solvent_cad_face_kind(cad: *mut c_void,face: c_int) -> c_int;
    fn solvent_cad_surface_grid(cad: *mut c_void,face: c_int,nu: c_int,nv: c_int,output: *mut f64) -> c_int;
}

/// A body built as one sector patterned (`sweep_boundary::sector`): the sector's material with its
/// two side faces, the sides it was cut by, the fuzzy value they are found to, and the turn
/// (`count` copies a `pitch` apart about the line through `origin`, mm, along `axis`).
#[derive(Clone,Copy,Debug)]
pub(crate) struct Patterned {
    pub piece: c_int,
    pub sides: [c_int;2],
    pub fuzzy: f64,
    pub origin: [f64;3],
    pub axis: [f64;3],
    pub count: usize,
    pub pitch: f64,
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
    /// Split tools split by each other, as one tool: a split by it does not intersect them again.
    pub(crate) fn fused_tools(&self,tools: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_fused_tools(self.0,tools.as_ptr(),tools.len() as c_int) })
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
    #[cfg(test)]
    pub(crate) fn samples(&self,solid: c_int,capacity: usize,measure: usize) -> Result<Vec<([f64;3],f64)>,String> {
        Ok(self.samples_of(&[solid],capacity,measure)?.pop().unwrap_or_default())
    }
    /// `samples` of several cells, each cell on its own core.
    pub(crate) fn samples_of(&self,solids: &[c_int],capacity: usize,measure: usize) -> Result<Vec<Vec<([f64;3],f64)>>,String> {
        let mut data = vec![0.;4*capacity*solids.len()];
        let mut written = vec![0 as c_int;solids.len()];
        self.result(unsafe { solvent_cad_solids_samples(self.0,solids.as_ptr(),solids.len() as c_int,data.as_mut_ptr(),capacity as c_int,
            measure as c_int,written.as_mut_ptr()) })?;
        Ok(written.iter().enumerate().map(|(k,&n)| (0..n as usize).map(|i| {
            let d = &data[4*(capacity*k+i)..];
            ([d[0],d[1],d[2]],d[3])
        }).collect()).collect())
    }
    pub(crate) fn fuse(&self,ids: &[c_int]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_fuse(self.0,ids.as_ptr(),ids.len() as c_int) })
    }
    /// A solid and its copies turned by `angles` about the line through `origin` (mm) along `axis`,
    /// united: its faces on `sides` left out and the rest of every copy sewn to `fuzzy` (mm), no face
    /// intersected. Unchecked: `pattern_check` checks and measures it, and gives the handle to keep.
    pub(crate) fn pattern(&self,solid: c_int,origin: [f64;3],axis: [f64;3],angles: &[f64],sides: [c_int;2],fuzzy: f64)
        -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_pattern(self.0,solid,origin.as_ptr(),axis.as_ptr(),angles.as_ptr(),angles.len() as c_int,
            sides.as_ptr(),fuzzy) })
    }
    /// A pattern's union checked and measured: its handle, or another where the union unified does
    /// not check and the union as it was sewn does.
    pub(crate) fn pattern_check(&self,made: c_int) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_pattern_check(self.0,made) })
    }
    /// Mesh a patterned body's sector afresh, at an absolute chordal `deflection` (mm) and an
    /// `angular` one (radians), and, asked, the chordal sag its faces but the sides have (mm) and where.
    pub(crate) fn sector_mesh(&self,sector: &Patterned,deflection: f64,angular: f64,sag: bool) -> Result<Option<(f64,[f64;3])>,String> {
        self.sector_mesh_with(sector,deflection,deflection,angular,sag)
    }
    /// The same, the faces' interiors meshed at their own chordal deflection (mm).
    pub(crate) fn sector_mesh_with(&self,sector: &Patterned,deflection: f64,interior: f64,angular: f64,sag: bool)
        -> Result<Option<(f64,[f64;3])>,String> {
        let mut data = [0.;4];
        self.result(unsafe { solvent_cad_sector_mesh(self.0,sector.piece,sector.sides.as_ptr(),sector.fuzzy,deflection,interior,angular,
            if sag { data.as_mut_ptr() } else { std::ptr::null_mut() }) })?;
        Ok(sag.then(|| (data[0],[data[1],data[2],data[3]])))
    }
    /// The binary STL of a patterned body from its meshed sector: the sector's triangles but its
    /// sides', turned into every copy, neighbouring copies sharing their seam points exactly, each
    /// point of one side's seam moved to its partner's turn on the other side's, at most `reach`
    /// (mm). Returns the triangles written and the farthest a seam point moved (mm).
    pub(crate) fn sector_stl(&self,sector: &Patterned,reach: f64,path: &str) -> Result<(usize,f64),String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        let mut moved = [0.];
        let triangles = self.result(unsafe { solvent_cad_sector_stl(self.0,sector.piece,sector.sides.as_ptr(),sector.fuzzy,
            sector.origin.as_ptr(),sector.axis.as_ptr(),sector.count as c_int,sector.pitch,reach,name.as_ptr(),moved.as_mut_ptr()) })?;
        Ok((triangles as usize,moved[0]))
    }
    /// A solid of revolution about the line through `origin` (mm) along `axis`, made again by
    /// turning its meridian section about it, so that every face's frame is on that line, its
    /// parameters starting on the half-plane towards `seam`.
    pub(crate) fn revolved(&self,solid: c_int,origin: [f64;3],axis: [f64;3],seam: [f64;3]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_revolved(self.0,solid,origin.as_ptr(),axis.as_ptr(),seam.as_ptr()) })
    }
    /// A solid of revolution: a planar region turned a whole turn about a line in its plane.
    pub(crate) fn revolve_region(&self,region: c_int,origin: [f64;3],axis: [f64;3]) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_revolve_region(self.0,region,origin.as_ptr(),axis.as_ptr()) })
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
    /// Write a solid's STEP file and verify it as `step_check::verification` says: its text parsed and
    /// checked against the solid (`step_check::verify`), and — `Full` — read back by the kernel,
    /// repaired as a consumer's reader repairs it, checked and measured. What was verified, said.
    pub(crate) fn step(&self,solid: c_int,path: &str) -> Result<String,String> {
        self.step_verified(solid,path,step_check::verification())
    }
    pub(crate) fn step_verified(&self,solid: c_int,path: &str,how: step_check::Verification) -> Result<String,String> {
        self.result(unsafe { solvent_cad_validate(self.0,solid) })?;
        self.step_written(solid,path,how,false)
    }
    /// The same of a pattern's union that is being checked beside it (`pattern_check`), which the
    /// file's light verification does not wait for: written from the union as it is stored.
    pub(crate) fn step_beside_check(&self,solid: c_int,path: &str) -> Result<String,String> {
        self.step_written(solid,path,step_check::Verification::Light,true)
    }
    fn step_written(&self,solid: c_int,path: &str,how: step_check::Verification,unchecked: bool) -> Result<String,String> {
        let name = CString::new(path).map_err(|e| e.to_string())?;
        let full = how == step_check::Verification::Full;
        let started = std::time::Instant::now();
        // the solid's summary read while the kernel writes the file
        let (written,summary) = std::thread::scope(|scope| {
            let summary = scope.spawn(|| self.brep_summary(solid));
            let written = self.result(unsafe { solvent_cad_step(self.0,solid,name.as_ptr(),c_int::from(full),c_int::from(unchecked)) });
            (written,summary.join().unwrap_or_else(|e| std::panic::resume_unwind(e)))
        });
        written?;
        let wrote = started.elapsed();
        let clock = std::time::Instant::now();
        let text = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
        let text = std::str::from_utf8(&text).map_err(|_| "the STEP file is not UTF-8 text".to_string())?;
        let verified = step_check::verify(text,&summary?).map_err(|e| {
            // (`SOLVENT_KEEP_REJECTED`: the refused file kept beside where a refused mesh is)
            if let Ok(kept) = std::env::var("SOLVENT_KEEP_REJECTED") { let _ = std::fs::write(format!("{kept}.step"),text); }
            format!("the STEP file does not describe the solid: {e}")
        })?;
        Ok(format!("{} entities; {} faces ({} on B-splines), {} edges and {} vertices, each the solid's{} (written {:?}, verified {:?})",
            verified.entities,verified.faces,verified.splines,verified.edges,verified.vertices,
            if full { ", and read back by the kernel as the solid" } else { "" },wrote,clock.elapsed()))
    }
    /// What a STEP file of a stored solid must say of it (`solvent_cad_brep_summary`).
    pub(crate) fn brep_summary(&self,solid: c_int) -> Result<step_check::Solid,String> {
        let count = self.result(unsafe { solvent_cad_brep_summary(self.0,solid,std::ptr::null_mut(),0) })?;
        let mut data = vec![0.;count as usize];
        let actual = self.result(unsafe { solvent_cad_brep_summary(self.0,solid,data.as_mut_ptr(),count) })?;
        if actual != count { return Err("the solid's summary changed between count and retrieval".into()); }
        step_check::Solid::read(&data)
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
    /// For each point, the face's support normal (unoriented) at its nearest foot and the distance
    /// to it, None where no foot is found: each foot searched from a guess (`u`, `v` as fractions of
    /// the face's UV box, NaN for none), falling back to the global search where the local one does
    /// not improve on the guess or lands farther than `trust` (mm).
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
