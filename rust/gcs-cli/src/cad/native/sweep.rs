//! Exact endpoint candidates of a declared finite sweep, owned by one CAD session.
use super::*;
use gcs_core::{envelope::Motion,model::{Sketch,SolidDef},motion::Family,solid::cad};

extern "C" {
    fn solvent_cad_faces(cad: *mut c_void,source: c_int,output: *mut c_int,capacity: c_int) -> c_int;
    fn solvent_cad_face_point(cad: *mut c_void,face: c_int,u: f64,v: f64,tolerance: f64,output: *mut f64) -> c_int;
}

pub(crate) struct SweepCap {
    pub parameter: f64,
    /// Model-unit pose, retained for material queries and provenance.
    pub pose: Motion,
    pub solid: c_int,
    pub faces: Vec<c_int>,
}
pub(crate) struct SweepCaps {
    pub source: c_int,
    /// Both endpoints retain every native source face. No face is selected as
    /// exposed without trimming and a query against the complete swept material.
    pub endpoints: [SweepCap;2],
}
pub(crate) struct FacePoint {
    /// Native millimetres, with the face's oriented unit normal.
    pub position: [f64;3],
    pub normal: [f64;3],
    pub on_trim: bool,
}

impl Session {
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

    pub(crate) fn sweep_caps(&self,sk: &Sketch,solid: usize) -> Result<SweepCaps,String> {
        gcs_core::solid::validate(sk,solid)?;
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[solid].def else {
            return Err("endpoint construction requires a continuous swept solid".into());
        };
        let recipe = cad::recipe(sk,*source as usize)?;
        let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
        let motion = Family::read(sk,*motion as usize)?;
        let poses = [motion.at(from.value)?,motion.at(to.value)?];
        let source = self.construct(&recipe)?;
        let endpoint = |parameter,pose| -> Result<SweepCap,String> {
            let matrix = cad::placement_matrix(pose,scale);
            let solid = self.result(unsafe { solvent_cad_transform(self.0,source,matrix.as_ptr()) })?;
            let faces = self.faces(solid)?;
            if faces.is_empty() { return Err("native endpoint has no boundary faces".into()); }
            Ok(SweepCap {parameter,pose,solid,faces})
        };
        Ok(SweepCaps {source,endpoints:[endpoint(from.value,poses[0])?,endpoint(to.value,poses[1])?]})
    }
}
