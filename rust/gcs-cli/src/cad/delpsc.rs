//! A body's boundary meshed from its material field by Delaunay refinement with protected
//! features (CGAL's Mesh_3; `backend/delpsc.cpp`). An experiment behind the `cgal` feature,
//! `--stl-backend cgal`: the field is asked only for signs, and no sheet, fit or kernel
//! Boolean enters. Delaunay refinement needs the domain's sharp curves given as features: here
//! the static blank's sharp edges where material survives, and the curves where the swept
//! boundary meets each blank face, both found from the material field alone (`features`).
use gcs_core::solid::MaterialField;
use std::ffi::{CString,c_char,c_int,c_void};

unsafe extern "C" {
    fn solvent_cgal_mesh(field: extern "C" fn(*mut c_void,*const f64) -> f64,context: *mut c_void,center: *const f64,
        radius: f64,points: *const f64,counts: *const c_int,curves: c_int,sizes: *const f64,scale: f64,
        path: *const c_char,error: *mut c_char,capacity: c_int) -> c_int;
}

/// What the callback reads the field with, and what it has seen.
struct Oracle { field: MaterialField,queries: usize,started: std::time::Instant,shown: f64,spent: std::time::Duration }

/// The side of the boundary a point lies on, as Mesh_3 asks it: the field's plain floating-point
/// side (`MaterialField::side`), since a mesher needs a side and not a proof. The certified
/// probe judges the finished mesh.
extern "C" fn sign(context: *mut c_void,p: *const f64) -> f64 {
    let oracle = unsafe { &mut *(context as *mut Oracle) };
    let p = unsafe { [*p,*p.add(1),*p.add(2)] };
    oracle.queries += 1;
    let seen = oracle.started.elapsed().as_secs_f64();
    if seen > oracle.shown+3. {
        oracle.shown = seen;
        eprintln!("solventc: [{seen:7.1} s] Mesh_3: {} field queries, {:.1} s of them in the field",
            oracle.queries,oracle.spent.as_secs_f64());
    }
    let clock = std::time::Instant::now();
    let value = oracle.field.side(p);
    oracle.spent += clock.elapsed();
    if value.is_finite() { value } else { 1. }
}

/// Mesh `solid` from its field and write the STL at `path` (millimetres). Sizes default to a
/// fraction of the material's support and can be set for the experiment:
/// `SOLVENT_CGAL_FACET` (facet size, mm), `SOLVENT_CGAL_DISTANCE` (facet distance, mm),
/// `SOLVENT_CGAL_ANGLE` (degrees), `SOLVENT_CGAL_EDGE` (feature edge size, mm),
/// `SOLVENT_CGAL_BISECTION` (where a crossing is placed, relative to the bounding sphere).
pub fn export(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    use super::field_mesh::setting;
    let super::field_mesh::Region {field,scale,center,radius,diagonal} = super::field_mesh::region(sk,solid)?;
    let facet = setting("SOLVENT_CGAL_FACET",diagonal*scale/60.)/scale;
    let sizes = [facet,setting("SOLVENT_CGAL_DISTANCE",0.005)/scale,setting("SOLVENT_CGAL_ANGLE",25.),
        setting("SOLVENT_CGAL_EDGE",facet*scale)/scale,setting("SOLVENT_CGAL_BISECTION",1e-5)];
    let started = std::time::Instant::now();
    let mut oracle = Oracle {field:field.clone(),queries:0,started,shown:0.,spent:Default::default()};
    let name = CString::new(path).map_err(|e| e.to_string())?;
    let mut error = vec![0 as c_char;1024];
    let lines = super::field_mesh::features(sk,solid,&field,scale,diagonal)?;
    eprintln!("solventc: [{:7.1} s] Mesh_3: {} feature curves of {} points",started.elapsed().as_secs_f64(),lines.len(),
        lines.iter().map(Vec::len).sum::<usize>());
    let points: Vec<f64> = lines.iter().flatten().flatten().copied().collect();
    let counts: Vec<c_int> = lines.iter().map(|l| l.len() as c_int).collect();
    let count = unsafe {
        solvent_cgal_mesh(sign,&mut oracle as *mut Oracle as *mut c_void,center.as_ptr(),radius,points.as_ptr(),
            counts.as_ptr(),counts.len() as c_int,sizes.as_ptr(),scale,name.as_ptr(),error.as_mut_ptr(),error.len() as c_int)
    };
    if count < 0 {
        let message = unsafe { std::ffi::CStr::from_ptr(error.as_ptr()) }.to_string_lossy().into_owned();
        return Err(format!("Mesh_3 failed: {message}"));
    }
    let evaluations = gcs_core::solid::SIDE_EVALUATIONS.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!("solventc: Mesh_3: {count} triangles from {} field queries ({:.1} s in the field, {evaluations} sweep evaluations \
        in all) in {:?}, facet size {:.4} mm, distance {:.4} mm",oracle.queries,oracle.spent.as_secs_f64(),started.elapsed(),
        sizes[0]*scale,sizes[1]*scale);
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    gcs_core::mesh::stl_shells(&bytes).map_err(|e| format!("the Mesh_3 STL fails its shell check: {e}"))?;
    super::mark("stl");
    super::native::field_agreement(sk,solid,&bytes)?;
    super::mark("written");
    Ok(())
}
