//! Optional native CAD support; the core and WebAssembly have no OCCT dependency.
#[cfg(feature="occt")]
mod native;
#[cfg(feature="manifold")]
pub mod manifold;
#[cfg(all(feature="manifold",feature="occt"))]
pub mod mesh_sweep;

/// How much the mesh path reports while it works (`--verbose`); without it
/// there is nothing to report and the flag is accepted and ignored.
pub fn set_verbosity(level: u8) {
    #[cfg(all(feature="manifold",feature="occt"))]
    mesh_sweep::VERBOSITY.store(level,std::sync::atomic::Ordering::Relaxed);
    let _ = level;
}

/// A body with swept cuts is judged against its own material field before anything is
/// written, whichever backend built it: probes a little inside and outside the triangles of
/// its STL (`gcs_core::solid::agreement`, millimetres), the check the traced-sheet
/// arrangement failed while its volume and its shell passed. A disagreement refuses the export.
#[cfg_attr(not(feature="occt"),allow(dead_code))]
pub fn field_agreement(sk: &gcs_core::model::Sketch,body: usize,stl: &[u8]) -> Result<(),String> {
    use gcs_core::solid::{agreement,MaterialField};
    let scale = sk.units.length.ok_or("CAD export requires an explicit model length unit")?.1;
    let started = std::time::Instant::now();
    let (vertices,triangles) = agreement::stl_triangles(stl,scale)?;
    let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
    let options = agreement::Options {offset:0.1/scale,confirm:0.025/scale,value_tolerance:0.02/scale,..Default::default()};
    let total = (triangles.len()+(triangles.len()/options.triangles.max(1)).max(1)-1)/(triangles.len()/options.triangles.max(1)).max(1);
    eprintln!("solventc: probing {total} of {} triangles against the material field",triangles.len());
    let mut shown = 0;
    let report = agreement::of_triangles_observed(&vertices,&triangles,&mut material,&options,&mut |r| {
        if r.probed_triangles >= shown+total.div_ceil(10) {
            shown = r.probed_triangles;
            eprintln!("solventc:   {} of {total} triangles probed, {} disagree ({:?})",r.probed_triangles,r.disagreements.len(),started.elapsed());
        }
    })?;
    eprintln!("solventc: field agreement: {} of {} triangles probed {:.2} mm off each side, {} probes unresolved, \
        {} withdrawn beside another face, {} disagree ({:?})",report.probed_triangles,report.triangles,
        options.offset*scale,report.unresolved,report.withdrawn,report.disagreements.len(),started.elapsed());
    if report.agrees() { return Ok(()); }
    for d in report.disagreements.iter().take(10) {
        eprintln!("solventc:   {} the mesh at ({:.4}, {:.4}, {:.4}) the field reads [{:.4}, {:.4}]",
            if d.inside_mesh { "inside" } else { "outside" },d.point[0]*scale,d.point[1]*scale,d.point[2]*scale,
            d.field[0]*scale,d.field[1]*scale);
    }
    Err(format!("`{}`: the exported surface disagrees with the material field at {} of {} probes; nothing was written",
        sk.solids[body].name,report.disagreements.len(),report.probes))
}

/// Whether the mesh path applies: the solid cuts continuous sweeps and both
/// native features are built in.
pub fn swept_mesh_applies(sk: &gcs_core::model::Sketch,solid: usize) -> bool {
    cfg!(all(feature="manifold",feature="occt"))
        && gcs_core::solid::cad::recipe_static(sk,solid).map(|r| !r.sweeps.is_empty()).unwrap_or(false)
}

/// STL of a body with swept cuts by mesh arrangement and material
/// classification, written through the core's checked STL writer.
pub fn export_swept_stl(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    #[cfg(all(feature="manifold",feature="occt"))]
    {
        let session = native::Session::new()?;
        let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
        let sheets = |swept: usize,inside: &dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>| {
            let sheet = native::sweep_boundary::swept_sheet_grid(&session,sk,swept,inside)?;
            Ok(vec![mesh_sweep::SheetGrid {points:sheet.points.iter().map(|p| p.map(|v| v/scale)).collect(),
                normals:sheet.normals.clone(),times:(0..sheet.columns).map(|c| c as f64).collect(),rows:sheet.rows,columns:sheet.columns,closed_rows:false}])
        };
        let (vertices,triangles) = mesh_sweep::construct(sk,solid,&sheets)?;
        let bytes = mesh_sweep::stl(&vertices,&triangles,&sk.solids[solid].name)?;
        field_agreement(sk,solid,&bytes)?;
        std::fs::write(path,bytes).map_err(|e| format!("{path}: {e}"))?;
        return Ok(());
    }
    #[cfg(not(all(feature="manifold",feature="occt")))]
    {
        let _ = (sk,solid,path);
        Err("swept STL export needs the `manifold` and `occt` features".into())
    }
}

pub fn export(sk: &gcs_core::model::Sketch,solid: usize,step: Option<&str>,stl: Option<&str>) -> Result<(),String> {
    #[cfg(feature="occt")]
    { return native::export(sk,solid,step,stl); }
    #[cfg(not(feature="occt"))]
    {
        let _ = (sk,solid,step,stl);
        Err("CAD export requires native OCCT support; build with `make solventc OCCT=1` \
            or Cargo's `--features occt`".into())
    }
}
