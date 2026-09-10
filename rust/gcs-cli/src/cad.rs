//! Optional native CAD support; the core and WebAssembly have no OCCT dependency.
#[cfg(feature="occt")]
mod native;
#[cfg(feature="manifold")]
pub mod manifold;
#[cfg(all(feature="manifold",feature="occt"))]
pub mod mesh_sweep;

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
