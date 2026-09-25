//! Optional native CAD support; the core and WebAssembly have no OCCT dependency.
#[cfg(feature="occt")]
mod native;
#[cfg(feature="occt")]
pub mod field_mesh;

/// A completed stage for a harness (`SOLVENT_STAGE_TRACE`); nothing without native support.
pub fn mark(key: &str) {
    #[cfg(feature="occt")]
    native::sweep_boundary::mark(key);
    let _ = key;
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
