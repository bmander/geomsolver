//! Optional native CAD support; the core and WebAssembly have no OCCT dependency.
#[cfg(feature="occt")]
mod native;

pub fn step(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    #[cfg(feature="occt")]
    { return native::step(sk,solid,path); }
    #[cfg(not(feature="occt"))]
    {
        let _ = (sk,solid,path);
        Err("STEP export requires native OCCT support; build with `make solventc OCCT=1` \
            or Cargo's `--features occt`".into())
    }
}
