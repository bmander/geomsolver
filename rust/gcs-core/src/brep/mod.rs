//! A boundary-representation kernel of our own (docs/rust-kernel-plan.md): exact analytic faces
//! and edges with their topology, built from the CAD recipe the native host is handed, measured
//! without meshing, and — as the plan's rungs land — combined, meshed and written as STEP.
pub mod geom;
pub mod mesh;
pub mod nurbs;
pub mod topo;
pub mod boolean;
pub mod build;
pub mod props;
pub mod step;
pub mod step_check;
pub mod query;
pub mod ssi;
pub mod recipe;
pub mod json;
pub mod pattern;
pub mod planar;
pub mod corners;
pub mod section;
pub mod sweep;
pub mod export;
pub mod fillet;

use crate::json::Json;
use std::sync::OnceLock;

/// Three numbers of a recipe or a dump (`[x, y, z]`).
pub(crate) fn vec3(j: &Json) -> geom::V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }

/// `j`'s field `k`, or an error naming it under `what` (`recipe`, `profile`, `brep json`).
pub(crate) fn field<'a>(j: &'a Json,k: &str,what: &str) -> Result<&'a Json,String> {
    j.get(k).ok_or(format!("{what}: no `{k}`"))
}

/// `SOLVENT_BREP_DEBUG` is set: the kernel narrates its Booleans and traces (read once).
pub(crate) fn debug() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_BREP_DEBUG").is_some())
}

/// `SOLVENT_BREP_TIME` is set: the kernel times its slow Boolean steps (read once).
pub(crate) fn timing() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_BREP_TIME").is_some())
}
