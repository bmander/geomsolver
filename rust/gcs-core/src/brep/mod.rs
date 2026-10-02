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
pub mod section;
pub mod sweep;
pub mod export;
