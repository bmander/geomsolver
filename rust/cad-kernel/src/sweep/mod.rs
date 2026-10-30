//! Solid sweeps: what a tool passes through under a rigid motion, as exact geometry
//! (docs/generating-sweeps.md in the solver this kernel was carved from).  A motion and its rate
//! (`envelope`), the families of rigid motions a sweep is read under (`motion`).
//!
//! Nothing here reads a document: a caller builds the tool and the motion from plain data, or
//! reads them through the `Source` traits its own document implements.
pub mod envelope;
pub mod motion;
pub mod condition;
pub mod refusal;
pub mod sector;
pub mod surface;
pub mod tool;
pub mod trace;
pub mod characteristic;

/// How far from a revolution's axis an endpoint is read as on it, wherever a revolved tool's
/// faces, fields or contacts are read (the tolerance a revolved snapshot takes).
pub const AXIS_TOLERANCE: f64 = 1e-10;
