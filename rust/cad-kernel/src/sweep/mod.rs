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
pub mod cutter;
pub mod sheet;
pub mod helical;
pub mod admit;
pub mod cells;

use refusal::Stage;

/// How the pipeline speaks: a line of progress, and the stage just passed.
pub struct Say<'a> { pub stage: &'a (dyn Fn(&str)+Sync),pub mark: &'a (dyn Fn(Stage)+Sync) }

/// How far from a revolution's axis an endpoint is read as on it, wherever a revolved tool's
/// faces, fields or contacts are read (the tolerance a revolved snapshot takes).
pub const AXIS_TOLERANCE: f64 = 1e-10;
