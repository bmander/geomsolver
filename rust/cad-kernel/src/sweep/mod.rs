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
