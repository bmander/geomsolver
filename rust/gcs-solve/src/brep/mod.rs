//! The B-rep kernel (`gcs-brep`), and what the solver builds with it: the generating class's
//! swept bodies (`sweep`) and the export of a sketch's solids (`export`).
pub use gcs_brep::*;
pub mod sweep;
pub mod export;
