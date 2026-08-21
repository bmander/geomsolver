//! gcs — geometric constraint solver.
//!
//! Numerical code indexes; that is what the algorithms are written in and what the papers show.
//! Rewriting the inner loops as iterator chains would obscure the linear algebra, so the
//! index-based lints are off here rather than worked around one loop at a time.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::type_complexity)]
//!
//! One implementation of everything: the model, the numerics, structural diagnosis, cluster
//! decomposition, witness analysis and solution management.  The Python and TypeScript packages
//! are thin bindings over the flat C ABI in `gcs-ffi`; there is no second copy of any algorithm.
pub mod callout;
pub mod cgraph;
pub mod complex;
pub mod constraints;
pub mod decompose;
pub mod diagnose;
pub mod examples;
pub mod fdcheck;
pub mod graph;
pub mod homotopy;
pub mod io;
pub mod json;
pub mod kernels;
pub mod linalg;
pub mod model;
pub mod newton;
pub mod report;
pub mod rng;
pub mod solve;
pub mod sparse;
pub mod system;
pub mod witness;
