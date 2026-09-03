//! gcs — geometric constraint solver.
//!
//! Numerical code indexes; that is what the algorithms are written in and what the papers show.
//! Rewriting the inner loops as iterator chains would obscure the linear algebra, so the
//! index-based lints are off here rather than worked around one loop at a time.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::type_complexity)]
//!
//! One implementation of everything: the model, the numerics, structural diagnosis, cluster
//! decomposition, witness analysis and solution management.  The TypeScript package is a thin
//! binding over the flat C ABI in `gcs-ffi`; there is no second copy of any algorithm.
pub mod callout;
pub mod cgraph;
pub mod clear;
pub mod complex;
pub mod constraints;
pub mod curve;
pub mod decompose;
pub mod diagnose;
pub mod edit;
pub mod examples;
pub mod expr;
pub mod fdcheck;
pub mod fixtures;
pub mod flatten;
pub mod graph;
pub mod hidden;
pub mod homotopy;
pub mod io;
pub mod json;
pub mod kernels;
pub mod library;
pub mod linalg;
pub mod locus;
pub mod model;
pub mod modules;
pub mod newton;
pub mod overview;
pub mod plane;
pub mod program;
pub mod report;
pub mod rng;
pub mod solid;
pub mod solve;
pub mod sparse;
pub mod csg;
pub mod mesh;
pub mod style;
pub mod svg;
pub mod syntax;
pub mod system;
pub mod tape;
pub mod units;
pub mod witness;
