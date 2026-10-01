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
pub mod brep;
pub mod cgraph;
pub mod clear;
pub mod complex;
pub mod constraints;
pub mod curve;
pub mod decompose;
pub mod delaunay;
pub mod diagnose;
pub mod drawing;
pub mod edit;
pub mod envelope;
mod intersection;
pub mod patch;
pub mod seam;
pub mod motion;
pub mod examples;
pub mod expr;
pub mod fdcheck;
pub mod fixtures;
pub mod flatten;
pub mod gltf;
pub mod graph;
pub mod hidden;
pub mod homotopy;
pub mod io;
pub mod interval;
pub mod json;
pub mod kernels;
pub mod library;
pub mod linalg;
pub mod locus;
pub mod measure;
pub mod model;
pub mod modules;
pub mod newton;
pub mod overview;
pub mod par;
pub mod clock;
pub mod fmath;
pub mod plane;
pub mod program;
pub mod report;
pub mod renderer;
pub mod rng;
pub mod solid;
pub mod roots;
pub mod space;
pub mod solve;
pub mod sparse;
pub mod csg;
pub mod mesh;
pub mod style;
pub mod semantics;
pub mod svg;
pub mod syntax;
pub mod system;
pub mod tape;
pub mod topology;
pub mod vertex;
pub mod edge;
pub mod spatial_face;
pub mod units;
pub mod witness;

pub mod ir;
