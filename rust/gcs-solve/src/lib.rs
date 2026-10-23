//! gcs-solve — the geometric constraint solver, without the language.
//!
//! Numerical code indexes; that is what the algorithms are written in and what the papers show.
//! Rewriting the inner loops as iterator chains would obscure the linear algebra, so the
//! index-based lints are off here rather than worked around one loop at a time.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::type_complexity)]
//!
//! The model, its constraints and kernels, the solvers, structural diagnosis, cluster
//! decomposition, witness analysis and solution management, and what the model evaluates: the
//! curves, the solids, the motions and the B-rep (#409, step 7).  Nothing here knows Solvent:
//! the language is `gcs-core`, built on this crate, and the compiler holds the line — this crate
//! cannot name a module of the one that depends on it.
pub mod amd;
pub mod beams;
pub mod bodies;
pub mod brep;
pub mod cgraph;
pub mod cholesky;
pub mod clear;
pub mod clock;
pub mod complex;
pub mod constraints;
pub mod contact_rate;
pub mod csg;
pub mod curve;
pub mod decompose;
pub mod deformed;
pub mod delaunay;
pub mod eigen;
pub mod diagnose;
pub mod envelope;
pub mod expr;
pub mod extremal;
pub mod fmath;
pub mod follow;
pub mod generate;
pub mod graft;
pub mod graph;
pub mod homotopy;
pub mod integral;
pub mod interior;
pub mod interval;
pub mod json;
pub mod kernels;
pub mod linalg;
pub mod lines;
pub mod locus;
pub mod mesh;
pub mod model;
pub mod motion;
pub mod newton;
pub mod ode;
pub mod par;
pub mod plane;
pub mod prisms;
pub mod progress;
pub mod rates;
pub mod rng;
pub mod roots;
pub mod semantics;
pub mod solid;
pub mod solve;
pub mod space;
pub mod sparse;
pub mod system;
pub mod tape;
pub mod taylor;
pub mod topology;
pub mod traced;
pub mod units;
pub mod variational;
pub mod vocab;
pub mod witness;

// Internal to the engine, and public only so the language's renderer, seams and patches (in
// `gcs-core`) reach the same bounding-volume tree and intersections.
#[doc(hidden)]
pub mod bvh;
#[doc(hidden)]
pub mod intersection;
pub mod measured;
