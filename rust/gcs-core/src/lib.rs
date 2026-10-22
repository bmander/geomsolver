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
//!
//! The solver and what the model evaluates are `gcs-solve`, which knows no Solvent (#409, step
//! 7); this crate is the language and what is built on it.  The solver's modules are re-exported
//! here under their own names, so `crate::model` and `gcs_core::model` name what they always did.
pub use gcs_solve::{
    amd, beams, brep, cgraph, cholesky, clear, clock, complex, constraints, contact_rate, csg,
    curve, decompose, delaunay, diagnose, eigen, envelope, expr, extremal, fmath, follow, generate,
    graft, graph, homotopy, integral, interior, interval, json, kernels, linalg, lines, locus, mesh,
    model, motion, newton, ode, par, plane, progress, rates, rng, roots, semantics, solid, solve,
    space, sparse, system, tape, taylor, topology, traced, units, variational, vocab, witness,
};
pub(crate) use gcs_solve::{bvh, intersection};

pub mod boundary;
pub mod callout;
pub mod drawing;
pub mod edge;
pub mod edit;
pub mod examples;
pub mod fdcheck;
pub mod fixtures;
pub mod flatten;
pub mod followed;
pub mod gltf;
pub mod hidden;
pub mod io;
pub mod ir;
pub mod library;
pub mod lowering;
pub mod modules;
pub mod overview;
pub mod parts;
pub mod patch;
pub mod program;
pub mod renderer;
pub mod report;
pub mod seam;
pub mod spatial_face;
pub mod style;
pub mod svg;
pub mod syntax;
pub mod vertex;
