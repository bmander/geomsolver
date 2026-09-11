//! The certified boundary of a swept volume: a closed triangle mesh of the
//! material a tool sweeps under a motion, every triangle judged against the
//! sweep's own one-Lipschitz field. The traced analytic sheets seed the
//! construction and never stand as its truth: a vertex is kept only where the
//! field brackets the boundary about it, a triangle only where the field reads
//! material a probe distance inside it and exterior the same distance outside.
//!
//! Two facts of the field's contract carry everything here. A strict sign
//! change between two points puts a boundary point on the segment between them,
//! which needs no gradient. And the contract gives no gradient, so no distance
//! is ever read off a value's magnitude, and an enclosure containing zero is
//! never a sign. See `docs/swept-boundary.md`.
pub mod judge;
pub mod seeds;
pub mod project;
pub mod trim;
pub mod certify;

pub use certify::{Certificate,Failure,certify};
pub use judge::{FieldJudge,JudgeError,Projection,QueryStats,Sign};
pub use project::{Label,Labelled,directions,label_patch,label_sheets};
pub use seeds::seeds;
pub use trim::{KeptMesh,boundary_loops,kept_triangles};

/// Controls of the construction. Lengths are in the model's own units.
#[derive(Clone,Copy,Debug)]
pub struct SweptBoundaryOptions {
    /// The chord error the sheets are traced and simplified to.
    pub sagitta: f64,
    /// The target spacing of a sheet's columns along the motion.
    pub spacing: f64,
    /// How far a vertex may be from the boundary once judged: a quarter of the
    /// sagitta unless given.
    pub vertex_tolerance: Option<f64>,
    /// How far inside and outside every triangle the field is asked to agree:
    /// twice the sagitta unless given.
    pub probe_distance: Option<f64>,
    /// Roll evaluations a query near the boundary may spend before refusing.
    pub near_budget: usize,
    /// Roll evaluations a query a probe distance from the boundary may spend.
    pub far_budget: usize,
    /// Motion poses cached per sweep evaluator.
    pub cached_poses: usize,
}

impl Default for SweptBoundaryOptions {
    fn default() -> Self {
        Self {sagitta:0.02,spacing:0.5,vertex_tolerance:None,probe_distance:None,near_budget:4000,far_budget:1000,cached_poses:4096}
    }
}

impl SweptBoundaryOptions {
    pub fn vertex_tolerance(&self) -> f64 { self.vertex_tolerance.unwrap_or(self.sagitta/4.) }
    pub fn probe_distance(&self) -> f64 { self.probe_distance.unwrap_or(2.*self.sagitta) }
}
