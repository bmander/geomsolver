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
pub mod audit;
pub use audit::{AuditOptions,AuditError,SpatialAudit,SurfaceWitness,CoverageWitness};
pub mod adjacency;
pub mod judge;
pub mod seeds;
pub mod project;
pub mod trim;
pub mod certify;
pub mod caps;
pub mod stitch;
pub mod planar;
pub mod crease;
pub mod construct;
pub mod grazing;
pub mod hygiene;
pub mod window;

pub use caps::{Cap,CapComponent,CutMesh,End,Origin,caps};
pub use construct::{BoundaryCandidate,ConstructError,Stage,SweptBoundary,candidate,candidate_from,construct,construct_from,validate};
pub use crate::space::{Region,altitude,closest_on_triangle};
pub use certify::{Certificate,Failure,certify,triangle_normal};
pub use crease::{Rim,chains,clip_sheets,merge_creases};
pub use planar::planar_union;
pub use stitch::{collapse_needles,collapse_short_edges,dedupe,drop_doubled_slivers,rim_zip,split_at_vertices,split_where,unpinch,weld,weld_boundary_ends,zip_loops};
pub use judge::{BoundaryBracket,FieldJudge,JudgeError,Projection,QueryStats,Sign};
pub use project::{Label,Labelled,directions,label_patch,label_seeds,label_sheets,orientation};
pub use seeds::{Grazing,Seed,grazing_seeds,seeds};
pub use grazing::{GrazingFace,PlaneMotion,grazing_faces,swept_region};
pub use hygiene::{Hygiene,hygiene};
pub use window::{Near,Window,window};
pub use trim::{KeptMesh,Span,boundary_loops,centroid_kept,clip_overlaps,covered_by,kept_triangles,loop_span,retained,uncovered,without_overlaps};

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

/// Every tolerance of the construction, derived here and only here.
impl SweptBoundaryOptions {
    /// Spatial acceptance is deliberately bounded; exhausted coverage remains a refusal.
    pub fn audit(&self) -> AuditOptions { AuditOptions {tolerance:self.probe_distance(),max_cells:100000,max_depth:24} }
    /// How far a judged vertex may be from the boundary.
    pub fn vertex_tolerance(&self) -> f64 { self.vertex_tolerance.unwrap_or(self.sagitta/4.) }
    /// How far inside and outside every triangle the certificate asks the field to agree.
    pub fn probe_distance(&self) -> f64 { self.probe_distance.unwrap_or(2.*self.sagitta) }
    /// The least distance the certificate halves its probe to, for thin material: twice the
    /// vertex tolerance. A triangle with no altitude above it is a sliver.
    pub fn least_probe(&self) -> f64 { 2.*self.vertex_tolerance() }
    /// How far along its direction a vertex is searched for the boundary before it is judged
    /// inner or off the material.
    pub fn reach(&self) -> f64 { self.sagitta }
    /// Field-value refinement width, not a spatial proximity guarantee.
    pub fn judge_tolerance(&self) -> f64 { self.vertex_tolerance()/2. }
    /// How near a column point or a crossing must come to a vertex of the tool's mesh to be it.
    pub fn snap(&self) -> f64 { self.vertex_tolerance() }
    /// How far apart two rims' vertices on one crease may be to be merged, and how near a rim
    /// vertex must come to another rim's edge to split it.
    pub fn crease_merge(&self) -> (f64,f64) { (1.5*self.sagitta,4.*self.vertex_tolerance()) }
    /// How near a covering sheet must pass to a triangle to cover it.
    pub fn coverage(&self) -> f64 { 2.*self.sagitta }
    /// How near two points are to be one (a weld, a plane's membership).
    pub fn coincidence(&self) -> f64 { 1e-7 }
    /// Vertices nearer than this are merged.
    pub fn shortest_edge(&self) -> f64 { self.vertex_tolerance()/2. }
    /// How far off an edge a vertex may lie and still split it as a T-junction, and the zip's
    /// split tolerance.
    pub fn junction(&self) -> f64 { 2.*self.sagitta }
    /// The axis tolerance the sweep's solved geometry is read with.
    pub fn axis_tolerance(&self) -> f64 { 1e-10 }
}
