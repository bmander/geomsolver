//! Delaunay machinery for meshing an implicit domain's boundary by Delaunay refinement
//! (docs/field-meshing.md): exact expansions, filtered orientation and power predicates, and
//! the incremental regular (weighted Delaunay) triangulation that protecting balls need.
pub mod expansion;
pub mod predicates;
pub mod regular;
pub mod refine;

pub use predicates::{orient,power,Weighted};
pub use regular::{Regular,Inserted,Tet,NONE,spatial_order};
