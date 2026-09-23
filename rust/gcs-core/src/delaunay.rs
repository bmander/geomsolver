//! Delaunay machinery for meshing a material field by Delaunay refinement
//! (docs/field-meshing.md): exact expansions, filtered orientation and power predicates, and
//! the incremental regular (weighted Delaunay) triangulation that protecting balls need.
pub mod expansion;
pub mod predicates;
pub mod regular;

pub use predicates::{orient,power,Weighted};
pub use regular::{Regular,Inserted,Tet,NONE,spatial_order};
