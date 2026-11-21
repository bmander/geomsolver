//! A solid walked from its shells down into a `Brep`: faces, their bounds, each bound's oriented
//! edges, the edges and the vertices, each read once however often it is used, so the boundary's
//! identity is the file's own and never found by proximity.
use super::context::Found;
use super::{Given,Refusal};
use crate::step_file::File;

/// The solid `found` names, read; refused naming the entity that could not be.
pub fn solid(_file: &File,found: Found) -> Result<Given,Refusal> {
    Err(Refusal::unsupported(found.solid,"a solid is not read yet"))
}
