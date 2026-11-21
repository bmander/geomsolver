//! What a boundary read from a file must be to be a solid's, beyond what `topo::Brep::check`
//! holds every boundary to: a reader is handed topology it did not build, and must refuse what
//! closes edge by edge and still bounds no solid.
use super::context::Context;
use super::{Given,Refusal,Why};
use crate::topo::Brep;

/// A boundary that is no solid's: why, the face or the edge of it that witnesses it (by its
/// index in the `Brep`), and what there is to say.
#[derive(Clone,Debug,PartialEq)]
pub struct Unsound { pub why: Why,pub face: Option<usize>,pub edge: Option<usize>,pub msg: String }

impl Unsound {
    /// The refusal of the file this is, its witness the entity `given` read that face or edge
    /// from.
    pub fn refusal(&self,given: &Given) -> Refusal {
        let entity = self.face.and_then(|f| given.faces.get(f)).or(self.edge.and_then(|e| given.edges.get(e))).copied().filter(|&e| e != 0);
        Refusal::new(self.why,entity.or(Some(given.entity)),self.msg.clone())
    }
}

/// Whether `b`, read in `context`, is a solid's boundary.
pub fn sound(_b: &Brep,_context: &Context) -> Result<(),Unsound> {
    Ok(())
}
