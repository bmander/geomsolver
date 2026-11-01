//! The region a planar tool sweeps (#606), read off a drawing: the region itself is the kernel's
//! (`brep::swept::region`, #721 F2c). What is here is the section's material, its members'
//! `PlanarField`s joined as their tree joins them (`section`), and the kernel's refusals said in
//! the drawing's words, by row (`said`).
use crate::brep::swept::planar::Region;
use crate::brep::swept::region::{self as kernel,Seen};
use crate::solid::admission::Condition;
use crate::solid::PlanarField;
use crate::solid::surface::Edge;
use std::sync::Arc;

pub use kernel::{keeps,parity,Piece,Plane,Section,Setting,SweptRegion,PER_DEGREE};

type P = [f64;2];

/// Why no region: the row, what was found, and a point of the plane where.
#[derive(Clone,Debug)]
pub struct Refusal { pub row: Condition,pub message: String,pub witness: Option<P> }

/// How a section's members combine: a member, the union of some, or what they hold in common.
#[derive(Clone,Debug)]
pub enum Tree { Member(usize),Union(Vec<Tree>),Common(Vec<Tree>) }

/// The tool's section at one height of `members` (each member's loops walked with its material on
/// the left, none where it does not reach the height) joined as `tree` says: its field the members'
/// `PlanarField`s joined as the tree joins them, refused where a member's loops are no field.
pub fn section(members: Vec<Option<Vec<Vec<Edge>>>>,tree: &Tree) -> Result<Section,String> {
    fn join(members: &[Option<Vec<Vec<Edge>>>],t: &Tree) -> Result<Option<PlanarField>,String> {
        let (ts,common) = match t {
            Tree::Member(m) => return members[*m].as_ref().map(|l| PlanarField::from_loops(l,0.)).transpose(),
            Tree::Union(ts) => (ts,false),
            Tree::Common(ts) => (ts,true),
        };
        let mut out: Option<PlanarField> = None;
        for t in ts {
            // what is held in common with nothing is nothing
            let Some(f) = join(members,t)? else { if common { return Ok(None) } else { continue } };
            out = Some(match out {
                None => f,
                Some(o) => if common { o.intersection(f) } else { o.union(f) }.map_err(|e| format!("{e:?}"))?,
            });
        }
        Ok(out)
    }
    let field = join(&members,tree)?;
    Ok(Section::new(members,field.map(|f| Arc::new(f) as Arc<dyn Region>)))
}

/// The kernel's refusal of the region the tool sweeps under the motion `name`, said in the
/// drawing's words.
pub fn said(r: kernel::Refusal,name: &str) -> Refusal {
    let row = match r.seen.row() {
        kernel::Row::Plane => Condition::Plane,
        kernel::Row::Period => Condition::Period,
        kernel::Row::Kept => Condition::Kept,
    };
    let message = match r.seen {
        Seen::Motion(e) => e.msg,
        Seen::LeavesPlane => format!("`{name}` carries the tool out of its plane, or turns it over"),
        Seen::Unread => "the motion cannot be read".into(),
        Seen::Backwards => "the roll increases".into(),
        Seen::NotInterval => "the roll is no interval".into(),
        Seen::Folds => "the swept region's boundary folds back on itself".into(),
        Seen::EndUnread => "a kept stretch cannot be read at its end".into(),
        Seen::LeavesBox => "the swept region's boundary leaves the box and never comes back into it".into(),
        Seen::Open => "the swept region's boundary does not close: a kept stretch meets no other where it ends".into(),
        Seen::Branches => "the swept region's boundary branches".into(),
        Seen::Crosses => "the swept region's boundary crosses itself".into(),
    };
    Refusal {row,message,witness:r.witness}
}
