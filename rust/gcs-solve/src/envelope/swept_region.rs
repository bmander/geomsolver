//! The region a planar tool sweeps (#606), read off a drawing: the region itself is the kernel's
//! (`brep::swept::region`, #721 F2c), and so is the section its tool is read as, from the tool's
//! B-rep (`brep::swept::subtraction`, #721 F1c). What is here is the kernel's refusals said in the
//! drawing's rows (`said`).
use crate::brep::swept::region as kernel;
use crate::solid::admission::Condition;

pub use kernel::{keeps,parity,Piece,Plane,Section,Setting,SweptRegion,PER_DEGREE};

type P = [f64;2];

/// Why no region: the row, what was found, and a point of the plane where.
#[derive(Clone,Debug)]
pub struct Refusal { pub row: Condition,pub message: String,pub witness: Option<P> }

/// The kernel's refusal of the region the tool sweeps under the motion `name`, said in the
/// drawing's words.
pub fn said(r: kernel::Refusal,name: &str) -> Refusal {
    let row = match r.seen.row() {
        kernel::Row::Plane => Condition::Plane,
        kernel::Row::Period => Condition::Period,
        kernel::Row::Kept => Condition::Kept,
    };
    let message = r.words(name);
    Refusal {row,message,witness:r.witness}
}
