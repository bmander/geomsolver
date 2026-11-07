//! The region a planar tool sweeps (#606), read off a drawing: the region itself is the kernel's
//! (`brep::swept::region`, #721 F2c), and so is the section its tool is read as, from the tool's
//! B-rep (`brep::swept::subtraction`, #721 F1c), and the words its refusals are said in
//! (`region::Refusal::words`).
use crate::brep::swept::region as kernel;

pub use kernel::{keeps,parity,Piece,Plane,Section,Setting,SweptRegion,PER_DEGREE};
