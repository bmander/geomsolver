//! Sweeps of constant twist (docs/generating-sweeps.md): a tool carried by a screw, or by either
//! of its limits, a turn about one axis and a slide along one line (#684). The characteristic, its
//! walk and the class's rows S1–S4 asked of it are the kernel's (`brep::swept::constant_twist`,
//! #721 F2b); here the solver hands the kernel the tool and the motion it read off the drawing
//! (`SweepContacts`, its motion lowered to the kernel's trajectory) and says the row a walk fails
//! as its admission's (`Condition`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::Condition;
use super::SweepContacts;
pub use crate::brep::swept::constant_twist::{Ask,Characteristic,Node,Row,Sweep,crossing,once_round,outward};
use crate::brep::swept::constant_twist::NO_TWIST;

type V = [f64;3];

/// A row of the class that fails, what was seen and where: in the sweep's frame, at a time the
/// tool point is in the blank.
pub(super) type Failure = (Condition,String,Option<V>);

impl From<Row> for Condition {
    fn from(row: Row) -> Condition {
        match row {
            Row::Motion => Condition::Motion,Row::Reach => Condition::Reach,Row::Regular => Condition::Regular,
            Row::Once => Condition::Once,Row::Edgewise => Condition::Edgewise,Row::Lead => Condition::Lead,
            Row::Clearance => Condition::Clearance,Row::Tool => Condition::Tool,
        }
    }
}

/// Walk the characteristic of `contacts`' tool under its motion through the blank `ask.inside`
/// reads (`Characteristic::walk`), or refuse with the row it fails. A motion that could not be
/// lowered to the kernel's trajectory is no constant twist.
pub fn walk(contacts: &SweepContacts,ask: &Ask) -> Result<Characteristic,Failure> {
    match contacts.motion().path() {
        Ok(path) => Characteristic::walk(&Sweep {tool:contacts.tool(),path,whole:contacts.whole()},ask)
            .map_err(|(row,m,w)| (Condition::from(row),m,w)),
        Err(_) => Err((Condition::Motion,NO_TWIST.to_string(),None)),
    }
}
