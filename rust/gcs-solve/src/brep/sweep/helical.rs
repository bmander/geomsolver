//! A screw cut's sheet is the kernel's (`brep::swept::helical`, #721 F2b): here the cut's name,
//! scale, motion and roll are handed to it.
use super::sheet::{Fitted,SweptCut};
use super::Say;
use crate::solid::constant_twist::Characteristic;
use crate::solid::contact_trace::Inside;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

type V = [f64;3];

/// The sheet of a screw cut whose characteristic `found` walked (`brep::swept::helical::helical_sheet`).
pub fn helical_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let path = cut.family.path().at(Stage::Reach)?;
    say.kernel(|say| crate::brep::swept::helical::helical_sheet(&cut.name,cut.scale,path,cut.limits,found,heights,inside,near,tolerance,say))
        .map_err(ExportRefusal::from)
}
