//! A screw cut's sheet is the kernel's (`brep::swept::helical`, #721 F2b): here the cut's name,
//! scale, motion and roll are handed to it.
use super::sheet::{Fitted,SweptCut};
use super::Say;
use crate::solid::constant_twist::Characteristic;
use crate::brep::swept::trace::Inside;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

type V = [f64;3];

/// The sheet of a screw cut whose characteristic `found` walked (`brep::swept::helical::helical_sheet`):
/// where it parts at concave corners into stretches (#721 F1d), each stretch's sheet, the others'
/// carried beside the first's (`Fitted::parted`), all of which the blank is split by at once and its
/// cells judged, so that each is trimmed against the next where they cross.
pub fn helical_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let path = cut.family.path().at(Stage::Reach)?;
    let one = |found: &Characteristic| say.kernel(|say| crate::brep::swept::helical::helical_sheet(&cut.name,cut.scale,path,cut.limits,found,
        heights,inside,near,tolerance,say)).map_err(ExportRefusal::from);
    let mut fitted = one(found)?;
    for other in &found.others {
        let more = one(other)?;
        fitted.error = fitted.error.max(more.error);
        fitted.parted.push(more.faces);
    }
    Ok(fitted)
}
