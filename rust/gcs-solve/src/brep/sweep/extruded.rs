//! A prism's sheet under a motion that carries its profile's plane within itself (issue #70, part
//! 1), built from the planar envelope and not traced: the kernel's (`brep::swept::extruded`, its
//! cuts read in closed form, `brep::swept::planar_cut`, #721 F2c). What is here is the drawing's
//! reading: the cut's cutter, its motion in the profile's plane (`Family::in_frame`), and which
//! corners are convex, the cutter's own reading as admission's.
use crate::brep::swept::cutter::Cutter;
use super::sheet::{Fitted,SweptCut};
use super::Say;
use crate::brep::geom::Frame;
use crate::brep::swept::trace::Inside;
use crate::solid::export::{ExportRefusal,Stage,Tolerance};

/// The sheet of `cut`, built from its profile's planar envelope, when its cutter is a prism and its
/// motion carries the profile's plane within itself; none otherwise (it is traced). `inside` asks
/// the blank (mm); `tolerance` sets the fit's bar (half of it), else `FIT`.
pub fn planar_sheet(cut: &SweptCut,inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Option<Result<Fitted,ExportRefusal>> {
    let cutter = Cutter::read(&cut.recipe).ok()?;
    let (walks,extent) = cutter.prism_profile()?;
    let frame = Frame::about(cutter.origin,cutter.axis);
    let program = cut.family.in_frame(&frame,cut.scale)?;
    // which corners are convex is the cutter's own reading, as admission's
    let convex: Vec<Vec<bool>> = match cutter.profile(0.) {
        Ok(loops) => loops.iter().map(|l| l.convex()).collect(),
        Err(e) => return Some(Err(ExportRefusal::at(Stage::Reach,e))),
    };
    let roll = cut.limits.map(f64::to_degrees);
    Some(say.kernel(|say| crate::brep::swept::extruded::sheet(&cut.name,walks,&convex,extent,frame,&program,roll,inside,tolerance,say))
        .map_err(ExportRefusal::from))
}

