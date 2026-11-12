//! A prism's sheet under a motion that carries its profile's plane within itself (issue #70, part
//! 1), built from the planar envelope and not traced: the kernel's (`brep::swept::extruded`, its
//! cuts read in closed form, `brep::swept::planar_cut`, #721 F2c). What is here is the drawing's
//! reading: the cut's cutter, its motion in the profile's plane (`Family::in_frame`), and which
//! corners are convex, the cutter's own reading as admission's. So too a prism leaned across the
//! blank under a rack's roll, a helical rack (#826): its sections square to the blank's axis are
//! read in the plane the roll keeps, the observer's, and the sheet is their envelope carried up
//! the helix they turn along (`brep::swept::extruded::Screwed`).
use super::sheet::{Fitted,SweptCut};
use super::Say;
use crate::brep::geom::Frame;
use crate::brep::swept::trace::Inside;
use crate::solid::export::{ExportRefusal,Stage,Tolerance};

/// The sheet of `cut`, built from its profile's planar envelope, when its cutter is a prism and its
/// motion carries the profile's plane within itself; none otherwise (it is traced). `inside` asks
/// the blank (mm); `tolerance` sets the fit's bar (half of it), else `FIT`.
pub fn planar_sheet(cut: &SweptCut,inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Option<Result<Fitted,ExportRefusal>> {
    let cutter = cut.cutter().ok()?;
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


/// The sheet of `cut`, a leaned prism under a rack's roll (#826), built from the envelope of its
/// section square to the observer's axis carried up a helix (`brep::swept::extruded::screwed`):
/// where its cutter is an undrafted prism of lines and arcs, its motion a slide seen from a turn,
/// and its extrusion leans along the slide; none otherwise (it is traced). `inside` asks the blank
/// (mm); `tolerance` sets the fit's bar (half of it), else the kernel's.
pub fn screwed_sheet(cut: &SweptCut,inside: Inside,tolerance: Option<Tolerance>,say: &Say) -> Option<Result<Fitted,ExportRefusal>> {
    // (the prism the tool's B-rep reads, as every sheet of it is sectioned by, #721 F1d)
    let cutter = cut.cutter().ok()?;
    let (walks,extent) = cutter.prism_profile()?;
    let crate::brep::swept::generating::Form::Rack {observer,..} = crate::brep::swept::generating::Form::of(cut.family.path().ok()?).ok()?
        else { return None };
    let frame = Frame::about(observer.origin.map(|x| x*cut.scale),observer.axis);
    let program = cut.family.in_frame(&frame,cut.scale)?;
    let screwed = crate::brep::swept::extruded::Screwed::read(walks,cutter.axis,extent,frame,&program)?;
    let roll = cut.limits.map(f64::to_degrees);
    Some(say.kernel(|say| crate::brep::swept::extruded::screwed(&cut.name,&screwed,roll,inside,tolerance,say)).map_err(ExportRefusal::from))
}
