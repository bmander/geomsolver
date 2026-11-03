//! A body cut by sweeps of the closed class (`solid::closed_class`, #721) built by this kernel: at
//! each placement of each sweep, the tool and its screw placed where the body cuts it, the blank so
//! far split by every candidate face of the sweep's boundary at once, each cell kept where the
//! kernel's membership reads it no member of the sweep (`brep::swept::closed::cut_from`, #735's one
//! arrangement). No sheet is fitted, no sector laid out and no closed solid built
//! and subtracted: the blank and the sweep's candidates are one arrangement, classified once.
use super::{Prepared,Say};
use crate::brep::topo::Brep;
use crate::model::Sketch;
use crate::solid::admission::Class;
use crate::solid::cad::StaticRecipe;
use crate::solid::closed_class;
use crate::solid::export::{ExportRefusal,Stage,Tolerance};

/// How finely the tool is meshed for its membership (mm): a polyhedral tool's mesh is its faces, so
/// the bar is a cell's margin of doubt about its boundary, not a fit.
const MEMBERSHIP_BAR: f64 = 2e-3;
/// The arrangement's tolerance (mm): the kernel's split's own (`swept::sector::SPLIT_TOL`).
const ARRANGED: f64 = crate::solid::sector::SPLIT_TOL;

/// Whether every sweep of the body was admitted to the closed class.
pub fn all(prepared: &Prepared) -> bool { prepared.classes.iter().all(|c| matches!(c,Class::Closed(_))) }

/// The body: its blank less each closed sweep, at each placement, each cut on one arrangement.
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,_tolerance: Option<Tolerance>,say: &Say)
    -> Result<Brep,ExportRefusal> {
    let scale = prepared.field.scale();
    let mut solid = prepared.blank.clone();
    for (k,&swept) in prepared.distinct.iter().enumerate() {
        let name = &sk.solids[swept].name;
        let Class::Closed(found) = &prepared.classes[k] else {
            return Err(ExportRefusal::at(Stage::Admission,format!("`{name}` is not a closed sweep")))
        };
        for c in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            let clock = crate::clock::Instant::now();
            // the tool and its screw placed where the body cuts it, the blank left where it stands
            let place = super::sector::placed(c.pose,scale);
            let path = crate::brep::trajectory::Trajectory::Twist(found.screw.moved(&place));
            solid = crate::brep::swept::closed::cut_from(&solid,&found.tool.moved(&place),&path,found.roll,MEMBERSHIP_BAR,ARRANGED)
                .map_err(|u| {
                    let (condition,message,at) = closed_class::said(u,&found.tool_name);
                    let at = at.map_or(String::new(),|p| format!(" at ({:.4}, {:.4}, {:.4}) mm",p[0],p[1],p[2]));
                    ExportRefusal::at(Stage::Split,format!("`{}`: less `{name}` ({}: {}): {message}{at}",sk.solids[body].name,
                        condition.code(),condition.rule()))
                })?;
            (say.stage)(&format!("`{name}` ({}) cut from the blank on one arrangement: {} faces ({:?})",found.kind(),solid.faces.len(),
                clock.elapsed()));
        }
        (say.mark)(Stage::Reach);
    }
    (say.mark)(Stage::Split);
    Ok(solid)
}
