//! A body cut by sweeps of the planar generating class (`solid::planar_class`, #65) built by this
//! kernel: what a pocketed prism turned through a whole period leaves of the blank is the blank
//! within the pocket's inner envelope carried along the prisms. The envelope is fitted a cubic
//! B-spline a piece, its corners the pieces' shared ends, extruded through the blank and kept in
//! common with it, at every placement of every sweep. Nothing is traced or split: the envelope's
//! own contacts, solved exactly (`InnerEnvelope::point`), are what the splines are fitted through.
use super::{Prepared,Say};
use crate::brep::boolean::Op;
use crate::brep::topo::Brep;
use crate::model::Sketch;
use crate::solid::admission::Class;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

/// Whether every sweep of the body was admitted to the planar class.
pub fn all(prepared: &Prepared) -> bool { prepared.classes.iter().all(|c| matches!(c,Class::Planar(_))) }

/// The body: its blank kept in common with each planar sweep's envelope prism, at each placement.
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,tolerance: Option<Tolerance>,say: &Say) -> Result<Brep,ExportRefusal> {
    let scale = prepared.field.scale();
    // as a curve in a static profile is fitted, within the tolerance's share for the fit when finer
    let fit = tolerance.map_or(cad::FIT_MM,|t| t.fit().min(cad::FIT_MM))/scale;
    let mut solid = prepared.blank.clone();
    for (k,&swept) in prepared.distinct.iter().enumerate() {
        let Class::Planar(found) = &prepared.classes[k] else {
            return Err(ExportRefusal::at(Stage::Admission,format!("`{}` is not a planar sweep",sk.solids[swept].name)))
        };
        let clock = crate::clock::Instant::now();
        let env = &found.envelope;
        let prism = env.prism(found.heights,fit,scale,&sk.solids[swept].name).at(Stage::Reach)?;
        (say.stage)(&format!("`{}`: its inner envelope fitted, {} pieces, and carried through the blank ({:?})",sk.solids[swept].name,
            env.pieces.len(),clock.elapsed()));
        (say.mark)(Stage::Reach);
        for c in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            let placed = prism.moved(&super::sector::placed(c.pose,scale));
            solid = crate::brep::boolean::combined(&solid,&placed,Op::Common,0.)
                .map_err(|e| ExportRefusal::at(Stage::Split,format!("`{}`: kept within `{}`'s envelope: {e}",sk.solids[body].name,sk.solids[swept].name)))?;
        }
    }
    (say.mark)(Stage::Split);
    Ok(solid)
}
