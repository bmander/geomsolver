//! A body cut by sweeps of the planar subtraction class (`solid::planar_subtraction`, #606) built by
//! this kernel: in each slab of the blank, the region the tool's section sweeps
//! (`envelope::swept_region`) carried along the plane's normal through the slab — a hair past the
//! blank where the slab ends at its face — and taken from the blank, at every placement of every
//! sweep. A loop of the region is a profile's: each kept stretch a cubic B-spline fitted through
//! its exact points, each step along the box a line, its corners the stretches' shared ends; a
//! loop counter-clockwise is an outer loop, one clockwise a hole in the one about it. A drafted
//! slab (#607) is lofted between its stations, each stretch a B-spline surface through the
//! stretch's points at every station. Nothing is traced, split or judged cell by cell.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Prepared,Say};
use crate::brep::boolean::Op;
use crate::brep::topo::Brep;
use crate::brep::swept::region::{lofted,profiles};
use crate::model::Sketch;
use crate::solid::admission::Class;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};


/// Whether every sweep of the body was admitted to the planar subtraction class.
pub fn all(prepared: &Prepared) -> bool { prepared.classes.iter().all(|c| matches!(c,Class::Subtraction(_))) }

/// The body: its blank less each subtraction sweep's swept region carried through each slab, at
/// each placement.
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,tolerance: Option<Tolerance>,say: &Say) -> Result<Brep,ExportRefusal> {
    let scale = prepared.field.scale();
    // as a curve in a static profile is fitted, within the tolerance's share for the fit when finer
    let fit = tolerance.map_or(cad::FIT_MM,|t| t.fit().min(cad::FIT_MM))/scale;
    let mut solid = prepared.blank.clone();
    for (k,&swept) in prepared.distinct.iter().enumerate() {
        let name = &sk.solids[swept].name;
        let Class::Subtraction(found) = &prepared.classes[k] else {
            return Err(ExportRefusal::at(Stage::Admission,format!("`{name}` is not a planar subtraction sweep")))
        };
        let clock = crate::clock::Instant::now();
        let mut cutters: Vec<Brep> = Vec::new();
        for slab in &found.slabs {
            match slab.stations.as_slice() {
                [(_,region)] => for profile in profiles(region,scale,fit,name).at(Stage::Reach)? {
                    let p = crate::brep::build::Profile::from_json(&profile).at(Stage::Reach)?;
                    cutters.push(crate::brep::build::prism(&p,slab.reach[0]*scale,slab.reach[1]*scale).at(Stage::Reach)?);
                },
                stations => cutters.extend(lofted(stations,scale,fit,name).at(Stage::Reach)?),
            }
        }
        (say.stage)(&format!("`{name}`: its swept region fitted in {} slabs, {} loops, and carried through the blank ({:?})",
            found.slabs.len(),cutters.len(),clock.elapsed()));
        (say.mark)(Stage::Reach);
        for c in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            for cutter in &cutters {
                let placed = cutter.moved(&super::sector::placed(c.pose,scale));
                solid = crate::brep::boolean::combined(&solid,&placed,Op::Cut,0.)
                    .map_err(|e| ExportRefusal::at(Stage::Split,format!("`{}`: less `{name}`'s swept region: {e}",sk.solids[body].name)))?;
            }
        }
    }
    (say.mark)(Stage::Split);
    Ok(solid)
}

