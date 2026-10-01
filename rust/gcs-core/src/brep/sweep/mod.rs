//! A body with swept cuts of the generating class built by this kernel alone (phase 5 of
//! docs/rust-kernel-plan.md): each distinct sweep's sheet traced and fitted (`sheet`), the body
//! built as one sector split by its sides and its sheets and judged by the material field
//! (`sector`), and that sector patterned round its axis (`brep::pattern`). What a native host did
//! with its kernel's shapes is done here with this kernel's, so the export needs no native kernel,
//! and runs where the core does — in the browser's worker too.
pub mod cutter;
pub mod sector;
pub mod sheet;

use crate::brep::pattern::{self,Built as Patterned};
use crate::model::Sketch;
use crate::solid::admission::Admission;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

/// How the pipeline speaks: a line of progress, and the stage just passed.
pub struct Say<'a> { pub stage: &'a (dyn Fn(&str)+Sync),pub mark: &'a (dyn Fn(Stage)+Sync) }

/// A swept body built: as one sector, and the sector patterned into the whole (its B-rep, its mesh),
/// or whole where a sector's premises do not hold.
pub enum Built { Sector { sector: sector::Sector,pattern: Patterned },Whole(crate::brep::topo::Brep) }

/// How near a sector's far side's vertex must be to its near side's turned for the pattern to
/// take them for one (mm): the far side is the near one turned, so they agree to rounding.
const PATTERN_MATCH: f64 = 1e-6;

/// `body` (its static recipe and its admission to the generating-sweep class, which only
/// `admission::admit_body` makes) built as one sector patterned, or — where a sector's premises do
/// not hold, which is said — whole, or refused with the stage that refused it.
pub fn build(sk: &Sketch,body: usize,recipe: &StaticRecipe,admission: &Admission,tolerance: Option<Tolerance>,say: &Say)
    -> Result<Built,ExportRefusal> {
    let name = &sk.solids[body].name;
    if admission.body() != body {
        return Err(ExportRefusal::at(Stage::Admission,format!("`{name}`: the admission presented is another body's")))
    }
    let scale = cad::millimetres(sk).at(Stage::Blank)?;
    let (field,_) = crate::solid::admission::static_remainder(sk,body,cad::AXIS_TOLERANCE).at(Stage::Blank)?;
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    let cuts = distinct.iter().map(|&swept| sheet::SweptCut::read(sk,swept)).collect::<Result<Vec<_>,_>>()?;
    // the blank, by its meridian where it is a solid of revolution
    let started = crate::clock::Instant::now();
    let blank = match crate::brep::recipe::meridian(&recipe.recipe,[1.,0.,0.]).at(Stage::Blank)? {
        Ok((b,..)) => b,
        Err(_) => crate::brep::recipe::build(&recipe.recipe).at(Stage::Blank)?,
    };
    (say.stage)(&format!("`{name}`: the static blank by this kernel: {:.6} mm³, {} faces ({:?})",crate::brep::props::volume(&blank),
        blank.faces.len(),started.elapsed()));
    (say.mark)(Stage::Blank);
    // each sweep's sheet, side by side where the target has threads
    let inside = |points: &[[f64;3]]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect()) };
    let near = |p: [f64;3]| field.value(p.map(|x| x/scale))*scale;
    let sheets = crate::par::map(&cuts,|cut| sheet::swept_sheet(cut,&inside,&near,tolerance,say)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let premises = match sector::premises(sk,body,recipe,&blank,&field,&distinct,&sheets,scale,say) {
        Ok(premises) => premises,
        Err(why) => {
            (say.stage)(&format!("built whole, not as one sector: {why}"));
            return Ok(Built::Whole(sector::whole(sk,body,recipe,&blank,&distinct,&sheets,scale,say).at(Stage::Split)?))
        }
    };
    let sector = sector::construct(recipe,&distinct,premises,say).at(Stage::Split)?;
    let started = crate::clock::Instant::now();
    let pattern = pattern::pattern(&sector.piece,sector.origin,sector.axis,sector.count,PATTERN_MATCH).at(Stage::Fuse)?;
    (say.stage)(&format!("turned the sector into {} copies: its sides faces {} and {}, their vertices matched within {:.1e} mm ({:?})",
        sector.count,pattern.sides[0],pattern.sides[1],pattern.matched,started.elapsed()));
    (say.mark)(Stage::Fuse);
    Ok(Built::Sector {sector,pattern})
}
