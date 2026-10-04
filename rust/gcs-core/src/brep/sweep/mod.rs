//! A body with swept cuts of the generating class built by this kernel alone (phase 5 of
//! docs/rust-kernel-plan.md): each distinct sweep's sheet traced and fitted (`sheet`), the body
//! built as one sector split by its sides and its sheets and judged by the material field
//! (`sector`), and that sector patterned round its axis (`brep::pattern`). What a native host did
//! with its kernel's shapes is done here with this kernel's, so the export needs no native kernel,
//! and runs where the core does — in the browser's worker too.
pub mod cutter;
pub mod extruded;
pub mod helical;
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

impl Built {
    /// The whole solid: the pattern's, or the one built whole.
    pub fn into_solid(self) -> crate::brep::topo::Brep { match self { Built::Sector {pattern,..} => pattern.solid,Built::Whole(s) => s } }
}

/// How near a sector's far side's vertex must be to its near side's turned for the pattern to
/// take them for one (mm): the far side is the near one turned, so they agree to rounding.
const PATTERN_MATCH: f64 = 1e-6;

/// What every stage of a swept body's build reads: millimetres a model unit, the static
/// remainder's analytic field and the box it lies in (model units), the distinct sweeps, their cuts and
/// the class each was admitted to, and the blank (mm).
pub struct Prepared { scale: f64,field: crate::solid::SpatialField,bounds: Option<([f64;3],[f64;3])>,distinct: Vec<usize>,
    cuts: Vec<sheet::SweptCut>,classes: Vec<crate::solid::admission::Class>,blank: crate::brep::topo::Brep }

impl Prepared {
    /// How many distinct sweeps the body cuts, each one sheet.
    pub fn sweeps(&self) -> usize { self.cuts.len() }
    /// The document's name of sweep `k`.
    pub fn sweep_name<'a>(&self,sk: &'a Sketch,k: usize) -> &'a str { &sk.solids[self.distinct[k]].name }
}

/// The first stage: the body's field, its sweeps and its blank, by its meridian where it is a solid
/// of revolution. `admission` must be this body's (which only `admission::admit_body` makes).
pub fn prepare(sk: &Sketch,body: usize,recipe: &StaticRecipe,admission: &Admission,say: &Say) -> Result<Prepared,ExportRefusal> {
    let name = sk.solids[body].name.clone();
    if admission.body() != body {
        return Err(ExportRefusal::at(Stage::Admission,format!("`{name}`: the admission presented is another body's")))
    }
    let scale = cad::millimetres(sk).at(Stage::Blank)?;
    let (field,_) = crate::solid::admission::static_remainder(sk,body,cad::AXIS_TOLERANCE).at(Stage::Blank)?;
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    let cuts = distinct.iter().map(|&swept| sheet::SweptCut::read(sk,swept)).collect::<Result<Vec<_>,_>>()?;
    let classes = distinct.iter().map(|&swept| admission.sweeps().iter().find(|e| e.sweep == swept).map(|e| e.class.clone())
        .ok_or_else(|| ExportRefusal::at(Stage::Admission,format!("`{name}`: `{}` was not admitted",sk.solids[swept].name))))
        .collect::<Result<Vec<_>,_>>()?;
    let bounds = field.support_bounds().ok().flatten().map(|b| (b.map(|x| x.bounds()[0]),b.map(|x| x.bounds()[1])));
    let started = crate::clock::Instant::now();
    let blank = match crate::brep::recipe::meridian(&recipe.recipe,[1.,0.,0.]).at(Stage::Blank)? {
        Ok((b,..)) => b,
        Err(_) => crate::brep::recipe::build(&recipe.recipe).at(Stage::Blank)?,
    };
    (say.stage)(&format!("`{name}`: the static blank by this kernel: {:.6} mm³, {} faces ({:?})",crate::brep::props::volume(&blank),
        blank.faces.len(),started.elapsed()));
    (say.mark)(Stage::Blank);
    Ok(Prepared {scale,field,bounds,distinct,cuts,classes,blank})
}

/// Sweep `k`'s sheet fitted against the blank: a prism's under a motion keeping its profile's
/// plane, its profile's planar envelope extruded (`extruded::planar_sheet`); traced, under any other
/// relative rotation (`sheet::swept_sheet`); or its characteristic carried, under a screw
/// (`helical::helical_sheet`).
pub fn sheet(prepared: &Prepared,k: usize,tolerance: Option<Tolerance>,say: &Say) -> Result<sheet::Fitted,ExportRefusal> {
    let (field,scale) = (&prepared.field,prepared.scale);
    let inside = |points: &[[f64;3]]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect()) };
    let near = |p: [f64;3]| field.value(p.map(|x| x/scale))*scale;
    match &prepared.classes[k] {
        crate::solid::admission::Class::Generating => match extruded::planar_sheet(&prepared.cuts[k],&inside,tolerance,say) {
            Some(fitted) => fitted,
            None => traced(prepared,k,tolerance,say),
        },
        crate::solid::admission::Class::ConstantTwist(found) => {
            let heights = prepared.bounds.map(|(lo,hi)| found.screw.extent(lo,hi,crate::envelope::Motion::identity()).0);
            helical::helical_sheet(&prepared.cuts[k],found,heights,&inside,&near,tolerance,say)
        }
    }
}

/// Sweep `k`'s sheet traced, as every generating sweep but a prism's under a motion keeping its
/// profile's plane is built (kept for that one too, so that the two can be compared).
pub fn traced(prepared: &Prepared,k: usize,tolerance: Option<Tolerance>,say: &Say) -> Result<sheet::Fitted,ExportRefusal> {
    let (field,scale) = (&prepared.field,prepared.scale);
    let inside = |points: &[[f64;3]]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect()) };
    let near = |p: [f64;3]| field.value(p.map(|x| x/scale))*scale;
    sheet::swept_sheet(&prepared.cuts[k],&inside,&near,tolerance,say)
}

/// How the body is cut from its blank: as one sector, its premises holding, or whole, and why.
pub enum Plan { Sector(sector::Premises),Whole(String) }

/// Whether the body's sweeps are turns of one placement about one axis with a side through their
/// gaps (`sector::premises`), or it is built whole.
pub fn plan(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,sheets: &[sheet::Fitted],say: &Say) -> Plan {
    match sector::premises(sk,body,recipe,&prepared.blank,&prepared.field,&prepared.distinct,sheets,prepared.scale,say) {
        Ok(premises) => Plan::Sector(premises),
        Err(why) => { (say.stage)(&format!("built whole, not as one sector: {why}")); Plan::Whole(why) }
    }
}

/// The body's material by its plan: the sector cut from the blank by its sides and sheets, its cells
/// judged, or the whole blank so cut (the `Whole` built already).
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,sheets: &[sheet::Fitted],plan: Plan,say: &Say)
    -> Result<Cut,ExportRefusal> {
    Ok(match plan {
        Plan::Sector(premises) => Cut::Sector(sector::construct(recipe,&prepared.distinct,premises,say).at(Stage::Split)?),
        Plan::Whole(_) => Cut::Whole(sector::whole(sk,body,recipe,&prepared.blank,&prepared.distinct,sheets,prepared.scale,say)
            .at(Stage::Split)?),
    })
}

/// The material cut: one sector, or the whole body.
pub enum Cut { Sector(sector::Sector),Whole(crate::brep::topo::Brep) }

/// The body built from its cut material: a sector patterned round its axis, or the whole as it is.
pub fn finish(cut: Cut,say: &Say) -> Result<Built,ExportRefusal> {
    let sector = match cut { Cut::Sector(sector) => sector,Cut::Whole(solid) => return Ok(Built::Whole(solid)) };
    let started = crate::clock::Instant::now();
    let pattern = pattern::pattern(&sector.piece,sector.origin,sector.axis,sector.count,PATTERN_MATCH).at(Stage::Fuse)?;
    (say.stage)(&format!("turned the sector into {} copies: its sides faces {} and {}, their vertices matched within {:.1e} mm ({:?})",
        sector.count,pattern.sides[0],pattern.sides[1],pattern.matched,started.elapsed()));
    (say.mark)(Stage::Fuse);
    Ok(Built::Sector {sector,pattern})
}

/// `body` (its static recipe and its admission to the generating-sweep class, which only
/// `admission::admit_body` makes) built as one sector patterned, or — where a sector's premises do
/// not hold, which is said — whole, or refused with the stage that refused it: the stages above in
/// turn, each sweep's sheet side by side where the target has threads.
pub fn build(sk: &Sketch,body: usize,recipe: &StaticRecipe,admission: &Admission,tolerance: Option<Tolerance>,say: &Say)
    -> Result<Built,ExportRefusal> {
    let prepared = prepare(sk,body,recipe,admission,say)?;
    let sheets = crate::par::indices(prepared.sweeps(),|k| sheet(&prepared,k,tolerance,say)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let plan = plan(sk,body,recipe,&prepared,&sheets,say);
    finish(cut(sk,body,recipe,&prepared,&sheets,plan,say)?,say)
}

