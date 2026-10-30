//! A body built as one sector (docs/native-speed-plan.md, by this kernel alone): where every
//! swept cut is a turn of one placement about one axis and the blank reads alike under that turn,
//! the blank between a side through the gaps the cuts leave (`solid::sector`) and that side turned
//! by a pitch, cut by the placements that fall in it, its cells judged by the material field, and
//! the material cell returned to be patterned round the axis (`brep::pattern`). The split, the
//! judgement and the cell contract are the kernel's (`cad_kernel::sweep::cells`); here is what
//! reads them off the sketch: the body's recipe, its sweeps' placements and keys, and its material
//! field as the probe the cells are judged by.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::Say;
use super::sheet::Fitted;
use crate::brep::geom::Rigid;
use crate::brep::recipe::Recipe;
use crate::brep::topo::Brep;
use crate::envelope::Motion;
use crate::model::Sketch;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::{MaterialField,Millimetres,ProbeState,SpatialField};
use cad_kernel::sweep::cells::{self,Ball,Cuts};

pub use cad_kernel::sweep::cells::{COLUMNS,MARGIN,PROBES,READ_BACK,Premises,Sector,sphere_centres};

type V = [f64;3];

/// A placement's rigid motion in millimetres.
pub(crate) fn placed(pose: Motion,scale: f64) -> Rigid { cells::placed(pose,scale) }

/// The body's material field, as the probe its cells are judged by.
type Field = Result<Millimetres<MaterialField>,String>;

fn field(sk: &Sketch,body: usize) -> Field { Millimetres::<MaterialField>::read(sk,body,cad::AXIS_TOLERANCE) }

/// The body's material field (read already; its failure to read said at each ball) as the probe
/// its cells are judged by, each ball on every core.
fn probe(field: &Field) -> impl Fn(&[(V,f64)]) -> Vec<Result<Ball,String>>+Sync+'_ {
    move |asked: &[(V,f64)]| {
        let field = match field {
            Ok(field) => field,
            Err(e) => return asked.iter().map(|_| Err(e.clone())).collect(),
        };
        crate::par::indices_with(asked.len(),|| field.evaluator(cad::POSE_CACHE),|material,i| {
            let (point,radius) = asked[i];
            material.ball(point,radius).map(|state| match state {
                ProbeState::InteriorBall => Ball::Material,
                ProbeState::ExteriorBall => Ball::Removed,
                state => Ball::Other(format!("{state:?}")),
            })
        })
    }
}

/// The body's distinct sweeps' placements (model units) and keys, and their names.
fn placements(sk: &Sketch,recipe: &StaticRecipe,distinct: &[usize]) -> (Vec<Vec<Motion>>,Vec<Vec<String>>,Vec<String>) {
    let sweeps: Vec<Vec<_>> = distinct.iter().map(|&s| recipe.sweeps.iter().filter(|c| c.swept == s).collect()).collect();
    (sweeps.iter().map(|c| c.iter().map(|c| c.pose).collect()).collect(),sweeps.iter().map(|c| c.iter().map(|c| c.key.clone()).collect()).collect(),
        distinct.iter().map(|&s| sk.solids[s].name.clone()).collect())
}

/// Whether `body` can be built as one sector, and what it is cut from, or why not
/// (`cells::premises`). `sheets` are the distinct sweeps' fitted sheets in the order of `distinct`;
/// `blank` the static blank (mm), `field` its analytic field, whose scale the placements' motions
/// are read by.
#[allow(clippy::too_many_arguments)]
pub fn premises(sk: &Sketch,body: usize,recipe: &StaticRecipe,blank: &Brep,field: &Millimetres<SpatialField>,distinct: &[usize],
    sheets: &[Fitted],say: &Say) -> Result<Premises,String> {
    let (poses,keys,names) = placements(sk,recipe,distinct);
    let cuts = Cuts {poses:&poses,keys:&keys,names:&names,sheets,scale:field.scale()};
    cells::premises(&cuts,&Recipe::read(&recipe.recipe)?,blank,&|p| field.value(p),&probe(&self::field(sk,body)),say)
}

/// The sector its premises describe (`cells::construct`), its cells judged by the body's material
/// field.
pub fn construct(sk: &Sketch,body: usize,recipe: &StaticRecipe,distinct: &[usize],premises: Premises,say: &Say) -> Result<Sector,String> {
    cells::construct(&Recipe::read(&recipe.recipe)?,distinct.len(),premises,&probe(&field(sk,body)),say)
}

/// A body whose premises for a sector do not hold, built whole (`cells::whole`): the blank, made
/// again with its seams opposite the cuts where it is a solid of revolution, split by every
/// placement of every sheet that reaches it, its cells judged by the body's material field.
/// `sheets` and `distinct` as for `premises`.
#[allow(clippy::too_many_arguments)]
pub fn whole(sk: &Sketch,body: usize,recipe: &StaticRecipe,blank: &Brep,distinct: &[usize],sheets: &[Fitted],scale_mm: f64,say: &Say)
    -> Result<Brep,String> {
    let (poses,keys,_) = placements(sk,recipe,distinct);
    let tools = poses.iter().zip(&keys).zip(sheets).flat_map(|((poses,keys),sheet)| poses.iter().zip(keys).map(move |(&pose,key)| {
        let tool = cells::keyed(sheet.faces.moved(&placed(pose,scale_mm)),key);
        let bounds = tool.bounds();
        (tool,bounds)
    })).collect();
    let tools = cells::reaching(blank,tools);
    let turned = cells::facing(&Recipe::read(&recipe.recipe)?,blank,&tools)?;
    cells::whole(&turned,tools,&poses.iter().map(Vec::len).collect::<Vec<_>>(),&probe(&field(sk,body)),say)
}
