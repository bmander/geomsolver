//! A swept cut's sheet: its contacts traced at the cutter's stations over the band its declared
//! roll reaches the blank in (`brep::swept::trace`), laid out on a grid widened until its edges
//! leave the blank, interpolated as a B-spline surface (`nurbs::interpolate_net`), and judged at the
//! contacts withheld from the fit — held to a tolerance, refined where it misses until it fits.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::swept::cutter::Cutter;
use super::Say;
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::brep::swept::trace::Inside;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{cad,SweepContacts};
/// A sheet fitted, judged and cut into faces is the kernel's (`brep::swept::sheet`, #721 F2b); what
/// is here is how a generating sweep's contacts are traced for it.
pub use crate::brep::swept::sheet::{Fitted,MOST_COLUMNS,MOST_REFINEMENTS,MOST_ROWS,feet_near,surface_grid};

type V = [f64;3];

/// A swept cut as its sheet is built: what the sketch says of it, read where the sketch is, so
/// that the sheets can be built side by side.
pub struct SweptCut {
    pub name: String,
    pub scale: f64,
    /// The cutter's recipe, the motion and the declared roll's limits.
    pub recipe: crate::json::Json,
    pub family: Family,
    pub limits: [f64;2],
    /// The contacts, whose failure to read is the reach's.
    pub contacts: Result<SweepContacts,String>,
    /// The route to each primitive of the cutter (`document::operand_paths`), by the key its
    /// faces are named under: a cut face is named by the cutter's face that made it (#212).
    pub routes: std::collections::BTreeMap<String,String>,
}

impl SweptCut {
    pub fn read(sk: &Sketch,swept: usize) -> Result<SweptCut,ExportRefusal> {
        let name = sk.solids[swept].name.clone();
        let scale = cad::millimetres(sk);
        let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
            return Err(ExportRefusal::at(Stage::Clearance,format!("`{name}` is not a continuous sweep")))
        };
        let recipe = cad::recipe(sk,*source as usize).at(Stage::Clearance)?;
        let family = Family::read(sk,*motion as usize).at(Stage::Clearance)?;
        let limits = crate::solid::roll::interval(sk,swept).at(Stage::Clearance)?;
        let contacts = SweepContacts::read(sk,swept,cad::AXIS_TOLERANCE);
        // a member of a union tool's faces are named by their routes in the union (#684 C)
        let SolidDef::Swept {source:tool,..} = sk.solids[crate::solid::distributed::union_of(sk,swept)].def else {
            unreachable!("a member's union is a sweep")
        };
        let routes = crate::solid::operand_paths(sk,tool as usize);
        Ok(SweptCut {name,scale,recipe,family,limits,contacts,routes})
    }
}

/// The sheet of one swept cut against the blank `inside` describes: its contacts reach the blank
/// over a band of the cutter's stations (`Cutter::reach`), its grid is laid out by each row
/// placement in turn and widened until its edges leave the blank, and the first placement whose
/// fit holds is the sheet; held to a tolerance, a fit that misses is refined where it misses.
/// (The roll's clearance of the blank at both limits is admission's row E1, asked before this.)
pub fn swept_sheet(cut: &SweptCut,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    let cutter = Cutter::read(&cut.recipe).at(Stage::Reach)?;
    traced_sheet(cut,&cutter,inside,near,tolerance,say)
}

/// The sheet of a prism whose cap reaches the blank (#684 F): `swept_sheet`'s, its stations the
/// prism's sections through its extrusion at distances along the direction admission's row T3
/// chose, each its chord carried along the extrusion, so the walk runs from a side over a cap's
/// rim and across the cap.
pub fn capped_sheet(cut: &SweptCut,capped: &crate::solid::admission::Capped,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let middle = 0.5*(capped.band[0]+capped.band[1]);
    let cutter = Cutter::read(&cut.recipe).and_then(|c| c.across(capped.across,middle)).at(Stage::Reach)?;
    traced_sheet(cut,&cutter,inside,near,tolerance,say)
}

/// `swept_sheet` of the cutter's stations: the kernel's (`brep::swept::trace::traced_sheet`), each
/// band named by the cutter's face that made it, by its path in the cutter (#212).
fn traced_sheet(cut: &SweptCut,cutter: &Cutter,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    let chart = cut.contacts.as_ref().map_err(Clone::clone).and_then(SweepContacts::chart).at(Stage::Reach)?;
    let face_name = |tag: u32| -> String {
        // (`UNNAMED` carries the fan's bit, so names nothing)
        let Some((key,face)) = cutter.face_of(tag) else { return String::new() };
        crate::solid::faces::FacePath::new(cut.routes.get(&key).map_or("",String::as_str),&face).text()
    };
    Ok(say.kernel(|say| crate::brep::swept::trace::traced_sheet(&cut.name,cut.scale,cutter,chart,inside,near,tolerance,&face_name,say))?)
}
