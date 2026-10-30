//! A swept cut's sheet: the kernel's (`cad_kernel::sweep::sheet`), and what reads a swept cut off
//! the sketch — the kernel's `Source` for it.
#[allow(unused_imports)]
use crate::fmath::Det;
pub use cad_kernel::sweep::sheet::*;
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::solid::export::{AtStage,ExportRefusal,Stage};
use crate::solid::{cad,SweepContacts};

impl Source for Sketch {
    fn cut(&self,swept: usize) -> Result<SweptCut,ExportRefusal> { read(self,swept) }
}

/// Swept cut `swept` as the sketch now stands: what it says of it, read where the sketch is, so
/// that the sheets can be built side by side.
fn read(sk: &Sketch,swept: usize) -> Result<SweptCut,ExportRefusal> {
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
