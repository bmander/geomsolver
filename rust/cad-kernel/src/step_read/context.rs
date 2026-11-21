//! The solids a file represents, and the context each is represented in: its units, as
//! millimetres and radians to a unit of the file's, and the uncertainty the file states.
use super::{Refusal,Why};
use crate::step_file::{File,Value};

/// A representation's context: what the file's numbers measure.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Context {
    /// The context's entity.
    pub entity: u32,
    /// Millimetres to the file's unit of length.
    pub length: f64,
    /// Radians to the file's unit of plane angle.
    pub angle: f64,
    /// The uncertainty the file states for its lengths, in millimetres; none where it states
    /// none.
    pub uncertainty: Option<f64>,
}

/// A solid a file represents: its entity (a `MANIFOLD_SOLID_BREP` or a `BREP_WITH_VOIDS`), its
/// product's name (empty where the file gives none), and its representation's context.
#[derive(Clone,Debug,PartialEq)]
pub struct Found { pub solid: u32,pub name: String,pub context: Context }

/// Metres to an SI unit's prefix.
fn prefix(word: Option<&str>) -> Option<f64> {
    Some(match word { None => 1.,Some("MILLI") => 1e-3,Some("CENTI") => 1e-2,Some("DECI") => 1e-1,Some("MICRO") => 1e-6,
        Some("KILO") => 1e3,_ => return None })
}

/// The context `r`: an SI unit of length under any prefix above and the radian are read; anything
/// else is refused, naming the unit.
fn context(file: &File,r: u32) -> Result<Context,Refusal> {
    let e = file.get(r);
    let units = e.part("GLOBAL_UNIT_ASSIGNED_CONTEXT").and_then(|v| v.first()).and_then(Value::list)
        .ok_or_else(|| Refusal::malformed(r,"a representation context that assigns no units"))?;
    let (mut length,mut angle) = (None,None);
    for u in units.iter().filter_map(Value::reference) {
        let unit = file.get(u);
        let si = unit.part("SI_UNIT");
        if unit.is("LENGTH_UNIT") {
            let metres = si.filter(|v| v.len() == 2 && v[1].word() == Some("METRE")).and_then(|v| prefix(v[0].word()))
                .ok_or_else(|| Refusal::unsupported(u,"a unit of length that is no SI metre"))?;
            length = Some(metres*1e3);
        } else if unit.is("PLANE_ANGLE_UNIT") {
            si.filter(|v| v.len() == 2 && v[0] == Value::Unset && v[1].word() == Some("RADIAN"))
                .ok_or_else(|| Refusal::unsupported(u,"a unit of plane angle that is not the radian"))?;
            angle = Some(1.);
        }
    }
    let length = length.ok_or_else(|| Refusal::malformed(r,"a representation context without a unit of length"))?;
    let angle = angle.ok_or_else(|| Refusal::malformed(r,"a representation context without a unit of plane angle"))?;
    // the uncertainty, in the context's own unit of length
    let uncertainty = e.part("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT").and_then(|v| v.first()).and_then(Value::list).unwrap_or(&[])
        .iter().filter_map(Value::reference).find_map(|u| match file.get(u).simple() {
            Some(("UNCERTAINTY_MEASURE_WITH_UNIT",v)) => match v.first() {
                Some(Value::Typed("LENGTH_MEASURE",x)) => x.first().and_then(Value::number),
                _ => None,
            },
            _ => None,
        }).map(|x| x*length);
    Ok(Context {entity:r,length,angle,uncertainty})
}

/// Every solid the file's advanced B-rep shape representations list, in the file's order.
pub fn solids(file: &File) -> Result<Vec<Found>,Refusal> {
    let mut out = Vec::new();
    for e in file.entities.iter().filter(|e| e.is("ADVANCED_BREP_SHAPE_REPRESENTATION")) {
        let v = e.part("ADVANCED_BREP_SHAPE_REPRESENTATION").unwrap();
        let items = v.get(1).and_then(Value::list).ok_or_else(|| Refusal::malformed(e.id,"a shape representation without items"))?;
        let at = v.get(2).and_then(Value::reference).ok_or_else(|| Refusal::malformed(e.id,"a shape representation without a context"))?;
        let mut read = None;
        for solid in items.iter().filter_map(Value::reference).filter(|&i| file.get(i).is("MANIFOLD_SOLID_BREP") || file.get(i).is("BREP_WITH_VOIDS")) {
            let context = match read { Some(c) => c,None => { let c = context(file,at)?; read = Some(c); c } };
            out.push(Found {solid,name:String::new(),context});
        }
    }
    if out.is_empty() && file.count("MANIFOLD_SOLID_BREP") > 0 {
        return Err(Refusal::new(Why::Unsupported,None,"the file's solids are items of no advanced B-rep shape representation"))
    }
    Ok(out)
}
