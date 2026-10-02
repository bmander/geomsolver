//! A static solid read through this kernel's B-rep (rung 3 of docs/rust-kernel-plan.md): its CAD
//! recipe built node by node, every face named by the path the faceted kernel gives it —
//! `bore.wall`, a placed copy's `placed.bore.wall` — so a body's surviving faces, its provenance
//! and its round features read the same whichever kernel evaluated it. A boolean never renames:
//! a face keeps the name of the operand face it is a piece of.
use super::*;
use crate::brep::topo::Brep;
use std::collections::BTreeSet;

/// Between a face's solid and its own name while the recipe is built: a placement renames the
/// solid half and leaves the face half alone, and neither half may hold it.
const SEP: char = '\u{1}';

/// A static solid's exact B-rep, in millimetres (`mm` of them a model unit, `cad::millimetres`)
/// about `origin` (model units, the point the facet term is evaluated about too), each face named
/// by its document path.
#[derive(Clone, Debug)]
pub struct Exact {
    pub brep: Brep,
    pub mm: f64,
    pub origin: [f64; 3],
    /// The faces whose facets lead, as the facet term's do — a prism's caps and a sweep's — so an
    /// edge between two faces, named by the face that leads, is named as it was.
    pub leading: BTreeSet<String>,
}

/// Solid `si`'s exact B-rep, or why this kernel does not build it (no length unit, a sweep, a
/// recipe node it refuses) — a caller then evaluates the facet term instead.
pub(crate) fn build(sk: &Sketch, si: usize) -> Result<Exact, String> {
    let mm = cad::millimetres(sk)?;
    let origin = frame_origin(sk, si, REPORT_UNIT);
    let recipe = cad::shifted(&cad::recipe(sk, si)?, origin.map(|x| x * mm));
    // the rounding of coordinates read where the solid stands, before the shift: a few units in
    // the last place of the largest
    let floor = 16.0 * f64::EPSILON * origin.iter().fold(0.0f64, |m, x| m.max(x.abs())) * mm;
    let field = |j: &'_ crate::json::Json, k: &str| -> Result<crate::json::Json, String> {
        j.get(k).cloned().ok_or(format!("recipe: no `{k}`"))
    };
    let mut built = BTreeMap::new();
    let mut leading = BTreeSet::new();
    for n in field(&recipe, "nodes")?.arr() {
        let id = field(n, "id")?.as_i64();
        let mut b = crate::brep::recipe::node_named(n, &built, &BTreeMap::new(), floor)?;
        let sol = &sk.solids[id as usize];
        match &sol.def {
            SolidDef::Placed { source, .. } => {
                let paths = operand_paths(sk, *source as usize);
                for f in &mut b.faces {
                    let (of, face) = f.name.split_once(SEP).unwrap_or((f.name.as_str(), ""));
                    let path = paths.get(of).map(String::as_str).unwrap_or(of);
                    let placed = super::document::placed_name(&sol.name, path);
                    if leading.contains(&format!("{of}.{face}")) {
                        leading.insert(format!("{placed}.{face}"));
                    }
                    f.name = format!("{placed}{SEP}{face}");
                }
            }
            // a body's faces are its operands', named as they were
            SolidDef::Body { .. } => {}
            _ => {
                let caps: &[&str] = match sol.def {
                    SolidDef::Prism { .. } | SolidDef::Through { .. } => &["near", "far"],
                    SolidDef::Loft { .. } => &["start", "end"],
                    _ => &[],
                };
                for f in &mut b.faces {
                    if caps.contains(&f.name.as_str()) {
                        leading.insert(format!("{}.{}", sol.name, f.name));
                    }
                    f.name = format!("{}{SEP}{}", sol.name, f.name);
                }
            }
        }
        built.insert(id, b);
    }
    let mut brep = built.remove(&field(&recipe, "root")?.as_i64()).ok_or("recipe: no root")?;
    for f in &mut brep.faces {
        f.name = match f.name.split_once(SEP) {
            Some((of, "")) => of.to_string(),
            Some((of, face)) => format!("{of}.{face}"),
            None => std::mem::take(&mut f.name),
        };
    }
    Ok(Exact { brep, mm, origin, leading })
}
