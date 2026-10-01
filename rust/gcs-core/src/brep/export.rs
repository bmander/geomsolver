//! A solid's exact export by this kernel, as every host makes it (phase 5 of
//! docs/rust-kernel-plan.md): the terminal's `solventc` and the browser's worker call the same
//! functions, so a file is one file wherever it is written. A body is built from its CAD recipe,
//! or — with swept cuts of the generating class, admitted — as one sector patterned
//! (`brep::sweep`); its STEP is written and parsed back against it (`step_check`); its STL is meshed
//! within the tolerance's deflection (the measured sag, a bound and not a control), its shells
//! checked, and a swept body's judged against its material field (`agreement`), nothing given back
//! unless it agrees.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::pattern::Built as Patterned;
use super::sweep::Say;
use super::topo::Brep;
use crate::model::Sketch;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{admission,agreement,cad,contracts,MaterialField};

/// The mesh's deflection with no tolerance given: the export's gross bar (mm).
pub const GROSS: f64 = 0.01;
/// The mesh's angular control (radians).
pub const ANGULAR: f64 = 0.2;

/// A body built exactly: its B-rep, and, built as one sector, the pattern that meshes it and the
/// turn its field agreement reads each probe in.
pub struct Exact { pub solid: Brep,pub pattern: Option<Patterned>,pub indexed: Option<([f64;3],[f64;3],usize)>,
    /// Whether the body cuts sweeps, so its mesh is held to its material field.
    pub swept: bool }

/// Solid `body` of the sketch built by this kernel. A body with swept cuts is admitted to the
/// generating-sweep class first (`admission` where the host made it, else here).
pub fn exact(sk: &Sketch,body: usize,admitted: Option<&admission::Admission>,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    let recipe = cad::recipe_static(sk,body).at(Stage::Blank)?;
    if !recipe.sweeps.is_empty() {
        let made;
        let admission = match admitted {
            Some(a) => a,
            None => { made = admission::admit_body(sk,body,&admission::Options::default())?; (say.mark)(Stage::Admission); &made }
        };
        return Ok(match super::sweep::build(sk,body,&recipe,admission,tolerance,say)? {
            super::sweep::Built::Sector {sector,pattern} => {
                (say.stage)(&format!("the solid by the Rust kernel: {} sectors, {} faces, {} edges",sector.count,pattern.solid.faces.len(),
                    pattern.solid.edges.len()));
                Exact {solid:pattern.solid.clone(),pattern:Some(pattern),indexed:Some((sector.origin,sector.axis,sector.count)),swept:true}
            }
            super::sweep::Built::Whole(solid) => {
                (say.stage)(&format!("the solid by the Rust kernel: {} faces, {} edges",solid.faces.len(),solid.edges.len()));
                Exact {solid,pattern:None,indexed:None,swept:true}
            }
        })
    }
    let started = crate::clock::Instant::now();
    let solid = super::recipe::build(&recipe.recipe).at(Stage::Blank)?;
    let tol = 1e-9*solid.size().max(1.);
    solid.check(10.*tol).map_err(|e| ExportRefusal::at(Stage::Blank,format!("the built boundary is invalid: {e}")))?;
    if let Some(p) = solid.pinches().first() {
        return Err(ExportRefusal::at(Stage::Blank,format!("the solid pinches to a point at [{:.4}, {:.4}, {:.4}] mm (two of its \
            walls touch there without an edge between them): no manifold file describes it",p[0],p[1],p[2])))
    }
    (say.stage)(&format!("built the solid by the Rust kernel: {:.6} mm³, {} faces, {} edges ({:?})",super::props::volume(&solid),
        solid.faces.len(),solid.edges.len(),started.elapsed()));
    Ok(Exact {solid,pattern:None,indexed:None,swept:false})
}

/// The STEP file of a built solid, parsed back and held to it (every reference, the topology's
/// counts, units, and each face's surface and its numbers), with what the check counted.
pub fn step(exact: &Exact,name: &str,tolerance: Option<Tolerance>,say: &Say) -> Result<String,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let text = super::step::write(&exact.solid,name,tolerance.map_or(1e-4,|t| t.deflection()*0.1)).at(Stage::Step)?;
    let verified = super::step_check::verify(&text,&super::step_check::Solid::of(&exact.solid))
        .map_err(|e| ExportRefusal::at(Stage::Step,format!("the STEP file does not describe the solid: {e}")))?;
    (say.stage)(&format!("the STEP output: {} entities; {} faces, {} edges and {} vertices, each the solid's ({:?})",
        verified.entities,verified.faces,verified.edges,verified.vertices,started.elapsed()));
    (say.mark)(Stage::Step);
    Ok(text)
}

/// The binary STL of a built solid: meshed within the tolerance's deflection (a sector's mesh
/// turned into its copies), no triangle turned against its surface, its shells closed — and a
/// swept body's held to its material field, refused where they disagree.
pub fn stl(sk: &Sketch,body: usize,exact: &Exact,tolerance: Option<Tolerance>,say: &Say) -> Result<Vec<u8>,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let bar = tolerance.map_or(GROSS,|t| t.deflection());
    let m = match &exact.pattern { Some(p) => p.mesh(bar,ANGULAR),None => super::mesh::mesh(&exact.solid,bar,ANGULAR) }.at(Stage::Mesh)?;
    if m.turned > 0 { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh has {} triangles facing against their surfaces",m.turned))) }
    if m.sag > bar { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh sags {:.3} µm against {:.3} µm",m.sag*1e3,bar*1e3))) }
    let bytes = crate::mesh::stl_of(&m.triangles(),&sk.solids[body].name);
    crate::mesh::stl_shells(&bytes).at(Stage::Stl)?;
    (say.stage)(&format!("the STL output: {} triangles, sagging {:.3} µm at most against {:.3} µm, its shells checked ({:?})",
        m.tris.len(),m.sag*1e3,bar*1e3,started.elapsed()));
    (say.mark)(Stage::Stl);
    if exact.swept { field_agreement(sk,body,&bytes,tolerance,exact.indexed,say)?; }
    Ok(bytes)
}

/// A swept body's STL held to its material field: no cluster of microscopic triangles, and every
/// probe a little off each side of the mesh read on the side the mesh says (an indexed body's turned
/// into one sector and read by the cuts that reach it).
pub fn field_agreement(sk: &Sketch,body: usize,stl: &[u8],tolerance: Option<Tolerance>,indexed: Option<([f64;3],[f64;3],usize)>,say: &Say)
    -> Result<(),ExportRefusal> {
    let mesh = |message: String| ExportRefusal::at(Stage::Mesh,message);
    let refused = |message: String| ExportRefusal::at(Stage::Agreement,message);
    let scale = cad::millimetres(sk).map_err(mesh)?;
    let started = crate::clock::Instant::now();
    let (vertices,triangles) = agreement::stl_triangles(stl,scale).map_err(mesh)?;
    let verdict = match tolerance {
        None => {
            let tiny = contracts::tiny_triangles(&vertices,&triangles,scale);
            (say.stage)(&format!("mesh: {} of {} triangles under {} mm²",tiny.count,tiny.total,contracts::TINY_AREA));
            tiny.verdict()
        }
        Some(t) => {
            let tiny = contracts::tiny_triangles_under(&vertices,&triangles,scale,(t.deflection()/10.).dpowi(2));
            (say.stage)(&format!("mesh: {} of {} triangles under {:.1e} mm², at most {} in a millimetre cube",tiny.count,tiny.total,
                tiny.under,tiny.densest));
            tiny.clustered()
        }
    };
    verdict.map_err(mesh)?;
    (say.mark)(Stage::Mesh);
    let field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE).map_err(refused)?;
    let [offset,confirm,value_tolerance] = tolerance.map_or([0.1,0.025,0.02],|t| t.probe());
    let options = agreement::Options {offset:offset/scale,confirm:confirm/scale,value_tolerance:value_tolerance/scale,..Default::default()};
    let (report,how) = match indexed {
        Some((origin,axis,count)) => {
            let at = agreement::Indexed {origin:origin.map(|x| x/scale),axis,count};
            let (report,folding) = agreement::of_triangles_indexed(&vertices,&triangles,&field,at,cad::POSE_CACHE,&options).map_err(refused)?;
            (report,if folding.alike {
                format!(", each turned into one sector and read there by {} of its {} cuts (the rest proved clear of {} boxes about it; \
                    {} probes outside them read where they stand)",folding.kept,folding.operands,folding.cells,folding.whole)
            } else { ", each where it stands (the field did not read alike turned by a pitch)".into() })
        }
        None => (agreement::of_triangles_parallel(&vertices,&triangles,&field,cad::POSE_CACHE,&options).map_err(refused)?,String::new()),
    };
    let off = if tolerance.is_some() { format!("{:.4}",options.offset*scale) } else { format!("{:.2}",options.offset*scale) };
    (say.stage)(&format!("field agreement: {} of {} triangles probed {off} mm off each side{how}, {} probes unresolved, \
        {} withdrawn beside another face, {} disagree ({:?})",report.probed_triangles,report.triangles,
        report.unresolved,report.withdrawn,report.disagreements.len(),started.elapsed()));
    if report.agrees() { (say.mark)(Stage::Agreement); return Ok(()) }
    Err(ExportRefusal {stage:Stage::Agreement,condition:None,witness:report.disagreements.first().map(|d| d.point),
        message:format!("`{}`: the exported surface disagrees with the material field at {} of {} probes; nothing was written",
        sk.solids[body].name,report.disagreements.len(),report.probes)})
}
