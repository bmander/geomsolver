//! The export by the core's own kernel (docs/rust-kernel-plan.md, `--kernel rust`): a static solid
//! built from its CAD recipe by `gcs_core::brep`, its STEP written and its STL meshed within the
//! tolerance's deflection (the measured sag, not a control), each checked — the boundary valid, the
//! STL's shells closed — and staged, so a failure anywhere leaves every output as it was.
use super::{output,progress::stage,Body,ExportRefusal,Stage};
use gcs_core::brep;
use gcs_core::model::Sketch;
use gcs_core::solid::export::Tolerance;

/// The mesh's deflection with no tolerance given: the native export's gross bar.
const GROSS: f64 = 0.01;

pub fn export(sk: &Sketch,body: &Body,step: Option<&str>,stl: Option<&str>,tolerance: Option<Tolerance>) -> Result<(),ExportRefusal> {
    if body.swept() {
        return Err(ExportRefusal::at(Stage::Blank,"the Rust kernel builds static solids only, not yet a body with swept cuts; \
            use `--kernel occt`"))
    }
    let recipe = body.recipe.as_ref().map_err(|e| ExportRefusal::at(Stage::Blank,e.clone()))?;
    let started = std::time::Instant::now();
    let solid = brep::recipe::build(&recipe.recipe).map_err(|e| ExportRefusal::at(Stage::Blank,e))?;
    let tol = 1e-9*solid.size().max(1.);
    solid.check(10.*tol).map_err(|e| ExportRefusal::at(Stage::Blank,format!("the built boundary is invalid: {e}")))?;
    if let Some(p) = solid.pinches().first() {
        return Err(ExportRefusal::at(Stage::Blank,format!("the solid pinches to a point at [{:.4}, {:.4}, {:.4}] mm (two of its \
            walls touch there without an edge between them): no manifold file describes it",p[0],p[1],p[2])))
    }
    let volume = brep::props::volume(&solid);
    stage(&format!("built the solid by the Rust kernel: {volume:.6} mm³, {} faces, {} edges ({:?})",solid.faces.len(),
        solid.edges.len(),started.elapsed()));
    let name = sk.solids[body.index].name.clone();
    let mut staged = output::Staged::new();
    if let Some(path) = step {
        let started = std::time::Instant::now();
        let text = brep::step::write(&solid,&name,tolerance.map_or(1e-4,|t| t.deflection()*0.1));
        staged.write(path,"step",text.as_bytes()).map_err(|e| ExportRefusal::at(Stage::Step,e))?;
        stage(&format!("staged the STEP output: {} faces ({:?})",solid.faces.len(),started.elapsed()));
    }
    if let Some(path) = stl {
        let started = std::time::Instant::now();
        let bar = tolerance.map_or(GROSS,|t| t.deflection());
        let m = brep::mesh::mesh(&solid,bar,0.2).map_err(|e| ExportRefusal::at(Stage::Mesh,e))?;
        if m.sag > bar {
            return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh sags {:.3} µm against {:.3} µm",m.sag*1e3,bar*1e3)))
        }
        let bytes = gcs_core::mesh::stl_of(&m.triangles(),&name);
        gcs_core::mesh::stl_shells(&bytes).map_err(|e| ExportRefusal::at(Stage::Stl,e))?;
        staged.write(path,"stl",&bytes).map_err(|e| ExportRefusal::at(Stage::Stl,e))?;
        stage(&format!("staged the STL output: {} triangles, sagging {:.3} µm at most against {:.3} µm, its shells checked ({:?})",
            m.tris.len(),m.sag*1e3,bar*1e3,started.elapsed()));
    }
    staged.commit().map_err(|e| ExportRefusal::at(Stage::Written,e))
}
