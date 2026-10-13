//! The export by the core's own kernel (docs/rust-kernel-plan.md, the default; `--kernel occt` asks
//! for OCCT): a solid built by `gcs_core::brep` — from its CAD recipe, or with swept cuts of the
//! generating class as one sector patterned — its STEP written and parsed back against it, its STL
//! meshed within the tolerance's deflection and a swept body's held to its material field, by the
//! same functions the browser's export calls (`brep::export`); staged, so a failure anywhere leaves
//! every output as it was.
use super::{output,progress::stage,Body,ExportRefusal,Stage};
use gcs_core::brep;
use gcs_core::model::Sketch;
use gcs_core::solid::export::Tolerance;

pub fn export(sk: &Sketch,body: &Body,step: Option<&str>,stl: Option<&str>,tolerance: Option<Tolerance>) -> Result<(),ExportRefusal> {
    super::progress::start();
    let say = brep::sweep::Say {stage:&|line: &str| stage(line),mark:&|m| super::mark(m)};
    let exact = brep::export::exact(sk,body.index,body.admission.as_ref(),tolerance,&say)?;
    let name = sk.solids[body.index].name.clone();
    let mut staged = output::Staged::new();
    if let Some(path) = step {
        let text = brep::export::step(&exact,&name,tolerance,&say)?;
        let file = staged.file(path,"step").map_err(|e| ExportRefusal::at(Stage::Step,e))?;
        std::fs::write(&file,text.as_bytes()).map_err(|e| ExportRefusal::at(Stage::Step,format!("{path}: {e}")))?;
        // a build with the kernel reads the file back by it, as a consumer's reader would, when
        // verification is full
        #[cfg(feature="occt")]
        if super::native::step_check::verification() == super::native::step_check::Verification::Full {
            let started = std::time::Instant::now();
            let session = super::native::Session::new().map_err(|e| ExportRefusal::at(Stage::Step,e))?;
            let said = super::read_back(&session,&exact.solid,&file.to_string_lossy()).map_err(|e| ExportRefusal::at(Stage::Step,e))?;
            stage(&format!("the STEP file, each the solid's{said} ({:?})",started.elapsed()));
        }
        stage("staged the STEP output");
    }
    // a swept body is meshed and judged against its material field whether or not an STL is asked for
    if stl.is_some() || exact.swept {
        let bytes = brep::export::stl(sk,body.index,&exact,tolerance,&say)?;
        if let Some(path) = stl {
            staged.write(path,"stl",&bytes).map_err(|e| ExportRefusal::at(Stage::Stl,e))?;
            stage("staged the STL output");
        }
    }
    staged.commit().map_err(|e| ExportRefusal::at(Stage::Written,e))?;
    super::mark(Stage::Written);
    Ok(())
}
