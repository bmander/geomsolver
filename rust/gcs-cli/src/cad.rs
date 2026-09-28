//! Exporting a solid: which body, whether its sweeps are admitted, how it is built, and the
//! one stager every output goes through (`output`). The native kernel is optional; the core
//! and WebAssembly have no OCCT dependency.
pub mod progress;
pub mod output;
pub mod measure;
#[cfg(feature="occt")]
mod native;
#[cfg(feature="occt")]
pub mod field_mesh;

pub use progress::{mark,Stage};
use gcs_core::{model::Sketch,solid::{admission::{self,Admission},cad,export::ExportRefusal}};

/// A body to export: its static recipe, read once, and — when it cuts continuous sweeps —
/// the admission that lets the native construction build them.
pub struct Body {
    pub index: usize,
    recipe: Result<cad::StaticRecipe,String>,
    admission: Option<Admission>,
}

impl Body {
    pub fn read(sk: &Sketch,index: usize) -> Body {
        Body {index,recipe:cad::recipe_static(sk,index),admission:None}
    }

    /// Whether the body cuts continuous sweeps. A body whose recipe cannot be read is not
    /// said to; the export that needs the recipe reports why it cannot be read.
    pub fn swept(&self) -> bool { self.recipe.as_ref().is_ok_and(|r| !r.sweeps.is_empty()) }

    /// Admit the body's sweeps to the generating-sweep class (docs/generating-sweeps.md): a
    /// refusal names the row it fails and a point where, and nothing is built.
    pub fn admit(&mut self,sk: &Sketch) -> Result<(),ExportRefusal> {
        let a = admission::admit_body(sk,self.index,&admission::Options::default())?;
        mark(Stage::Admission);
        for s in a.sweeps() {
            let checked = s.placements.iter().filter(|p| p.equivalent_to.is_none()).count();
            let admission::Basis::Sampled {rows,columns} = s.basis;
            let alike = if s.placements.len() > checked {
                format!(" ({checked} of {} placements checked, the rest reading the blank alike)",s.placements.len())
            } else { String::new() };
            eprintln!("solventc: `{}` is in the generating-sweep class, sampled {rows}x{columns} per face{alike}",s.name);
        }
        self.admission = Some(a);
        Ok(())
    }
}

/// Build `body` natively once, stage and check every requested format, judge a swept body
/// against its material field, then replace the outputs: a failure anywhere leaves them all
/// as they were.
pub fn export(sk: &Sketch,body: &Body,step: Option<&str>,stl: Option<&str>) -> Result<(),ExportRefusal> {
    #[cfg(feature="occt")]
    {
        use progress::stage;
        use gcs_core::solid::export::AtStage;
        let recipe = body.recipe.as_ref().map_err(Clone::clone).at(Stage::Blank)?;
        let session = native::Session::new().at(Stage::Blank)?;
        let solid = native::sweep_boundary::construct_solid(&session,sk,body.index,recipe,body.admission.as_ref())?;
        let mut staged = output::Staged::new();
        let utf8 = |p: &std::path::Path| p.to_str().map(str::to_owned).ok_or_else(|| "CAD path must be UTF-8".to_string());
        for (kind,stage_of,path) in [("step",Stage::Step,step),("stl",Stage::Stl,stl)] {
            let Some(path) = path else { continue };
            let written = |staged: &mut output::Staged| -> Result<(),String> {
                let temporary = staged.file(path,kind)?;
                if kind == "step" { session.step(solid,&utf8(&temporary)?)?; } else {
                    session.stl(solid,&utf8(&temporary)?)?;
                    let bytes = std::fs::read(&temporary).map_err(|e| e.to_string())?;
                    output::check_stl(&bytes,"native float32 STL validation failed")?;
                }
                Ok(())
            };
            written(&mut staged).at(stage_of)?;
            stage(&format!("staged the {} output",kind.to_uppercase()));
            mark(stage_of);
        }
        // A swept body is judged on the STL being written when there is one, meshed once.
        if body.swept() {
            let meshed = |staged: &mut output::Staged| -> Result<Vec<u8>,String> {
                Ok(match staged.staged("stl") {
                    Some(path) => std::fs::read(path).map_err(|e| e.to_string())?,
                    None => {
                        let path = staged.scratch("stl")?;
                        let started = std::time::Instant::now();
                        session.stl(solid,&utf8(&path)?)?;
                        let bytes = std::fs::read(&path).map_err(|e| e.to_string())?;
                        stage(&format!("meshed the solid for its field agreement ({:?})",started.elapsed()));
                        bytes
                    }
                })
            };
            let stl = meshed(&mut staged).at(Stage::Mesh)?;
            output::field_agreement(sk,body.index,&stl)?;
        }
        staged.commit().at(Stage::Written)?;
        mark(Stage::Written);
        Ok(())
    }
    #[cfg(not(feature="occt"))]
    {
        let _ = (sk,body,step,stl);
        Err(ExportRefusal::at(Stage::Blank,"CAD export requires native OCCT support; build with `make solventc OCCT=1` \
            or Cargo's `--features occt`"))
    }
}
