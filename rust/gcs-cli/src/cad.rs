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
use gcs_core::{model::Sketch,solid::{admission::{self,Admission},cad,export::{ExportRefusal,Tolerance}}};

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
        self.admission = Some(admitted(sk,self.index)?);
        Ok(())
    }
}

/// The body `index` admitted to the generating-sweep class, or its refusal; what was checked said.
fn admitted(sk: &Sketch,index: usize) -> Result<Admission,ExportRefusal> {
    let a = admission::admit_body(sk,index,&admission::Options::default())?;
    mark(Stage::Admission);
    for s in a.sweeps() {
        let checked = s.placements.iter().filter(|p| p.equivalent_to.is_none()).count();
        let admission::Basis::Sampled {rows,columns} = s.basis;
        let alike = if s.placements.len() > checked {
            let why = match s.equivalence {
                admission::Equivalence::Revolved {..} => "turns of it about the axis the blank is a revolution about",
                admission::Equivalence::Sampled => "reading the blank alike",
            };
            format!(" ({checked} of {} placements checked, the rest {why})",s.placements.len())
        } else { String::new() };
        eprintln!("solventc: `{}` is in the generating-sweep class, sampled {rows}x{columns} per face{alike}",s.name);
        if std::env::var_os("SOLVENT_ADMISSION_TIMES").is_some() {
            eprintln!("admission: `{}`: {} samples, {} contacts, spacing {:e}, least area factor {:e}, {} near double roots, {} near tangent pairs",
                s.name,s.samples,s.contacts,s.spacing,s.least_area_factor,s.near_double_roots,s.near_tangent_pairs);
        }
    }
    Ok(a)
}

/// The mesher's angular deflection (radians) held to a tolerance, and how many times a mesh is made
/// finer before its sag refuses the export.
#[cfg(feature="occt")]
const ANGULAR: f64 = 0.2;
#[cfg(feature="occt")]
const MOST_MESHES: usize = 4;
/// How many times a sector is meshed for its seams to pair before the export is refused.
#[cfg(feature="occt")]
const SEAM_TRIES: usize = 4;
/// How far along its seam a point of a mesh without a tolerance (0.01 mm deflection) may be moved onto
/// its partner (mm): the two sides are meshed apart, and at that deflection the mesher has placed a
/// side's points 5.6 and 11 µm from their partners' turns; along the seam, where both lie on the same
/// two surfaces, such a move adds nothing to the mesh's distance from them. A quarter of the seam's
/// least spacing still bounds it.
#[cfg(feature="occt")]
const GROSS_SEAM: f64 = 0.02;

/// Build `body` natively once (admitting it first where it cuts sweeps and has not been, the
/// admission beside the blank and the sheets), stage and check every requested format, judge a
/// swept body against its material field, then replace the outputs: a failure anywhere leaves them
/// all as they were. Held to a `tolerance`, a swept body's sheets are refined into it, the STL is
/// meshed within the rest of it and the field is probed as near as it says; without one the
/// export keeps its gross bars (`Tolerance`).
pub fn export(sk: &Sketch,body: &Body,step: Option<&str>,stl: Option<&str>,tolerance: Option<Tolerance>) -> Result<(),ExportRefusal> {
    #[cfg(feature="occt")]
    {
        use progress::stage;
        use gcs_core::solid::export::AtStage;
        progress::start();
        let recipe = body.recipe.as_ref().map_err(Clone::clone).at(Stage::Blank)?;
        let session = native::Session::new().at(Stage::Blank)?;
        // A body with swept cuts not yet admitted is admitted beside its blank and sheets.
        let built = if body.swept() && body.admission.is_none() {
            native::sweep_boundary::construct_admitting(&session,sk,body.index,recipe,&|| admitted(sk,body.index),tolerance)?
        } else { native::sweep_boundary::construct_built(&session,sk,body.index,recipe,body.admission.as_ref(),tolerance)? };
        let solid = built.solid;
        // An indexed body's field agreement reads each probe turned into one sector.
        let indexed = built.sector.as_ref().map(|s| (s.origin,s.axis,s.count));
        // A body built as one sector patterned is meshed as that sector, its triangles turned into
        // every copy (`SOLVENT_SECTOR_STL=off` meshes the patterned solid whole).
        let sector = built.sector.filter(|_| std::env::var("SOLVENT_SECTOR_STL").map_or(true,|v| v != "off"));
        let mut staged = output::Staged::new();
        let utf8 = |p: &std::path::Path| p.to_str().map(str::to_owned).ok_or_else(|| "CAD path must be UTF-8".to_string());
        // The mesh's absolute chordal deflection (mm) and its angular one: the angle bounds how far a
        // facet's normal turns from its neighbours' and so is kept whatever the tolerance; the
        // deflection is the distance the tolerance is about. The mesher's deflection is a control
        // and not a bound: at 5 µm the pinion's mesh left 0.9 mm edges across its fillet, whose
        // centripetal parameters crowd it into a narrow band of the face's chart, 50 µm off the
        // face, and chords of the tip cone's trimmed edges 7 µm off it (a finer angle, 0.05 rad,
        // mended the first and not the second). So a mesh held to a tolerance is read for the sag
        // every triangle has (`mesh_sag`) and meshed again, finer, until that is within its share.
        // The sector's mesh turned into its copies, its seam points paired within `reach`; where the
        // mesher put the two sides' points unlike each other (the pairing refused), meshed again a tenth
        // finer, at most `SEAM_TRIES` times, each finer mesh's sag read again where a `bar` holds it.
        let turned_copies = |sector: &native::kernel::Patterned,mut deflection: f64,angular: f64,reach: f64,bar: Option<f64>,path: &str|
            -> Result<(usize,f64,f64),String> {
            for tries in 1.. {
                match session.sector_stl(sector,reach,path) {
                    Ok((triangles,moved)) => return Ok((triangles,moved,deflection)),
                    Err(e) if e.starts_with("the sector's mesh") && tries < SEAM_TRIES => {
                        deflection *= 0.9;
                        stage(&format!("the sector's seams did not pair ({e}); meshing it again at {:.2} µm",deflection*1e3));
                        match (session.sector_mesh(sector,deflection,angular,bar.is_some())?,bar) {
                            (Some((sag,at)),Some(bar)) if sag > bar => return Err(format!("the mesh sags {:.2} µm at {:?} meshed \
                                again for its seams, against {:.2} µm",sag*1e3,at.map(|x| (x*1e3).round()/1e3),bar*1e3)),
                            _ => {}
                        }
                    }
                    Err(e) => return Err(e),
                }
            }
            unreachable!("the tries end in a return")
        };
        let mesh = |path: &str| -> Result<(),String> {
            let Some(t) = tolerance else {
                let Some(sector) = &sector else { return session.stl(solid,path) };
                let started = std::time::Instant::now();
                session.sector_mesh(sector,0.01,0.2,false)?;
                let (triangles,moved,_) = turned_copies(sector,0.01,0.2,GROSS_SEAM,None,path)?;
                stage(&format!("meshed one sector and turned it into {} copies: {triangles} triangles, seam points moved {:.3} µm \
                    at most onto their partners ({:?})",sector.count,moved*1e3,started.elapsed()));
                return Ok(());
            };
            let mut deflection = t.deflection();
            let what = if sector.is_some() { "one sector" } else { "the solid" };
            for round in 0.. {
                let started = std::time::Instant::now();
                let (sag,at) = match &sector {
                    Some(sector) => session.sector_mesh(sector,deflection,ANGULAR,true)?.expect("a sag asked for"),
                    None => { session.remesh(solid,deflection,ANGULAR)?; session.mesh_sag(solid)? }
                };
                stage(&format!("meshed {what} at {:.2} µm deflection: it sags {:.2} µm at most, at {:?}, against {:.2} µm ({:?})",
                    deflection*1e3,sag*1e3,at.map(|x| (x*1e3).round()/1e3),t.deflection()*1e3,started.elapsed()));
                if sag <= t.deflection() { break }
                if round+1 == MOST_MESHES {
                    return Err(format!("the mesh still sags {:.2} µm at {:?} after {MOST_MESHES} meshings, against {:.2} µm, half \
                        the {} µm tolerance",sag*1e3,at.map(|x| (x*1e3).round()/1e3),t.deflection()*1e3,t.millimetres*1e3));
                }
                // sag goes as the deflection where the mesher heeds it; never more than a quarter at once
                deflection *= (0.8*t.deflection()/sag).max(0.25);
            }
            match &sector {
                Some(sector) => {
                    let started = std::time::Instant::now();
                    let (triangles,moved,_) = turned_copies(sector,deflection,ANGULAR,deflection,Some(t.deflection()),path)?;
                    stage(&format!("turned the sector's mesh into {} copies: {triangles} triangles, seam points moved {:.3} µm at \
                        most onto their partners ({:?})",sector.count,moved*1e3,started.elapsed()));
                    Ok(())
                }
                None => session.stl_with(solid,path,deflection,ANGULAR),
            }
        };
        let step_file = step.map(|path| staged.file(path,"step").and_then(|t| utf8(&t))).transpose().at(Stage::Step)?;
        let stl_file = stl.map(|path| staged.file(path,"stl").and_then(|t| utf8(&t))).transpose().at(Stage::Stl)?;
        // A swept body is judged on the STL being written when there is one, meshed once.
        let scratch = if body.swept() && stl_file.is_none() { Some(staged.scratch("stl").and_then(|t| utf8(&t)).at(Stage::Mesh)?) }
            else { None };
        let write_step = || -> Result<(),ExportRefusal> {
            let Some(file) = &step_file else { return Ok(()) };
            session.step(solid,file).at(Stage::Step)?;
            stage("staged the STEP output");
            mark(Stage::Step);
            Ok(())
        };
        let the_rest = || -> Result<(),ExportRefusal> {
            // The STL written, then its shells checked beside the field agreement's reading of the same
            // bytes, what each says said in that order.
            let bytes = match &stl_file {
                Some(file) => Some((|| -> Result<Vec<u8>,String> { mesh(file)?; std::fs::read(file).map_err(|e| e.to_string()) })()
                    .at(Stage::Stl)?),
                None => None,
            };
            let check = || -> Result<(),ExportRefusal> {
                let Some(bytes) = &bytes else { return Ok(()) };
                let started = std::time::Instant::now();
                output::check_stl(bytes,"native float32 STL validation failed").at(Stage::Stl)?;
                stage(&format!("staged the STL output, its shells checked in {:?}",started.elapsed()));
                mark(Stage::Stl);
                Ok(())
            };
            let judge = || -> Result<(),ExportRefusal> {
                if !body.swept() { return Ok(()) }
                let owned;
                let stl: &[u8] = match (&bytes,&scratch) {
                    (Some(bytes),_) => bytes,
                    (None,Some(path)) => {
                        let meshed = || -> Result<Vec<u8>,String> {
                            let started = std::time::Instant::now();
                            mesh(path)?;
                            let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
                            stage(&format!("meshed the solid for its field agreement ({:?})",started.elapsed()));
                            Ok(bytes)
                        };
                        owned = meshed().at(Stage::Mesh)?;
                        &owned
                    }
                    (None,None) => unreachable!("a swept body's mesh has a file"),
                };
                output::field_agreement(sk,body.index,stl,tolerance,indexed)
            };
            let (checked,judged) = progress::beside(check,judge,Result::is_err);
            checked?;
            judged
        };
        // The STEP is written beside the mesh where the mesh is the sector's, not the solid's.
        let (wrote,rest) = if sector.is_some() { progress::beside(write_step,the_rest,Result::is_err) } else {
            let wrote = write_step();
            let rest = if wrote.is_ok() { the_rest() } else { Ok(()) };
            (wrote,rest)
        };
        wrote?;
        rest?;
        staged.commit().at(Stage::Written)?;
        mark(Stage::Written);
        Ok(())
    }
    #[cfg(not(feature="occt"))]
    {
        let _ = (sk,body,step,stl,tolerance);
        Err(ExportRefusal::at(Stage::Blank,"CAD export requires native OCCT support; build with `make solventc OCCT=1` \
            or Cargo's `--features occt`"))
    }
}
