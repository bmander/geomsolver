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
pub mod brep;

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

/// Which kernel builds a static solid for STEP and STL: OCCT where the build has it, the core's
/// own (`gcs_core::brep`) where asked (`--kernel rust`, `SOLVENT_KERNEL=rust`) or where it does not.
static RUST_KERNEL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Whether OCCT was asked for by name (`--kernel occt`, `--stl-backend occt`): a build without it
/// then refuses rather than export by another kernel.
static OCCT_ASKED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Ask for the core's own kernel (`--kernel rust`).
pub fn use_rust_kernel() { RUST_KERNEL.store(true,std::sync::atomic::Ordering::Relaxed); }

/// Ask for OCCT by name.
pub fn ask_occt() { OCCT_ASKED.store(true,std::sync::atomic::Ordering::Relaxed); }

/// Whether the core's own kernel builds this export.
pub fn rust_kernel() -> bool {
    (!cfg!(feature="occt") && !OCCT_ASKED.load(std::sync::atomic::Ordering::Relaxed))
        || RUST_KERNEL.load(std::sync::atomic::Ordering::Relaxed)
        || std::env::var("SOLVENT_KERNEL").is_ok_and(|v| v == "rust")
}

/// Verify every STEP file written in full: the kernel's read-back as well as the light check
/// (`--verify-step full`; `SOLVENT_STEP_VERIFY=full` asks the same).
pub fn verify_step_fully() {
    #[cfg(feature="occt")]
    native::step_check::ask_full();
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

/// The STEP of the kernel's `solid`, read into the core's B-rep as `b`, written by the core's writer
/// to `file` and verified: parsed back against `b` (`step_check`), and — with full verification —
/// read back by the kernel as a valid solid with as many faces, that reading handed back to the core
/// and its volume measured there the same as `b`'s to 1e-5. The reader repairs what it reads (its own
/// STEP of the pinion comes back 1.1e-6 smaller by the core's measure, ours 1.2e-6), and the kernel's
/// own volume is only good to about 1e-5 on spline faces (on the pinion it is 9e-6 off its own
/// sectors' sum), so that is held to 1e-4, a gross bar.
#[cfg(feature="occt")]
fn ours_step(session: &native::Session,solid: std::ffi::c_int,b: &gcs_core::brep::topo::Brep,name: &str,tolerance: Option<Tolerance>,
    file: &str) -> Result<String,String> {
    let started = std::time::Instant::now();
    let text = gcs_core::brep::step::write(b,name,tolerance.map_or(1e-4,|t| t.deflection()*0.1))?;
    let verified = native::step_check::verify(&text,&native::step_check::Solid::of(b))
        .map_err(|e| format!("the STEP file does not describe the solid: {e}"))?;
    std::fs::write(file,&text).map_err(|e| format!("{file}: {e}"))?;
    let wrote = started.elapsed();
    let mut said = format!("{} entities; {} faces ({} on B-splines), {} edges and {} vertices, each the solid's",verified.entities,
        verified.faces,verified.splines,verified.edges,verified.vertices);
    if native::step_check::verification() == native::step_check::Verification::Full {
        let read = session.read_step(file)?;
        session.validate(read).map_err(|e| format!("the STEP file, read back by the kernel, is not a valid solid: {e}"))?;
        let (faces,theirs) = (session.faces(read)?.len(),session.faces(solid)?.len());
        if faces != theirs { return Err(format!("the STEP file, read back by the kernel, has {faces} faces against {theirs}")) }
        let ours = gcs_core::brep::props::volume(b);
        let text = session.brep_json(read)?;
        if let Ok(path) = std::env::var("SOLVENT_BREP_DUMP") { std::fs::write(format!("{path}.read"),&text).map_err(|e| e.to_string())?; }
        let reading = gcs_core::brep::json::read(&text)?;
        let read_ours = gcs_core::brep::props::volume(&reading);
        if (read_ours-ours).abs() > 1e-5*ours.abs() {
            return Err(format!("the STEP file, read back by the kernel, measures {read_ours:.6} mm³ against {ours:.6} mm³"))
        }
        let (after,before) = (session.volume(read)?,session.volume(solid)?);
        if (after-before).abs() > 1e-4*before.abs() {
            return Err(format!("the STEP file, read back by the kernel, measures {after:.6} mm³ by the kernel against {before:.6} mm³"))
        }
        said += &format!(", and read back by the kernel as a valid solid of {faces} faces, measuring {read_ours:.6} mm³ ({:.1e} of \
            the solid's; {:.1e} by the kernel's own measure)",(read_ours-ours).abs()/ours.abs(),(after-before).abs()/before.abs());
    }
    Ok(format!("{said} (written {wrote:?}, verified {:?})",started.elapsed()-wrote))
}

/// Build `body` natively once (admitting it first where it cuts sweeps and has not been, the
/// admission beside the blank and the sheets), stage and check every requested format, judge a
/// swept body against its material field, then replace the outputs: a failure anywhere leaves them
/// all as they were. Held to a `tolerance`, a swept body's sheets are refined into it, the STL is
/// meshed within the rest of it and the field is probed as near as it says; without one the
/// export keeps its gross bars (`Tolerance`).
pub fn export(sk: &Sketch,body: &Body,step: Option<&str>,stl: Option<&str>,tolerance: Option<Tolerance>) -> Result<(),ExportRefusal> {
    if rust_kernel() { return brep::export(sk,body,step,stl,tolerance) }
    #[cfg(feature="occt")]
    {
        use progress::stage;
        use gcs_core::solid::export::AtStage;
        progress::start();
        let recipe = body.recipe.as_ref().map_err(Clone::clone).at(Stage::Blank)?;
        let session = native::Session::new().at(Stage::Blank)?;
        // `SOLVENT_WRITER=rust` (phase 1 of docs/rust-kernel-plan.md): the solid the kernel built read
        // into the core's own B-rep and its STEP and STL written from that, by our writer and mesher,
        // the solid whole (its union made before its files)
        let ours = std::env::var("SOLVENT_WRITER").is_ok_and(|v| v == "rust");
        // A body with swept cuts not yet admitted is admitted beside its blank and sheets; built as one
        // sector patterned and verified lightly, its union is checked beside its files (below).
        let defer = native::step_check::verification() == native::step_check::Verification::Light
            && std::env::var("SOLVENT_SECTOR_STL").map_or(true,|v| v != "off") && !ours;
        let admitting = body.swept() && body.admission.is_none();
        let build = |defer: bool,whole: bool| if admitting {
            native::sweep_boundary::construct_admitting(&session,sk,body.index,recipe,&|| admitted(sk,body.index),tolerance,defer,whole)
        } else { native::sweep_boundary::construct_built(&session,sk,body.index,recipe,body.admission.as_ref(),tolerance) };
        let mut staged = output::Staged::new();
        let utf8 = |p: &std::path::Path| p.to_str().map(str::to_owned).ok_or_else(|| "CAD path must be UTF-8".to_string());
        let step_file = step.map(|path| staged.file(path,"step").and_then(|t| utf8(&t))).transpose().at(Stage::Step)?;
        let stl_file = stl.map(|path| staged.file(path,"stl").and_then(|t| utf8(&t))).transpose().at(Stage::Stl)?;
        // A swept body is judged on the STL being written when there is one, meshed once.
        let scratch = if body.swept() && stl_file.is_none() { Some(staged.scratch("stl").and_then(|t| utf8(&t)).at(Stage::Mesh)?) }
            else { None };
        // Every file of a body built: Ok(Some(reason)) where its sector's union, made and checked beside
        // them, cannot be made or does not check (nothing is kept of them, and the body is built whole
        // instead).
        let solid_name = sk.solids[body.index].name.clone();
        let files = |built: &native::sweep_boundary::Built| -> Result<Option<String>,ExportRefusal> {
            let solid = built.solid;
            // the finished shape handed over for the core's own B-rep (phase 1 of the Rust kernel plan):
            // `SOLVENT_BREP_DUMP=PATH` writes it as JSON, and its sector beside it where it is built as one
            if let Ok(path) = std::env::var("SOLVENT_BREP_DUMP") {
                let dump = |shape,path: String| session.brep_json(shape).and_then(|t| std::fs::write(&path,t).map_err(|e| e.to_string()));
                // (a sector's union still to be made has no solid yet: its sector is dumped alone)
                if built.unchecked.is_none() { dump(solid,path.clone()).at(Stage::Step)?; }
                if let Some(s) = &built.sector { dump(s.piece,format!("{path}.sector")).at(Stage::Step)?; }
            }
            let converted = if ours {
                let started = std::time::Instant::now();
                let b = session.brep_json(solid).and_then(|t| gcs_core::brep::json::read(&t)).at(Stage::Step)?;
                b.check(1e-7).map_err(|e| format!("the kernel's solid, read into the core's B-rep, is invalid: {e}")).at(Stage::Step)?;
                let worst = b.edges.iter().map(|e| e.tol).fold(0.,f64::max);
                stage(&format!("read the kernel's solid into the core's B-rep: {} faces, {} edges, each within {:.3} µm of its faces \
                    ({:?})",b.faces.len(),b.edges.len(),worst*1e3,started.elapsed()));
                Some(b)
            } else { None };
            // An indexed body's field agreement reads each probe turned into one sector.
            let indexed = built.sector.as_ref().map(|s| (s.origin,s.axis,s.count));
            // A body built as one sector patterned is meshed as that sector, its triangles turned into
            // every copy (`SOLVENT_SECTOR_STL=off` meshes the patterned solid whole).
            let sector = built.sector.filter(|_| std::env::var("SOLVENT_SECTOR_STL").map_or(true,|v| v != "off"));
            // A sector whose union is still to be made is meshed as a copy of it, beside the union's making.
            let sector = match (&built.unchecked,sector) {
                (Some(_),Some(s)) => Some(native::kernel::Patterned {piece:session.copy(s.piece).at(Stage::Mesh)?,..s}),
                (_,sector) => sector,
            };
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
            let turned_copies = |sector: &native::kernel::Patterned,mut deflection: f64,mut interior: f64,angular: f64,reach: f64,bar: Option<f64>,
                path: &str|
                -> Result<(usize,f64,f64),String> {
                for tries in 1.. {
                    match session.sector_stl(sector,reach,path) {
                        Ok((triangles,moved)) => return Ok((triangles,moved,deflection)),
                        Err(e) if e.starts_with("the sector's mesh") && tries < SEAM_TRIES => {
                            deflection *= 0.9; interior *= 0.9;
                            stage(&format!("the sector's seams did not pair ({e}); meshing it again at {:.2} µm",deflection*1e3));
                            match (session.sector_mesh_with(sector,deflection,interior,angular,bar.is_some())?,bar) {
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
                if let Some(b) = &converted {
                    // the core's mesher measures its sag: the bar is a bound, met or refused
                    let started = std::time::Instant::now();
                    let bar = tolerance.map_or(0.01,|t| t.deflection());
                    let m = gcs_core::brep::mesh::mesh(b,bar,ANGULAR)?;
                    if m.sag > bar { return Err(format!("the mesh sags {:.3} µm against {:.3} µm",m.sag*1e3,bar*1e3)) }
                    if m.turned > 0 { return Err(format!("the mesh has {} triangles facing against their surfaces",m.turned)) }
                    std::fs::write(path,gcs_core::mesh::stl_of(&m.triangles(),&solid_name)).map_err(|e| format!("{path}: {e}"))?;
                    stage(&format!("meshed the solid by the core: {} triangles, sagging {:.3} µm at most against {:.3} µm ({:?})",
                        m.tris.len(),m.sag*1e3,bar*1e3,started.elapsed()));
                    return Ok(())
                }
                let Some(t) = tolerance else {
                    let Some(sector) = &sector else { return session.stl(solid,path) };
                    let started = std::time::Instant::now();
                    session.sector_mesh(sector,0.01,0.2,false)?;
                    let (triangles,moved,_) = turned_copies(sector,0.01,0.01,0.2,GROSS_SEAM,None,path)?;
                    stage(&format!("meshed one sector and turned it into {} copies: {triangles} triangles, seam points moved {:.3} µm \
                        at most onto their partners ({:?})",sector.count,moved*1e3,started.elapsed()));
                    return Ok(());
                };
                // A sector meshed again finer has its edges made finer by what its sag asks and its faces'
                // interiors by half as much: the sag the mesher leaves is mostly its edges' chords (the
                // 10 µm pinion's fillet: 31 µm at 5 µm, 7.7 µm with only its edges at a quarter of that,
                // under 4 µm with its interiors at half; at a quarter all through, the same within the bar
                // and 25% more triangles).
                let (mut deflection,mut interior) = (t.deflection(),t.deflection());
                let what = if sector.is_some() { "one sector" } else { "the solid" };
                // Finer the second time by at most a quarter (a quarter where it sags past 3.2 times its
                // share): a sector is meshed so, on a copy, beside its first meshing, the copy's meshing
                // abandoned where the first holds; where it does not and a quarter is what its sag asks,
                // the copy is the second round's (`SOLVENT_MESH_AHEAD=off` meshes a round at a time).
                let bar = t.deflection();
                let mut meshed = sector;
                type Ahead = (native::kernel::Patterned,Result<Option<(f64,[f64;3])>,String>,std::time::Duration);
                let mut ahead: Option<Ahead> = None;
                let mut first: Option<(Result<Option<(f64,[f64;3])>,String>,std::time::Duration)> = None;
                if let Some(s) = sector.filter(|_| std::env::var("SOLVENT_MESH_AHEAD").map_or(true,|v| v != "off")) {
                    let copy = native::kernel::Patterned {piece:session.copy(s.piece)?,..s};
                    let cancel = std::sync::atomic::AtomicI32::new(0);
                    let (d,i) = (deflection*0.25,(interior*0.5).min(interior));
                    let (mine,theirs) = std::thread::scope(|scope| {
                        let theirs = scope.spawn(|| {
                            let started = std::time::Instant::now();
                            (session.sector_mesh_abandoned(&copy,d,i,ANGULAR,true,Some(&cancel)),started.elapsed())
                        });
                        let started = std::time::Instant::now();
                        let mine = session.sector_mesh_with(&s,deflection,interior,ANGULAR,true);
                        if mine.as_ref().is_ok_and(|r| r.is_some_and(|(sag,_)| sag <= bar)) {
                            cancel.store(1,std::sync::atomic::Ordering::Relaxed);
                        }
                        let elapsed = started.elapsed();
                        ((mine,elapsed),theirs.join().unwrap_or_else(|e| std::panic::resume_unwind(e)))
                    });
                    first = Some(mine);
                    ahead = Some((copy,theirs.0,theirs.1));
                }
                for round in 0.. {
                    let started = std::time::Instant::now();
                    let taken = match (round,first.take()) {
                        (0,Some((result,elapsed))) => Some((result,elapsed)),
                        _ => match ahead.take() {
                            Some((copy,result,elapsed)) if round == 1 && deflection == t.deflection()*0.25 && result.is_ok() => {
                                meshed = Some(copy); Some((result,elapsed))
                            }
                            _ => None,
                        },
                    };
                    let (sag,at,elapsed) = match (taken,&meshed) {
                        (Some((result,elapsed)),_) => { let (sag,at) = result?.expect("a sag asked for"); (sag,at,elapsed) }
                        (None,Some(sector)) => {
                            let (sag,at) = session.sector_mesh_with(sector,deflection,interior,ANGULAR,true)?.expect("a sag asked for");
                            (sag,at,started.elapsed())
                        }
                        (None,None) => { session.remesh(solid,deflection,ANGULAR)?; let (sag,at) = session.mesh_sag(solid)?; (sag,at,started.elapsed()) }
                    };
                    let within = if interior != deflection { format!(" ({:.2} µm within its faces)",interior*1e3) } else { String::new() };
                    stage(&format!("meshed {what} at {:.2} µm deflection{within}: it sags {:.2} µm at most, at {:?}, against {:.2} µm ({:?})",
                        deflection*1e3,sag*1e3,at.map(|x| (x*1e3).round()/1e3),t.deflection()*1e3,elapsed));
                    if sag <= bar { break }
                    if round+1 == MOST_MESHES {
                        return Err(format!("the mesh still sags {:.2} µm at {:?} after {MOST_MESHES} meshings, against {:.2} µm, half \
                            the {} µm tolerance",sag*1e3,at.map(|x| (x*1e3).round()/1e3),t.deflection()*1e3,t.millimetres*1e3));
                    }
                    // sag goes as the deflection where the mesher heeds it; never more than a quarter at once
                    let finer = (0.8*t.deflection()/sag).max(0.25);
                    deflection *= finer;
                    interior = if sector.is_some() { (interior*2.*finer).min(interior) } else { deflection };
                }
                let sector = meshed;
                match &sector {
                    Some(sector) => {
                        let started = std::time::Instant::now();
                        let (triangles,moved,_) = turned_copies(sector,deflection,interior,ANGULAR,deflection,Some(t.deflection()),path)?;
                        stage(&format!("turned the sector's mesh into {} copies: {triangles} triangles, seam points moved {:.3} µm at \
                            most onto their partners ({:?})",sector.count,moved*1e3,started.elapsed()));
                        Ok(())
                    }
                    None => session.stl_with(solid,path,deflection,ANGULAR),
                }
            };

            // The STEP, of the union as it was made where its check runs beside the writing.
            let write_step = |solid: std::ffi::c_int,beside: bool| -> Result<(),ExportRefusal> {
                let Some(file) = &step_file else { return Ok(()) };
                if let Some(b) = &converted {
                    let verified = ours_step(&session,solid,b,&solid_name,tolerance,file).at(Stage::Step)?;
                    stage(&format!("staged the STEP output by the core's writer: {verified}"));
                    mark(Stage::Step);
                    return Ok(())
                }
                let verified = if beside { session.step_beside_check(solid,file) } else { session.step(solid,file) }.at(Stage::Step)?;
                stage(&format!("staged the STEP output: {verified}"));
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

            // A sector's union still to be checked is checked beside its STEP, and both beside the
            // mesh; written again where the check keeps the union as it was sewn instead. Otherwise the
            // STEP is written beside the mesh where the mesh is the sector's, not the solid's.
            let checked_step = || -> Result<Option<String>,ExportRefusal> {
                let Some(union) = &built.unchecked else { return write_step(solid,false).map(|_| None) };
                let made = match union.make(&session) { Ok(made) => made, Err(reason) => return Ok(Some(reason)) };
                let (checked,wrote) = progress::under(|| union.check(&session,made),|| write_step(made,true),Result::is_err);
                let kept = match checked { Ok(kept) => kept, Err(reason) => return Ok(Some(reason)) };
                if kept != made { write_step(kept,false)?; } else { wrote?; }
                Ok(None)
            };
            let (wrote,rest) = if sector.is_some() { progress::beside(checked_step,the_rest,|r| !matches!(r,Ok(None))) } else {
                let wrote = checked_step();
                let rest = if matches!(wrote,Ok(None)) { the_rest() } else { Ok(()) };
                (wrote,rest)
            };
            if let Some(reason) = wrote? { return Ok(Some(reason)) }
            rest?;
            Ok(None)
        };
        let built = build(defer,false)?;
        if let Some(reason) = files(&built)? {
            stage(&format!("`{}` is built whole: the sector construction does not apply ({reason})",sk.solids[body.index].name));
            let whole = build(false,true)?;
            if let Some(reason) = files(&whole)? { return Err(ExportRefusal::at(Stage::Fuse,reason)) }
        }
        staged.commit().at(Stage::Written)?;
        mark(Stage::Written);
        Ok(())
    }
    #[cfg(not(feature="occt"))]
    {
        let _ = (sk,body,step,stl,tolerance);
        Err(ExportRefusal::at(Stage::Blank,"CAD export by OCCT requires native OCCT support; build with `make solventc OCCT=1` \
            or Cargo's `--features occt`, or export by the core's kernel (`--kernel rust`)"))
    }
}
