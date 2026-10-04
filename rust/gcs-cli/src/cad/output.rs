//! Every file an export writes, and the gate a swept body's mesh passes first. Each output is
//! staged beside its destination, checked, and only then renamed into place, so a refusal at
//! any stage leaves every earlier output as it was; a refused mesh is kept for inspection in
//! one place (`SOLVENT_KEEP_REJECTED`). A backend produces bytes or a staged file and nothing
//! else: whether it may be written is decided here.
// Without the native kernel, only the plain stager is reached.
#![cfg_attr(not(feature="occt"),allow(dead_code))]
use super::progress::{mark,stage,Stage};
use gcs_core::solid::export::ExportRefusal;
use std::{path::{Path,PathBuf},sync::atomic::{AtomicU64,Ordering}};

/// Outputs staged in private directories, renamed into place together by `commit`. Dropped
/// uncommitted, it removes what it staged and leaves the destinations alone.
#[derive(Default)]
pub struct Staged { files: Vec<(PathBuf,PathBuf)>,directories: Vec<PathBuf> }

static NEXT: AtomicU64 = AtomicU64::new(0);

impl Staged {
    pub fn new() -> Staged { Staged::default() }

    /// A fresh directory under `parent`, named for this process.
    fn directory(&mut self,parent: &Path,prefix: &str) -> Result<PathBuf,String> {
        loop {
            let d = parent.join(format!("{prefix}-{}-{}",std::process::id(),NEXT.fetch_add(1,Ordering::Relaxed)));
            match std::fs::create_dir(&d) {
                Ok(()) => { self.directories.push(d.clone()); return Ok(d) }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(format!("cannot create CAD output directory: {e}")),
            }
        }
    }

    /// Where the file for `path` is staged, `kind` its extension: beside the destination, so
    /// the rename that commits it never crosses a filesystem.
    pub fn file(&mut self,path: &str,kind: &str) -> Result<PathBuf,String> {
        let output = Path::new(path);
        let parent = output.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
        let destination = parent.canonicalize().map_err(|e| e.to_string())?
            .join(output.file_name().ok_or("CAD output needs a filename")?);
        if self.files.iter().any(|(_,p)| p == &destination) {
            return Err("STEP and STL need distinct output paths".into());
        }
        let temporary = self.directory(parent,".solvent-cad")?.join(format!("solid.{kind}"));
        self.files.push((temporary.clone(),destination));
        Ok(temporary)
    }

    /// Stage `bytes` as the file for `path`.
    pub fn write(&mut self,path: &str,kind: &str,bytes: &[u8]) -> Result<(),String> {
        let temporary = self.file(path,kind)?;
        std::fs::write(&temporary,bytes).map_err(|e| format!("{path}: {e}"))
    }

    /// A file for a check whose output nobody asked for, never committed.
    pub fn scratch(&mut self,kind: &str) -> Result<PathBuf,String> {
        Ok(self.directory(&std::env::temp_dir(),"solvent-agreement")?.join(format!("probe.{kind}")))
    }

    /// Rename every staged file into place.
    pub fn commit(self) -> Result<(),String> {
        for (temporary,output) in &self.files {
            std::fs::rename(temporary,output).map_err(|e| format!("cannot replace CAD output: {e}"))?;
        }
        Ok(())
    }
}

impl Drop for Staged {
    fn drop(&mut self) { for d in &self.directories { let _ = std::fs::remove_dir_all(d); } }
}

/// One file, staged and renamed into place: an output that has no check of its own is still
/// never left half written.
pub fn write(path: &str,bytes: &[u8]) -> Result<(),String> {
    let kind = Path::new(path).extension().and_then(|e| e.to_str()).unwrap_or("out");
    let mut staged = Staged::new();
    staged.write(path,kind,bytes)?;
    staged.commit()
}

/// A diagnostic, never the requested output: the refused mesh, where `SOLVENT_KEEP_REJECTED`
/// says to keep it.
pub fn keep_rejected(stl: &[u8]) {
    let Ok(path) = std::env::var("SOLVENT_KEEP_REJECTED") else { return };
    match std::fs::write(&path,stl) { Ok(()) => stage(&format!("kept the refused mesh at {path}")),
        Err(e) => stage(&format!("could not keep the refused mesh at {path}: {e}")) }
}

/// The shell check every STL passes before it is staged; `what` says whose STL failed.
pub fn check_stl(stl: &[u8],what: &str) -> Result<(),String> {
    gcs_core::mesh::stl_shells(stl).map(|_| ()).map_err(|e| { keep_rejected(stl); format!("{what}: {e}") })
}

/// A body with swept cuts is judged against its own material field before anything is
/// written, whichever backend built it: probes a little inside and outside the triangles of
/// its STL (`gcs_core::solid::agreement`, millimetres), the check the traced-sheet
/// arrangement failed while its volume and its shell passed. A disagreement refuses the export.
/// First the mesh contract (`solid::contracts`): no cluster of microscopic triangles. Held to a
/// `tolerance`, the probes stand off as it says (`Tolerance::probe`); otherwise 0.1 mm.
/// An indexed body (`indexed`: its axis in millimetres and its count) is probed a sector at a time
/// (`agreement::of_triangles_indexed`); `SOLVENT_AGREEMENT=whole` probes it as any other.
pub fn field_agreement(sk: &gcs_core::model::Sketch,body: usize,stl: &[u8],tolerance: Option<gcs_core::solid::export::Tolerance>,
    indexed: Option<([f64;3],[f64;3],usize)>) -> Result<(),ExportRefusal> {
    use gcs_core::solid::{agreement,cad,contracts,MaterialField};
    let mesh = |message: String| ExportRefusal::at(Stage::Mesh,message);
    let refused = |message: String| ExportRefusal::at(Stage::Agreement,message);
    let scale = cad::millimetres(sk).map_err(mesh)?;
    let started = std::time::Instant::now();
    let (vertices,triangles) = agreement::stl_triangles(stl,scale).map_err(mesh)?;
    let under = tolerance.map_or(contracts::TINY_AREA,|t| (t.deflection()/10.).powi(2));
    let tiny = contracts::tiny_triangles_under(&vertices,&triangles,scale,under);
    stage(&format!("mesh: {} of {} triangles under {:.1e} mm², at most {} in a millimetre cube",tiny.count,tiny.total,
        tiny.under,tiny.densest));
    if let Err(e) = tiny.clustered() { keep_rejected(stl); return Err(mesh(e)); }
    mark(Stage::Mesh);
    let field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE).map_err(refused)?;
    let [offset,confirm,value_tolerance] = tolerance.map_or([0.1,0.025,0.02],|t| t.probe());
    let options = agreement::Options {offset:offset/scale,confirm:confirm/scale,value_tolerance:value_tolerance/scale,..Default::default()};
    let indexed = indexed.filter(|_| std::env::var("SOLVENT_AGREEMENT").map_or(true,|v| v != "whole"));
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
    stage(&format!("field agreement: {} of {} triangles probed {off} mm off each side{how}, {} probes unresolved, \
        {} withdrawn beside another face, {} disagree ({:?})",report.probed_triangles,report.triangles,
        report.unresolved,report.withdrawn,report.disagreements.len(),started.elapsed()));
    if report.agrees() { mark(Stage::Agreement); return Ok(()); }
    keep_rejected(stl);
    for d in report.disagreements.iter().take(10) {
        eprintln!("solventc:   {} the mesh at ({:.4}, {:.4}, {:.4}) the field reads [{:.4}, {:.4}]",
            if d.inside_mesh { "inside" } else { "outside" },d.point[0]*scale,d.point[1]*scale,d.point[2]*scale,
            d.field[0]*scale,d.field[1]*scale);
    }
    Err(ExportRefusal {stage:Stage::Agreement,condition:None,witness:report.disagreements.first().map(|d| d.point),
        message:format!("`{}`: the exported surface disagrees with the material field at {} of {} probes; nothing was written",
        sk.solids[body].name,report.disagreements.len(),report.probes)})
}
