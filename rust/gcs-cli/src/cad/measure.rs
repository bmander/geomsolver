//! Measuring an exported file against the exact surface of the body it was exported from
//! (`gcs_core::solid::accuracy`, docs/native-hypoid-plan.md). The samples are this
//! crate's to take — a STEP file's faces through the native kernel, an STL's triangles from its
//! bytes — and every reading, and every word of the report, is the core's.
#![cfg_attr(not(feature="occt"),allow(dead_code))]
use gcs_core::{model::Sketch,solid::{accuracy::{self,Meter,Measurement,Options,Sample},agreement,cad}};
use super::progress::stage;

/// Samples of an exported file, and the names of the classes of face they are numbered by.
pub struct Samples { pub samples: Vec<Sample>, pub classes: Vec<String> }

/// An STL's triangles as samples in model units: at most `most` samples, seven a triangle
/// (its centroid, its edges' midpoints, its corners), the triangles chosen by area.
pub fn stl_samples(bytes: &[u8],millimetres: f64,most: usize) -> Result<Samples,String> {
    let (vertices,triangles) = agreement::stl_triangles(bytes,millimetres)?;
    Ok(Samples {samples:accuracy::mesh_samples(&vertices,&triangles,(most/7).max(1)),classes:vec!["triangles".into()]})
}

/// OCCT's `GeomAbs_SurfaceType` names, in its order.
const SURFACE_KINDS: [&str;11] = ["plane","cylinder","cone","sphere","torus","Bezier","B-spline","revolution",
    "extrusion","offset","other"];

/// A STEP file's faces as samples in model units: a grid over each face's parameter box, the
/// points its trims keep, about `most` in all, each with the face's oriented normal and its kind.
#[cfg(feature="occt")]
pub fn step_samples(path: &str,millimetres: f64,most: usize) -> Result<Samples,String> {
    let session = super::native::Session::new()?;
    let shape = session.read_step(path)?;
    let faces = session.faces(shape)?;
    let per = ((most as f64/faces.len().max(1) as f64).sqrt().floor() as usize).clamp(3,64);
    let mut classes: Vec<String> = Vec::new();
    let mut samples = Vec::new();
    let mut failed = 0;
    for (index,&face) in faces.iter().enumerate() {
        let kind = session.face_kind(face)?;
        let name = SURFACE_KINDS.get(kind as usize).copied().unwrap_or("other");
        let class = classes.iter().position(|c| c == name).unwrap_or_else(|| { classes.push(name.into()); classes.len()-1 });
        for i in 0..per { for j in 0..per {
            let (u,v) = ((i as f64+0.5)/per as f64,(j as f64+0.5)/per as f64);
            match session.face_point(face,u,v,1e-7) {
                Ok(Some(p)) => samples.push(Sample {position:p.position.map(|x| x/millimetres),normal:Some(p.normal),
                    face:index as u32,class:class as u32,kind:accuracy::SampleKind::Face}),
                Ok(None) => {}
                // a singular point of a face's parametrisation (a sphere's pole) says nothing
                Err(_) => failed += 1,
            }
        } }
    }
    stage(&format!("sampled {} points on {} STEP faces ({per}x{per} per face, {failed} singular)",samples.len(),faces.len()));
    Ok(Samples {samples,classes})
}

/// Both routes' readings of every sample, over every core the machine has.
pub fn measure(meter: &Meter,samples: &[Sample]) -> Vec<Measurement> {
    use std::sync::{Mutex,atomic::{AtomicUsize,Ordering}};
    let started = std::time::Instant::now();
    let threads = std::thread::available_parallelism().map_or(1,|n| n.get()).min(samples.len().max(1));
    let chunk = samples.len().div_ceil(threads*16).max(1);
    let next = AtomicUsize::new(0);
    let done = Mutex::new((0_usize,0_usize));
    let results = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
                let start = next.fetch_add(chunk,Ordering::Relaxed);
                if start >= samples.len() { break; }
                let end = (start+chunk).min(samples.len());
                let read: Vec<Measurement> = samples[start..end].iter().map(|s| meter.measure(s.position)).collect();
                results.lock().unwrap().push((start,read));
                let mut d = done.lock().unwrap();
                d.0 += end-start;
                if d.0*10 >= (d.1+1)*samples.len() {
                    d.1 = d.0*10/samples.len();
                    stage(&format!("  {} of {} samples measured ({:.1} s)",d.0,samples.len(),started.elapsed().as_secs_f64()));
                }
            });
        }
    });
    // the chunks tile the samples: in order, they are every reading in the samples' order
    let mut results = results.into_inner().unwrap();
    results.sort_by_key(|(start,_)| *start);
    results.into_iter().flat_map(|(_,read)| read).collect()
}

/// Measure the file at `path` (STEP by its extension, otherwise binary STL) against `body`,
/// and the report's lines; with a `tolerance` (native millimetres), the last says whether every
/// exact face is within it, and an `Err` carries the report when one is not.
pub fn report(sk: &Sketch,body: usize,path: &str,most: usize,tolerance: Option<f64>) -> Result<Vec<String>,String> {
    let millimetres = cad::millimetres(sk)?;
    let started = std::time::Instant::now();
    let meter = Meter::read(sk,body,Options::in_units(millimetres))?;
    stage(&format!("read the exact faces of `{}`: {} of them ({:?})",sk.solids[body].name,meter.surfaces().len(),started.elapsed()));
    let step = std::path::Path::new(path).extension().and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"));
    let samples = if step {
        #[cfg(feature="occt")]
        { step_samples(path,millimetres,most)? }
        #[cfg(not(feature="occt"))]
        { return Err("measuring a STEP file requires a build with the `occt` feature".into()) }
    } else {
        let s = stl_samples(&std::fs::read(path).map_err(|e| format!("{path}: {e}"))?,millimetres,most)?;
        stage(&format!("sampled {} points on the STL's triangles",s.samples.len()));
        s
    };
    let measurements = measure(&meter,&samples.samples);
    let mut lines = vec![format!("{path}: measured against `{}`",sk.solids[body].name)];
    lines.extend(accuracy::report(&meter,&samples.samples,&measurements,&samples.classes,millimetres));
    if let Some(t) = tolerance {
        let (ok,line) = accuracy::within(&meter,&samples.samples,&measurements,t/millimetres,millimetres);
        lines.push(line);
        if !ok { return Err(lines.join("\n")); }
    }
    Ok(lines)
}
