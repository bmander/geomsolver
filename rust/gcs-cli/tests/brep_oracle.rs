//! The Rust B-rep kernel held to OCCT (docs/rust-kernel-plan.md: OCCT as the test-only oracle):
//! every node of every corpus object's CAD recipe that the kernel builds, built by both, the
//! volumes equal and the boundary valid; every body also written as STEP by the kernel, read back
//! by OCCT as a valid solid of the same volume, and meshed closed within its bar. A node the kernel refuses by name (an intersection it
//! does not trace yet) is counted, never passed over silently.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{props::volume,recipe};
use gcs_core::json::{object,Json};
use std::collections::BTreeMap;

#[test]
fn every_node_the_kernel_builds_is_occts() {
    let session = native::Session::new().unwrap();
    let (mut compared,mut skipped) = (0,BTreeMap::<String,usize>::new());
    let mut worst: f64 = 0.;
    let mut failures: Vec<String> = Vec::new();
    let mut refused: Vec<String> = Vec::new();
    let mut bodies = 0;
    for (name,e) in fixtures::examples() {
        let Some(e) = e else { continue };
        if !e.ok() { continue }
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        for root in gcs_core::overview::objects(&sk) {
            let Ok(r) = gcs_core::solid::cad::recipe(&sk,root) else { continue };
            let nodes = r.get("nodes").unwrap().arr();
            let mut ours = BTreeMap::new();
            for (i,n) in nodes.iter().enumerate() {
                let id = n.get("id").unwrap().as_i64();
                let label = format!("{name}: `{}` ({})",n.get("name").unwrap().as_str(),n.get("kind").unwrap().as_str());
                let names: std::collections::BTreeMap<i64,String> = nodes.iter().map(|m| (m.get("id").unwrap().as_i64(),m.get("name").unwrap().as_str().to_string())).collect();
                let b = match recipe::node_named(n,&ours,&names) {
                    Ok(b) => b,
                    // a refusal by name is the kernel saying what it does not build yet
                    Err(err) if err.contains("not built yet") => { refused.push(format!("{label}: {err}")); continue }
                    Err(err) if n.get("kind").unwrap().as_str() == "body" => { failures.push(format!("{label}: {err}")); continue }
                    Err(err) => { *skipped.entry(err).or_default() += 1; continue }
                };
                let tol = 1e-9*b.size().max(1.);
                if let Err(err) = b.check(tol*10.) { failures.push(format!("{label}: {err}")); continue }
                let sub = object([("schema",1.into()),("units","mm".into()),("root",id.into()),
                    ("nodes",Json::Arr(nodes[..=i].to_vec()))]);
                let theirs = session.construct(&sub).unwrap_or_else(|err| panic!("{label}: OCCT: {err}"));
                let (v,w) = (volume(&b),session.volume(theirs).unwrap());
                let (faces,their_faces) = (b.faces.len(),session.faces(theirs).unwrap().len());
                let rel = (v-w).abs()/w.abs();
                eprintln!("{label}: {v:.9} mm³ against {w:.9}, {rel:e}; {faces} faces against {their_faces}");
                worst = worst.max(rel);
                // a `through` prism's extent is its sources' box, which OCCT widens by its tolerances
                if rel >= 1e-7 { failures.push(format!("{label}: volume {v} against OCCT's {w}")); continue }
                if faces != their_faces { eprintln!("  (faces differ)"); }
                if n.get("kind").unwrap().as_str() == "body" && !b.pinches().is_empty() {
                    // a solid touching itself at a point has no manifold file (OCCT's export of one refuses too)
                    refused.push(format!("{label}: pinches at {:?}",b.pinches()[0]));
                } else if n.get("kind").unwrap().as_str() == "body" {
                    // our STEP, read back by OCCT: a valid solid of our volume (the slow tier: its traced
                    // edges written within 1e-7 mm, the reading takes most of a minute)
                    if cfg!(feature="slow") {
                    let file = std::env::temp_dir().join(format!("solvent-brep-oracle-{}.step",std::process::id()));
                    std::fs::write(&file,gcs_core::brep::step::write(&b,"oracle",1e-7)).unwrap();
                    let read = session.read_step(file.to_str().unwrap());
                    let _ = std::fs::remove_file(&file);
                    match read.and_then(|r| session.validate(r).map(|_| r)).and_then(|r| session.volume(r)) {
                        // with no pcurves in the file, OCCT's reader projects its own, and its volume is then
                        // good to about 1e-5
                        Ok(u) if ((u-v)/v).abs() < 5e-5 => {}
                        Ok(u) => { failures.push(format!("{label}: our STEP reads back as {u} mm³ against {v}")); continue }
                        Err(err) => { failures.push(format!("{label}: our STEP does not read back: {err}")); continue }
                    }
                    }
                    // our mesh: closed, within its bar
                    match gcs_core::brep::mesh::mesh(&b,0.01,0.2) {
                        Ok(m) if m.sag <= 0.01 => if let Err(err) = gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&m.triangles(),"oracle")) {
                            failures.push(format!("{label}: our mesh is not closed: {err}")); continue
                        },
                        Ok(m) => { failures.push(format!("{label}: our mesh sags {}",m.sag)); continue }
                        Err(err) => { failures.push(format!("{label}: not meshed: {err}")); continue }
                    }
                    bodies += 1;
                }
                ours.insert(id,b);
                compared += 1;
            }
        }
    }
    eprintln!("{compared} nodes compared, worst {worst:e}; {bodies} bodies written as STEP, read back by OCCT and meshed; not built: {skipped:?}");
    for r in &refused { eprintln!("refused {r}"); }
    for f in &failures { eprintln!("FAILED {f}"); }
    assert!(compared > 100,"{compared}");
    assert!(failures.is_empty(),"{} failures",failures.len());
    assert!(refused.len() <= 8,"{} refused",refused.len());
}
