//! The Rust B-rep kernel held to OCCT (docs/rust-kernel-plan.md: OCCT as the test-only oracle):
//! every node of every corpus object's CAD recipe that the kernel builds, built by both, the
//! volumes equal and the boundary valid.
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
fn every_primitive_the_kernel_builds_is_occts() {
    let session = native::Session::new().unwrap();
    let (mut compared,mut skipped) = (0,BTreeMap::<String,usize>::new());
    let mut worst: f64 = 0.;
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
                let b = match recipe::node(n,&ours) {
                    Ok(b) => b,
                    Err(err) => { *skipped.entry(err).or_default() += 1; continue }
                };
                let tol = 1e-9*b.size().max(1.);
                b.check(tol).unwrap_or_else(|err| panic!("{label}: {err}"));
                let sub = object([("schema",1.into()),("units","mm".into()),("root",id.into()),
                    ("nodes",Json::Arr(nodes[..=i].to_vec()))]);
                let theirs = session.construct(&sub).unwrap_or_else(|err| panic!("{label}: OCCT: {err}"));
                let (v,w) = (volume(&b),session.volume(theirs).unwrap());
                let (faces,their_faces) = (b.faces.len(),session.faces(theirs).unwrap().len());
                let rel = (v-w).abs()/w.abs();
                eprintln!("{label}: {v:.9} mm³ against {w:.9}, {rel:e}; {faces} faces against {their_faces}");
                worst = worst.max(rel);
                // a `through` prism's extent is its sources' box, which OCCT widens by its tolerances
                assert!(rel < 1e-7,"{label}: volume {v} against OCCT's {w}");
                assert_eq!(faces,their_faces,"{label}: faces");
                ours.insert(id,b);
                compared += 1;
            }
        }
    }
    eprintln!("{compared} nodes compared, worst {worst:e}; not built: {skipped:?}");
    assert!(compared > 100,"{compared}");
}
