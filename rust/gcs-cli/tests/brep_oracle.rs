//! The Rust B-rep kernel held to OCCT (docs/rust-kernel-plan.md: OCCT as the test-only oracle):
//! every node of every corpus object's CAD recipe that the kernel builds, built by both, the
//! volumes equal, faces equal by kind and the boundary valid; every object against the core's
//! faceted kernel; every body written as STEP and parsed back against itself (read back by OCCT as
//! a valid solid with its faces, in the slow tier), and meshed closed within its bar. A node the kernel refuses by name (an intersection it
//! does not trace yet) is counted, never passed over silently.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{geom::Surface,props::volume,recipe};
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
        // `BREP_ORACLE_ONLY=name` runs one example
        if std::env::var("BREP_ORACLE_ONLY").is_ok_and(|only| !name.contains(&only)) { continue }
        let Some(e) = e else { continue };
        if !e.ok() { continue }
        let mut sk = e.sketch.clone();
        gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        for root in gcs_core::overview::objects(&sk) {
            let Ok(r) = gcs_core::solid::cad::recipe(&sk,root) else { continue };
            let root_id = r.get("root").unwrap().as_i64();
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
                let (v,faces) = (volume(&b),b.faces.len());
                // OCCT builds no lofts: for one (and what is built of it) the faceted kernel is the reference
                let theirs = match session.construct(&sub) {
                    Ok(t) => Some(t),
                    Err(err) if err.contains("along-guide lofts") => { eprintln!("{label}: {v:.9} mm³ (OCCT builds no lofts)"); None }
                    Err(err) => panic!("{label}: OCCT: {err}"),
                };
                if let Some(theirs) = theirs {
                let w = session.volume(theirs).unwrap();
                let their_faces = session.faces(theirs).unwrap().len();
                let rel = (v-w).abs()/w.abs();
                eprintln!("{label}: {v:.9} mm³ against {w:.9}, {rel:e}; {faces} faces against {their_faces}");
                worst = worst.max(rel);
                // a `through` prism's extent is its sources' box, which OCCT widens by its tolerances and,
                // about a spline face, takes from its poles, where this kernel samples the face: the
                // prism's depth is then a little different and nothing cut by it is
                let swept = |b: &gcs_core::brep::topo::Brep| b.faces.iter().any(|f| matches!(f.surface,Surface::Extrusion(..) | Surface::Revolution(..)));
                let through_splines = n.get("kind").unwrap().as_str() == "through"
                    && n.get("sources").unwrap().arr().iter().any(|s| ours.get(&s.as_i64()).is_some_and(swept));
                // OCCT integrates a face swept from a spline (its adaptive rule asked for 1e-9) to a few
                // parts in a million, where this kernel's Gauss rule is exact on each knot span: on
                // `solid_tooth` fitted within 1e-7 mm this kernel meets the involute's closed form to
                // 1e-9 and OCCT is 4e-8 off; fitted within 1e-4 mm, OCCT is 2e-6 off (`tests/brep.rs`
                // and `brep_census.rs` hold this kernel to the closed forms)
                let bar = if through_splines { 1e-3 } else if swept(&b) { 1e-5 } else { 1e-7 };
                if rel >= bar { failures.push(format!("{label}: volume {v} against OCCT's {w}")); continue }
                // faces by the kind of their surface (OCCT's `GeomAbs_SurfaceType` order): OCCT may split
                // a plane along a line where another face only touches it, which this kernel does not
                let kinds = |ks: &mut Vec<i32>| { ks.sort(); ks.clone() };
                let our_kinds = kinds(&mut b.faces.iter().map(|f| match f.surface {
                    Surface::Plane(_) => 0,Surface::Cylinder(..) => 1,Surface::Cone(..) => 2,Surface::Sphere(..) => 3,Surface::Torus(..) => 4,
                    Surface::Revolution(..) => 7,Surface::Extrusion(..) => 8,Surface::Blend(..) | Surface::BSpline(..) => 6,
                }).collect());
                let their_kinds = kinds(&mut session.faces(theirs).unwrap().into_iter().map(|g| session.face_kind(g).unwrap()).collect());
                if our_kinds != their_kinds {
                    let extra_planes = their_kinds.iter().filter(|&&k| k != 0).eq(our_kinds.iter().filter(|&&k| k != 0))
                        && their_kinds.iter().filter(|&&k| k == 0).count() > our_kinds.iter().filter(|&&k| k == 0).count();
                    if extra_planes { eprintln!("  (OCCT splits a plane along a line it only touches: {their_kinds:?} against our {our_kinds:?})"); }
                    else { failures.push(format!("{label}: faces by kind {our_kinds:?} against OCCT's {their_kinds:?}")); continue }
                }
                }
                let whole = n.get("kind").unwrap().as_str() == "body" || id == root_id;
                if whole && !b.pinches().is_empty() {
                    // a solid touching itself at a point has no manifold file (OCCT's export of one refuses too)
                    refused.push(format!("{label}: pinches at {:?}",b.pinches()[0]));
                } else if whole {
                    // our STEP, parsed back and checked against our solid face by face
                    let text = match gcs_core::brep::step::write(&b,"oracle",1e-5) {
                        Ok(text) => text,
                        Err(err) => { failures.push(format!("{label}: not written as STEP: {err}")); continue }
                    };
                    if let Err(err) = native::step_check::verify(&text,&native::step_check::Solid::of(&b)) {
                        failures.push(format!("{label}: our STEP does not describe it: {err}")); continue
                    }
                    // the slow tier: read back by OCCT as a valid solid with our faces. Its volume of the
                    // reading is only a check against gross misreading: the meter finds the file's faces
                    // on their surfaces exactly where OCCT's integration of it is 2e-4 off
                    if cfg!(feature="slow") {
                        let file = std::env::temp_dir().join(format!("solvent-brep-oracle-{}.step",std::process::id()));
                        std::fs::write(&file,&text).unwrap();
                        let read = session.read_step(file.to_str().unwrap());
                        let _ = std::fs::remove_file(&file);
                        let read = read.and_then(|r| session.validate(r).map(|_| r))
                            .and_then(|r| Ok((session.volume(r)?,session.faces(r)?.len())));
                        match read {
                            Ok((u,n)) if n == faces && ((u-v)/v).abs() < 1e-3 => {}
                            Ok((u,n)) => { failures.push(format!("{label}: our STEP reads back as {u} mm³ in {n} faces against {v} in {faces}")); continue }
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
                // the object itself against the core's faceted kernel, cut as it cuts a mesh (its sagitta a
                // small fraction of the object's diagonal): to its faceting, a part in 200
                if id == root_id {
                    let scale = gcs_core::solid::cad::millimetres(&sk).unwrap();
                    let facets = sk.evaluated_solid(root,gcs_core::solid::ApproximationPolicy::Mesh).map(|e| e.volume()*scale.powi(3));
                    match facets {
                        Ok(w) if ((v-w)/v).abs() < 5e-3 => {}
                        Ok(w) => { failures.push(format!("{label}: the faceted kernel's volume {w} against {v}")); continue }
                        Err(err) => eprintln!("  (not faceted: {err})"),
                    }
                }
                ours.insert(id,b);
                compared += 1;
            }
        }
    }
    eprintln!("{compared} nodes compared, worst {worst:e}; {bodies} bodies written as STEP and checked (read back by OCCT in the slow tier), and meshed; not built: {skipped:?}");
    for r in &refused { eprintln!("refused {r}"); }
    for f in &failures { eprintln!("FAILED {f}"); }
    assert!(compared > 100 || std::env::var_os("BREP_ORACLE_ONLY").is_some(),"{compared}");
    assert!(failures.is_empty(),"{} failures",failures.len());
    assert!(refused.len() <= 8,"{} refused",refused.len());
}
