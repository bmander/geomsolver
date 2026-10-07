//! **A fillet held to OCCT's** (issue #66, rung 1): every fillet of the corpus's examples, its
//! pieces built by this kernel and added to (or taken from) what it rounds, against OCCT's own
//! rolling-ball fillet (`BRepFilletAPI_MakeFillet`) of the same edges, picked by a point of each,
//! at the same radius: the volumes equal and the faces alike by kind.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{boolean::Op,geom::Surface,props::volume,recipe};
use gcs_core::model::SolidDef;
use gcs_core::solid::cad;

fn kinds(b: &gcs_core::brep::topo::Brep) -> Vec<i32> {
    let mut k: Vec<i32> = b.faces.iter().map(|f| match f.surface {
        Surface::Plane(_) => 0,Surface::Cylinder(..) => 1,Surface::Cone(..) => 2,Surface::Sphere(..) => 3,Surface::Torus(..) => 4,
        Surface::Revolution(..) => 7,Surface::Extrusion(..) => 8,Surface::Blend(..) | Surface::BSpline(..) => 6,
    }).collect();
    k.sort();
    k
}

/// Every fillet of `e`, ours against OCCT's; how many were compared.
fn held(session: &native::Session,name: &str,e: &gcs_core::program::Elaborated) -> usize {
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    let mm = cad::millimetres(&sk).unwrap();
    let mut compared = 0;
    for (i,s) in sk.solids.iter().enumerate() {
        let SolidDef::Fillet {r,..} = &s.def else { continue };
        let blend = sk.fillet_blend(i).unwrap_or_else(|e| panic!("{name}: `{}`: {e}",s.name));
        let what = gcs_core::solid::fillet::operands(&sk,i);
        let ours_of = |root: u32| recipe::build(&cad::recipe(&sk,root as usize).unwrap()).unwrap();
        let mut ours = ours_of(what[0]);
        for &o in &what[1..] { ours = recipe::combined(&ours,&ours_of(o),Op::Union,0.).unwrap(); }
        let op = if blend.concave { Op::Union } else { Op::Cut };
        let ours = recipe::combined(&ours,&ours_of(i as u32),op,0.).unwrap_or_else(|e| panic!("{name}: `{}`: {e}",s.name));
        let mut theirs = session.construct(&cad::recipe(&sk,what[0] as usize).unwrap()).unwrap();
        for &o in &what[1..] {
            let other = session.construct(&cad::recipe(&sk,o as usize).unwrap()).unwrap();
            theirs = session.boolean(theirs,other,"on").unwrap();
        }
        let points: Vec<[f64;3]> = blend.pieces.iter().map(|p| p.edge_point())
            .chain(blend.rolls.iter().map(|r| r.edge_point())).map(|p| p.map(|x| x*mm)).collect();
        let theirs = session.fillet(theirs,r.value*mm,&points).unwrap_or_else(|e| panic!("{name}: `{}`: OCCT: {e}",s.name));
        let (v,w) = (volume(&ours),session.volume(theirs).unwrap());
        let rel = (v-w).abs()/w.abs();
        let their_kinds = {
            let mut k: Vec<i32> = session.faces(theirs).unwrap().into_iter().map(|g| session.face_kind(g).unwrap()).collect();
            k.sort();
            k
        };
        eprintln!("{name}: `{}`: {v:.9} mm³ against OCCT's {w:.9} ({rel:e}); faces {:?} against {their_kinds:?}",s.name,kinds(&ours));
        // a ball rolled along a traced loop: OCCT's own fillet stands off this kernel's in the piece
        // — 0.33% of the tee's crotch, 0.8% of the bore's rim (1e-5 of the bodies) — whatever bars
        // its approximations are held to, where a reference made without either kernel, the normal
        // sections swept along the spine (`gcs-core/tests/fillet.rs`), agrees with this kernel's to
        // 1e-6: OCCT is held to the body's volume at 1e-4 there, and to its faces by kind
        let bar = if blend.rolls.is_empty() { 1e-7 } else { 1e-4 };
        assert!(rel < bar,"{name}: `{}`: volume {v} against OCCT's {w}",s.name);
        // OCCT may leave a plane split where this kernel's Boolean joined it, or the reverse: the
        // curved faces must agree. Each fillet's own face is a cylinder or a torus here; OCCT builds
        // it so where its blend has a closed form and approximates it by a B-spline (kind 6) where
        // not (a ball between a cylinder and a sphere) — within the volume bar above — so a fillet's
        // own face is set aside on both sides before the rest are compared
        let curved = |k: &[i32]| k.iter().copied().filter(|&k| k != 0).collect::<Vec<_>>();
        let (mut mine,mut its) = (curved(&kinds(&ours)),curved(&their_kinds));
        for p in &blend.pieces {
            let own = match p.carry { gcs_core::solid::fillet::Carry::Prism {..} => 1,gcs_core::solid::fillet::Carry::Turn {..} => 4 };
            let take = |ks: &mut Vec<i32>,k: i32| ks.iter().position(|&x| x == k).map(|i| ks.remove(i)).is_some();
            assert!(take(&mut mine,own),"{name}: `{}`: no face of its own",s.name);
            assert!(take(&mut its,own) || take(&mut its,6),"{name}: `{}`: OCCT made no fillet face",s.name);
        }
        // a corner's patch of the ball: a sphere, or OCCT's approximation of its blend there
        for _ in &blend.corners {
            let take = |ks: &mut Vec<i32>,k: i32| ks.iter().position(|&x| x == k).map(|i| ks.remove(i)).is_some();
            assert!(take(&mut mine,3),"{name}: `{}`: no corner of its own",s.name);
            assert!(take(&mut its,3) || take(&mut its,6),"{name}: `{}`: OCCT made no corner",s.name);
        }
        // (OCCT builds the face of a run that ends, cut off by the face it meets, in pieces)
        for roll in &blend.rolls {
            let take = |ks: &mut Vec<i32>,k: i32| ks.iter().position(|&x| x == k).map(|i| ks.remove(i)).is_some();
            assert!(take(&mut mine,6),"{name}: `{}`: no canal face of its own",s.name);
            assert!(take(&mut its,6),"{name}: `{}`: OCCT made no fillet face",s.name);
            if !roll.rolled.trims.is_empty() { while take(&mut its,6) {} }
        }
        assert_eq!(mine,its,"{name}: `{}`: curved faces by kind",s.name);
        compared += 1;
    }
    compared
}

#[test]
fn every_fillet_is_occts() {
    let session = native::Session::new().unwrap();
    let mut compared = 0;
    for (name,e) in fixtures::examples() {
        let Some(e) = e else { continue };
        if !e.ok() || !e.sketch.solids.iter().any(|s| matches!(s.def,SolidDef::Fillet {..})) { continue }
        compared += held(&session,&name,&e);
    }
    assert!(compared >= 5,"only {compared} fillets compared");
}

