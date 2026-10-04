//! **A fillet held to OCCT's** (issue #66, rung 1): every fillet of the corpus's example and of the
//! cases below, its pieces built by this kernel and added to (or taken from) what it rounds, against
//! OCCT's own rolling-ball fillet (`BRepFilletAPI_MakeFillet`) of the same edges, picked by a point
//! of each, at the same radius: the volumes equal and the faces alike by kind.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use gcs_core::brep::{boolean::Op,geom::Surface,props::volume,recipe};
use gcs_core::model::SolidDef;
use gcs_core::solid::cad;

/// A disc of radius 40, thickness 10, and a frustum on it (radius 15 to 5, 20 tall), turned about
/// one axis: its root filleted, its top rim rounded.
const TURNED: &str = "\
unit mm
a0 := point
a1 := point hint(x: 0, y: 40)
fix(x == 0, y == 0) a0
fix(x == 0, y == 40) a1
axis := line(a0, a1)
p0 := point
p1 := point hint(x: 40, y: 0)
p2 := point hint(x: 40, y: 10)
p3 := point hint(x: 0, y: 10)
fix(x == 0, y == 0) p0
fix(x == 40, y == 0) p1
fix(x == 40, y == 10) p2
fix(x == 0, y == 10) p3
(pbot := line(p0, p1)) -> (pside := line(p1, p2)) -> (ptop := line(p2, p3)) -> (paxis := line(p3, p0)) -> close
pf := face(pbot, pside, ptop, paxis)
q0 := point hint(x: 0, y: 10)
q1 := point hint(x: 15, y: 10)
q2 := point hint(x: 5, y: 30)
q3 := point hint(x: 0, y: 30)
fix(x == 0, y == 10) q0
fix(x == 15, y == 10) q1
fix(x == 5, y == 30) q2
fix(x == 0, y == 30) q3
(cbot := line(q0, q1)) -> (slant := line(q1, q2)) -> (ctop := line(q2, q3)) -> (caxis := line(q3, q0)) -> close
cf := face(cbot, slant, ctop, caxis)
plate := solid(pf, about: axis)
frustum := solid(cf, about: axis)
body := solid(plate)
frustum union body
root := fillet(frustum, plate, r: 3mm)
root union body
rim := fillet(frustum.ctop, frustum.slant, r: 1mm)
rim cut body
";

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
        let SolidDef::Fillet {a,b,r} = &s.def else { continue };
        let blend = sk.fillet_blend(i).unwrap_or_else(|e| panic!("{name}: `{}`: {e}",s.name));
        let mut what: Vec<u32> = gcs_core::solid::fillet::stands_for(&sk,i,a.solid);
        what.extend(gcs_core::solid::fillet::stands_for(&sk,i,b.solid));
        what.dedup();
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
        let points: Vec<[f64;3]> = blend.pieces.iter().map(|p| p.edge_point.map(|x| x*mm)).collect();
        let theirs = session.fillet(theirs,r.value*mm,&points).unwrap_or_else(|e| panic!("{name}: `{}`: OCCT: {e}",s.name));
        let (v,w) = (volume(&ours),session.volume(theirs).unwrap());
        let rel = (v-w).abs()/w.abs();
        let their_kinds = {
            let mut k: Vec<i32> = session.faces(theirs).unwrap().into_iter().map(|g| session.face_kind(g).unwrap()).collect();
            k.sort();
            k
        };
        eprintln!("{name}: `{}`: {v:.9} mm³ against OCCT's {w:.9} ({rel:e}); faces {:?} against {their_kinds:?}",s.name,kinds(&ours));
        assert!(rel < 1e-7,"{name}: `{}`: volume {v} against OCCT's {w}",s.name);
        // OCCT may leave a plane split where this kernel's Boolean joined it, or the reverse: the
        // curved faces — the fillet's own among them — must agree
        let curved = |k: &[i32]| k.iter().copied().filter(|&k| k != 0).collect::<Vec<_>>();
        assert_eq!(curved(&kinds(&ours)),curved(&their_kinds),"{name}: `{}`: curved faces by kind",s.name);
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
    compared += held(&session,"turned",&fixtures::read(TURNED));
    assert!(compared >= 5,"only {compared} fillets compared");
}
