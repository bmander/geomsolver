//! **A reflected B-rep held to OCCT's mirror** (#578): a mirrored instance's solid is its body
//! built where it was solved and reflected (`Brep::reflected`).  Here the same body is built by
//! OCCT from its recipe, mirrored by `gp_Trsf::SetMirror` in the plane the instance names, and
//! dumped back (`brep::json`, which makes OCCT's left-handed placements right-handed by the rule
//! the reflection shares): the volume, the faces by kind and every vertex agree with ours.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use std::os::raw::{c_int,c_void};

extern "C" {
    fn solvent_cad_mirror(cad: *mut c_void,source: c_int,at: *const f64,normal: *const f64) -> c_int;
}

/// A turned part off the mid-plane — planes, a cylinder, cones and a torus — and a plate drawn in
/// the front, each a closed definition standing in the world.
const PARTS: &str = "unit mm
use std
use std.place (offset, horizontal)
component Turned(s0: Length) {
  in std.side {
    construction ax := line(std.side.origin, hint((10, 0)))
    horizontal ax
    length(10mm) ax
    p1 := point hint((s0, 2))
    p2 := point hint((s0 + 6mm, 2))
    p3 := point hint((s0 + 6mm, 5))
    p4 := point hint((s0 + 4mm, 6))
    k := point hint((s0 + 3mm, 6))
    p5 := point hint((s0 + 3mm, 7))
    p6 := point hint((s0, 9))
    p1 offset(dx: s0, dy: 2mm) std.side.origin
    p2 offset(dx: s0 + 6mm, dy: 2mm) std.side.origin
    p3 offset(dx: s0 + 6mm, dy: 5mm) std.side.origin
    p4 offset(dx: s0 + 4mm, dy: 6mm) std.side.origin
    k offset(dx: s0 + 3mm, dy: 6mm) std.side.origin
    p5 distance(s0 + 3mm, along: u) std.side
    p6 offset(dx: s0, dy: 9mm) std.side.origin
    profile := (l1 := line(p1, p2)) -> (l2 := line(p2, p3)) -> (l3 := line(p3, p4)) ->
               (round := arc(center: k, start: p4, end: p5)) -> (l5 := line(p5, p6)) ->
               (l6 := line(p6, p1)) -> close
    body := solid(profile, about: ax)
  }
}
component Plate(w: Length, d: Length, h: Length) {
  in std.front {
    a := point hint((1, 2))
    b := point hint((w, 2))
    c := point hint((w, d))
    e := point hint((1, d + 3mm))
    a offset(dx: 1mm, dy: 2mm) std.origin
    b offset(dx: w, dy: 2mm) std.origin
    c offset(dx: w, dy: d) std.origin
    e offset(dx: 1mm, dy: d + 3mm) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: h)
  }
}
";

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, OCCT's mirror")]
fn a_reflection_is_occts_mirror() {
    let session = native::Session::new().unwrap();
    for (call,word,normal) in [("Turned(s0: 4mm)","front",[0.,1.,0.]),("Turned(s0: 4mm)","side",[1.,0.,0.]),
                               ("Plate(w: 30mm, d: 20mm, h: 5mm)","top",[0.,0.,1.])] {
        let src = format!("{PARTS}left := {call} mirrored({word})\n");
        let (prog,errs,linked) = gcs_core::library::parse_linked(&src);
        assert!(errs.is_empty() && linked.is_empty(),"{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.ok(),"{:?}",e.diags);
        let mut sk = e.sketch.clone();
        assert!(gcs_core::solve::solve(&mut sk,Default::default()).success);
        let l = e.map.entity_path(&sk,"left.body").unwrap().i();
        // the body where it was solved, built by OCCT, and mirrored there
        let recipe = gcs_core::solid::cad::recipe(&sk,l).unwrap();
        let built = session.construct(&recipe).unwrap();
        let at = [0.;3];
        let mirrored = session.result(unsafe { solvent_cad_mirror(session.as_ptr(),built,at.as_ptr(),normal.as_ptr()) })
            .unwrap_or_else(|e| panic!("{word}: OCCT's mirror: {e}"));
        let theirs = gcs_core::brep::json::read(&session.brep_json(mirrored).unwrap()).unwrap();
        theirs.check(1e-7).unwrap_or_else(|e| panic!("{word}: OCCT's mirror read invalid: {e}"));
        let ours = sk.exact_solid(l).unwrap();
        let (v,w) = (gcs_core::brep::props::volume(&ours.brep),session.volume(mirrored).unwrap());
        assert!(((v-w)/w).abs() < 1e-9,"{word}: {v} against OCCT's {w}");
        let kinds = |b: &gcs_core::brep::topo::Brep| {
            let mut k: Vec<&str> = b.faces.iter().map(|f| f.surface.kind()).collect();
            k.sort();
            k
        };
        assert_eq!(kinds(&ours.brep),kinds(&theirs),"{word}");
        // every vertex of ours one of OCCT's, both in millimetres in the world
        let ours_at: Vec<[f64;3]> = ours.brep.vertices.iter().map(|p| std::array::from_fn(|k| p.p[k]+ours.origin[k]*ours.mm)).collect();
        for p in &ours_at {
            let d = theirs.vertices.iter().map(|q| gcs_core::space::distance(*p,q.p)).fold(f64::INFINITY,f64::min);
            assert!(d < 1e-7,"{word}: {p:?} is {d} from OCCT's nearest vertex");
        }
        assert_eq!(ours_at.len(),theirs.vertices.len(),"{word}");
    }
}
