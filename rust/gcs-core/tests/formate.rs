//! The Formate pair (`spiral_bevel/formate.sv`, #283 step 1): the gear formed, every flank of its
//! tooth spaces one of the face mill's cones copied round its axis with no roll, and the pinion its
//! blank less the gear's tooth swept under the mesh — the gear's turn seen from the pinion's, at
//! the ratio of their teeth. No crown enters the pinion: its cutter is the region the gear's teeth
//! are copies of, and its motion turns about the two shafts and nothing else. Its sweeps are in
//! the generating class.
use gcs_core::brep::{export,sweep::Say};
use gcs_core::model::{MotionDef,SolidDef};
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Options};

/// The pair at its configured design with no fabrication allowances, the conjugate pair.
fn conjugate() -> String { fixtures::gear::design(&fixtures::gear::formate_source(),25.,12.5,25.) }

/// The line a rotation turns about, and its ratio.
fn rotation(e: &Elaborated,m: u32) -> (usize,f64) {
    let motion = &e.sketch.motions[m as usize];
    let MotionDef::Rotation {axis,..} = motion.def else { panic!("`{}` is not a rotation",motion.name) };
    (axis as usize,motion.rotation(&e.sketch).unwrap().0)
}

/// The pinion's sweep cuts with the gear's tooth, under the gear's turn about its own axis seen
/// from the pinion's turn about its own, the gear turning the pinion's teeth over its own as far;
/// the gear is cut by copies alone, one a tooth.
#[test]
fn the_pinion_is_cut_by_the_gears_tooth_under_the_mesh() {
    let e = fixtures::gear::unsolved(&conjugate());
    let sk = &e.sketch;
    let SolidDef::Swept {source,motion,..} = sk.solids[fixtures::solid(&e,"pair.pinion.cut.removal")].def else {
        panic!("the pinion's cut is a sweep")
    };
    assert_eq!(sk.solids[source as usize].name,"pair.gear_tooth.crown");
    let MotionDef::Relative {source,observer} = sk.motions[motion as usize].def else { panic!("not a relative motion") };
    let axis = |name: &str| e.map.ent_named(name).unwrap().i();
    let (gear,ratio) = rotation(&e,source);
    let (pinion,own) = rotation(&e,observer);
    assert_eq!((gear,pinion),(axis("pair.gear_cone.ax"),axis("pair.pinion_cone.ax")),"the mesh turns about the two shafts");
    assert_eq!((ratio,own),(-24./48.,1.),"at the ratio of the teeth");
    assert!(!sk.is_swept(fixtures::solid(&e,"pair.gear.body")),"the gear is formed, not generated");
    let copies = sk.solids.iter().filter(|s| s.name.starts_with("pair.gear.cut.") && s.name.ends_with(".indexed")).count();
    assert_eq!(copies,48,"a space copied at every tooth");
}

/// The generated pinion is in the class: one placement checked for all 24 (its blank a revolution
/// about the axis every index turns about), its flanks with no fold.
#[test]
fn the_conjugate_pinion_is_admitted() {
    let e = fixtures::gear::read(&conjugate());
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"pair.pinion.body"),&Options::default()).unwrap();
    let s = &a.sweeps()[0];
    eprintln!("{} samples, {} contacts, spacing {:.4}, least J {:.3}",s.samples,s.contacts,s.spacing,s.least_area_factor);
    assert_eq!(a.sweeps().len(),1);
    assert_eq!(s.placements.len(),24);
    assert!(matches!(s.equivalence,admission::Equivalence::Revolved {..}),"{:?}",s.equivalence);
    assert!(s.least_area_factor > 0.1,"{}",s.least_area_factor);
}

/// As configured, backlash, tip relief and end relief in: the gear tooth's sweep and the relief's
/// are both in the class.
#[test]
fn the_configured_pinion_is_admitted() {
    let e = fixtures::gear::read(&fixtures::gear::formate_source());
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"pair.pinion.body"),&Options::default()).unwrap();
    assert_eq!(a.sweeps().len(),2);
    assert!(a.sweeps().iter().all(|s| s.least_area_factor > 0.1));
}

/// A kernel reproduction, not a check of the pair: the configured gear with each space bounded by
/// the tooth ahead's levels and round as well as its flank (`hypoid.sv`'s `Space` turned a gear
/// pitch) — a second floor 3.35° off the first — fails the kernel's Boolean at one copy of 48
/// ("not a manifold", edge 723: a cone of an earlier copy meets that floor in a trace no edge
/// crosses). With no backlash grown into the flanks it builds. `formate.sv`'s `Space` bounds the tooth ahead by its
/// flank alone because of it; when this builds, the space may take its own floor back.
#[test]
#[ignore = "a kernel reproduction, about two minutes: fails today (brep::boolean), see formate.sv's Space"]
fn a_formed_gear_with_two_floors_a_space_builds() {
    let text = fixtures::gear::formate_source();
    let far = "far := { p | p inside concave; p inside cap; p outside below; p inside above }";
    assert!(text.contains(far),"the space bounds the tooth ahead by its flank alone");
    let e = fixtures::gear::read(&text.replace(far,"concave_round := std.Rounded(concave, tip, r: r)\n  \
        far := { p | p inside concave; p inside tip; p inside base; p inside concave_round }"));
    let say = Say {stage:&|l: &str| eprintln!("{l}"),mark:&|_| {}};
    let exact = export::exact(&e.sketch,fixtures::solid(&e,"pair.gear.body"),None,None,&say);
    assert!(exact.is_ok(),"{}",exact.err().unwrap());
}
