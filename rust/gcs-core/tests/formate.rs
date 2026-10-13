//! The Formate pair (`spiral_bevel/formate.sv`, #283 step 1): the gear formed, its tooth spaces the
//! face mill's cones copied round its axis with no roll, and the pinion its blank less the gear's
//! tooth swept under the mesh — the gear's turn seen from the pinion's, at the ratio of their
//! teeth. No crown enters the pinion: its cutter is a solid the gear's teeth are copies of, and
//! its motion turns about the two shafts and nothing else. Its sweep is in the generating class.
use gcs_core::model::{MotionDef,SolidDef,Sketch};
use gcs_core::solid::admission::{self,Options};

/// The pair at its configured design with no fabrication allowances: the conjugate pair.
fn conjugate() -> gcs_core::program::Elaborated {
    fixtures::gear::read_formate(&fixtures::gear::design(&fixtures::gear::formate_source(),25.,12.5,25.))
}

/// Every solid `id` is made from, itself included.
fn graph(sk: &Sketch,id: usize) -> Vec<usize> {
    let mut out = vec![id];
    let mut at = 0;
    while at < out.len() {
        for s in sk.solids[out[at]].operands() {
            if !out.contains(&(s as usize)) { out.push(s as usize) }
        }
        at += 1;
    }
    out
}

/// The line a rotation turns about, and its ratio.
fn rotation(e: &gcs_core::program::Elaborated,m: u32) -> (usize,f64) {
    let motion = &e.sketch.motions[m as usize];
    let MotionDef::Rotation {axis,..} = motion.def else { panic!("`{}` is not a rotation",motion.name) };
    (axis as usize,motion.rotation(&e.sketch).unwrap().0)
}

/// The pinion's one sweep cuts with the gear's tooth, under the gear's turn about its own axis seen
/// from the pinion's turn about its own, the gear turning the pinion's teeth over its own as far;
/// the gear is cut by placements alone.
#[test]
fn the_pinion_is_cut_by_the_gears_tooth_under_the_mesh() {
    let e = conjugate();
    let sk = &e.sketch;
    let sweeps: Vec<usize> = graph(sk,fixtures::solid(&e,"pair.pinion.body")).into_iter()
        .filter(|&s| matches!(sk.solids[s].def,SolidDef::Swept {..})).collect();
    assert_eq!(sweeps.len(),1,"the pinion has one sweep, with no tip relief");
    let SolidDef::Swept {source,motion,..} = sk.solids[sweeps[0]].def else { unreachable!() };
    assert_eq!(sk.solids[source as usize].name,"pair.gear_tooth.body");
    let MotionDef::Relative {source,observer} = sk.motions[motion as usize].def else { panic!("not a relative motion") };
    let axis = |name: &str| e.map.ent_named(name).unwrap().i();
    let (gear,ratio) = rotation(&e,source);
    let (pinion,own) = rotation(&e,observer);
    assert_eq!((gear,pinion),(axis("pair.gear_cone.ax"),axis("pair.pinion_cone.ax")),"the mesh turns about the two shafts");
    assert_eq!((ratio,own),(-24./48.,1.),"at the ratio of the teeth");
    let gear = graph(sk,fixtures::solid(&e,"pair.gear.body"));
    assert!(gear.iter().all(|&s| !matches!(sk.solids[s].def,SolidDef::Swept {..})),"the gear is formed, not generated");
    // its spaces copied at every tooth
    let placed = gear.iter().filter(|&&s| matches!(sk.solids[s].def,SolidDef::Placed {..})).count();
    assert!(placed >= 48,"{placed} placements");
}

/// The generated pinion is in the class: one placement checked for all 24 (its blank a revolution
/// about the axis every index turns about), its flanks with no fold.
#[test]
fn the_conjugate_pinion_is_admitted() {
    let e = conjugate();
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
    let e = fixtures::gear::formate_as_configured();
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"pair.pinion.body"),&Options::default()).unwrap();
    assert_eq!(a.sweeps().len(),2);
    assert!(a.sweeps().iter().all(|s| s.least_area_factor > 0.1));
}
