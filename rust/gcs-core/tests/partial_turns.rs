//! **A revolution over part of a turn is a tool like any other** (#721 F1d, G9): the drawing's
//! field reads it (`RevolvedRegion::spatial`: its whole turn's material in common with the wedge
//! its turn spans), and the kernel reads it by its faces, its ends planes through its axis; T1
//! asks such a plane only where the motion's contact lies on it at some pose of the roll
//! (`admission::holds_none`). Held here on the lantern pinion: its pin turned 160° about its axis,
//! the wedge's edge along the axis convex, is admitted by the generating class as the pin's
//! prism is, where before the drawing refused it as no full revolution.
use gcs_core::solid::admission::{self,Class,Options};

/// The lantern pinion's wheel, its pin a revolution through `turn`.
fn lantern(turn: &str) -> gcs_core::program::Elaborated {
    let src = include_str!("../../examples/lantern_generation.sv");
    let text = src.replace("Revolution(pin_section, about: qa))",&format!("Revolution(pin_section, about: qa, {turn}))"));
    assert!(text.contains(turn));
    fixtures::read(&text)
}

/// **A pin turned 160° is admitted by the generating class**: its field read, its ends no
/// refusal, its sweep the generating class's as the whole pin's is.
#[test]
fn a_pin_turned_part_of_a_turn_is_the_generating_classs() {
    let e = lantern("to: 160deg");
    let wheel = fixtures::solid(&e,"wheel");
    let a = admission::admit_body(&e.sketch,wheel,&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    assert!(matches!(s.class,Class::Generating),"{:?}",std::mem::discriminant(&s.class));
}
