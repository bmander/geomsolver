//! **A ring in contact all round** (#684 X): where a ring of a revolved tool has every point of it
//! on the boundary, the characteristic branches there. Under a slide along the tool's own axis each
//! ring meets the motion at one angle all round, so it touches nowhere or all round, and the ring
//! of greatest radius sweeps the cylinder of that radius: where the tool's shadow along the slide
//! is its whole disc and it passes wholly through the blank's heights, the sweep within the blank
//! is that cylinder's, written exactly (`constant_twist::along_axis`, `build::tube`). A drill pushed
//! through a plate bores its radius (`solid_sweeps/drill.sv`,
//! `solid_sweeps_examples::the_drill_bores_its_radius_through_a_plate`); fed as it turns, the same;
//! a lemon, whose greatest radius is a ring within its one arc, its greatest radius. Under a turn
//! about an axis the tool's own meets, every other ring touches at its two points in the plane of
//! the two axes, and the ring whose normals pass through where they meet sweeps a zone of the sphere
//! about that point: the walk turns onto it half round and off onto the branch that leaves the
//! zone, the branch it trims proven within (`constant_twist::rounds`, `Walker::sides`, `under`). A
//! pin swung along a radius cuts a slot whose end wall is that zone (`solid_sweeps/spoke.sv`), and
//! the torus leaning toward the turn's axis, the field gate's first find (refused before), its
//! shadow. Each volume is figured apart, and each exact solid's STL agrees with the field. Refused
//! by name: a torus slid along its axis, its shadow a ring (S2); a drill that does not pass wholly
//! through the blank's heights (E1); a ball-ended pin swung, its characteristic running through the
//! ball's pole on its axis, and a pin swung wholly within the block, through its base's centre (S1,
//! a pole); an upright torus turned across a taller block, its tube's top ring, whose normals lie
//! along its axis and the turn's, in contact all round in a plane the turn keeps (S1); and a small
//! leaning torus wholly within the block, its loop closing with no crease (S1, #684 W's row).
use gcs_core::solid::admission::{Class,Condition};
use std::f64::consts::PI;

use crate::solid_sweeps_examples::{class,text};
use crate::turns_and_slides::{admit,exported,near,refused};

/// `drill.sv` with its plate named `part`, as `admit` reads it.
fn drill() -> String { text("drill.sv").replace("plate := solid","part := solid").replace("hole cut plate","hole cut part") }

/// `drill.sv` with its drill drawn as `face`'s edges (in its bench, the radius out along `du` and
/// the axis up along `dv`, its point 2 above the plate), turned about `spindle`.
fn drawn(edges: &str,face: &str) -> String {
    let text = drill();
    let (start,end) = (text.find("in bench {").unwrap(),text.find("// the push down").unwrap());
    format!("{}in bench {{\n{edges}}}\nconstruction drill := solid(face({face}), about: spindle)\n\n{}",&text[..start],&text[end..])
}

/// The drill fed down through the plate as it turns about its own axis, once over the roll: a screw
/// about the axis carries each ring into itself, so the drill touches as it does slid, along its
/// ring of greatest radius all round, and bores the same hole.
#[test]
fn a_drill_fed_as_it_turns_bores_the_same_hole() {
    let text = drill().replace("use std.motion (slides)","use std.motion (screws)")
        .replace("  private spindle := line(back, tip)","  spindle := line(back, tip)")
        .replace("ram slides(20mm) down","ram screws(20mm) spindle");
    let e = fixtures::read(&text);
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(!found.screw.is_slide() && !found.screw.is_turn(),"read as a screw");
    assert!(found.closed && found.cylinder().is_some(),"its characteristic the ring of greatest radius");
    near(exported(&e,"part"),800.-16.*PI,1e-9);
}

/// A lemon slid along its axis: its profile one arc from its point below to its point above, of
/// radius 3.25 about (−1.25, 0) in its bench, so its greatest radius, 2, is a ring within the arc,
/// read where the arc's radii are read greatest and refined between. The hole is that radius's.
#[test]
fn a_lemon_slid_along_its_axis_bores_its_greatest_radius() {
    let e = fixtures::read(&drawn("  private c := point
  private low := point
  private high := point
  c coords(du: -1.25mm, dv: 3mm) bench
  low coords(du: 0mm, dv: 0mm) bench
  high coords(du: 0mm, dv: 6mm) bench
  private rind := arc(center: c, start: low, end: high) hint(r: 3.25)
  private spindle := line(high, low)
","rind, spindle"));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    let (_,_,radius) = found.cylinder().expect("its characteristic the ring of greatest radius");
    assert!((radius-2.).abs() <= 1e-9,"its greatest radius {radius}");
    near(exported(&e,"part"),800.-16.*PI,1e-9);
}

/// What a slide along the tool's own axis still refuses: a torus, its shadow along the slide a
/// ring from 1.5 to 3.5 about its axis, whose inner ring in contact all round sweeps a second wall,
/// the plate's core left standing (S2); and the drill pushed through the plate beside a post 30
/// high, whose heights it does not pass wholly through, though it is clear of the post at both
/// ends (E1: the class reads the hole only where the tool passes the blank's every height).
#[test]
fn what_a_slide_along_the_tools_axis_still_refuses() {
    let torus = refused(admit(&drawn("  private ta := point
  private tb := point
  ta coords(du: 0mm, dv: 0mm) bench
  tb coords(du: 0mm, dv: 1mm) bench
  private spindle := line(tb, ta)
  private tc := point
  tc coords(du: 2.5mm, dv: 2mm) bench
  private tube := circle(center: tc) hint(r: 1)
  radius(1mm) tube
","tube")));
    assert_eq!(torus.condition,Condition::Once);
    assert!(torus.message.contains("the tool's shadow along the slide is no whole disc: none of it stands between 0.0000 and 1.5000"),
        "{}",torus.message);
    let post = drill().replace("part := solid(face(near_side, far_end, far_side, near_end), from: 0mm, to: thick)",
        "part := solid(face(near_side, far_end, far_side, near_end), from: 0mm, to: thick)
in std.top {
  private p0 := point
  private p1 := point
  private p2 := point
  private p3 := point
  p0 coords(du: 0mm, dv: -5mm) std.top
  p1 coords(du: 3mm, dv: -5mm) std.top
  p2 coords(du: 3mm, dv: -2mm) std.top
  p3 coords(du: 0mm, dv: -2mm) std.top
  private q0 := line(p0, p1)
  private q1 := line(p1, p2)
  private q2 := line(p2, p3)
  private q3 := line(p3, p0)
}
post := solid(face(q0, q1, q2, q3), from: 0mm, to: 30mm)
post union part");
    let post = refused(admit(&post));
    assert_eq!(post.condition,Condition::Clearance);
    assert!(post.message.contains("does not pass wholly through the blank's heights"),"{}",post.message);
}

// -- a turn about an axis the tool's own meets -------------------------------------------------

/// `spoke.sv` with its block named `part`, its pin set from `start` to `reach` along its radius.
fn spoke(start: f64,reach: f64) -> String {
    text("spoke.sv").replace("block := solid","part := solid").replace("wedge common block","wedge common part")
        .replace("slot cut block","slot cut part").replace("start := 6mm",&format!("start := {start}mm"))
        .replace("reach := 11mm",&format!("reach := {reach}mm"))
}

/// `turns_and_slides`'s torus leaning 25° toward the turn's axis, so that its own meets that one at
/// (0, 0, −5/tan 25°) below: its rings touch at their two points in the plane of the two axes, its
/// tube's two circles there, and its ring farthest from where the axes meet touches all round,
/// sweeping a zone of the sphere about that point which bounds the groove's floor and its outer
/// side within the block. The walk runs up one tube circle's outer half onto that ring, and the
/// circle's inner half, which the ring trims, lies within the zone. The field gate saw this ring
/// before S1 did, and it was refused until #684 X. What the turn removes is Pappus over the torus's
/// shadow, read from its centre circle alone (`turns_and_slides::turned_away`): within 2e-5 (2.3e-6).
/// Turned a whole turn round the whole ring (#684 N), six times the sector's shadow (7.5e-7).
#[test]
fn a_torus_leaning_toward_the_turns_axis_grooves_its_shadow() {
    use crate::turns_and_slides::{turned,turned_away,turned_in,turned_torus,Lean};
    let e = fixtures::read(&turned(Lean::Toward(25.),-140.,140.));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn() && !found.closed,"a turn, its characteristic open");
    assert_eq!(found.rounds(),["a ring of `wheel.tube` in contact all round"]);
    let away = turned_away(&turned_torus(Lean::Toward(25.)));
    near(exported(&e,"part"),PI/3.*80.-away,2e-5);
    let e = fixtures::read(&turned_in(Lean::Toward(25.),0.,360.,false));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.whole && found.rounds().len() == 1,"a whole turn, round the ring");
    near(exported(&e,"part"),2.*PI*80.-6.*away,2e-5);
}

/// What a turn about an axis the tool's own meets still refuses: `spoke.sv`'s pin with its end
/// domed, a cap of a ball of radius 1.3 about a point of its axis 1.2 short of its end, whose
/// characteristic in the plane of the two axes runs over the dome's pole on the pin's axis, where
/// the walk would cross to each ring's other root (S1, a pole); the pin set wholly within the
/// block, from 9 to 11, its characteristic closing through its base's centre, a pole too (S1); the
/// upright torus moved out to 7 and turned across a block three times as tall, its axis along the
/// turn's, the ring at its tube's top, whose normals all lie along both, in contact all round in
/// the plane the turn keeps, a strip of it (S1, read at the walk's station there); and a torus
/// leaning 25° toward the turn's axis small enough to lie wholly within the block, ring 0.8 and tube
/// 0.3 about (10, 0, 0), whose loop — each tube circle's outer half joined to the far ring and the
/// near — closes smoothly, with no crease, the first case to reach #684 W's row (S1).
#[test]
fn what_a_turn_about_an_axis_the_tools_meets_still_refuses() {
    let ball = spoke(6.,10.5).replace("  private e3 := point\n","  private e3 := point\n  private e4 := point\n")
        .replace("  e3 coords(du: reach, dv: 0mm) holder\n","  e3 coords(du: reach + 0.1mm, dv: 0mm) holder\n  e4 coords(du: reach - 1.2mm, dv: 0mm) holder\n")
        .replace("  private tip := line(e2, e3)\n","  private tip := arc(center: e4, start: e3, end: e2) hint(r: 1.3)\n  radius(1.3mm) tip\n");
    let ball = refused(admit(&ball));
    assert_eq!(ball.condition,Condition::Regular);
    assert!(ball.message.contains("meets the tool's axis at a pole of its profile"),"{}",ball.message);
    let within = refused(admit(&spoke(9.,11.)));
    assert_eq!(within.condition,Condition::Regular);
    assert!(within.message.contains("`pin.base` meets the tool's axis at a pole of its profile"),"{}",within.message);
    let tall = crate::turns_and_slides::turned(crate::turns_and_slides::Lean::Across(0.),-130.,130.)
        .replace("fix(origin == (5, 0, 0)) lathe","fix(origin == (7, 0, 0)) lathe")
        .replace("fix((8, -1)) q0\n  fix((12, -1)) q1\n  fix((12, 1)) q2\n  fix((8, 1)) q3","fix((8, -3)) q0\n  fix((12, -3)) q1\n  fix((12, 3)) q2\n  fix((8, 3)) q3")
        .replace("from: -3mm, to: 3mm","from: -4mm, to: 4mm");
    let tall = refused(admit(&tall));
    assert_eq!(tall.condition,Condition::Regular);
    assert!(tall.message.contains("the characteristic on `wheel.tube` is in contact all round a ring"),"{}",tall.message);
    let small = crate::turns_and_slides::turned(crate::turns_and_slides::Lean::Toward(25.),-140.,140.)
        .replace("fix(origin == (5, 0, 0)) lathe","fix(origin == (10, 0, 0)) lathe")
        .replace("tc coords(du: 2.5mm, dv: 0mm) lathe","tc coords(du: 0.8mm, dv: 0mm) lathe")
        .replace("radius(2mm) tube","radius(0.3mm) tube").replace("hint(r: 2)","hint(r: 0.3)");
    let small = refused(admit(&small));
    assert_eq!(small.condition,Condition::Regular);
    assert!(small.message.contains("closes on itself through `wheel.tube` with no crease"),"{}",small.message);
}


