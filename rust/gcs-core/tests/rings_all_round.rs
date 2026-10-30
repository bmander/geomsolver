//! **A ring in contact all round** (#684 X): where a ring of a revolved tool has every point of it
//! on the boundary, the characteristic branches there. Under a slide along the tool's own axis each
//! ring meets the motion at one angle all round, so it touches nowhere or all round, and the ring
//! of greatest radius sweeps the cylinder of that radius: where the tool's shadow along the slide
//! is its whole disc and it passes wholly through the blank's heights, the sweep within the blank
//! is that cylinder's, written exactly (`constant_twist::along_axis`, `build::tube`). A drill pushed
//! through a plate bores its radius (`solid_sweeps/drill.sv`,
//! `solid_sweeps_examples::the_drill_bores_its_radius_through_a_plate`); fed as it turns, the same;
//! a lemon, whose greatest radius is a ring within its one arc, its greatest radius. Each volume is
//! a closed form, and each exact solid's STL agrees with the field. Refused by name: a torus slid
//! along its axis, its shadow a ring (S2), and a drill that does not pass wholly through the blank's
//! heights (E1).
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
