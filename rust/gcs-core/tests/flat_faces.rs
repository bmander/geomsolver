//! **Flat faces a turn or a slide keeps in their own planes** (#684 O): under a turn about a line
//! square to a flat face, or a slide along it, the face's contact `n·v` is nought all over, so every
//! point of it is on the boundary and it sweeps a region of its own plane. Each of its points is
//! carried onto the motion's section where it stands across it — its distance from the turn's
//! axis, its offset square to the slide — so the section reads it as the line between where it
//! stands least and most, and the sheet as that line carried, a strip of the plane, creased
//! against the walls where the faces beside it meet it in sharp edges (`constant_twist::Flat`,
//! `nurbs::interpolate_creased`). An end mill's flat bottom slid across a block cuts a slot
//! (`solid_sweeps/slot.sv`, `solid_sweeps_examples::the_end_mill_cuts_a_slot`), along its edge a
//! step, and turned across a ring sector a groove; a bull-nose's round joins its flat smoothly; a
//! tapered mill's walls are cones; and a bar lying along a turn cuts with its underside
//! (`prism_twists::a_side_square_to_the_turn_cuts_a_floor`). Each volume is a closed form, and each
//! exact solid's STL agrees with the field. A ring kept in its plane and a disc swept across the
//! turn's axis are refused by name.
use gcs_core::solid::admission::{Class,Condition};
use std::f64::consts::PI;

use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::{admit,exported,exported_at,near,refused};

/// A block 20 × 10 × 10 (x 0 to 20, y −5 to 5, z 0 to 10) and an end mill of radius `r`, its axis
/// upright through (−5, `y`), its flat bottom at `z` and its top 8 above, slid 30 along x over the
/// roll, from clear before the block to clear past it (`solid_sweeps/slot.sv`'s, its axis set across).
fn slid_mill(r: f64,y: f64,z: f64) -> String {
    format!("unit mm
use std
use std.place (coords)
use std.motion (slides)

in std.top {{
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: 0mm, dv: -5mm) std.top
  b1 coords(du: 20mm, dv: -5mm) std.top
  b2 coords(du: 20mm, dv: 5mm) std.top
  b3 coords(du: 0mm, dv: 5mm) std.top
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}}
part := solid(face(s0, s1, s2, s3), from: 0mm, to: 10mm)

private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (-5, {y}, {z})) bench
in bench {{
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: {r}mm, dv: 0mm) bench
  e2 coords(du: {r}mm, dv: 8mm) bench
  e3 coords(du: 0mm, dv: 8mm) bench
  private bottom := line(e0, e1)
  private side := line(e1, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}}
construction mill := solid(face(bottom, side, top, spindle), about: spindle)

private way := axis
fix(dir == (1, 0, 0)) way
feed := frame
feed slides(30mm) way
construction slot := sweep(mill over t in (0deg, 360deg), under: feed)
slot cut part
")
}

/// `turn.sv`'s block, a ring sector about z (radii 8 and 12, heights ±1, 60° round), and a mill
/// standing upright `d` from z, its flat bottom of radius `r` at height `z` and its side running
/// to radius `top` 4 above (a cone, or where `top` is `r` a cylinder), turned about z over the roll
/// from −140° to 140°, clear of the block at both ends.
fn turned_mill(r: f64,top: f64,d: f64,z: f64) -> String {
    format!("unit mm
use std
use std.place (coords)
use std.motion (turns)

in std.front {{
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 8mm, dv: -1mm) std.front
  q1 coords(du: 12mm, dv: -1mm) std.front
  q2 coords(du: 12mm, dv: 1mm) std.front
  q3 coords(du: 8mm, dv: 1mm) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}}
part := solid(face(wall_in, underside, wall_out, topside), about: ax)
in std.top {{
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 24mm, dv: 0mm) std.top
  w2 coords(du: 24mm * cos(60deg), dv: 24mm * sin(60deg)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}}
construction wedge := solid(face(wa, wb, wc), from: -2mm, to: 2mm)
wedge common part

private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == ({d}, 0, {z})) bench
in bench {{
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: {r}mm, dv: 0mm) bench
  e2 coords(du: {top}mm, dv: 4mm) bench
  e3 coords(du: 0mm, dv: 4mm) bench
  private bottom := line(e0, e1)
  private side := line(e1, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}}
construction mill := solid(face(bottom, side, top, spindle), about: spindle)

swing := frame
swing turns(1) ax
construction groove := sweep(mill over t in (-140deg, 140deg), under: swing)
groove cut part
")
}

/// The constant-twist evidence of `part`'s one sweep, its flat faces walked and how often the walk
/// runs onto or off one.
fn walked(text: &str) -> (Vec<String>,usize) {
    let e = fixtures::read(text);
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    (found.flats(),found.creases().len())
}

/// Set along the block's edge, the mill's axis on its side, it cuts a step: its near side's line up
/// the cutter is the wall, 2 in from the edge, and the floor's strip runs from it out past the
/// block, where the walk ends on the bottom's line. The block loses 2 × 3 along its 20.
#[test]
fn an_end_mill_slid_along_an_edge_cuts_a_step() {
    let text = slid_mill(2.,5.,7.);
    assert_eq!(walked(&text),(vec!["`mill.bottom`".to_string()],1));
    near(exported(&fixtures::read(&text),"part"),2000.-20.*2.*3.,1e-6);
}

/// A bull-nose mill, its flat bottom joined to its side by a quarter round of radius 0.5 tangent to
/// both: the round's ring where it meets the flat is in contact all round, its normals the flat's,
/// and the walk runs on from the flat's line across it at the line's ends, the slot's corners
/// rounded. The slot's section loses `(1 − π/4) 0.25` at each corner; at 0.001 mm (at 0.01 the fit
/// over the round removes 1.9e-5 of the part too little).
#[test]
fn a_bull_nose_mill_slid_rounds_the_slots_corners() {
    let text = slid_mill(2.,0.,7.).replace("  e1 coords(du: 2mm, dv: 0mm) bench","  e1 coords(du: 1.5mm, dv: 0mm) bench\n  \
        private k := point\n  k coords(du: 1.5mm, dv: 0.5mm) bench\n  private e5 := point\n  e5 coords(du: 2mm, dv: 0.5mm) bench")
        .replace("  private side := line(e1, e2)","  private nose := arc(center: k, start: e1, end: e5) hint(r: 0.5)\n  radius(0.5mm) nose\n  \
        private side := line(e5, e2)")
        .replace("face(bottom, side, top, spindle)","face(bottom, nose, side, top, spindle)");
    assert_eq!(walked(&text),(vec!["`mill.bottom`".to_string()],2));
    near(exported_at(&fixtures::read(&text),"part",0.001),2000.-20.*(12.-2.*(1.-PI/4.)*0.25),1e-6);
}

/// Turned about z, 10 from it, the mill's flat bottom of radius 1 sweeps the annulus from 9 to 11 in
/// its own plane, and its side's lines nearest and farthest the axis turn to the groove's walls,
/// cylinders: within the block's 60°, `π/6 (11² − 9²)` over the block's top millimetre.
#[test]
fn an_end_mill_turned_cuts_a_ring_groove() {
    let text = turned_mill(1.,1.,10.,0.);
    assert_eq!(walked(&text),(vec!["`mill.bottom`".to_string()],2));
    near(exported(&fixtures::read(&text),"part"),PI/3.*80.-PI/6.*40.,1e-5);
}

/// The same mill turned a whole turn (#684 N): its section revolved all the way round, closing on
/// itself, the floor a whole annulus; the block's 60° lose what they lost to the partial turn.
#[test]
fn an_end_mill_turned_a_whole_turn_cuts_the_same_groove() {
    let text = turned_mill(1.,1.,10.,0.).replace("(-140deg, 140deg)","(0deg, 360deg)");
    near(exported(&fixtures::read(&text),"part"),PI/3.*80.-PI/6.*40.,1e-5);
}

/// A tapered mill, its side a cone from radius 1 at its flat bottom to 2 four above: the cone's
/// lines nearest and farthest the axis turn to cones about z, `10 ∓ R(z)` from it with
/// `R(z) = 1 + z/4`, so within the 60° the block loses `π/6 ∫ 4·10·R dz = 2π/3 · 10 · 9/8` over its
/// top millimetre.
#[test]
fn a_tapered_mill_turned_cuts_a_groove_between_cones() {
    let text = turned_mill(1.,2.,10.,0.);
    assert_eq!(walked(&text),(vec!["`mill.bottom`".to_string()],2));
    near(exported(&fixtures::read(&text),"part"),PI/3.*80.-2.*PI/3.*10.*1.125,1e-5);
}

/// What is still refused (S1): a hollow mill's bottom, a ring the slide keeps in its plane, whose
/// line across the slide would cross its hole; and a mill turned a whole turn about an axis that
/// passes through its flat bottom, which sweeps across the axis (the block made a sector of a disc,
/// its inner radius 0).
#[test]
fn a_ring_or_a_disc_across_the_axis_kept_in_its_plane_is_refused() {
    let hollow = slid_mill(2.,0.,7.).replace("  e0 coords(du: 0mm, dv: 0mm) bench","  e0 coords(du: 1mm, dv: 0mm) bench")
        .replace("  e3 coords(du: 0mm, dv: 8mm) bench","  e3 coords(du: 1mm, dv: 8mm) bench")
        .replace("  private spindle := line(e3, e0)\n}","  private inside := line(e3, e0)\n  private a0 := point\n  private a1 := point\n  \
            a0 coords(du: 0mm, dv: 0mm) bench\n  a1 coords(du: 0mm, dv: 1mm) bench\n  private spindle := line(a0, a1)\n}")
        .replace("face(bottom, side, top, spindle)","face(bottom, side, top, inside)");
    let r = refused(admit(&hollow));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("`mill.bottom` is a ring kept in its own plane"),"{}",r.message);
    let across = turned_mill(1.5,1.5,1.,0.).replace("(-140deg, 140deg)","(0deg, 360deg)").replace("du: 8mm","du: 0mm");
    let r = refused(admit(&across));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("`mill.bottom` is kept in its own plane with the turn's axis through it"),"{}",r.message);
}
