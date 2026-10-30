//! **Prisms under a constant twist** (#684 M): a prism tool carried by a turn, a screw or a slide.
//! Each moves every ruling of the prism — a line along it from one cap to the other — as one
//! affine field, and a ruling's normal is one, so the contact `n·v` is linear along it: one point
//! of contact a ruling, fixed on the tool, or under a slide the whole ruling at each of the
//! silhouette's points. The characteristic is walked over the sides and round the convex corners'
//! fans where they meet, and carried as a revolution's is (`solid::constant_twist`). Each case's
//! volume is computed apart from the core, and each exact solid's STL agrees with the field; the
//! square thread (`rust/examples/solid_sweeps/square_thread.sv`) is
//! `solid_sweeps_examples::the_square_tool_leaves_a_groove_widening_outward`.
use gcs_core::solid::admission::{self,Class,Condition,Options};
use std::f64::consts::PI;

use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::{admit,exported,near,refused};

/// A bar turned about z across `turn.sv`'s block, a ring sector (radii 8 and 12, heights ±1, 60°
/// round): its section the quadrilateral `corners`, drawn in the plane through x leaning `lean`
/// from upright across the turn (`du` along x, `dv` across the turn, falling as it leans), and the
/// bar standing `half` either side of that plane along its length: drawn in the plane through
/// x set `half` back along the length, and extruded from it twice that, drafted `draft` degrees.
fn turned_bar(lean: f64,corners: [[f64;2];4],half: f64,draft: f64) -> String {
    let [c0,c1,c2,c3] = corners.map(|[u,v]| format!("du: {u}mm, dv: {v}mm"));
    let drafted = if draft == 0. { String::new() } else { format!(", draft: {draft}deg") };
    format!("unit mm
use std
use std.place (coords)
use std.motion (turns)

private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos({lean}deg), -sin({lean}deg))) along
holder := plane(u: across, v: along)
fix(origin == (0, -{half} * sin({lean}deg), -{half} * cos({lean}deg))) holder
in holder {{
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords({c0}) holder
  c1 coords({c1}) holder
  c2 coords({c2}) holder
  c3 coords({c3}) holder
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}}
construction bar := solid(face(lower, upper, outer, under), from: 0mm, to: 2 * {half}mm{drafted})

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
ring := solid(face(wall_in, underside, wall_out, topside), about: ax)
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
wedge common ring
part := solid(ring)

swing := frame
swing turns(1) ax
construction groove := sweep(bar over t in (-140deg, 140deg), under: swing)
groove cut part
")
}

/// A bar of square section stood on a corner `inner` from the axis, standing ±4 along its length.
fn diamond(inner: f64) -> [[f64;2];4] { [[inner,0.],[inner+1.5,1.5],[inner+3.,0.],[inner+1.5,-1.5]] }

/// A turn of the leaning bar: within the block the characteristic is the ruling through its inner
/// corner, round the corner's fan, a line leaning across the turn, so the groove's wall is that
/// line turned, a hyperboloid of one sheet. Its point at height z stands `z tan(lean)` along the
/// turn from the plane through x, so `√(inner² + z² tan² lean)` from the axis (the bar's sides run
/// outward from that corner, so nothing of it at that height stands nearer), and the bar reaches
/// past the block's outer radius: the block loses everything outside that radius, 60° round.
#[test]
fn a_leaning_bar_turned_leaves_a_hyperboloid() {
    let (lean,inner) = (20_f64,10_f64);
    let e = fixtures::read(&turned_bar(lean,diamond(inner),4.,0.));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn(),"read as a turn");
    assert_eq!(found.fans(),["the corner of `bar.under` and `bar.lower`"]);
    let t2 = lean.to_radians().tan().powi(2);
    let removed = PI/6.*(2.*(144.-inner*inner)-2./3.*t2);
    near(exported(&e,"part"),PI/3.*(144.-64.)-removed,1e-5);
}

/// The leaning bar drafted 2° (#684 G): each side is a plane through its edge's sections at the
/// bar's two ends, and the section shrinks along the bar, its inner corner moving `h tan 2° / sin 45°`
/// outward along its bisector at `h` along the bar. That corner is still a line, the characteristic
/// within the block, and the groove's wall is it turned: at `s = h − 4` it stands at
/// `(10 + √2 tan 2° (s + 4), s sin 20°, s cos 20°)`, so its square distance from the axis is
/// quadratic in the height, and Simpson's rule integrates the block's loss exactly.
#[test]
fn a_drafted_bar_turned_leaves_its_corners_line_turned() {
    let e = fixtures::read(&turned_bar(20.,diamond(10.),4.,2.));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    // a drafted prism's sides are read loop by loop as its sections walk them
    assert_eq!(found.fans(),["the corner of `bar.lower` and `bar.under`"]);
    let (sl,cl) = 20f64.to_radians().sin_cos();
    let shift = 2f64.sqrt()*2f64.to_radians().tan();
    let r2 = |z: f64| { let s = z/cl; (10.+shift*(s+4.)).powi(2)+(s*sl).powi(2) };
    let removed = PI/6.*crate::brep::simpson(-1.,1.,2,|z| 144.-r2(z));
    near(exported(&e,"part"),PI/3.*(144.-64.)-removed,1e-5);
}

/// Upright, the bar's length is the turn's axis, and the turn keeps the plane of its section: the
/// planar subtraction class reads its swept region whole, as it did before the constant twist read
/// prisms.
#[test]
fn an_upright_bar_turned_is_still_the_planar_subtraction_classs() {
    let e = fixtures::read(&turned_bar(0.,diamond(10.),4.,0.));
    assert!(matches!(class(&e,"part"),Class::Subtraction(_)),"read by the planar subtraction class");
}

/// A bar of square section lying along x, sharp corner down (`diamond`) or flat side down, slid
/// along `(0.3, 1, drop)` across the top of a 10 mm cube: from y = −4 to 14, its lowest point
/// starting 9.8 high, so `drop` −0.2 carries it from 9 over the cube's near face to 7 over its
/// far one. Its length stands from x = −8 to 18, and the slide carries it 5.4 along x.
fn slid_bar(diamond: bool,drop: f64) -> String {
    let section = if diamond { "c0 coords(du: -4mm, dv: 9.8mm) std.side
  c1 coords(du: -2mm, dv: 11.8mm) std.side
  c2 coords(du: -4mm, dv: 13.8mm) std.side
  c3 coords(du: -6mm, dv: 11.8mm) std.side" } else { "c0 coords(du: -2mm, dv: 9.8mm) std.side
  c1 coords(du: -2mm, dv: 12.8mm) std.side
  c2 coords(du: -6mm, dv: 12.8mm) std.side
  c3 coords(du: -6mm, dv: 9.8mm) std.side" };
    let length = (0.09+1.+drop*drop).sqrt();
    format!("unit mm
use std
use std.place (coords)
use std.motion (slides)

in std.top {{
  private k0 := point
  private k1 := point
  private k2 := point
  private k3 := point
  fix((0, 0)) k0
  k1 coords(du: 10mm, dv: 0mm) std.top
  k2 coords(du: 10mm, dv: 10mm) std.top
  k3 coords(du: 0mm, dv: 10mm) std.top
}}
part := solid(face(k0, k1, k2, k3, -> close), from: 0mm, to: 10mm)

in std.side {{
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  {section}
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}}
construction bar := solid(face(lower, upper, outer, under), from: -8mm, to: 18mm)

private way := axis
fix(dir == ({}, {}, {})) way
feed := frame
feed slides({}mm) way
construction groove := sweep(bar over t in (0deg, 360deg), under: feed)
groove cut part
",0.3/length,1./length,drop/length,18.*length)
}

/// A slide of the bar stood on its corner: the slide moves every ruling's points alike, so the
/// rulings in contact are whole, its silhouette, where the fan of its lowest corner turns square to
/// the slide. That ruling slid is a plane falling 0.2 along y, and the cube loses what lies above
/// it, from 9 high at y = 0 to 7 at y = 10: 10 × 10 × 2 on average. The slide carries the bar
/// along its length too, so it keeps no plane of its section.
#[test]
fn a_bar_slid_aslant_leaves_a_plane() {
    let e = fixtures::read(&slid_bar(true,-0.2));
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    near(exported(&e,"part"),1000.-200.,1e-6);
}

// -- refusals --------------------------------------------------------------------------------

/// Lying along the turn, a bar of oblong section's underside (`lower`) is square to the turn's axis: the turn
/// keeps it in its own plane, and every point of it is on the boundary, which the class does not
/// sweep (S1, box O of #684). Its ends stand clear of the block.
#[test]
fn a_side_square_to_the_turn_is_refused() {
    let r = refused(admit(&turned_bar(90.,[[10.,0.],[13.,0.],[13.,-3.],[10.,-3.]],9.,0.)));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("is in contact all over: the motion keeps it in its own plane"),"{}",r.message);
}

/// Flat side down and slid level, the bar's underside runs along the slide: every point of it moves
/// along it, the face kept in its own plane (S1).
#[test]
fn a_side_along_the_slide_is_refused() {
    let r = refused(admit(&slid_bar(false,0.)));
    assert_eq!(r.condition,Condition::Regular);
    assert!(r.message.contains("`bar.under` moves along itself"),"{}",r.message);
}

/// The leaning bar drafted both ways from its parting plane (#684 P): each side folds there, a crease
/// the walk does not cross under a turn (T1).
#[test]
fn a_bar_drafted_both_ways_turned_is_refused() {
    let text = turned_bar(20.,diamond(10.),4.,2.).replace("from: 0mm, to: 2 * 4mm","from: -4mm, to: 4mm")
        .replace("fix(origin == (0, -4 * sin(20deg), -4 * cos(20deg))) holder","fix(origin == (0, 0, 0)) holder");
    let r = refused(admit(&text));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("runs both ways from its parting plane"),"{}",r.message);
}

/// The square thread's bar made too short to stand past the rod either side: its caps pass through
/// the rod, and the class cuts with a cap only under a relative rotation (T1).
#[test]
fn a_cap_in_the_blank_under_a_screw_is_refused() {
    let text = crate::solid_sweeps_examples::text("square_thread.sv").replace("reach := 8mm","reach := 2.5mm");
    let e = fixtures::read(&text);
    let r = refused(admission::admit_body(&e.sketch,fixtures::solid(&e,"rod"),&Options::default()));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("a cap only of a prism standing alone as the tool, its profile one loop, under a relative rotation"),
        "{}",r.message);
}
