// D0 (#339): line contact on skew axes from the contact alone — #326's 4b — in the proposed
// spelling, over #289's Formate pair (90°, 25 mm). Does not parse: it stops at `initially`, and
// it needs #326 R2 (`reciprocal`), R4 (`under`) and R5 (surfaces over two formals) as well.
//
// The surface of action states the free function. For the check it is read off the formed gear:
// `Action` is the locus of points on the gear's outer flank cone (a cone in the gear, carried by
// the gear's turn) where the cone's normal is reciprocal to the relative twist (the meshing law,
// (1)). Unlike the parallel and intersecting pairs, the skew pair's relative twist has a pitch,
// so v₁₂ vanishes nowhere and (1) never loses rank.
//
// `Contact` marches #326's element in that surface from the contact line at the home phase:
//   (2) the normal square to the line;  (3) the pinion sees the contact slide on its flank;
//   the gauge: the line moves square to itself within the surface.
// Its normal is (1) and (2) together — `nl reciprocal(seen_from: pinion) gear` and
// `nl perpendicular rate(x over sigma)` — and the surface it moves in is the stated one.
//
// Closed form: `rust/examples/spiral_bevel/formate.sv`'s generated pinion flank (the held
// envelope of the gear's tooth under the mesh). D5's gate holds the marched flank to it within
// 1e-7 mm (README.md says why), with `meshing` reporting line contact.
unit mm
use std
use rust.examples.spiral_bevel.formate (FormatePair)

component Action(cone: set, gear: frame, pinion: frame, phi: Angle, s: Scalar) {
  x := point
  y := point
  y coincident(under: gear, t == -phi) x        // x as the gear sees it
  y coincident cone                             // on the gear's formed flank
  T := plane
  T.origin coincident y
  T tangent(at: y) cone
  T.normal reciprocal(seen_from: pinion) gear   // (1): the meshing law, read in the gear
  r := point
  r coincident T.normal
  x distance(s * 20mm, along: T.u) y            // σ: a place along the line, in the flank
}

component Contact(action: surface, line0: curve, pinion: frame, gear: frame, phi: Angle,
                  sigma: Scalar) {
  x := point
  x coincident action                           // the surface of action: the free function
  initially {
    x coincident(t == sigma) line0              // the Cauchy data: the line at the home phase
  }
  k := point
  nl := line(x, k)
  length(10mm) nl
  nl reciprocal(seen_from: pinion) gear         // (1)
  nl perpendicular rate(x over sigma)           // (2)
  rate(x over phi, seen_from: pinion) perpendicular nl       // (3)
  rate(x over phi) perpendicular rate(x over sigma)          // the gauge
  X := point
  X coincident(under: pinion, t == -phi) x      // the element as the pinion sees it
}

pair := FormatePair(std.top, std.y, std.z, pinion_teeth: 24, gear_teeth: 48, mean_module: 2mm,
  shaft_angle: 90deg, offset: 25mm, pressure_shift: 12.5deg, spiral_angle: 25deg,
  backlash: 0mm, tip_relief: 0mm, end_relief: 0mm)

action := Action(pair.mill.outer_flank, pair.gear_turn, pair.pinion_turn).x
  over (phi in (-10deg, 10deg), s in (-1, 1))
line0 := Action(pair.mill.outer_flank, pair.gear_turn, pair.pinion_turn, phi: 0deg).x
  over s in (-1, 1)
flank := Contact(action, line0, pair.pinion_turn, pair.gear_turn).X
  over (phi in (-10deg, 10deg), sigma in (-1, 1))
contact := meshing(flank, pair.gear_tooth.outer over t in (-10deg, 10deg), under: pair.gear_turn,
  seen_from: pair.pinion_turn)
