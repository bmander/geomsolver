// Step 2, the pinion's pitch cone, solved against the gear's. The shafts are
// stated, at the shaft angle and the offset apart, and the pinion's cone touches
// the pitch plane along its generator through M: its apex A lies in P, and its axis
// is drawn in Q, the view square to P along M -> A, from A's image. A turns from O
// about M by the offset angle, which the solve answers.
//
// What sizes the cone is the equal normal pitch. V is where the line MA crosses the
// square from O to the trace's heading, so O and V fall on one point of the heading:
// R cos(spiral) is |MV| cos(spiral + offset angle), with no cosine written. The
// virtual axis leaves V's image at the bevel pinion's pitch angle, which the caller
// ties to the gear's triangle (layout.sv), and the pitch radius at M is one circle
// about M that both axes touch: the pinion's radius is the one a bevel pinion with
// its apex at V would have. At no offset A and V are O and the two axes one.
use std
use design
use views
use pitch.gear
use pitch.trace

component PinionCone(p: plane, q: plane, generator: line, foot: line, gear_axis: line,
                     design: group) {
  // Seeds only: the offset angle as if the offset were R sin of it, and the bevel
  // pinion's pitch angle.
  param ct = sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  param r = design.module * ct / 2
  param e = asin(design.offset / r)
  param rp = r * cos(design.spiral) / cos(design.spiral + e)
  param ex = r * cos(e)
  in p {
    point A hint(x: r - rp * cos(e), y: rp * sin(e))
    point V hint(x: r - rp * cos(e), y: rp * sin(e))
    line hinge(generator.p2, A)
  }
  V on hinge
  V on foot
  in q {
    point apex hint(x: rp - ex, y: 0)
    point mean hint(x: -ex, y: 0)
    point virtual hint(x: rp - ex, y: 0)
    point tip hint(x: rp - ex - rp * design.gear_teeth / ct, y: -rp * design.pinion_teeth / ct)
    point virtual_tip hint(x: rp - ex - rp * design.gear_teeth / ct,
                           y: -rp * design.pinion_teeth / ct)
    line pitch_line(apex, mean)
    line axis(apex, tip)
    line virtual_line(virtual, mean)
    line virtual_axis(virtual, virtual_tip)
    circle pitch_radius(center: mean) hint(r: rp * design.pinion_teeth / ct)
  }
  apex on p
  mean on p
  virtual on p
  A project apex
  generator.p2 project mean
  V project virtual
  axis equal pitch_line
  virtual_axis equal virtual_line
  axis tangent(side: right) pitch_radius
  virtual_axis tangent(side: right) pitch_radius
  // The shafts.
  gear_axis angle(design.shaft) axis
  gear_axis distance(design.offset) axis
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  pinion: PinionCone(pitch.view, q.view, gear.generator, trace.foot, gear.axis, hypoid_design)
  q: FoldedView(pitch.view, pinion.hinge, span: hypoid_design.cutter_radius)
  gear.to_foot angle(pinion_angle) gear.to_apex
  pinion.virtual_line angle(pinion_angle) pinion.virtual_axis
}
