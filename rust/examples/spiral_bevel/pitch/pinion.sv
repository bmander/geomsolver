// Step 2, the pinion's pitch cone, solved against the gear's. The shafts are
// stated, at the shaft angle and the offset apart, and the pinion's cone touches
// the pitch plane along its generator through M: its apex A lies in P, and its axis
// is drawn in Q, the view square to P along M -> A, from A's image. A turns from O
// about M by the offset angle, which the solve answers.
//
// What sizes the cone is the equal normal pitch. V is where the line MA crosses the
// square from O to the trace's heading, so O and V fall on one point of the heading:
// R cos(spiral) is |MV| cos(spiral + offset angle), with no cosine written. The
// virtual axis leaves V's image at the bevel pinion's pitch angle, the angle at M in
// the gear's triangle, and the pitch radius at M is one circle about M that both
// axes touch: the pinion's radius is the one a bevel pinion with its apex at V would
// have. At no offset A and V are O and the two axes one.
use std
use design
use views
use pitch.gear
use pitch.trace

// `gear` is the GearCone, `foot` the trace's square from O to its heading.
component PinionCone(p: plane, q: plane, gear: group, foot: line, design: group) {
  // Seeds only, rough: A and V about the offset aside of O and back from it, and in Q
  // the bevel pinion's cone with its apex stood the offset along its pitch line.
  param ct = sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  param r = design.module * ct / 2
  in p {
    point A hint(x: -0.75 * design.offset, y: 1.25 * design.offset)
    point V hint(x: -0.75 * design.offset, y: 1.25 * design.offset)
    line hinge(gear.M, A)
  }
  V on hinge
  V on foot
  in q {
    point apex hint(x: design.offset, y: 0)
    point mean hint(x: -r, y: 0)
    point virtual hint(x: design.offset, y: 0)
    point tip hint(x: design.offset - r * design.gear_teeth / ct,
                   y: -r * design.pinion_teeth / ct)
    point virtual_tip hint(x: tip.x, y: tip.y)
    line pitch_line(apex, mean)
    line axis(apex, tip)
    line virtual_line(virtual, mean)
    line virtual_axis(virtual, virtual_tip)
    circle pitch_radius(center: mean) hint(r: r * design.pinion_teeth / ct)
  }
  apex on p
  mean on p
  virtual on p
  A project apex
  gear.M project mean
  V project virtual
  axis equal pitch_line
  virtual_axis equal virtual_line
  axis tangent(side: right) pitch_radius
  virtual_axis tangent(side: right) pitch_radius
  // The shafts.
  gear.axis angle(design.shaft) axis
  gear.axis distance(design.offset) axis
  // The bevel pinion's pitch angle: from the generator to the axis, the angle at M in
  // the gear's triangle is the angle at V.
  gear.to_apex angle(pinion_angle, sense: cw) gear.to_foot
  virtual_line angle(pinion_angle) virtual_axis
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  pinion: PinionCone(pitch.view, q.view, gear, trace.foot, hypoid_design)
  q: FoldedView(pitch.view, pinion.hinge, span: hypoid_design.cutter_radius)
}
