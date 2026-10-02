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
  ct := sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  r := design.module * ct / 2
  in p {
    A := point hint(x: -0.75 * design.offset, y: 1.25 * design.offset)
    V := point hint(x: -0.75 * design.offset, y: 1.25 * design.offset)
    hinge := line(gear.M, A)
  }
  V on hinge
  V on foot
  in q {
    apex := point hint(x: design.offset, y: 0)
    mean := point hint(x: -r, y: 0)
    virtual := point hint(x: design.offset, y: 0)
    tip := point hint(x: design.offset - r * design.gear_teeth / ct,
                   y: -r * design.pinion_teeth / ct)
    virtual_tip := point hint(x: tip.x, y: tip.y)
    pitch_line := line(apex, mean)
    axis := line(apex, tip)
    virtual_line := line(virtual, mean)
    virtual_axis := line(virtual, virtual_tip)
    pitch_radius := circle(center: mean) hint(r: r * design.pinion_teeth / ct)
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
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.hypoid_design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design.hypoid_design)
  pinion := PinionCone(pitch.view, q.view, gear, trace.foot, design.hypoid_design)
  q := views.FoldedView(pitch.view, pinion.hinge, span: design.hypoid_design.cutter_radius)
}
