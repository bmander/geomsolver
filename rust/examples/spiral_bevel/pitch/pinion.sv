// Step 2, the pinion's pitch cone, solved against the gear's. The shafts are stated, the
// shaft angle and the offset apart, and the cone touches P along its generator through M: its
// apex A lies in P, turned from O about M by the offset angle the solve answers, and its axis
// stands square to P on its generator A -> M.
//
// The equal normal pitch sizes it, with no cosine written. V, where MA crosses the square from
// O to the trace's heading, falls with O on one point of the heading, so R cos(spiral) is
// |MV| cos(spiral + offset angle). A virtual axis leaves V at the bevel pinion's pitch angle,
// the angle at M in the gear's triangle, and the pinion's pitch radius is one distance from M
// that both axes keep. With no offset, A and V are O and the two axes one.
use std (skew)
use design
use pitch.gear
use pitch.trace

// `gear` is the GearCone, `foot` the trace's square from O to its heading. A caller leaves
// `pinion_angle` and `pitch_radius` unbound: the triangles below construct them.
component PinionCone(p: plane, gear: group, foot: line, design: group, pinion_angle: Angle,
                     pitch_radius: Length) {
  // Seeds only, rough: A and V the offset aside of O and back from it, the tips where the
  // bevel pinion's cone puts its axis, above P.
  r := design.cone_distance
  d := design.offset
  c := design.gear_teeth / design.crown_teeth
  s := design.pinion_teeth / design.crown_teeth
  in p {
    A := point hint((-0.75 * d, 1.25 * d))
    V := point hint((-0.75 * d, 1.25 * d))
    pitch_line := line(A, gear.M)
  }
  V coincident pitch_line
  V coincident foot
  // The offset's hand: the pinion's apex on P's v side of the gear's generator.
  A distance(>= 0, along: v) p
  tip := point hint((-0.75 * d + (r + 0.75 * d) * c, 1.25 * d * (1 - c), r * s))
  virtual_tip := point hint(at: tip)
  ax := line(A, tip)
  virtual_line := line(V, gear.M)
  virtual_axis := line(V, virtual_tip)
  // Both axes stand square to P on the generator, above it: square to the level line square to it.
  square := axis hint(dir: (0, 1, 0))
  square perpendicular pitch_line
  square perpendicular gear.crown_axis
  ax perpendicular square
  virtual_axis perpendicular square
  tip outside p
  virtual_tip outside p
  ax equal pitch_line
  virtual_axis equal virtual_line
  gear.M distance(pitch_radius) ax
  gear.M distance(pitch_radius) virtual_axis
  // The shafts.
  gear.ax skew(theta: design.shaft, e: design.offset) ax
  // The bevel pinion's pitch angle, generator to axis: the angle at M in the gear's triangle
  // is the angle at V.
  gear.to_apex angle(pinion_angle) gear.to_foot
  virtual_line angle(pinion_angle) virtual_axis
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, design.hypoid_design)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  pinion := PinionCone(std.top, gear, trace.foot, design.hypoid_design)
}
