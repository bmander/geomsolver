// Step 2, the pinion's pitch cone, solved against the gear's. The shafts are stated, the
// shaft angle and the offset apart, and the cone touches P along its generator through M: its
// apex A lies in P, turned from O about M by the offset angle the solve answers, and its axis
// is drawn in Q, the view square to P along M -> A.
//
// The equal normal pitch sizes it, with no cosine written. V, where MA crosses the square from
// O to the trace's heading, falls with O on one point of the heading, so R cos(spiral) is
// |MV| cos(spiral + offset angle). A virtual axis leaves V at the bevel pinion's pitch angle,
// the angle at M in the gear's triangle, and the pinion's pitch radius is one circle about M
// that both axes touch. With no offset, A and V are O and the two axes one.
use std
use design
use views
use pitch.gear
use pitch.trace

// `gear` is the GearCone, `foot` the trace's square from O to its heading. A caller leaves
// `pinion_angle` unbound: the two triangles below construct it.
component PinionCone(p: plane, q: plane, gear: group, foot: line, design: group,
                     pinion_angle: Angle) {
  // Seeds only, rough: A and V the offset aside of O and back from it, and in Q the bevel
  // pinion's cone, its apex the offset along its pitch line.
  r := design.cone_distance
  d := design.offset
  in p {
    A := point hint((-0.75 * d, 1.25 * d))
    V := point hint((-0.75 * d, 1.25 * d))
    hinge := line(gear.M, A)
  }
  V coincident hinge
  V coincident foot
  in q {
    apex := point hint((d, 0))
    mean := point hint((-r, 0))
    virtual := point hint((d, 0))
    tip := point hint(x: d - r * design.gear_teeth / design.crown_teeth,
                      y: -r * design.pinion_teeth / design.crown_teeth)
    virtual_tip := point hint(at: tip)
    pitch_line := line(apex, mean)
    ax := line(apex, tip)
    virtual_line := line(virtual, mean)
    virtual_axis := line(virtual, virtual_tip)
    pitch_radius := circle(center: mean) hint(r: r * design.pinion_teeth / design.crown_teeth)
  }
  apex coincident p
  mean coincident p
  virtual coincident p
  A project apex
  gear.M project mean
  V project virtual
  ax equal pitch_line
  virtual_axis equal virtual_line
  ax tangent(side: right) pitch_radius
  virtual_axis tangent(side: right) pitch_radius
  // The shafts.
  gear.ax angle(design.shaft) ax
  gear.ax distance(design.offset) ax
  // The bevel pinion's pitch angle, generator to axis: the angle at M in the gear's triangle
  // is the angle at V.
  gear.to_apex angle(pinion_angle, sense: cw) gear.to_foot
  virtual_line angle(pinion_angle) virtual_axis
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  pinion := PinionCone(std.top, q.view, gear, trace.foot, design.hypoid_design)
  q := views.FoldedView(std.top, pinion.hinge)
}
