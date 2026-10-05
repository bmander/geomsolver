// Step 2, the gear's pitch cone. The gear apex O and the mean point M lie in the pitch plane
// P; in G, the view square to P along O -> M, the right triangle O-M-F has the two pitch radii
// at M for its legs, F the foot of M on the gear axis. The pitch angle and the mean cone
// distance follow; nothing states them.
use std
use design
use views

component GearCone(p: plane, g: plane, design: group) {
  // Seeds only, rough: F on the side of OM the pitch angle opens to.
  r := design.cone_distance
  in p {
    O := point hint(x: 0, y: 0)
    M := point hint(x: r, y: 0)
    generator := line(O, M)
  }
  O coincident p.origin
  M distance(0mm, along: v) p
  in g {
    apex := point hint(x: 0, y: 0)
    mean := point hint(x: r, y: 0)
    foot := point hint(x: r / 4, y: r / 2)
    top := point hint(x: 0, y: -r)
    mirror := point hint(x: 0, y: r)
    pitch_line := line(apex, mean)
    to_apex := line(mean, apex)
    to_foot := line(mean, foot)
    axis := line(apex, foot)
    crown_axis := line(apex, top)
    opposite := line(apex, mirror)
  }
  apex coincident p
  mean coincident p
  O project apex
  M project mean
  to_foot perpendicular axis
  mean distance(design.gear_teeth * design.module / 2) foot
  apex distance(design.pinion_teeth * design.module / 2) foot
  // The crown's axis, square to P at the apex and as long as the cone.
  pitch_line angle(90deg, sense: cw) crown_axis
  crown_axis equal pitch_line
  // The generator across the axis from M, where the gear's blank is drawn.
  mean symmetry(axis) mirror
}

preview {
  unit mm
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.hypoid_design.cutter_radius)
}
