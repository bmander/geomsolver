// Step 2, the gear's pitch cone. The gear apex O and the mean point M lie in the
// pitch plane; in G, the view square to it along O -> M, the right triangle O-M-F
// has the two pitch radii at M for its legs, F the foot of M on the gear axis.
// The pitch angle and the mean cone distance follow; nothing states them.
use std
use design
use views

component GearCone(p: plane, g: plane, design: group) {
  // Seeds only, rough: the triangle's foot on the side of OM its apex angle opens to.
  r := design.module * sqrt(design.pinion_teeth^2 + design.gear_teeth^2) / 2
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
  apex on p
  mean on p
  O project apex
  M project mean
  to_foot perpendicular axis
  mean distance(design.gear_teeth * design.module / 2) foot
  apex distance(design.pinion_teeth * design.module / 2) foot
  // The crown's axis stands square to the pitch plane at the apex, as long as the cone.
  pitch_line angle(90deg, sense: cw) crown_axis
  crown_axis equal pitch_line
  // The generator opposite M, where the gear's blank is drawn.
  mean symmetry(axis) mirror
}

preview {
  unit mm
  pitch := PitchView(std.front, span: hypoid_design.cutter_radius)
  gear := GearCone(pitch.view, g.view, hypoid_design)
  g := FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
}
