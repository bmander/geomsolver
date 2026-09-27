// Step 2, the gear's pitch cone. The gear apex O and the mean point M lie in the
// pitch plane; in G, the view square to it along O -> M, the right triangle O-M-F
// has the two pitch radii at M for its legs, F the foot of M on the gear axis.
// The pitch angle and the mean cone distance follow; nothing states them.
use std
use design
use views

component GearCone(p: plane, g: plane, design: group) {
  // Seeds only: the closed forms the triangle solves to.
  param ct = sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  param r = design.module * ct / 2
  param c = design.pinion_teeth / ct
  param s = design.gear_teeth / ct
  in p {
    point O hint(x: 0, y: 0)
    point M hint(x: r, y: 0)
    line generator(O, M)
  }
  O coincident p.origin
  M distance(0mm, along: v) p
  in g {
    point apex hint(x: 0, y: 0)
    point mean hint(x: r, y: 0)
    point foot hint(x: r * c * c, y: r * c * s)
    point top hint(x: 0, y: -r)
    point mirror hint(x: r * (c * c - s * s), y: 2 * r * c * s)
    line pitch_line(apex, mean)
    line to_apex(mean, apex)
    line to_foot(mean, foot)
    line axis(apex, foot)
    line crown_axis(apex, top)
    line back_cone(apex, mirror)
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
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
}
