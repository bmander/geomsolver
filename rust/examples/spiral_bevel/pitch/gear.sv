// Step 2, the gear's pitch cone. The gear apex O and the mean point M lie in the pitch plane P;
// the right triangle O-M-F stands square to P on the generator O -> M, F the foot of M on the gear
// axis, and has the two pitch radii at M for its legs. The pitch angle and the mean cone distance
// follow; nothing states them.
use std
use design

component GearCone(p: plane, design: group) {
  // Seeds only, rough.
  r := design.cone_distance
  in p {
    O := point hint((0, 0))
    M := point hint((r, 0))
    generator := line(O, M)
  }
  O coincident p.origin
  M level(v) p
  // The triangle, in space on the generator, square to P: F below it.
  foot := point hint((r / 4, 0, -r / 2))
  top := point hint((0, 0, r))
  pitch_line := line(O, M)
  to_apex := line(M, O)
  to_foot := line(M, foot)
  ax := line(O, foot)
  crown_axis := line(O, top)
  O level(p.v) foot
  // The gear's axis runs down from the pitch plane.
  foot inside p
  to_foot perpendicular ax
  M distance(design.gear_teeth * design.module / 2) foot
  O distance(design.pinion_teeth * design.module / 2) foot
  // The crown's axis, square to P at the apex, up, and as long as the cone.
  O level(p.u) top
  O level(p.v) top
  top outside p
  crown_axis equal pitch_line
}

preview {
  unit mm
  gear := GearCone(std.top, design.hypoid_design)
}
