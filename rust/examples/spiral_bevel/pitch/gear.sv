// Step 2, the gear's pitch cone. The gear apex O and the mean point M lie in the pitch plane P and
// in G, the view square to P along O -> M; in G the right triangle O-M-F has the two pitch radii at
// M for its legs, F the foot of M on the gear axis. The pitch angle and the mean cone distance
// follow; nothing states them.
use std
use design
use views

component GearCone(p: plane, g: plane, design: group) {
  // Seeds only, rough.
  r := design.cone_distance
  // O and M lie on the line where P and G meet, so each is drawn in both.
  O := point hint((0, 0)) in p, g
  M := point hint((r, 0)) in p, g
  in p {
    generator := line(O, M)
  }
  O coincident p.origin
  M level(v) p
  in g {
    foot := point hint((r / 4, r / 2))
    top := point hint((0, -r))
    mirror := point hint((0, r))
    pitch_line := line(O, M)
    to_apex := line(M, O)
    to_foot := line(M, foot)
    ax := line(O, foot)
    crown_axis := line(O, top)
    opposite := line(O, mirror)
  }
  // The gear's axis runs down from the pitch plane.
  foot inside p
  to_foot perpendicular ax
  M distance(design.gear_teeth * design.module / 2) foot
  O distance(design.pinion_teeth * design.module / 2) foot
  // The crown's axis, square to P at the apex and as long as the cone.
  pitch_line angle(90deg, sense: cw) crown_axis
  crown_axis equal pitch_line
  // The generator across the axis from M, where the gear's blank is drawn.
  M symmetry(ax) mirror
}

preview {
  unit mm
  gear := GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
}
