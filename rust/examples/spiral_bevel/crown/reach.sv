// Step 4, how far the gear's space cutter reaches: a cylinder about the cutter's axis through the
// point `reach` past the gear's apex on the line from C through it, clear of every blank point.
// crown/mate.sv closes the cutter's outer mate there.
use std
use design
use pitch.gear
use pitch.trace
use crown.thickness
use crown.cutter

// The cap: in the pitch plane `p`, the point `reach` past the apex on the line from C through it,
// and the cylinder about the cutter's `ax` through it.
component CutterReach(p: plane, apex: point, normal: line, ax: line, reach: Length) {
  // Seed, rough: as far beyond the apex as C is short of it.
  in p {
    beyond := point hint(at: apex, toward: normal.p1, by: -1)
    private to_apex := line(normal.p1, apex)
    private past := line(apex, beyond)
  }
  to_apex angle(0deg) past
  distance(reach) past
  cap := std.Cylinder(ax, r: hint(2 * reach))
  beyond coincident cap
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, design.hypoid_design)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  cutter := crown.cutter.Cutter(gear.crown_axis, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design, normal_module: trace.normal_module)
  reach := CutterReach(std.top, gear.O, trace.normal, cutter.ax,
    reach: design.hypoid_design.space_reach)
}
