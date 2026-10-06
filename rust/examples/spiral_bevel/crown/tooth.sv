// Step 4, the pinion's generator: the crown tooth in N, the view square to the pitch plane
// along the trace normal C -> M, which shows it in true shape. The tooth's pitch points are the
// images of the thickness's, and the cutter's axis stands at C's image, square to the pitch
// plane and pointing out of the tooth's tip.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section

// Drawn in N (`in n`); `normal` runs from C to M in the pitch plane `p`, and `inner` and
// `outer` are the pitch points there.
component CrownTooth(p: plane, normal: line, inner: point, outer: point, design: group,
                     normal_module: Length) {
  // Seeds: the pitch-plane points read in N, where they lie on the fold, and the axis the trace
  // normal's length, |CM|, turned square to it.
  center := point hint(at: normal.p1)
  lp := point hint(at: inner)
  rp := point hint(at: outer)
  top := point hint(at: center, along: normal, turn: -90deg)
  center coincident p
  lp coincident p
  rp coincident p
  normal.p1 project center
  inner project lp
  outer project rp
  rack := crown.section.RackSection(lp, rp, design, normal_module: normal_module)
  ax := line(center, top)
  rack.pitch angle(90deg) ax
  center distance(design.cutter_radius) top
  construction crown := solid(rack.profile, about: ax)
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  n := views.FoldedView(std.top, trace.normal)
  tooth := CrownTooth(std.top, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design.hypoid_design, normal_module: trace.normal_module) in n.view
}
