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
  center on p
  lp on p
  rp on p
  normal.p1 project center
  inner project lp
  outer project rp
  rack := crown.section.RackSection(lp, rp, design, normal_module: normal_module)
  axis := line(center, top)
  rack.pitch angle(90deg) axis
  center distance(design.cutter_radius) top
  construction crown := solid(rack.profile, about: axis)
}

preview {
  unit mm
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.hypoid_design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(pitch.view, gear.generator, trace.normal,
    design.hypoid_design)
  n := views.FoldedView(pitch.view, trace.normal, span: design.hypoid_design.cutter_radius)
  tooth := CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design.hypoid_design) in n.view
  // Alone, each component's normal module is its own unknown: the trace constructs it.
  trace.K distance(tooth.normal_module) trace.normal
}
