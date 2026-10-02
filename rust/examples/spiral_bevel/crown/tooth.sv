// Step 4, the pinion's generator: the crown tooth in the normal view N, the view
// square to the pitch plane along the trace normal C -> M. The cutter's axis
// stands at C's image, square to the pitch plane; the tooth's pitch points are
// the images of the thickness's, beyond C, so N shows the tooth in true shape.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section

// Drawn in N (`in n`); `normal` runs from C to M in the pitch plane `p`, and
// `inner` and `outer` are the pitch points there.
component CrownTooth(p: plane, normal: line, inner: point, outer: point, design: group,
                     normal_module: Length) {
  // Seeds: a pitch-plane point's place along the trace normal is its x in N.
  center := point hint(x: (normal.p1.x * (normal.p2.x - normal.p1.x)
      + normal.p1.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  lp := point hint(x: (inner.x * (normal.p2.x - normal.p1.x) + inner.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  rp := point hint(x: (outer.x * (normal.p2.x - normal.p1.x) + outer.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  top := point hint(x: center.x, y: -design.cutter_radius)
  center on p
  lp on p
  rp on p
  normal.p1 project center
  inner project lp
  outer project rp
  rack := crown.section.RackSection(lp, rp, design, normal_module: normal_module)
  // The cutter's axis, pointing out of the tooth's tip.
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
  thickness := crown.thickness.CrownThickness(pitch.view, gear.generator, trace.normal, design.hypoid_design)
  n := views.FoldedView(pitch.view, trace.normal, span: design.hypoid_design.cutter_radius)
  tooth := CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design.hypoid_design) in n.view
  // Alone, the tooth's depths are in the trace's normal module: K stands that far from MC.
  trace.K distance(tooth.normal_module) trace.normal
}
