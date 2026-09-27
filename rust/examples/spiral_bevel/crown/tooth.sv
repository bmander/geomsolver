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
component CrownTooth(p: plane, normal: line, inner: point, outer: point, design: group) {
  // Seeds: a pitch-plane point's place along the trace normal is its x in N.
  point center hint(x: (normal.p1.x * (normal.p2.x - normal.p1.x)
      + normal.p1.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  point lp hint(x: (inner.x * (normal.p2.x - normal.p1.x) + inner.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  point rp hint(x: (outer.x * (normal.p2.x - normal.p1.x) + outer.y * (normal.p2.y - normal.p1.y))
    / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
  point top hint(x: center.x, y: -design.cutter_radius)
  center on p
  lp on p
  rp on p
  normal.p1 project center
  inner project lp
  outer project rp
  rack: RackSection(lp, rp, design)
  // The cutter's axis, pointing out of the tooth's tip.
  line axis(center, top)
  rack.pitch angle(90deg) axis
  center distance(design.cutter_radius) top
  construction solid crown(rack.profile, about: axis)
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
  n: FoldedView(pitch.view, trace.normal, span: hypoid_design.cutter_radius)
  tooth: CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    hypoid_design) in n.view
}
