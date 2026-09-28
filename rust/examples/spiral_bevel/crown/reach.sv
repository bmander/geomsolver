// Step 4, how far the gear's space cutter reaches: a cap in N standing |CO| + reach
// from the cutter's axis, clear of every blank point (crown/space.sv closes the
// cutter's sections there).
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section
use crown.tooth
use crown.mate

// The cap: in the pitch plane, the point `reach` past the apex on the ray from C
// through it, carried onto the trace normal beyond C, and its image in N; drawn
// there along the cutter's `axis`.
component CutterReach(p: plane, n: plane, apex: point, normal: line, axis: line, reach: Length) {
  in p {
    // Seeds, rough: beyond the apex from C, and about as far again beyond C from M.
    point beyond hint(x: 2 * apex.x - normal.p1.x, y: 2 * apex.y - normal.p1.y)
    point radial hint(x: 3 * normal.p1.x - 2 * normal.p2.x, y: 3 * normal.p1.y - 2 * normal.p2.y)
    private line to_apex(normal.p1, apex)
    private line past(apex, beyond)
    private line reach_line(normal.p1, beyond)
    private line cap_radius(normal.p1, radial)
  }
  to_apex angle(0deg) past
  distance(reach) past
  normal angle(180deg) cap_radius
  cap_radius equal reach_line
  in n {
    // Seed, rough: about twice the reach beyond the cutter's axis, away from M.
    point foot hint(x: axis.p1.x - 2 * reach, y: 0)
    point end hint(x: foot.x, y: foot.y + axis.p2.y - axis.p1.y)
    line cap(foot, end)
  }
  foot on p
  radial project foot
  axis angle(0deg) cap
  cap equal axis
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
  // Alone, the tooth's depths are in the trace's normal module: K stands that far from MC.
  trace.K distance(tooth.normal_module) trace.normal
  mate: CrownMate(tooth, hypoid_design) in n.view
  trace.K distance(mate.normal_module) trace.normal
  reach: CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: hypoid_design.space_reach)
}
