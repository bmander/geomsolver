// Step 4, the gear's generator: the crown tooth's mate. Its two sections
// (crown/mate_section.sv) lie on the tooth's own flank lines, one tooth's width
// outward and one inward along the pitch line, each with its own tip and roundings,
// and it turns about the cutter's axis pointing the other way. The shift is stated
// once, on the tooth, and the two crowns are complementary by construction.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section
use crown.tooth
use crown.mate_section

// The mate of `tooth` (a CrownTooth), revolved about the cutter's axis turned tip down.
component CrownMate(tooth: group, design: group, normal_module: Length) {
  point outer_far hint(x: 2 * tooth.rack.pitch.p2.x - tooth.rack.pitch.p1.x,
                       y: tooth.rack.pitch.p2.y)
  point inner_far hint(x: 2 * tooth.rack.pitch.p1.x - tooth.rack.pitch.p2.x,
                       y: tooth.rack.pitch.p1.y)
  point bottom hint(x: tooth.axis.p1.x, y: 2 * tooth.axis.p1.y - tooth.axis.p2.y)
  construction line outer_span(tooth.rack.pitch.p1, outer_far)
  construction line inner_span(inner_far, tooth.rack.pitch.p2)
  tooth.rack.pitch.p2 midpoint outer_span
  tooth.rack.pitch.p1 midpoint inner_span
  outer: MateSection(tooth.rack.pitch.p2, outer_far, tooth.rack.outer, tooth.rack.inner, design,
    normal_module: normal_module)
  inner: MateSection(inner_far, tooth.rack.pitch.p1, tooth.rack.outer, tooth.rack.inner, design,
    normal_module: normal_module)
  line axis(tooth.axis.p1, bottom)
  tooth.axis angle(180deg) axis
  axis equal tooth.axis
  construction solid outer_crown(outer.profile, about: axis)
  construction solid inner_crown(inner.profile, about: axis)
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
}
