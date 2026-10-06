// Step 4, the gear's generator: the crown tooth's mate, two sections on the tooth's own flank
// lines (crown/mate_section.sv), one tooth's width outward and one inward along the pitch line,
// each with its own tip and roundings, revolved about the cutter's axis turned tip down. The
// shift is stated once, on the tooth, so the two crowns are complementary by construction.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.tooth
use crown.mate_section

// The mate of `tooth`, a CrownTooth.
component CrownMate(tooth: group, design: group, normal_module: Length) {
  // Seeds: the tooth's pitch points reflected through each other, and its axis turned.
  outer_far := point hint(at: tooth.rack.pitch.p2, toward: tooth.rack.pitch.p1, by: -1)
  inner_far := point hint(at: tooth.rack.pitch.p1, toward: tooth.rack.pitch.p2, by: -1)
  bottom := point hint(at: tooth.ax.p1, toward: tooth.ax.p2, by: -1)
  construction outer_span := line(tooth.rack.pitch.p1, outer_far)
  construction inner_span := line(inner_far, tooth.rack.pitch.p2)
  tooth.rack.pitch.p2 midpoint outer_span
  tooth.rack.pitch.p1 midpoint inner_span
  outer := crown.mate_section.MateSection(tooth.rack.pitch.p2, outer_far, tooth.rack.outer,
    tooth.rack.inner, design, normal_module: normal_module)
  inner := crown.mate_section.MateSection(inner_far, tooth.rack.pitch.p1, tooth.rack.outer,
    tooth.rack.inner, design, normal_module: normal_module)
  ax := line(tooth.ax.p1, bottom)
  tooth.ax angle(180deg) ax
  ax equal tooth.ax
  construction outer_crown := solid(outer.profile, about: ax)
  construction inner_crown := solid(inner.profile, about: ax)
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  n := views.FoldedView(std.top, trace.normal)
  tooth := crown.tooth.CrownTooth(std.top, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design, normal_module: trace.normal_module) in n.view
  mate := CrownMate(tooth, design.hypoid_design,
    normal_module: trace.normal_module) in n.view
}
