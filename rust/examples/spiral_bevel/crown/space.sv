// Step 4, the gear's space cutter: the stretch between two neighbouring mate teeth, each active
// flank closed far from the working blank. It is the outer mate within the inner one indexed a
// crown pitch round, the outer section closed at the reach's cap (crown/reach.sv), the inner at
// the cutter's axis.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.tooth
use crown.mate
use crown.reach

// One active rounded flank, closed at `cap` along its base and tip. The walk enters the flank
// from whichever end meets its round.
component FlankSection(base: line, flank: line, corner: arc, tip: line, cap: line) {
  private base_end := point hint(x: cap.p1.x, y: base.p1.y)
  private tip_end := point hint(x: cap.p1.x, y: tip.p1.y)
  base_end on base
  base_end on cap
  tip_end on tip
  tip_end on cap
  profile := face(base_end, flank, corner, tip_end, -> close)
}

// The space between the outer mate section's inner flank and the inner mate section's outer
// flank, one crown pitch round: the outer crown within its indexed neighbour. Drawn in N, about
// the mate's `axis`.
component ComplementarySpace(axis: line, outside: group, inside: group, indexing: motion,
                             cap: line) {
  private outer := FlankSection(outside.base, outside.inner, outside.inner_round, outside.tip,
    cap)
  private inner := FlankSection(inside.base, inside.outer, inside.outer_round, inside.tip, axis)
  private construction outer_crown := solid(outer.profile, about: axis)
  private construction inner_crown := solid(inner.profile, about: axis)
  private construction neighbor := solid(inner_crown, under: indexing, at: 0deg)
  body := solid(outer_crown)
  neighbor bound body
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
  tooth := crown.tooth.CrownTooth(pitch.view, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design) in n.view
  mate := crown.mate.CrownMate(tooth, design.hypoid_design) in n.view
  // Alone, each component's normal module is its own unknown: the trace constructs it.
  trace.K distance(tooth.normal_module) trace.normal
  trace.K distance(mate.normal_module) trace.normal
  reach := crown.reach.CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: design.hypoid_design.space_reach)
  crown_neighbor := motion(about: gear.crown_axis,
    phase: -4 * length(thickness.ahead) / radius(thickness.ahead) * 1rad)
  space := ComplementarySpace(mate.axis, mate.outer, mate.inner, crown_neighbor,
    reach.cap) in n.view
}
