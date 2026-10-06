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
// from whichever end meets its round. Each end is where two lines cross, so needs no seed.
component FlankSection(base: line, flank: line, corner: arc, tip: line, cap: line) {
  private base_end := point
  private tip_end := point
  base_end coincident base
  base_end coincident cap
  tip_end coincident tip
  tip_end coincident cap
  profile := face(base_end, flank, corner, tip_end, -> close)
}

// The space between the outer mate section's inner flank and the inner mate section's outer
// flank, one crown pitch round: the outer crown within its indexed neighbour. Drawn in N, about
// the mate's `axis`.
component ComplementarySpace(ax: line, outside: group, inside: group, indexing: motion,
                             cap: line) {
  private outer := FlankSection(outside.base, outside.inner, outside.inner_round, outside.tip,
    cap)
  private inner := FlankSection(inside.base, inside.outer, inside.outer_round, inside.tip, ax)
  private construction outer_crown := solid(outer.profile, about: ax)
  private construction inner_crown := solid(inner.profile, about: ax)
  private construction neighbor := solid(inner_crown, under: indexing, at: 0deg)
  body := solid(outer_crown)
  neighbor bound body
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
  mate := crown.mate.CrownMate(tooth, design.hypoid_design,
    normal_module: trace.normal_module) in n.view
  reach := crown.reach.CutterReach(std.top, n.view, gear.O, trace.normal, mate.ax,
    reach: design.hypoid_design.space_reach)
  crown_neighbor := motion(about: gear.crown_axis,
    phase: -4 * length(thickness.ahead) / radius(thickness.ahead) * 1rad)
  space := ComplementarySpace(mate.ax, mate.outer, mate.inner, crown_neighbor,
    reach.cap) in n.view
}
