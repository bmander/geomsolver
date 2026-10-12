// Step 4, the gear's generator: the crown tooth's mate, tip down, as the gear's space cutter —
// the stretch between two neighbouring mate teeth, each a region about the cutter's axis on the
// flank cones the tooth stands on (crown/cutter.sv), so the two crowns are complementary by
// construction. The outer mate lies beyond the tooth's outer cone, closed at the reach's cap
// (crown/reach.sv); the inner within its inner cone, out to the axis, and turned one crown pitch
// round. Each grows across its cone a quarter of the backlash, as the tooth does (crown/tooth.sv).
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.cutter
use crown.reach
use generation
use pitch.pinion

// A crown space, tip down: beyond the flank cone `a` and within the cap `cap`, within the flank
// cone `b` turned by `index` — each cone grown by `grow` — above the level `lo` and below `hi`,
// the edges of its tips at `lo` rounded by `r`. Each set it is within names the face it bounds:
// `inner`, `outer`, `tip`, `base`, `cap` and the rounds `inner_round` and `outer_round`; the outer
// mate is `outer_crown`, the inner `inner_crown`, turned `neighbor`.
component Space(a: set, b: set, lo: set, hi: set, cap: set, index: motion, grow: Length,
                r: Length) {
  inner := { q | q outside(grow) a }
  outer := std.Offset(b, d: grow)
  tip := { q | q outside lo }
  base := { q | q inside hi }
  inner_round := std.Rounded(inner, tip, r: r)
  outer_round := std.Rounded(outer, tip, r: r)
  near := { p | p inside inner; p inside cap; p inside tip; p inside base; p inside inner_round }
  far := { p | p inside outer; p inside tip; p inside base; p inside outer_round }
  private construction outer_crown := solid(near)
  private construction inner_crown := solid(far)
  private construction neighbor := solid(inner_crown, under: index, at: 0deg)
  body := solid(outer_crown)
  neighbor bound body
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  pinion := pitch.pinion.PinionCone(std.top, q.view, gear, trace.foot, design.hypoid_design)
  q := views.FoldedView(std.top, pinion.hinge)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  cutter := crown.cutter.Cutter(gear.crown_axis, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design, normal_module: trace.normal_module)
  reach := crown.reach.CutterReach(std.top, gear.O, trace.normal, cutter.ax,
    reach: design.hypoid_design.space_reach)
  generation := generation.Generation(gear, pinion, thickness)
  gear_space := Space(cutter.outer_flank, cutter.inner_flank, cutter.mate_tip, cutter.mate_base,
    reach.cap, generation.crown_neighbor, grow: design.hypoid_design.backlash / 4,
    r: design.hypoid_design.rounding * trace.normal_module)
}
