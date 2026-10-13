// Step 4, the pinion's generator: the crown tooth, a region about the cutter's axis (§6.21) —
// between the flank cones through the pitch points (crown/cutter.sv), from its base to its tip,
// its tip's edges rounded — and its solid, the line-and-arc meridian the region folds into turned
// once about the axis.
//
// Backlash: the flank cones (crown/cutter.sv) are the ones the tooth shares with the gear's space
// (crown/mate.sv), and both grow across each a quarter of the backlash. The two cutters overlap by
// half the backlash across each cone, and since the envelope of an offset surface is the offset of
// the envelope, each generated flank stands a quarter inside its conjugate one: half the backlash
// between each flank pair, all of it once one pair touches.
use std
use design
use pitch.gear
use pitch.trace
use crown.thickness
use crown.cutter

// A crown tooth about a cutter's axis: beyond the flank cone `a` and within `b`, each grown by
// `grow`, above the level `lo` and below `hi`, the edges of its tip at `hi` rounded by `r`; and
// its solid, `crown`. Each set the region is within names the face it bounds: `inner`, `outer`,
// `base`, `tip`, and the rounds `inner_round` and `outer_round`.
component Tooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := { q | q outside(grow) a }
  outer := std.Offset(b, d: grow)
  base := { q | q outside lo }
  tip := { q | q inside hi }
  inner_round := std.Rounded(inner, tip, r: r)
  outer_round := std.Rounded(outer, tip, r: r)
  region := { p | p inside inner; p inside outer; p inside base; p inside tip
                  p inside inner_round; p inside outer_round }
  construction crown := solid(region)
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, design.hypoid_design)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  cutter := crown.cutter.Cutter(gear.crown_axis, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design, normal_module: trace.normal_module)
  tooth := Tooth(cutter.inner_flank, cutter.outer_flank, cutter.tooth_base, cutter.tooth_tip,
    grow: design.hypoid_design.backlash / 4, r: design.hypoid_design.rounding * trace.normal_module)
}
