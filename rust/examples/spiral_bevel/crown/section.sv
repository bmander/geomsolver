// Step 4, the crown tooth's section, in the normal view: straight flanks at the pressure angle
// split by the shift, a base and a tip at their depths, and the tip roundings. Revolved about
// the cutter's axis the flanks are cones and the roundings tori.
//
// Backlash: the flank lines through the pitch points `lp` (inner) and `rp` (outer) are the ones
// the tooth shares with its mate (crown/mate_section.sv), and both stand each flank a quarter of
// the backlash outside them. The two cutters overlap by half the backlash across each line, and
// since the envelope of an offset surface is the offset of the envelope, each generated flank
// stands a quarter inside its conjugate one: half the backlash between each flank pair, all of
// it once one pair touches.
use std
use crown.rounding

component RackSection(lp: point, rp: point, design: group, normal_module: Length) {
  // Seeds only, rough, in pitch widths (the `pitch` line's run, half a normal pitch, so a
  // normal module is about 2 / pi of one). Each flank runs from its pitch point, square to the
  // pitch line and leaning by its pressure angle, to the base's depth and to its round's; each
  // round's centre stands a rounding inside its flank's end, and the tip's end a rounding above.
  outer_angle := design.pressure - design.shift
  inner_angle := design.pressure + design.shift
  base_depth := design.base * 2 / pi
  join_depth := (design.dedendum - design.rounding) * 2 / pi
  corner := design.rounding * 2 / pi
  construction pitch := line(lp, rp)
  private bi := point hint(at: lp, along: pitch, by: base_depth / cos(inner_angle),
                           turn: -90deg - inner_angle)
  private bo := point hint(at: rp, along: pitch, by: base_depth / cos(outer_angle),
                           turn: outer_angle - 90deg)
  private oj := point hint(at: rp, along: pitch, by: join_depth / cos(outer_angle),
                           turn: 90deg + outer_angle)
  private ij := point hint(at: lp, along: pitch, by: join_depth / cos(inner_angle),
                           turn: 90deg - inner_angle)
  private co := point hint(at: oj, along: pitch, by: -corner)
  private ci := point hint(at: ij, along: pitch, by: corner)
  private ot := point hint(at: co, along: pitch, by: corner, turn: 90deg)
  private it := point hint(at: ci, along: pitch, by: corner, turn: 90deg)
  profile := (base := line(bi, bo)) -> (outer := line(bo, oj)) -> tangent
             (outer_round := arc(center: co)) -> tangent (tip := line(ot, it)) -> tangent
             (inner_round := arc(center: ci)) -> tangent (inner := line(ij, bi)) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  // Each flank a quarter of the backlash outside its shared line; with none, on it.
  repeat design.lashed {
    lp distance(design.backlash / 4, side: left) inner
    rp distance(design.backlash / 4, side: left) outer
  }
  repeat 1 - design.lashed {
    lp coincident inner
    rp coincident outer
  }
  rounding := crown.rounding.TipRounding(pitch, base, tip, outer_round, inner_round, design,
    normal_module: normal_module)
}

preview {
  unit mm
  // The 24:48 pair's crown at module 2, symmetric, revolved about the page's y axis at eight
  // tenths of the cone distance; crown.svd draws it.
  pitch_radius := 0.8 * 2mm * hypot(24, 48) / 2
  proportions := {pressure: 20deg, shift: 0deg,
                       base: 1, dedendum: 1, rounding: 0.3, backlash: 0mm, lashed: 0}
  in std.front {
    lp := point hint(x: pitch_radius - 1.3mm, y: 0)
    rp := point hint(x: pitch_radius + 1.3mm, y: 0)
    std.origin distance(pitch_radius - 1.3mm, along: right) lp
    std.origin distance(0mm, along: up) lp
    std.origin distance(pitch_radius + 1.3mm, along: right) rp
    std.origin distance(0mm, along: up) rp
    rack := RackSection(lp, rp, proportions, normal_module: 2mm)
    construction centerline axis := line(std.origin, hint(x: 0, y: 1))
    fix(x == 0, y == 1) axis.p2
  }
  crown := solid(rack.profile, about: axis)
  outer := surface(crown, rack.outer)
  outer_round := surface(crown, rack.outer_round)
  inner := surface(crown, rack.inner)
  inner_round := surface(crown, rack.inner_round)
  tip := surface(crown, rack.tip)
}
