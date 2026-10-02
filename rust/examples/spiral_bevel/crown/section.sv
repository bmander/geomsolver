// Step 4, the crown tooth's section, in the normal view: straight flanks at the pressure
// angle split by the shift, a base and a tip at their depths, and the tip roundings.
// Revolved about the cutter's axis the flanks are cones and the roundings tori.
//
// The flank lines through the pitch points `lp` (inner) and `rp` (outer) are the ones the
// tooth shares with its mate (crown/mate_section.sv); each flank stands a quarter of the
// backlash outside its line, and so does the mate's. The two cutters then overlap by half the
// backlash across each shared line, and since the envelope of an offset surface is the offset
// of the envelope, each generated flank stands a quarter of the backlash inside its conjugate
// one: the pair's normal clearance is half the backlash at each flank, the whole of it once
// one flank pair touches. The walk is counter-clockwise, so the tooth lies left of both flanks.
use std
use crown.rounding

component RackSection(lp: point, rp: point, design: group, normal_module: Length) {
  // Seeds only, rough, in pitch widths `rp.x - lp.x`: a normal module is about 2 / pi of
  // one (the width is half a normal pitch), and its sign says which way the section is
  // drawn (the tip on its left). Each flank leans by about its pressure angle in radians,
  // and each end of the tip stands a rounding inside it.
  outer_lean := (design.pressure - design.shift) / 1rad
  inner_lean := (design.pressure + design.shift) / 1rad
  base_depth := design.base * 2 / pi
  tip_depth := design.dedendum * 2 / pi
  join_depth := (design.dedendum - design.rounding) * 2 / pi
  end_depth := (design.dedendum - design.rounding / 2) * 2 / pi
  corner := design.rounding * 2 / pi
  private bi := point hint(x: lp.x - (rp.x - lp.x) * base_depth * inner_lean,
                        y: lp.y - (rp.x - lp.x) * base_depth)
  private bo := point hint(x: rp.x + (rp.x - lp.x) * base_depth * outer_lean, y: bi.y)
  private oj := point hint(x: rp.x - (rp.x - lp.x) * join_depth * outer_lean,
                        y: lp.y + (rp.x - lp.x) * join_depth)
  private ot := point hint(x: rp.x - (rp.x - lp.x) * (end_depth * outer_lean + corner),
                        y: lp.y + (rp.x - lp.x) * tip_depth)
  private it := point hint(x: lp.x + (rp.x - lp.x) * (end_depth * inner_lean + corner), y: ot.y)
  private ij := point hint(x: lp.x + (rp.x - lp.x) * join_depth * inner_lean, y: oj.y)
  private co := point hint(x: ot.x, y: oj.y)
  private ci := point hint(x: it.x, y: oj.y)
  construction pitch := line(lp, rp)
  profile := (base := line(bi, bo)) -> (outer := line(bo, oj)) -> tangent
            (outer_round := arc(center: co) hint(r: abs(ot.y - co.y))) -> tangent
            (tip := line(ot, it)) -> tangent
            (inner_round := arc(center: ci) hint(r: abs(ot.y - co.y))) -> tangent
            (inner := line(ij, bi)) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  // Each flank a quarter of the backlash outside its shared line; with none, on it.
  repeat design.lashed {
    lp distance(design.backlash / 4, side: left) inner
    rp distance(design.backlash / 4, side: left) outer
  }
  repeat 1 - design.lashed {
    lp on inner
    rp on outer
  }
  rounding := crown.rounding.TipRounding(pitch, base, tip, outer_round, inner_round, design,
    normal_module: normal_module)
}

preview {
  unit mm
  // The 24:48 pair's crown at module 2, symmetric, revolved about the page's y axis at
  // eight tenths of the cone distance; crown.svd draws it.
  pitch_radius := 0.8 * 2mm * hypot(24, 48) / 2
  proportions := group(pressure: 20deg, shift: 0deg,
                    base: 1, dedendum: 1, rounding: 0.3, backlash: 0mm, lashed: 0)
  lp := point hint(x: pitch_radius - 1.3mm, y: 0)
  rp := point hint(x: pitch_radius + 1.3mm, y: 0)
  std.origin distance(pitch_radius - 1.3mm, along: right) lp
  std.origin distance(0mm, along: up) lp
  std.origin distance(pitch_radius + 1.3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack := RackSection(lp, rp, proportions, normal_module: 2mm)
  construction centerline axis := line(std.origin, std.up.toward)
  crown := solid(rack.profile, about: axis)
  outer := surface(crown, rack.outer)
  outer_round := surface(crown, rack.outer_round)
  inner := surface(crown, rack.inner)
  inner_round := surface(crown, rack.inner_round)
  tip := surface(crown, rack.tip)
}
