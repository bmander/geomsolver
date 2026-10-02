// Step 4, a section of the crown tooth's mate. It stands on the tooth's flank lines
// with its tip pointing the other way along the cutter's axis, so in N it walks its
// edges the other way round from RackSection (base, inner, inner_round, tip,
// outer_round, outer), and each flank carries the pressure angle of the tooth's
// flank it lies on. Its walk is counter-clockwise too, and each flank stands a quarter
// of the backlash outside the shared line, as the tooth's does (crown/section.sv).
use std
use crown.section
use crown.rounding

// A mate section between pitch points `lp` (inner) and `rp` (outer), its inner flank
// along `inner_along` and its outer flank along `outer_along`, both through its pitch
// points: the tooth's outer and inner flanks, carried a tooth's width.
component MateSection(lp: point, rp: point, inner_along: line, outer_along: line,
                      design: group, normal_module: Length) {
  // Seeds only: RackSection's, the leans exchanged and the tip down.
  inner_lean := (design.pressure - design.shift) / 1rad
  outer_lean := (design.pressure + design.shift) / 1rad
  base_depth := design.base * 2 / pi
  tip_depth := design.dedendum * 2 / pi
  join_depth := (design.dedendum - design.rounding) * 2 / pi
  end_depth := (design.dedendum - design.rounding / 2) * 2 / pi
  corner := design.rounding * 2 / pi
  private bi := point hint(x: lp.x - (rp.x - lp.x) * base_depth * inner_lean,
                        y: lp.y + (rp.x - lp.x) * base_depth)
  private bo := point hint(x: rp.x + (rp.x - lp.x) * base_depth * outer_lean, y: bi.y)
  private ij := point hint(x: lp.x + (rp.x - lp.x) * join_depth * inner_lean,
                        y: lp.y - (rp.x - lp.x) * join_depth)
  private it := point hint(x: lp.x + (rp.x - lp.x) * (end_depth * inner_lean + corner),
                        y: lp.y - (rp.x - lp.x) * tip_depth)
  private ot := point hint(x: rp.x - (rp.x - lp.x) * (end_depth * outer_lean + corner), y: it.y)
  private oj := point hint(x: rp.x - (rp.x - lp.x) * join_depth * outer_lean, y: ij.y)
  private ci := point hint(x: it.x, y: ij.y)
  private co := point hint(x: ot.x, y: ij.y)
  construction pitch := line(lp, rp)
  profile := (base := line(bo, bi)) -> (inner := line(bi, ij)) -> tangent
            (inner_round := arc(center: ci) hint(r: abs(it.y - ci.y))) -> tangent
            (tip := line(it, ot)) -> tangent
            (outer_round := arc(center: co) hint(r: abs(it.y - ci.y))) -> tangent
            (outer := line(oj, bo)) -> close
  inner_along angle(180deg) inner
  outer_along angle(180deg) outer
  // Each flank a quarter of the backlash outside its shared line; with none, on it.
  repeat design.lashed {
    lp distance(design.backlash / 4, side: left) inner
    rp distance(design.backlash / 4, side: left) outer
  }
  repeat 1 - design.lashed {
    lp on inner
    rp on outer
  }
  rounding := TipRounding(pitch, base, tip, inner_round, outer_round, design,
    normal_module: normal_module)
}

preview {
  unit mm
  // The preview crown section's outer mate: the tooth one width outward on its flanks.
  pitch_radius := 0.8 * 2mm * hypot(24, 48) / 2
  proportions := group(pressure: 20deg, shift: 0deg,
                    base: 1, dedendum: 1, rounding: 0.3, backlash: 0mm, lashed: 0)
  lp := point hint(x: pitch_radius - 1.3mm, y: 0)
  rp := point hint(x: pitch_radius + 1.3mm, y: 0)
  far := point hint(x: pitch_radius + 3.9mm, y: 0)
  std.origin distance(pitch_radius - 1.3mm, along: right) lp
  std.origin distance(0mm, along: up) lp
  std.origin distance(pitch_radius + 1.3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack := RackSection(lp, rp, proportions, normal_module: 2mm)
  construction span := line(lp, far)
  rp midpoint span
  mate := MateSection(rp, far, rack.outer, rack.inner, proportions, normal_module: 2mm)
}
