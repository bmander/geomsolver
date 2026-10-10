// Step 4, a section of the crown tooth's mate. It stands on the tooth's flank lines with its
// tip pointing the other way along the cutter's axis, so in N its counter-clockwise walk runs
// the other way round from RackSection's (base, inner, inner_round, tip, outer_round, outer),
// and each flank carries the pressure angle of the tooth's flank it lies on. Its flanks stand a
// quarter of the backlash outside the shared lines, as the tooth's do (crown/section.sv).
use std
use crown.section
use crown.rounding

// A mate section between pitch points `lp` (inner) and `rp` (outer), its inner flank along
// `inner_along` and its outer along `outer_along`, through its pitch points: the tooth's outer
// and inner flank lines, carried a tooth's width.
component MateSection(lp: point, rp: point, inner_along: line, outer_along: line,
                      design: group, normal_module: Length) {
  // Seeds only: RackSection's, the angles exchanged and the tip down.
  inner_angle := design.pressure - design.shift
  outer_angle := design.pressure + design.shift
  base_depth := design.base * 2 / pi
  join_depth := (design.dedendum - design.rounding) * 2 / pi
  corner := design.rounding * 2 / pi
  construction pitch := line(lp, rp)
  private bi := point hint(at: lp, along: pitch, by: base_depth / cos(inner_angle),
                           turn: 90deg + inner_angle)
  private bo := point hint(at: rp, along: pitch, by: base_depth / cos(outer_angle),
                           turn: 90deg - outer_angle)
  private ij := point hint(at: lp, along: pitch, by: join_depth / cos(inner_angle),
                           turn: inner_angle - 90deg)
  private oj := point hint(at: rp, along: pitch, by: join_depth / cos(outer_angle),
                           turn: -90deg - outer_angle)
  private ci := point hint(at: ij, along: pitch, by: corner)
  private co := point hint(at: oj, along: pitch, by: -corner)
  private it := point hint(at: ci, along: pitch, by: corner, turn: -90deg)
  private ot := point hint(at: co, along: pitch, by: corner, turn: -90deg)
  profile := (base := line(bo, bi)) -> (inner := line(bi, ij)) -> tangent
             (inner_round := arc(center: ci)) -> tangent (tip := line(it, ot)) -> tangent
             (outer_round := arc(center: co)) -> tangent (outer := line(oj, bo)) -> close
  inner_along angle(180deg) inner
  outer_along angle(180deg) outer
  // Each flank a quarter of the backlash outside its shared line.
  lp distance(design.backlash / 4, side: left) inner
  rp distance(design.backlash / 4, side: left) outer
  // The tip is a land: the rounds meeting in a point satisfy the section's rows too, and are
  // not the section (§9.6). A hundredth of a module; the narrowest land, the configured
  // hypoid's, is a sixth.
  distance(>= 0.01 * design.module) tip
  rounding := crown.rounding.TipRounding(pitch, base, tip, inner_round, outer_round, design,
    normal_module: normal_module)
}

preview {
  unit mm
  // The preview crown section's outer mate: the tooth one width outward on its flanks.
  pitch_radius := 0.8 * 2mm * hypot(24, 48) / 2
  proportions := {pressure: 20deg, shift: 0deg, module: 2mm,
                       base: 1, dedendum: 1, rounding: 0.3, backlash: 0mm}
  in std.front {
    lp := point
    rp := point
    far := point hint((pitch_radius + 3.9mm, 0))
    fix((pitch_radius - 1.3mm, 0)) lp
    fix((pitch_radius + 1.3mm, 0)) rp
    rack := crown.section.RackSection(lp, rp, proportions, normal_module: 2mm)
    construction span := line(lp, far)
    rp midpoint span
    mate := MateSection(rp, far, rack.outer, rack.inner, proportions, normal_module: 2mm)
  }
}
