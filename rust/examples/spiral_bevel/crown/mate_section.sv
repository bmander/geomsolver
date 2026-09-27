// Step 4, a section of the crown tooth's mate. It stands on the tooth's flank lines
// with its tip pointing the other way along the cutter's axis, so in N it walks its
// edges the other way round from RackSection (base, inner, inner_round, tip,
// outer_round, outer), and each flank carries the pressure angle of the tooth's
// flank it lies on.
use std
use crown.section
use crown.rounding

// A mate section between pitch points `lp` (inner) and `rp` (outer), its inner flank
// along `inner_along` and its outer flank along `outer_along`, both through its pitch
// points: the tooth's outer and inner flanks, carried a tooth's width.
component MateSection(lp: point, rp: point, inner_along: line, outer_along: line,
                      design: group) {
  // Seeds only: RackSection's, the pressure angles exchanged and the tip down.
  param nm = design.normal_module
  param inner_pressure = design.pressure - design.shift
  param outer_pressure = design.pressure + design.shift
  param th = design.dedendum * nm
  param tr = design.rounding * nm
  param bd = design.base * nm
  param jo = th - tr * (1 - sin(outer_pressure))
  param ji = th - tr * (1 - sin(inner_pressure))
  param ro = jo * tan(outer_pressure) + tr * cos(outer_pressure)
  param ri = ji * tan(inner_pressure) + tr * cos(inner_pressure)
  private point bi hint(x: lp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(inner_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point bo hint(x: rp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(outer_pressure),
                        y: bi.y)
  private point ij hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ji * tan(inner_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * ji)
  private point it hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ri,
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point ot hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * ro, y: it.y)
  private point oj hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * jo * tan(outer_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * jo)
  private point ci hint(x: it.x, y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * (th - tr))
  private point co hint(x: ot.x, y: ci.y)
  construction line pitch(lp, rp)
  profile = line base(bo, bi) -> line inner(bi, ij) -> tangent
            arc inner_round(center: ci) hint(r: tr) -> tangent line tip(it, ot) -> tangent
            arc outer_round(center: co) hint(r: tr) -> tangent line outer(oj, bo) -> close
  inner_along angle(180deg) inner
  outer_along angle(180deg) outer
  lp on inner
  rp on outer
  rounding: TipRounding(pitch, base, tip, inner_round, outer_round, design)
}

preview {
  unit mm
  // The preview crown section's outer mate: the tooth one width outward on its flanks.
  param pitch_radius = 0.8 * 2mm * hypot(24, 48) / 2
  group proportions(normal_module: 2mm, pressure: 20deg, shift: 0deg,
                    base: 1, dedendum: 1, rounding: 0.3)
  point lp hint(x: pitch_radius - 1.3mm, y: 0)
  point rp hint(x: pitch_radius + 1.3mm, y: 0)
  point far hint(x: pitch_radius + 3.9mm, y: 0)
  std.origin distance(pitch_radius - 1.3mm, along: right) lp
  std.origin distance(0mm, along: up) lp
  std.origin distance(pitch_radius + 1.3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack: RackSection(lp, rp, proportions)
  construction line span(lp, far)
  rp midpoint span
  mate: MateSection(rp, far, rack.outer, rack.inner, proportions)
}
