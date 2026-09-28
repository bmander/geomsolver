// Step 4, the crown tooth's section, in the normal view: straight flanks through
// the pitch points `lp` (inner) and `rp` (outer) at the pressure angle split by the
// shift, a base and a tip at their depths, and the tip roundings. Revolved about
// the cutter's axis the flanks are cones and the roundings tori.
use std
use crown.rounding

component RackSection(lp: point, rp: point, design: group, normal_module: Length) {
  // Seeds only, rough, in pitch widths `rp.x - lp.x`: a normal module is about 2 / pi of
  // one (the width is half a normal pitch), and its sign says which way the section is
  // drawn (the tip on its left). Each flank leans by about its pressure angle in radians,
  // and each end of the tip stands a rounding inside it.
  param outer_lean = (design.pressure - design.shift) / 1rad
  param inner_lean = (design.pressure + design.shift) / 1rad
  param base_depth = design.base * 2 / pi
  param tip_depth = design.dedendum * 2 / pi
  param join_depth = (design.dedendum - design.rounding) * 2 / pi
  param end_depth = (design.dedendum - design.rounding / 2) * 2 / pi
  param corner = design.rounding * 2 / pi
  private point bi hint(x: lp.x - (rp.x - lp.x) * base_depth * inner_lean,
                        y: lp.y - (rp.x - lp.x) * base_depth)
  private point bo hint(x: rp.x + (rp.x - lp.x) * base_depth * outer_lean, y: bi.y)
  private point oj hint(x: rp.x - (rp.x - lp.x) * join_depth * outer_lean,
                        y: lp.y + (rp.x - lp.x) * join_depth)
  private point ot hint(x: rp.x - (rp.x - lp.x) * (end_depth * outer_lean + corner),
                        y: lp.y + (rp.x - lp.x) * tip_depth)
  private point it hint(x: lp.x + (rp.x - lp.x) * (end_depth * inner_lean + corner), y: ot.y)
  private point ij hint(x: lp.x + (rp.x - lp.x) * join_depth * inner_lean, y: oj.y)
  private point co hint(x: ot.x, y: oj.y)
  private point ci hint(x: it.x, y: oj.y)
  construction line pitch(lp, rp)
  profile = line base(bi, bo) -> line outer(bo, oj) -> tangent
            arc outer_round(center: co) hint(r: abs(ot.y - co.y)) -> tangent
            line tip(ot, it) -> tangent
            arc inner_round(center: ci) hint(r: abs(ot.y - co.y)) -> tangent
            line inner(ij, bi) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  lp on inner
  rp on outer
  rounding: TipRounding(pitch, base, tip, outer_round, inner_round, design,
    normal_module: normal_module)
}

preview {
  unit mm
  // The 24:48 pair's crown at module 2, symmetric, revolved about the page's y axis at
  // eight tenths of the cone distance; crown.svd draws it.
  param pitch_radius = 0.8 * 2mm * hypot(24, 48) / 2
  group proportions(pressure: 20deg, shift: 0deg,
                    base: 1, dedendum: 1, rounding: 0.3)
  point lp hint(x: pitch_radius - 1.3mm, y: 0)
  point rp hint(x: pitch_radius + 1.3mm, y: 0)
  std.origin distance(pitch_radius - 1.3mm, along: right) lp
  std.origin distance(0mm, along: up) lp
  std.origin distance(pitch_radius + 1.3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack: RackSection(lp, rp, proportions, normal_module: 2mm)
  construction centerline line axis(std.origin, std.up.toward)
  solid crown(rack.profile, about: axis)
  surface outer(crown, rack.outer)
  surface outer_round(crown, rack.outer_round)
  surface inner(crown, rack.inner)
  surface inner_round(crown, rack.inner_round)
  surface tip(crown, rack.tip)
}
