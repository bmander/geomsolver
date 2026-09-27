// Step 4, the crown tooth's section, in the normal view: straight flanks through
// the pitch points `lp` (inner) and `rp` (outer) at the pressure angle split by the
// shift, a base and a tip at their depths, and the tip roundings. Revolved about
// the cutter's axis the flanks are cones and the roundings tori.
use std
use crown.rounding

component RackSection(lp: point, rp: point, design: group) {
  // Seeds only: the closed forms, turned to where the pitch points are drawn.
  param nm = design.normal_module
  param outer_pressure = design.pressure - design.shift
  param inner_pressure = design.pressure + design.shift
  param th = design.dedendum * nm
  param tr = design.rounding * nm
  param bd = design.base * nm
  param jo = th - tr * (1 - sin(outer_pressure))
  param ji = th - tr * (1 - sin(inner_pressure))
  param ro = jo * tan(outer_pressure) + tr * cos(outer_pressure)
  param ri = ji * tan(inner_pressure) + tr * cos(inner_pressure)
  private point bi hint(x: lp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(inner_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point bo hint(x: rp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(outer_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point oj hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * jo * tan(outer_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * jo)
  private point ot hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * ro,
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point it hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ri,
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point ij hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ji * tan(inner_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * ji)
  private point co hint(x: ot.x, y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * (th - tr))
  private point ci hint(x: it.x, y: co.y)
  construction line pitch(lp, rp)
  profile = line base(bi, bo) -> line outer(bo, oj) -> tangent
            arc outer_round(center: co) hint(r: tr) -> tangent line tip(ot, it) -> tangent
            arc inner_round(center: ci) hint(r: tr) -> tangent line inner(ij, bi) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  lp on inner
  rp on outer
  rounding: TipRounding(pitch, base, tip, outer_round, inner_round, design)
}

preview {
  unit mm
  // The 24:48 pair's crown at module 2, symmetric, revolved about the page's y axis at
  // eight tenths of the cone distance; crown.svd draws it.
  param pitch_radius = 0.8 * 2mm * hypot(24, 48) / 2
  group proportions(normal_module: 2mm, pressure: 20deg, shift: 0deg,
                    base: 1, dedendum: 1, rounding: 0.3)
  point lp hint(x: pitch_radius - 1.3mm, y: 0)
  point rp hint(x: pitch_radius + 1.3mm, y: 0)
  std.origin distance(pitch_radius - 1.3mm, along: right) lp
  std.origin distance(0mm, along: up) lp
  std.origin distance(pitch_radius + 1.3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack: RackSection(lp, rp, proportions)
  construction centerline line axis(std.origin, std.up.toward)
  solid crown(rack.profile, about: axis)
  surface outer(crown, rack.outer)
  surface outer_round(crown, rack.outer_round)
  surface inner(crown, rack.inner)
  surface inner_round(crown, rack.inner_round)
  surface tip(crown, rack.tip)
}
