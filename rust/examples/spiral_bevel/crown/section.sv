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
  param jr = th - tr * (1 - sin(outer_pressure))
  param jl = th - tr * (1 - sin(inner_pressure))
  param rr = jr * tan(outer_pressure) + tr * cos(outer_pressure)
  param rl = jl * tan(inner_pressure) + tr * cos(inner_pressure)
  private point bl hint(x: lp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(inner_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point br hint(x: rp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(outer_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point rj hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * jr * tan(outer_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * jr)
  private point rt hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * rr,
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point lt hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * rl,
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point lj hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * jl * tan(inner_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * jl)
  private point cr hint(x: rt.x, y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * (th - tr))
  private point cl hint(x: lt.x, y: cr.y)
  construction line pitch(lp, rp)
  profile = line base(bl, br) -> line outer(br, rj) -> tangent
            arc outer_round(center: cr) hint(r: tr) -> tangent line tip(rt, lt) -> tangent
            arc inner_round(center: cl) hint(r: tr) -> tangent line inner(lj, bl) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  lp on inner
  rp on outer
  rounding: TipRounding(pitch, base, tip, outer_round, inner_round, design)
}

preview {
  unit mm
  group proportions(normal_module: 1.8mm, pressure: 20deg, shift: 10deg,
                    base: 2, dedendum: 1.25, rounding: 0.3)
  point rp hint(x: 3, y: 0)
  std.origin distance(3mm, along: right) rp
  std.origin distance(0mm, along: up) rp
  rack: RackSection(std.origin, rp, proportions)
}
