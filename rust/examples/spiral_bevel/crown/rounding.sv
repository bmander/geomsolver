// Step 4, what every crown section shares: a base parallel to the pitch line and a tip
// running back along it, at their depths, and two tip roundings of one radius. The
// chain walks the section counter-clockwise, so the pitch line lies left of both.
use std

component TipRounding(pitch: line, base: line, tip: line, first: arc, second: arc,
                      design: group) {
  base parallel pitch
  base angle(180deg) tip
  pitch.p1 distance(design.base * design.normal_module, side: left) base
  pitch.p1 distance(design.dedendum * design.normal_module, side: left) tip
  radius(design.rounding * design.normal_module) first
  first equal second
}

preview {
  unit mm
  group proportions(normal_module: 2mm, base: 2, dedendum: 1.25, rounding: 0.3)
  point bi hint(x: -5, y: -4)
  point bo hint(x: 10, y: -4)
  point oj hint(x: 7, y: 1.5)
  point ot hint(x: 6, y: 2.5)
  point it hint(x: 0, y: 2.5)
  point ij hint(x: -1, y: 1.5)
  line pitch(std.origin, hint(x: 5, y: 0))
  std.origin distance(5mm, along: right) pitch.p2
  std.origin distance(0mm, along: up) pitch.p2
  profile = line base(bi, bo) -> line outer(bo, oj) ->
            tangent arc outer_round(center: hint(x: 6, y: 2)) -> tangent line tip(ot, it) ->
            tangent arc inner_round(center: hint(x: 0, y: 2)) -> tangent line inner(ij, bi) -> close
  base angle(110deg) outer
  base angle(250deg) inner
  std.origin on inner
  pitch.p2 on outer
  rounding: TipRounding(pitch, base, tip, outer_round, inner_round, proportions)
}
