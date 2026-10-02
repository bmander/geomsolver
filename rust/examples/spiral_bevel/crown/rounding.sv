// Step 4, what every crown section shares: a base parallel to the pitch line and a tip running
// back along it, at their depths, and two tip roundings of one radius. The chain walks the
// section counter-clockwise, so the pitch line lies left of both.
use std

component TipRounding(pitch: line, base: line, tip: line, first: arc, second: arc,
                      design: group, normal_module: Length) {
  base parallel pitch
  base angle(180deg) tip
  pitch.p1 distance(design.base * normal_module, side: left) base
  pitch.p1 distance(design.dedendum * normal_module, side: left) tip
  radius(design.rounding * normal_module) first
  first equal second
}

preview {
  unit mm
  proportions := group(base: 2, dedendum: 1.25, rounding: 0.3)
  bi := point hint(x: -5, y: -4)
  bo := point hint(x: 10, y: -4)
  oj := point hint(x: 7, y: 1.5)
  ot := point hint(x: 6, y: 2.5)
  it := point hint(x: 0, y: 2.5)
  ij := point hint(x: -1, y: 1.5)
  pitch := line(std.origin, hint(x: 5, y: 0))
  std.origin distance(5mm, along: right) pitch.p2
  std.origin distance(0mm, along: up) pitch.p2
  profile := (base := line(bi, bo)) -> (outer := line(bo, oj)) ->
             tangent (outer_round := arc(center: hint(x: 6, y: 2))) -> tangent
             (tip := line(ot, it)) -> tangent
             (inner_round := arc(center: hint(x: 0, y: 2))) -> tangent
             (inner := line(ij, bi)) -> close
  base angle(110deg) outer
  base angle(250deg) inner
  std.origin on inner
  pitch.p2 on outer
  rounding := TipRounding(pitch, base, tip, outer_round, inner_round, proportions,
    normal_module: 2mm)
}
