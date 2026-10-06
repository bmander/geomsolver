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
  proportions := {base: 2, dedendum: 1.25, rounding: 0.3}
  in std.front {
    bi := point hint((-5, -4))
    bo := point hint((10, -4))
    oj := point hint((7, 1.5))
    ot := point hint((6, 2.5))
    it := point hint((0, 2.5))
    ij := point hint((-1, 1.5))
    pitch := line(std.origin)
    fix((5, 0)) pitch.p2
    profile := (base := line(bi, bo)) -> (outer := line(bo, oj)) ->
               tangent (outer_round := arc(center: hint((6, 2)))) -> tangent
               (tip := line(ot, it)) -> tangent
               (inner_round := arc(center: hint((0, 2)))) -> tangent
               (inner := line(ij, bi)) -> close
    base angle(110deg) outer
    base angle(250deg) inner
    std.origin coincident inner
    pitch.p2 coincident outer
    rounding := TipRounding(pitch, base, tip, outer_round, inner_round, proportions,
      normal_module: 2mm)
  }
}
