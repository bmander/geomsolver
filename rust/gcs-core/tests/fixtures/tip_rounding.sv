// The crown section's tip rounding as it was drawn before the crown became a region (#214): two
// tangent arcs of one radius at a section's tip corners, the base and tip at their depths. Kept
// for the fillet checks that hold the rounds' words to it.
use std

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
    // the base and the tip at their depths, and the two rounds of one radius
    base parallel pitch
    base angle(180deg) tip
    pitch.p1 distance(proportions.base * 2mm, side: left) base
    pitch.p1 distance(proportions.dedendum * 2mm, side: left) tip
    radius(proportions.rounding * 2mm) outer_round
    outer_round equal inner_round
  }
}
