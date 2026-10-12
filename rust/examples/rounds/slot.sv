// A plate with an obround boss standing on it and a pocket with rounded corners cut into it,
// each rounded along a chain of edges that meet tangent: the boss's foot along its flat sides and
// round its half-circle ends, the pocket's rim along its walls and round its corners. A chain is
// not a statement of its own. Where two faces meet tangent there is no corner, so nothing ends a
// round there: the run along a flat side and the ring round the half circle beside it each stop at
// the section they share. Built exactly: each run a prism whose face is a cylinder, each stretch
// of a ring a torus's.
//
// Edit `r_foot`, `r_rim` or the obround's length and apply (⌘↵).
unit mm
use std (round)

r_foot := 2mm   // the boss's foot, filled
r_rim := 1mm    // the pocket's rim, taken away

in std.front {
  // the plate, 80 × 50
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((0, 0)) p0
  fix((80, 0)) p1
  fix((80, 50)) p2
  fix((0, 50)) p3
  (ab := line(p0, p1)) -> (bc := line(p1, p2)) -> (cd := line(p2, p3)) -> (da := line(p3, p0)) -> close
  top := face(ab, bc, cd, da)

  // the boss: an obround 12 wide, its ends half circles about centres 18 apart
  o0 := point
  o1 := point
  fix((11, 25)) o0
  fix((29, 25)) o1
  s0 := point
  s1 := point
  s2 := point
  s3 := point
  fix((11, 19)) s0
  fix((29, 19)) s1
  fix((29, 31)) s2
  fix((11, 31)) s3
  (near_side := line(s0, s1)) -> (east := arc(center: o1, start: s1, end: s2) hint(r: 6)) ->
    (far_side := line(s2, s3)) -> (west := arc(center: o0, start: s3, end: s0) hint(r: 6)) -> close
  outline := face(near_side, east, far_side, west)

  // the pocket: 20 × 16, its corners rounded to 4 in the sketch
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((52, 21)) k0
  fix((64, 21)) k1
  fix((64, 29)) k2
  fix((52, 29)) k3
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  q4 := point
  q5 := point
  q6 := point
  q7 := point
  fix((52, 17)) q0
  fix((64, 17)) q1
  fix((68, 21)) q2
  fix((68, 29)) q3
  fix((64, 33)) q4
  fix((52, 33)) q5
  fix((48, 29)) q6
  fix((48, 21)) q7
  (bottom := line(q0, q1)) -> (corner1 := arc(center: k1, start: q1, end: q2) hint(r: 4)) ->
    (right := line(q2, q3)) -> (corner2 := arc(center: k2, start: q3, end: q4) hint(r: 4)) ->
    (upper := line(q4, q5)) -> (corner3 := arc(center: k3, start: q5, end: q6) hint(r: 4)) ->
    (left := line(q6, q7)) -> (corner0 := arc(center: k0, start: q7, end: q0) hint(r: 4)) -> close
  mouth := face(bottom, corner1, right, corner2, upper, corner3, left, corner0)
}
plate := solid(top, depth: 10mm)
boss := solid(outline, from: 0mm, to: 8mm)
pocket := solid(mouth, from: 1mm, to: -6mm)
part := solid(plate)
boss union part
pocket cut part

// The boss's foot: concave, so filled, along four edges.
part.boss round(r: r_foot) part.plate
// The pocket's rim: convex, so taken away, along eight. Only where the pocket's walls meet the
// plate's near face: its floor's edges are the pocket's against the pocket's, and stay sharp.
part.plate round(r: r_rim) part.pocket
