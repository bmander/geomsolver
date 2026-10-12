// A plate with an obround boss standing on it and a pocket with rounded corners cut into it, each
// rounded where it meets the plate: rounds carried along chains of tangent edges. The boss's foot
// runs along its two sides and round its half-circle ends; the pocket's rim along its four sides
// and round its four corners. A chain is not a statement of its own: where two faces meet tangent
// there is no edge, so a run reaching such a seam ends there, and the next run carries on from it
// with no step between them.
//
// The foot is concave, so its round fills; the rim is convex, so its round cuts. Only where the
// pocket's walls meet the plate's near face is rounded: the pocket's floor meets only the pocket's
// own walls, and stays sharp. Orbit (right-drag) to see both chains. Edit a radius and apply
// (⌘↵); the surfaces refine in the background.
unit mm
use std (round)

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

  // the boss: an obround 30 long and 12 wide, its ends half circles about centres 18 apart
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
  (near_side := line(s0, s1)) -> (east := arc(center: o1) hint(r: 6)) -> (far_side := line(s2, s3)) ->
    (west := arc(center: o0) hint(r: 6)) -> close
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
  (bottom := line(q0, q1)) -> (corner1 := arc(center: k1) hint(r: 4)) -> (right := line(q2, q3)) ->
    (corner2 := arc(center: k2) hint(r: 4)) -> (upper := line(q4, q5)) ->
    (corner3 := arc(center: k3) hint(r: 4)) -> (left := line(q6, q7)) ->
    (corner0 := arc(center: k0) hint(r: 4)) -> close
  mouth := face(bottom, corner1, right, corner2, upper, corner3, left, corner0)
}
plate := solid(top, depth: 10mm)
boss := solid(outline, from: 0mm, to: 8mm)
pocket := solid(mouth, from: 1mm, to: -6mm)
part := solid(plate)
boss union part
pocket cut part

// the boss's foot: one chain of four edges, filled
part.boss round(r: 2mm) part.plate
// the pocket's rim: one chain of eight edges, cut
part.plate round(r: 1mm) part.pocket
