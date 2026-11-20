// A deck plate with a step along its lower edge, its top rim rounded 2 (#962). From the left the
// lower edge runs along y = -59.25 into a concave arc of 5, which turns down into a convex lobe of
// 12 about (-43, -55); the lobe runs up into a straight side 13.75 long, and the side into a
// concave arc of 8 that turns along y = -33.25 to the right. Every one of those four seams is
// tangent: but to the drawing's own numbers, as an author states them, five places after the
// point, so the lobe meets the arc of 5 half a millionth of a radian off its tangent, and the side
// leaves the lobe and meets the arc of 8 a ten-millionth off theirs.
//
// The body reads each as a seam (its two faces' normals one there to a millionth), so it is no end:
// the round goes on across it, a run along the side and a stretch of a ring round each arc, each
// glued to the next in the one section they share. Glued at what the two sections stand apart, a
// millionth of a millimetre or so, never unioned: a union of two caps that far apart could not
// tell what touched what, and the round was refused.
//
// The plate stands from 39 to 69 on the top plane. Edit the round's radius and apply (⌘↵).
unit mm
use std
use std.solids
use std.rounds (round)

in std.top {
  a := point
  fix((-125, -59.25)) a
  // the concave arc of 5, its centre under where the lower edge runs into it
  v9 := point
  fix((-57.26315, -59.25)) v9
  c9 := point
  fix((-57.26315, -64.25)) c9
  // where it meets the lobe
  v10 := point hint((-53.06811, -61.52941))
  fix(y == -61.52941mm) v10
  // the lobe, and where it runs into the side
  c10 := point
  fix((-43, -55)) c10
  v11 := point hint((-31, -55))
  fix(y == -55mm) v11
  // the side's top, and the concave arc of 8 beside it
  v12 := point
  fix((-31, -41.25)) v12
  c12 := point
  fix((-23, -41.25)) c12
  v13 := point hint((-23, -33.25))
  fix(x == -23mm) v13
  e := point
  fix((105, -33.25)) e
  f := point
  fix((105, 72)) f
  g := point
  fix((-125, 72)) g
  lower := line(a, v9)
  fillet := arc(center: c9, start: v9, end: v10, sense: cw)
  lobe := arc(center: c10, start: v10, end: v11)
  side := line(v11, v12)
  root := arc(center: c12, start: v12, end: v13, sense: cw)
  step_edge := line(v13, e)
  east := line(e, f)
  north := line(f, g)
  west := line(g, a)
  frame := face(lower, fillet, lobe, side, root, step_edge, east, north, west)
}
deck := solid(std.solids.Prism(frame, from: 39mm, to: 69mm))
deck.near round(r: 2mm) deck
