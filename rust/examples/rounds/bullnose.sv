// A boss on a bullnosed plate. The plate's top is a flat land that runs on smoothly into a
// bullnose, a quarter round of 10: where the two meet there is no edge, only a seam. A boss of 4
// stands up through the plate right on that seam, and its root is filled by one statement.
//
// The root's edge is two edges, one on each face: a half circle on the land, and a curve on the
// bullnose that no line or circle carries. On the land the round is a stretch of a ring. On the
// bullnose a ball is rolled along the traced curve, carried on past the seam, and cut off at the
// section where the ring's stretch ends, so the two pieces meet there with no step.
//
// The plate is drawn on the side plane and stands 40 wide across it. Orbit (right-drag) to see
// the root. Move the boss off the seam (fix `q` at (0, -1)) and the ball touches the seam in a
// different section than its edge crosses it, so no one section joins the two pieces: the export
// is refused, and the refusal says why.
unit mm
use std (round)

in std.side {
  a := point
  b := point
  c := point
  d := point
  o := point
  fix((-20, 0)) a
  fix((10, 0)) b
  fix((0, 10)) c
  fix((-20, 10)) d
  fix((0, 0)) o
  // the land is `cd`, the bullnose `bc`
  (ab := line(a, b)) -> (bc := arc(center: o) hint(r: 10)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
}
in std.top {
  q := point
  fix((0, 0)) q
  k := circle(center: q) hint(r: 4)
  radius(4mm) k
}
slab := solid(face(ab, bc, cd, da), from: -20mm, to: 20mm)
boss := solid(face(k), from: 0mm, to: 20mm)
part := solid(slab)
boss union part

// the root, across the seam
part.boss round(r: 1mm) part.slab
