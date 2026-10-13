// A post standing at the edge of a plate: a cylinder half outside the plate's near side, rising
// past its top. The plate's top edge along that side is rounded up to the post on either side of
// it. Each run ends on the plate's square side at one end and on the post's curved wall at the
// other: carried on into the post and cut off by its wall, so the round stops flush where the
// post rises, as the set says (`p inside crease.ends`: on the side of the wall the edge runs on).
//
// Drawn on the top plane, standing 10 (the plate) and 30 (the post) up from it. Orbit
// (right-drag) to see the rounds meet the post. Edit `r` and apply (⌘↵).
unit mm
use std (round)

r := 2mm   // the edge's radius

in std.top {
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((60, 0)) b
  fix((60, 40)) c
  fix((0, 40)) d
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o := point
  fix((30, 0)) o
  k := circle(center: o) hint(r: 8)
  radius(8mm) k
}
plate := solid(face(ab, bc, cd, da), from: 0mm, to: 10mm)
post := solid(face(k), from: 0mm, to: 30mm)
stand := solid(plate)
post union stand

// the plate's top along its near side, either side of the post
stand.plate.near round(r: r) stand.plate.ab
