// A square pyramid, the join of its apex and its base (§6.21, [0.212]): the points on a segment
// from the apex to some point of the base. The base is a `w` square drawn on the top plane about
// its middle, and the apex stands `h` over that middle; the join is read off the drawing as it
// solves, so edit `w` or `h` and the pyramid follows — its volume w² h / 3.
unit mm
use std
use std.sets

w := 20mm
h := 12mm

in std.top {
  m := point
  fix((0, 0)) m
  s0 := point hint((10, -10))
  s1 := point hint((10, 10))
  s2 := point hint((-10, 10))
  s3 := point hint((-10, -10))
  (e0 := line(s0, s1)) -> (e1 := line(s1, s2)) -> (e2 := line(s2, s3)) -> (e3 := line(s3, s0)) -> close
  vertical e0
  horizontal e1
  vertical e2
  horizontal e3
  e0 distance(w) e2
  e1 distance(w) e3
  m distance(w / 2) e0
  m distance(w / 2) e1
  base := face(e0, e1, e2, e3)
}
apex := point hint((0, 0, 10))
m distance(h, along: std.z) apex
m distance(0mm, along: std.x) apex
m distance(0mm, along: std.y) apex
pyramid := solid({ p | some q in base: p in std.sets.Segment(apex, q) })
