// A box joined to a point above it is a house — its walls and its roof in one solid, the hull of
// the box's corners and the peak (§6.21, [0.212]): a solid of planes meeting in convex edges is
// the hull of its corners, so it joins as a point set does. Beside it the same box rounded, the
// points a ball of 2 about some point of the box reaches: the box grown, built exactly. Move the
// peak, or edit the box, and both follow.
unit mm
use std
use std.sets
use std.solids

in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0, 0)) k0
  fix((20, 0)) k1
  fix((20, 12)) k2
  fix((0, 12)) k3
  (f0 := line(k0, k1)) -> (f1 := line(k1, k2)) -> (f2 := line(k2, k3)) -> (f3 := line(k3, k0)) -> close
  plate := face(f0, f1, f2, f3)
}
construction box := solid(std.solids.Prism(plate, from: 0mm, to: 6mm))
peak := point
fix((10, 6, 14)) peak
house := solid({ p | some q in box: p in std.sets.Segment(peak, q) })

in std.top {
  m0 := point
  m1 := point
  m2 := point
  m3 := point
  fix((40, 0)) m0
  fix((60, 0)) m1
  fix((60, 12)) m2
  fix((40, 12)) m3
  (g0 := line(m0, m1)) -> (g1 := line(m1, m2)) -> (g2 := line(m2, m3)) -> (g3 := line(m3, m0)) -> close
  block := face(g0, g1, g2, g3)
}
construction slab := solid(std.solids.Prism(block, from: 0mm, to: 6mm))
rounded := solid({ p | some q in slab: p distance(<= 2mm) q })
