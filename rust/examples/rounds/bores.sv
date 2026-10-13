// A slab drilled twice, the bores overlapping: a disc 40 across and 10 thick, a bore of 10 and one
// of 8 whose centres stand 14 apart. Where their walls meet, a cusp runs through the slab. Rounded
// all over: the rims round the disc and round each bore are rings, the cusps are runs between two
// cylinders, and where a cusp reaches a face of the slab, three convex edges meet — the cusp and
// both bores' rims — so the ball touching the face and both walls rounds the corner, its cell
// over a plane and two cylinders, glued to the cusp's run and each rim's stretch there.
//
// Drawn on the front plane, standing 10 deep in front of it. Orbit (right-drag) to see where the
// cusps meet the faces. Edit a radius and apply (⌘↵).
unit mm
use std (round)

r := 1mm   // every edge's radius

in std.front {
  o := point
  fix((5, 0)) o
  rim := circle(center: o) hint(r: 20)
  radius(20mm) rim
  c1 := point
  c2 := point
  fix((0, 0)) c1
  fix((14, 0)) c2
  big := circle(center: c1) hint(r: 10)
  small := circle(center: c2) hint(r: 8)
  radius(10mm) big
  radius(8mm) small
}
disc := solid(face(rim), from: 0mm, to: 10mm)
left := solid(face(big), from: 0mm, to: 10mm)
right := solid(face(small), from: 0mm, to: 10mm)
slab := solid(disc)
left cut slab
right cut slab

round(r: r) slab
