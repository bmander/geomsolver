// A V-wedge heeled about its apex (#953): an inclined waterline.  The wedge's sides run 30° either
// side of the vertical from its apex, up to 100; the water is heeled 10° about the apex, its
// surface `T` along the heeled vertical from it.  In the section the sides meet the waterline at
// `tᵢ = T / cos(β ∓ φ)` from the apex, so the immersed triangle is `½ t₁ t₂ sin 2β` and the stated
// displacement finds `T`, 40mm; the centre of buoyancy, driven onto the immersed body's centroid,
// is the mean of the triangle's corners, off the centreline toward the side the wedge heels to —
// exactly, each face a plane.  Edit the heel's angle and the waterline turns about the apex.
unit mm
use std
use std.solids
use std.sets

L := 100mm
param T: Length hint(35mm)

in std.front {
  apex := point
  a := point
  b := point
  fix((0mm, 0mm)) apex
  fix((57.735mm, 100mm)) a
  fix((-57.735mm, 100mm)) b
  side_a := line(apex, a)
  top := line(a, b)
  side_b := line(b, apex)
  section := face(side_a, top, side_b)
}
hull := solid(std.solids.Prism(section, from: 0mm, to: L))

keel := point
fix((0mm, 0mm, 0mm)) keel
// the heeled vertical: the world's turned 10° about the y axis
heel := axis hint(dir: (0.17, 0, 0.98))
heel perpendicular std.y
std.z angle(10deg, about: std.y) heel
water := std.sets.HalfSpace(keel, heel, d: T)
immersed := solid({ p | p in hull; p in water })
volume(962.456 * 1mm^2 * L) immersed

buoyancy := point hint((0, -50, 20))
buoyancy centroid immersed
