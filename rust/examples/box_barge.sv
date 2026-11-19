// A box barge floating (#953): a hull 200 long, 60 in beam and 100 deep, kept within the water
// below its waterline — `immersed := solid({ p | p in hull; p in water })`, the hull in common
// with the half-space up to `T` above its keel — and `T` an unknown its displacement finds:
// `volume(L · B · 40mm) immersed` holds the immersed volume, so `T` is 40mm.  The centre of
// buoyancy is a point the immersed body's volume is centred on, `b centroid immersed`, at
// `(L/2, 0, T/2)`; the waterplane is the immersed body's face `immersed.water`, its area `L B`
// and its second moment about the centreline drawn in it `L B³ / 12` — so its metacentric radius
// `BM = I / ∇ = B² / (12 T)`, 7.5mm, is `bm`, an unknown the second moment states.
unit mm
use std
use std.solids
use std.sets

L := 200mm
B := 60mm
param T: Length hint(30mm)
param bm: Length hint(5mm)

in std.top {
  a := point
  b := point
  c := point
  d := point
  fix((0mm, -B / 2)) a
  fix((L, -B / 2)) b
  fix((L, B / 2)) c
  fix((0mm, B / 2)) d
  side := line(a, b)
  stern := line(b, c)
  other := line(c, d)
  bow := line(d, a)
  plan := face(side, stern, other, bow)
}
hull := solid(std.solids.Prism(plan, from: 0mm, to: 100mm))

keel := point
fix((0mm, 0mm, 0mm)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({ p | p in hull; p in water })
volume(L * B * 40mm) immersed

// the centreline in the waterplane, `T` above the keel
fore := point hint((0, 0, 30))
aft := point hint((200, 0, 30))
fix(x == 0mm, y == 0mm) fore
fix(x == L, y == 0mm) aft
keel distance(T, along: std.z) fore
keel distance(T, along: std.z) aft
centreline := line(fore, aft)

buoyancy := point hint((100, 0, 20))
buoyancy centroid immersed
moment(bm * L * B * 40mm, about: centreline) immersed.water
claim area(L * B) immersed.water
