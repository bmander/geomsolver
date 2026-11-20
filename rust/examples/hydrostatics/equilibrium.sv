// A box floating where it weighs the water it displaces (§6.25): a pine box 200 long, 60 in beam
// and 100 deep, kept within the water below a waterline `T` above its keel, and one equation of
// two integrals — the box's mass, `∫ ρ dV` over the box, against the water's density times the
// immersed volume — written as it is, with no word between: the solve finds `T` where the two
// agree, `ρ / ρ_w · 100mm`, 45mm for pine of 450 kg/m³.  The centre of buoyancy is a point the
// immersed body's volume is centred on, `std.measures`'s `centroid`, at `(L/2, 0, T/2)`.
unit mm
use std
use std.solids
use std.sets
use std.measures (centroid)

L := 200mm
B := 60mm
param T: Length hint(30mm)
pine := material(E: 10GPa, nu: 0.3, rho: 450kg / 1m^3)
rho_w := 1000kg / 1m^3

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
box := solid(std.solids.Prism(plan, from: 0mm, to: 100mm)) of pine

keel := point
fix((0mm, 0mm, 0mm)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({ p | p in box; p in water })

// what it weighs is what it displaces
integral(density(p) over p in box) == rho_w * integral(1 over p in immersed)

buoyancy := point hint((100, 0, 20))
buoyancy centroid immersed
