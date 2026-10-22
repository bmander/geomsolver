// A beam built in at both ends (#510 S0), loaded at its middle and then by its own weight: the
// frame's cases of a load where no member ends, and of a load spread along it.  The middle is a
// point drawn where the span's middle is, so the scenario cuts the member there.  Under the load
// it sinks P L³ / (192 E I) and bends most at its ends and its middle, P L / 8; under its weight
// q = ρ A g a length it sinks q L⁴ / (384 E I) and bends most at its ends, q L² / 12.  Both are
// a frame's exact answers; the claims bracket them as the corpus does, within 1% and 3%.
unit mm
use std

L := 2000mm
h := 100mm
b := 50mm
P := 5kN
g := 9.81m / 1s^2
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

I := b * h^3 / 12
Sz := b * h^2 / 6
q := steel.rho * b * h * g
sinks := P * L^3 / (192 * steel.E * I)
sags := q * L^4 / (384 * steel.E * I)

in std.front {
  a := point
  c := point
  mid := point
  fix((0mm, 0mm)) a
  fix((L, 0mm)) c
  fix((L / 2, 0mm)) mid
  span := line(a, c)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
girder := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: I, J: 0.229 * h * b^3, Sy: h * b^2 / 6,
               Sz: Sz) of steel

scenario midspan {
  fixed a
  fixed c
  load mid P along: down
  claim deflection(mid, along: down) >= 0.99 * sinks
  claim deflection(mid, along: down) <= 1.01 * sinks
  claim safety(girder) >= 0.97 * steel.yield * Sz * 8 / (P * L)
  claim safety(girder) <= 1.03 * steel.yield * Sz * 8 / (P * L)
}

scenario weight {
  fixed a
  fixed c
  accelerate girder (0m / 1s^2, 0m / 1s^2, -g)
  claim deflection(mid, along: down) >= 0.99 * sags
  claim deflection(mid, along: down) <= 1.01 * sags
  claim safety(girder) >= 0.97 * steel.yield * Sz * 12 / (q * L^2)
  claim safety(girder) <= 1.03 * steel.yield * Sz * 12 / (q * L^2)
}
