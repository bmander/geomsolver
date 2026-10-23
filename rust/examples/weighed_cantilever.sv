// A cantilever sized by its sag and its mass (#510): the library's sized cantilever
// (`sized_cantilever.sv`) with its width `b` an unknown beside its depth `h`, its tip's
// deflection driven in its scenario and its mass stated of the drawing, `mass(m) arm` — what the
// beam is made of, `ρ b h L`, answers no load and is read with no solve.  Two rows, two unknowns:
// `b h = m / (ρ L)` and Euler–Bernoulli's `b h³ = 4 P L³ / (E sag)`, so the depth is
// √((4 P L³ / (E sag)) / (b h)) — 62.65mm — and the width 20.33mm.  The claim beside them is
// judged at the section found.
unit mm
use std

L := 1000mm
P := 1kN
sag := 4mm
m := 10kg
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param b: Length hint(20mm)
param h: Length hint(40mm)

in std.front {
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  fix((L, 0mm)) end
  span := line(foot, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * h * b^3,
            Sy: h * b^2 / 6, Sz: b * h^2 / 6) of steel
mass(m) arm

scenario tip_load {
  fixed foot
  load end P along: down
  deflection(end, along: down) == sag
  claim safety(arm) >= 3
}
