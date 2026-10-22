// How far a cantilever may reach (#510 S1): the library's sized cantilever turned round — its
// section stated, and its length the unknown.  The tip stands where the line's dimension `reach`
// puts it, and its deflection is a driving requirement, so the solve moves the tip out until it
// sinks `sag` under `P`: the deflection's row reads the tip's place as a column, and the line's
// length is the unknown that dimension and that row share.  Euler–Bernoulli's `P L³ / (3 E I)`
// says the reach is `(3 E I sag / P)^(1/3)`, 855.0mm.
unit mm
use std (horizontal)

b := 25mm
h := 50mm
P := 1kN
sag := 4mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param reach: Length hint(800mm)

in std.front {
  foot := point
  tip := point hint((800, 0))
  fix((0mm, 0mm)) foot
  foot horizontal tip
  foot distance(reach) tip
  span := line(foot, tip)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * h * b^3,
            Sy: h * b^2 / 6, Sz: b * h^2 / 6) of steel

scenario tip_load {
  fixed foot
  load tip P along: down
  deflection(tip, along: down) == sag
  claim safety(arm) >= 2
}
