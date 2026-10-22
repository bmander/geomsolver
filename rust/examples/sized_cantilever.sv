// A cantilever sized by how far it sags (#510 S1): the corpus's cantilever frame
// (`analysis/cantilever_frame.sv`), its depth `h` an unknown and its tip's deflection a driving
// requirement.  The solve finds the depth at which the tip sinks `sag` under `P`: the beam's
// deflection is a row of the drawing, its value the frame's solve and its rate in `h` the
// adjoint's.  Euler–Bernoulli's `P L³ / (3 E I)` with `I = b h³ / 12` says the depth is
// `(4 P L³ / (E b sag))^(1/3)`, 58.48mm; the claim beside it is judged at that depth.
unit mm
use std

L := 1000mm
b := 25mm
P := 1kN
sag := 4mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
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

scenario tip_load {
  fixed foot
  load end P along: down
  deflection(end, along: down) == sag
  claim safety(arm) >= 3
}
