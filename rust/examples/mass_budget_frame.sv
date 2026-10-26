// A frame sized by its mass budget (#510): an L bracket — a post built in at its foot and an arm
// off its head — of one square steel tube, 2mm in the wall, its outer side `s` an unknown, and
// `mass(1.5kg) frame` stated of the drawing.  The mass needs no support and no load: it is
// `ρ A (H + R)` over the frame's members, A = s² − (s − 4mm)², so the side the solve finds is
// (m / (ρ (H + R)) + 16mm²) / 8mm, 25.88mm.  The scenario beside it loads the shelf and claims how
// far the arm's end sinks, judged at the section the budget gave.
unit mm
use std
use std.place (horizontal)

H := 600mm
R := 400mm
w := 2mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param s: Length hint(30mm)

in std.front {
  foot := point
  corner := point
  end := point
  fix((0mm, 0mm)) foot
  fix((0mm, H)) corner
  fix((R, H)) end
  post := line(foot, corner)
  arm := line(corner, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
frame := beam(post, arm, A: s^2 - (s - 2 * w)^2, Iy: (s^4 - (s - 2 * w)^4) / 12,
              Iz: (s^4 - (s - 2 * w)^4) / 12, J: (s^4 - (s - 2 * w)^4) / 6) of steel
mass(1.5kg) frame

scenario shelf {
  fixed foot
  load arm 300N along: down
  claim deflection(end, along: down) <= 10mm
}
