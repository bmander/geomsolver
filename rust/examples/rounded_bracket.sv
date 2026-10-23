// A rounded shelf bracket (#510 S2): an L of steel plate, its post built in along its foot and a
// load hung along the end of its arm, read in plane stress.  The inside corner is rounded by the
// arc `round`, so its factor of safety has an answer — the yield over the greatest von Mises
// stress, which the round's curve concentrates.  Draw the corner sharp (or cut straight across)
// and the elastic stress there is unbounded: `safety(bracket)` then has no answer, and says where
// and why — `… where the material's angle is 225°: it grows as r^-0.326 toward the edge; round
// it` — while the deflection and the mass still answer.
unit mm
use std

H := 120mm
reach := 100mm
w := 20mm
r := 8mm
t := 10mm
P := 1kN
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
// the post, the arm, and the round's fillet of material: a square of `r` less its quarter circle
area := H * w + (reach - w) * w + r^2 * (1 - pi / 4)

in std.front {
  a := point
  b := point
  c := point
  d := point
  e := point
  f := point
  g := point
  o := point
  fix((0mm, 0mm)) a
  fix((w, 0mm)) b
  fix((w, H - w - r)) c
  fix((w + r, H - w - r)) o
  fix((w + r, H - w)) d
  fix((reach, H - w)) e
  fix((reach, H)) f
  fix((0mm, H)) g
  base := line(a, b)
  inner := line(b, c)
  round := arc(center: o, start: d, end: c) hint(r: r)
  under := line(d, e)
  tip := line(e, f)
  top := line(f, g)
  back := line(g, a)
  section := face(base, inner, round, under, tip, top, back)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
bracket := solid(section, from: -t / 2, to: t / 2) of steel

// what it is made of answers no load, so it is the drawing's, claimed of it
claim mass(>= 0.999 * steel.rho * t * area) bracket
claim mass(<= 1.001 * steel.rho * t * area) bracket

scenario shelf {
  plane stress bracket
  fixed bracket.base
  load bracket.tip P along: down
  claim deflection(f, along: down) <= 1mm
  claim safety(bracket) >= 1
}
