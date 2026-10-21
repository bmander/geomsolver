// Euler's column, the corpus's buckling case (#510 P8): a square steel strut built in at its foot
// and pressed on its free head.  It buckles at P = π² E I / (4 L²), the cantilever's effective
// length twice its own, so the factor on the load is that over the load carried.  At L/b = 40 a
// solid's shear takes 0.04% off it.  The section is square, so the strut buckles as readily
// either way: the first two factors are one, a repeated pair.
unit mm
use std

L := 800mm
b := 20mm
p := 10MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

I := b^4 / 12
euler := pi^2 * steel.E * I / (4 * L^2)

in std.front {
  a := point
  c := point
  d := point
  e := point
  fix((-b / 2, 0mm)) a
  fix((b / 2, 0mm)) c
  fix((b / 2, L)) d
  fix((-b / 2, L)) e
  side := (foot := line(a, c)) -> (right := line(c, d)) -> (head := line(d, e)) ->
          (left := line(e, a)) -> close
}
strut := solid(side, from: -b / 2, to: b / 2) of steel

scenario press {
  fixed strut.foot
  pressure strut.head p
  claim buckling(strut) >= 0.99 * euler / (p * b^2)
  claim buckling(strut) <= 1.01 * euler / (p * b^2)
}
