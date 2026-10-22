// Euler's column as a frame (#510 S0): the corpus's column (`column.sv`) drawn as the line its
// axis runs up, made a beam of its square section, built in at its foot and pressed on its free
// head by the load the solid's pressure comes to.  It buckles at P = π² E I / (4 L²), the
// cubic's to approximate; the section is square, so it buckles as readily either way and its
// first two factors are one, a repeated pair.
unit mm
use std

L := 800mm
b := 20mm
P := 10MPa * b^2
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

I := b^4 / 12
euler := pi^2 * steel.E * I / (4 * L^2)

in std.front {
  foot := point
  head := point
  fix((0mm, 0mm)) foot
  fix((0mm, L)) head
  post := line(foot, head)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
// a square's torsion constant, 0.141 b⁴
strut := beam(post, A: b^2, Iy: I, Iz: I, J: 0.141 * b^4) of steel

scenario press {
  fixed foot
  load head P along: down
  claim buckling(strut) >= 0.99 * euler / P
  claim buckling(strut) <= 1.01 * euler / P
}
