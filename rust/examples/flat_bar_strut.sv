// A flat bar's buckling (#510 S3): a steel flat bar stood on its edge, built in at its foot and
// pressed on its head.  It is drawn in the front, its width in that plane and its thickness swept
// across it, and it buckles across its thickness — out of the plane it is drawn in, where a
// reading in that plane has no move at all, so it is read as the solid it is.  Euler's load for a
// strut built in at one end and free at the other is π² E I / (4 L²), I = b t³ / 12 its weaker
// second moment, and the factor on the pressure at which it buckles is that over the load the
// pressure lays on its head.
unit mm
use std
use std.place (horizontal, vertical)

L := 600mm
b := 40mm
t := 6mm
p := 2MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
euler := pi^2 * steel.E * b * t^3 / 12 / (4 * L^2)

in std.front {
  a := point
  c := point hint((20, 0))
  d := point hint((20, 600))
  e := point hint((-20, 600))
  fix((-b / 2, 0mm)) a
  a horizontal c
  a distance(b) c
  c vertical d
  c distance(L) d
  d horizontal e
  e vertical a
  side := (foot := line(a, c)) -> (flank := line(c, d)) -> (head := line(d, e)) ->
          (back := line(e, a)) -> close
}
bar := solid(side, from: -t / 2, to: t / 2) of steel

scenario press {
  fixed bar.foot
  pressure bar.head p
  claim buckling(bar) >= 0.98 * euler / (p * b * t)
  claim buckling(bar) <= 1.02 * euler / (p * b * t)
}
