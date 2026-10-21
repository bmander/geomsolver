// NAFEMS LE1, the elliptic membrane (#510 P8): a quarter of a plate between two ellipses, pulled
// outward square to its outer edge at 10 MPa, in plane stress.  The benchmark's target is the
// hoop stress σyy at D, the inner ellipse's end on the major axis, 92.7 MPa.  Nothing else acts
// at D, a free edge whose normal is x, so the von Mises stress there is the same, and it is the
// membrane's greatest: the corpus states the target as the least factor of safety against a
// yield the benchmark does not give, 250 MPa.  Each quarter ellipse is drawn exactly, a rational
// cubic (`../nurbs.sv`'s quarter circle stretched, which a rational curve survives unchanged).
unit mm
use std

thickness := 100mm
pull := 10MPa
le1 := material(E: 210GPa, nu: 0.3, yield: 250MPa)
target := 92.7MPa

// the control points stand where a quarter circle's do, stretched to each ellipse
inner := 2 - sqrt(2)
heavy := (1 + sqrt(2)) / 3

in std.front {
  d := point
  d1 := point
  d2 := point
  a := point
  fix((2000mm, 0mm)) d
  fix((2000mm, inner * 1000mm)) d1
  fix((inner * 2000mm, 1000mm)) d2
  fix((0mm, 1000mm)) a
  c := point
  c1 := point
  c2 := point
  b := point
  fix((3250mm, 0mm)) c
  fix((3250mm, inner * 2750mm)) c1
  fix((inner * 3250mm, 2750mm)) c2
  fix((0mm, 2750mm)) b
  hole := spline(d, d1, d2, a) weights [1, heavy, heavy, 1]
  outer := spline(c, c1, c2, b) weights [1, heavy, heavy, 1]
  dc := line(d, c)
  ab := line(b, a)
  quarter := face(dc, outer, ab, hole)
}
membrane := solid(quarter, from: -thickness / 2, to: thickness / 2) of le1

scenario le1 {
  pinned membrane.dc along: std.z
  pinned membrane.ab along: std.x
  pinned d along: std.y
  pressure membrane.outer -pull
  claim safety(membrane) >= 0.97 * le1.yield / target
  claim safety(membrane) <= 1.03 * le1.yield / target
}
