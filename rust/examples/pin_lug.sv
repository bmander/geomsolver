// A pin lug (#510 S3): a steel lug built in at its root, a pin hole cut through its far end and a
// pull on the hole's wall — a body cut by another solid, so no prism, read by refining its exact
// boundary and filling it.  The hole is a face of the cutter, named from the body it cuts,
// `lug.pin.bore`: a line about a part of a body is about the body.  The pull is spread evenly over
// the wall, a pin bearing on all of it; the net section beside the hole carries it at P / ((w − d)
// t) on average and more at the hole's edge, and the lug claims its factor of safety over that
// average by more than its stress concentration.
unit mm
use std

l := 80mm
w := 40mm
t := 10mm
d := 16mm
P := 20kN
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
net := P / ((w - d) * t)

in std.front {
  a := point
  b := point
  c := point
  e := point
  centre := point
  fix((0mm, 0mm)) a
  fix((l, 0mm)) b
  fix((l, w)) c
  fix((0mm, w)) e
  fix((l - w / 2, w / 2)) centre
  outline := (low := line(a, b)) -> (end := line(b, c)) -> (high := line(c, e)) ->
             (root := line(e, a)) -> close
  bore := radius(d / 2) circle(center: centre) hint(r: 8)
  hole := face(bore)
}
lug := solid(outline, from: -t / 2, to: t / 2) of steel
pin := solid(hole, from: -t, to: t)
pin cut lug

scenario pull {
  fixed lug.root
  load lug.pin.bore P along: std.x
  claim deflection(b, along: std.x) >= 0mm
  claim safety(lug) >= steel.yield / (4 * net)
  claim safety(lug) <= steel.yield / net
}
