// A frustum standing on a disc, both turned about one axis (issue #66, rung 1): the ball rolled
// round the frustum's foot fills a ring whose face is a torus, and its top rim is rounded off.
unit mm
use std
in std.front {
  a0 := point
  a1 := point hint(x: 0, y: 40)
  fix(x == 0, y == 0) a0
  fix(x == 0, y == 40) a1
  spindle := line(a0, a1)
  p0 := point
  p1 := point hint(x: 40, y: 0)
  p2 := point hint(x: 40, y: 10)
  p3 := point hint(x: 0, y: 10)
  fix(x == 0, y == 0) p0
  fix(x == 40, y == 0) p1
  fix(x == 40, y == 10) p2
  fix(x == 0, y == 10) p3
  (pbot := line(p0, p1)) -> (pside := line(p1, p2)) -> (ptop := line(p2, p3)) -> (paxis := line(p3, p0)) -> close
  pf := face(pbot, pside, ptop, paxis)
  q0 := point hint(x: 0, y: 10)
  q1 := point hint(x: 15, y: 10)
  q2 := point hint(x: 5, y: 30)
  q3 := point hint(x: 0, y: 30)
  fix(x == 0, y == 10) q0
  fix(x == 15, y == 10) q1
  fix(x == 5, y == 30) q2
  fix(x == 0, y == 30) q3
  (cbot := line(q0, q1)) -> (slant := line(q1, q2)) -> (ctop := line(q2, q3)) -> (caxis := line(q3, q0)) -> close
  cf := face(cbot, slant, ctop, caxis)
  plate := solid(pf, about: spindle)
  frustum := solid(cf, about: spindle)
  body := solid(plate)
  frustum union body
  root := fillet(frustum, plate, r: 3mm)
  root union body
  rim := fillet(frustum.ctop, frustum.slant, r: 1mm)
  rim cut body
}
