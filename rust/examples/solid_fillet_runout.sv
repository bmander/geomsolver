// Fillets that run out onto a face (issue #66, rung 3): where the meeting of the two faces a fillet
// rounds is cut short by a third, the ball rolls on past it and the third face cuts the fillet off
// there, however it stands to the edge. A boss standing through a sloped top, off the wedge's near
// face, its root an arc of an ellipse ending obliquely at that face; and a hole drilled through
// another wedge's slope, breaking out of its low end, its rim rounded off.
unit mm
use std
in std.front {
  // a wedge 40 long and 24 deep, its top sloping from 20 high to 10
  w0 := point
  w1 := point hint(x: 40, y: 0)
  w2 := point hint(x: 40, y: 10)
  w3 := point hint(x: 0, y: 20)
  fix(x == 0, y == 0) w0
  fix(x == 40, y == 0) w1
  fix(x == 40, y == 10) w2
  fix(x == 0, y == 20) w3
  (wb := line(w0, w1)) -> (we := line(w1, w2)) -> (wt := line(w2, w3)) -> (ww := line(w3, w0)) -> close
  side_f := face(wb, we, wt, ww)

  // the same wedge 60 along
  v0 := point hint(x: 60, y: 0)
  v1 := point hint(x: 100, y: 0)
  v2 := point hint(x: 100, y: 10)
  v3 := point hint(x: 60, y: 20)
  fix(x == 60, y == 0) v0
  fix(x == 100, y == 0) v1
  fix(x == 100, y == 10) v2
  fix(x == 60, y == 20) v3
  (vb := line(v0, v1)) -> (ve := line(v1, v2)) -> (vt := line(v2, v3)) -> (vw := line(v3, v0)) -> close
  other_f := face(vb, ve, vt, vw)
}
in std.top {
  // a boss of radius 6 standing 3 in from the near face, so a third of it overhangs
  bc := point hint(x: 20, y: -3)
  fix(x == 20, y == -3) bc
  boss_k := circle(center: bc) hint(r: 6)
  radius(6) boss_k

  // a hole of radius 4 through the second wedge, 2 past its low end
  hc := point hint(x: 98, y: -12)
  fix(x == 98, y == -12) hc
  hole_k := circle(center: hc) hint(r: 4)
  radius(4) hole_k
}
wedge := solid(side_f, from: 0mm, to: 24mm)
boss := solid(face(boss_k), from: 0mm, to: 30mm)
lug := solid(wedge)
boss union lug
root := fillet(boss, wedge.wt, r: 2mm)
root union lug

block := solid(other_f, from: 0mm, to: 24mm)
hole := solid(face(hole_k), from: -1mm, to: 30mm)
drilled := solid(block)
hole cut drilled
vent := solid(drilled)
rim := fillet(drilled.block.vt, drilled.hole, r: 1mm)
rim cut vent
