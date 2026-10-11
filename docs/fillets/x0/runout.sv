// X0 (#183): solid_fillet_runout.sv, rounds that run out onto a face. A boss standing through a
// sloped top, off the wedge's near face, its root an arc of an ellipse ending obliquely at that
// face; and a hole through another wedge's slope, breaking out of its low end, its rim rounded
// off. Where the edge ends, the face beyond cuts the round off, however it stands. Nothing says
// so. Does not parse yet.
//
// The stack named an intermediate for the hole (`drilled`, then `vent`); here it is one body.
unit mm
use std
in std.front {
  // a wedge 40 long and 24 deep, its top sloping from 20 high to 10
  w0 := point
  w1 := point
  w2 := point
  w3 := point
  fix((0, 0)) w0
  fix((40, 0)) w1
  fix((40, 10)) w2
  fix((0, 20)) w3
  (wb := line(w0, w1)) -> (we := line(w1, w2)) -> (wt := line(w2, w3)) -> (ww := line(w3, w0)) -> close
  side_f := face(wb, we, wt, ww)

  // the same wedge 60 along
  v0 := point
  v1 := point
  v2 := point
  v3 := point
  fix((60, 0)) v0
  fix((100, 0)) v1
  fix((100, 10)) v2
  fix((60, 20)) v3
  (vb := line(v0, v1)) -> (ve := line(v1, v2)) -> (vt := line(v2, v3)) -> (vw := line(v3, v0)) -> close
  other_f := face(vb, ve, vt, vw)
}
in std.top {
  // a boss of radius 6 standing 3 in from the near face, so a third of it overhangs
  bc := point
  fix((20, -3)) bc
  boss_k := circle(center: bc) hint(r: 6)
  radius(6) boss_k

  // a hole of radius 4 through the second wedge, 2 past its low end
  hc := point
  fix((98, -12)) hc
  hole_k := circle(center: hc) hint(r: 4)
  radius(4) hole_k
}
wedge := solid(side_f, from: 0mm, to: 24mm)
boss := solid(face(boss_k), from: 0mm, to: 30mm)
lug := solid(wedge)
boss union lug
// Only against the slope: where the boss passes the wedge's near face it stays sharp.
lug.boss round(2mm) lug.wedge.wt

block := solid(other_f, from: 0mm, to: 24mm)
hole := solid(face(hole_k), from: -1mm, to: 30mm)
vent := solid(block)
hole cut vent
vent.block.vt round(1mm) vent.hole
