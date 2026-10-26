// Rounds that run out onto a face. Two wedges, each 40 long and 24 deep, their tops sloping from
// 20 high down to 10:
// - `lug`: a boss standing up through the first wedge's slope, its centre 3 in from the near
//   face, so a fifth of it overhangs. Its root on the slope is filled: an arc of an ellipse
//   ending where the near face cuts it, at a slant.
// - `vent`: a hole drilled down through the second wedge, 2 short of its low end, so it breaks out
//   of the end face. Its rim on the slope is rounded off, and ends where the end face cuts it.
//
// Where a run ends on a face that is not rounded, the run carries on past its edge's end and that
// face cuts it off, however the face stands; nothing in the statements says so. Only the slope is
// named, so where the boss passes the near face, and where the hole passes the end face, the edges
// stay sharp. Orbit (right-drag) to see the run-outs. Move the boss onto the near face (fix `bc`
// at (20, 0)) and the face cuts its root in half; move the hole in (fix `hc` at (90, -12)) and its
// rim runs all the way round.
unit mm
use std
use std.rounds (round)

in std.front {
  // the first wedge, its top sloping from 20 high to 10
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
  // the boss, 6 in radius, its centre 3 in from the first wedge's near face
  bc := point
  fix((20, -3)) bc
  boss_k := circle(center: bc) hint(r: 6)
  radius(6mm) boss_k

  // the hole, 4 in radius, its centre 2 short of the second wedge's low end
  hc := point
  fix((98, -12)) hc
  hole_k := circle(center: hc) hint(r: 4)
  radius(4mm) hole_k
}
wedge := solid(side_f, from: 0mm, to: 24mm)
boss := solid(face(boss_k), from: 0mm, to: 30mm)
lug := solid(wedge)
boss union lug
// the root, against the slope alone
lug.boss round(r: 2mm) lug.wedge.wt

block := solid(other_f, from: 0mm, to: 24mm)
hole := solid(face(hole_k), from: -1mm, to: 30mm)
vent := solid(block)
hole cut vent
// the rim, on the slope alone
vent.block.vt round(r: 1mm) vent.hole
