// A boss left open at its top: a chimney. The face left open is the boss's own top,
// `plinth.boss.near`, so its walls run up through it; the slab under it stays sealed.
//
// An open face is first carried out `2 · wall` by a prism of itself glued on, and the walls are
// what lies within `wall` of what is outside that: so they run out through the opening and the
// cavity crosses the rim.
//
// Built exactly: as `plinth.sv`, with the boss's inner 8 × 8 run up through the top, 5 rather
// than 4.
unit mm
use std
use std.rounds (shell)

wall := 1mm

in std.top {
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((0, 0)) q0
  fix((40, 0)) q1
  fix((40, 40)) q2
  fix((0, 40)) q3
  (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close
  slab := face(qs, qe, qn, qw)
  m0 := point
  m1 := point
  m2 := point
  m3 := point
  fix((15, 15)) m0
  fix((25, 15)) m1
  fix((25, 25)) m2
  fix((15, 25)) m3
  (ms := line(m0, m1)) -> (me := line(m1, m2)) -> (mn := line(m2, m3)) -> (mw := line(m3, m0)) -> close
  stud := face(ms, me, mn, mw)
}
plinth := solid(slab, from: 0mm, to: 10mm)
boss := solid(stud, from: 9mm, to: 15mm)
boss union plinth

plinth.boss.near shell(t: wall) plinth
