// A boss put on a body is shelled with it, hollow inside, the cavity running up into it.
//
// A boss is a union, and a union is not hollowed term by term: under the boss's foot the cavity
// reaches up from the slab into the boss, and within each edge of the foot — concave in the body
// — it is rounded by `wall` about the edge. So the shell is built by its walls: what lies within
// `wall` of what is outside the body, built as a body grown is about the body's faces turned
// inside out, each face's slab mitred at the convex edges and a quarter round about each concave
// one.
//
// Built exactly: 16500 less the slab's 38 × 38 × 8, the foot's 100 − 10π + 8/3 (the boss's square
// shrunk by the foot's quarter circle as it rises from the slab's inner top), and the boss's inner
// 8 × 8 × 4.
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

shell(t: wall) plinth
