// A plate carrying two bosses, a square one and a round post, shelled sealed: each hollow inside,
// each foot rounded within by `wall` — a square boss's foot by a quarter round along each edge, the
// post's by one turned round its circle.
//
// Built exactly: 24600 + 200π less the plate's 58 × 38 × 8; under the square boss its foot's
// 100 − 10π + 8/3 and its inner 8 × 8 × 5; under the post its foot's π (25 + 2/3) − 5π²/2 (a
// disc shrunk by the quarter circle, turned) and its inner disc of radius 4, 7 tall.
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
  fix((60, 0)) q1
  fix((60, 40)) q2
  fix((0, 40)) q3
  (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close
  slab := face(qs, qe, qn, qw)
  m0 := point
  m1 := point
  m2 := point
  m3 := point
  fix((10, 15)) m0
  fix((20, 15)) m1
  fix((20, 25)) m2
  fix((10, 25)) m3
  (ms := line(m0, m1)) -> (me := line(m1, m2)) -> (mn := line(m2, m3)) -> (mw := line(m3, m0)) -> close
  stud := face(ms, me, mn, mw)
  n0 := point
  fix((45, 20)) n0
  pin := radius(5mm) circle(center: n0)
}
plate := solid(slab, from: 0mm, to: 10mm)
boss := solid(stud, from: 9mm, to: 16mm)
boss union plate
post := solid(face(pin), from: 9mm, to: 18mm)
post union plate

shell(t: wall) plate
