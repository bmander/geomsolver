// What is not built exactly is read point by point — shown, meshed and measured like any solid —
// and an exact export (`--step`, `--stl`) refuses it, saying why.
//
// - The plinth is open at its slab's top, where its boss stands: a wall standing on an opening.
//   Within `wall` of the boss's side is kept, and below the opening that is a quarter round about
//   the boss's foot, hanging into the hollow; no block outside the body says it.
// - The L is shelled thicker than half its arms: its walls meet, and nothing is left between them
//   to hollow.
unit mm
use std
use std.solids (cut, union)
use std.rounds (shell)

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
in std.front {
  l0 := point
  l1 := point
  l2 := point
  l3 := point
  l4 := point
  l5 := point
  fix((0, 100)) l0
  fix((20, 100)) l1
  fix((20, 110)) l2
  fix((10, 110)) l3
  fix((10, 120)) l4
  fix((0, 120)) l5
  (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (lt := line(l2, l3)) -> (lr := line(l3, l4)) ->
    (ln := line(l4, l5)) -> (lw := line(l5, l0)) -> close
  ell := face(ls, le, lt, lr, ln, lw)
}
plinth := solid(std.solids.Prism(slab, from: 0mm, to: 10mm))
boss := solid(std.solids.Prism(stud, from: 9mm, to: 15mm))
boss union plinth
step := solid(std.solids.Prism(ell, from: -30mm, to: 0))

plinth.near shell(t: 1mm) plinth
shell(t: 6mm) step
