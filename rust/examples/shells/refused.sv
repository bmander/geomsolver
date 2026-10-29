// What is not built exactly is read point by point — shown, meshed and measured like any solid —
// and an exact export (`--step`, `--stl`) refuses it, saying why.
//
// - The plinth is open at its slab's top, where its boss stands: a wall standing on an opening.
//   Within `wall` of the boss's side is kept, and below the opening that is a quarter round about
//   the boss's foot, hanging into the hollow; no block outside the body says it.
// - The L is shelled thicker than half its arms: its walls meet, and nothing is left between them
//   to hollow.
// - The tray leaves its pocket's faces open: an opening is one of the stock's faces, never a
//   cut's.
unit mm
use std
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
  r0 := point
  r1 := point
  r2 := point
  r3 := point
  fix((60, 0)) r0
  fix((100, 0)) r1
  fix((100, 40)) r2
  fix((60, 40)) r3
  (rs := line(r0, r1)) -> (re := line(r1, r2)) -> (rn := line(r2, r3)) -> (rw := line(r3, r0)) -> close
  dish := face(rs, re, rn, rw)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((70, 12)) k0
  fix((90, 12)) k1
  fix((90, 28)) k2
  fix((70, 28)) k3
  (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close
  opening := face(ks, ke, kn, kw)
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
plinth := solid(slab, from: 0mm, to: 10mm)
boss := solid(stud, from: 9mm, to: 15mm)
boss union plinth
step := solid(ell, depth: 30mm)
tray := solid(dish, from: 0mm, to: 10mm)
pocket := solid(opening, from: 6mm, to: 11mm)
pocket cut tray

plinth.near shell(t: 1mm) plinth
shell(t: 6mm) step
tray.pocket shell(t: 1mm) tray
