// A sealed step: an L whose inner edge is concave. Within a concave edge the walls' inner faces
// do not meet square: every point more than `wall` from both faces is hollow, so the cavity is
// rounded there by an arc of radius `wall` about the edge. A wall moved in square would have left
// material in that corner further than `wall` from the outside.
//
// Built exactly: 3600 less 8 (160 − π), the L's section eroded (the two arms' 156, the corner's
// 4 − π) over its 8 between the caps.
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.front {
  l0 := point
  l1 := point
  l2 := point
  l3 := point
  l4 := point
  l5 := point
  fix((0, 0)) l0
  fix((20, 0)) l1
  fix((20, 10)) l2
  fix((10, 10)) l3
  fix((10, 20)) l4
  fix((0, 20)) l5
  (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (lt := line(l2, l3)) -> (lr := line(l3, l4)) ->
    (ln := line(l4, l5)) -> (lw := line(l5, l0)) -> close
  ell := face(ls, le, lt, lr, ln, lw)
}
step := solid(ell, depth: 12mm)

shell(t: wall) step
