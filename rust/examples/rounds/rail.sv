// A rail: a rod half sunk along the top of a plate, filled into the plate on both sides. The rod
// meets the plate's top along two lines, and one statement rounds both: each a run the plate's
// whole length, ending flush where the rod and the plate end together, cut off by the end faces.
// The rod's wall is curved, so each run's section is the corner between a circle and a line that
// a ball of radius `r` cannot reach.
//
// The section is drawn on the front plane and stands 40 deep in front of it. Orbit (right-drag)
// to see the runs end flush. Edit `r` and apply (⌘↵), or name the rod and the plate's top alone
// (`rail.rod round(r: r) rail.plate.cd`): the same two runs, since the rod meets no other face of
// the plate's.
unit mm
use std
use std.rounds (round)

r := 2mm   // the fillets' radius

in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((60, 0)) b
  fix((60, 10)) c
  fix((0, 10)) d
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o := point
  fix((30, 10)) o
  k := circle(center: o) hint(r: 6)
  radius(6mm) k
}
plate := solid(face(ab, bc, cd, da), from: 0mm, to: 40mm)
rod := solid(face(k), from: 0mm, to: 40mm)
rail := solid(plate)
rod union rail

// both sides at once: every edge where a face the rod gave the rail meets one the plate gave it
rail.rod round(r: r) rail.plate
