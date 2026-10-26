// A boss standing on a plate, filleted where it stands. `pad.boss round(r: …) pad.plate` rounds
// every edge where a face the boss gave the pad meets one the plate gave it: here the one circle
// where the boss's wall meets the plate's near face. That edge is concave, so the round fills it:
// a ring whose section is the corner a ball of radius `r` cannot reach, with no special case for a
// curved face.
//
// Edit `r` or the boss's radius and apply (⌘↵). Write `round(r: 1mm) pad.boss` beside it to
// round the boss's top rim too: an edge among the boss's own faces, convex, so cut.
unit mm
use std
use std.rounds (round)

r := 3mm   // the fillet's radius

in std.front {
  t0 := point
  t1 := point
  t2 := point
  t3 := point
  fix((0, 0)) t0
  fix((60, 0)) t1
  fix((60, 40)) t2
  fix((0, 40)) t3
  (ts := line(t0, t1)) -> (te := line(t1, t2)) -> (tn := line(t2, t3)) -> (tw := line(t3, t0)) -> close
  outline := face(ts, te, tn, tw)
  c := point
  fix((30, 20)) c
  rim := circle(center: c) hint(r: 8)
  radius(8mm) rim
}
plate := solid(outline, depth: 10mm)
boss := solid(face(rim), from: 0mm, to: 20mm)
pad := solid(plate)
boss union pad

pad.boss round(r: r) pad.plate
