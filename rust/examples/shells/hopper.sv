// Walls that meet their opening at a slant.
//
// The hopper's west wall leans in over its open top at 45°. A point just below the rim, past the
// wall's edge, is nearer the rim than the wall's plane, so it is hollow if more than `wall` from
// the rim: the cavity runs on round the corner on an arc of radius `wall` to the opening, and the
// wall ends in a rounded lip rather than a square one.
//
// The flared tray's walls lean out to its open top: every point near the rim is nearer its own
// wall's plane, and the walls' inner faces run straight up through the opening.
//
// Both built exactly: the hopper to its section's area less the cavity's (the walls moved in,
// clipped by the top, and the lip's 2 − π/2 below it) over its depth; the flared tray to its
// trapezoid less the eroded one between the floor and the rim.
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.front {
  h0 := point
  h1 := point
  h2 := point
  h3 := point
  h4 := point
  fix((0, 0)) h0
  fix((30, 0)) h1
  fix((30, 20)) h2
  fix((10, 20)) h3
  fix((0, 10)) h4
  (bottom := line(h0, h1)) -> (east := line(h1, h2)) -> (top := line(h2, h3)) -> (lean := line(h3, h4)) ->
    (west := line(h4, h0)) -> close
  section := face(bottom, east, top, lean, west)
  t0 := point
  t1 := point
  t2 := point
  t3 := point
  fix((45, 0)) t0
  fix((75, 0)) t1
  fix((80, 15)) t2
  fix((40, 15)) t3
  (floor := line(t0, t1)) -> (right := line(t1, t2)) -> (rim := line(t2, t3)) -> (left := line(t3, t0)) -> close
  trapezoid := face(floor, right, rim, left)
}
hopper := solid(section, depth: 12mm)
flared := solid(trapezoid, depth: 20mm)

hopper.top shell(t: wall) hopper
flared.rim shell(t: wall) flared
