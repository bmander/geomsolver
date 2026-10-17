// A four-bar at its change point, where two assembly modes cross.
//
// The ground 40, the crank 10, the coupler 30 and the rocker 20: the shortest and the longest
// together are exactly the other two (10 + 40 = 30 + 20), the boundary of Grashof's condition.
// At `t = 180deg` every rod lies on the ground line, the linkage folds flat, and the coupler can
// leave the fold above the ground or below it: the two assembly modes meet there and cross.
// Nothing the drawing states chooses between them.
//
// Followed from `t = 0`, the march stops at the fold, `t = 180deg`, and says so: `two of its
// branches cross there`.  Make the rocker a hair longer (20.0001) and the modes no longer meet: the
// march goes through the near-fold on the branch it started on, round the whole turn.

unit mm
use std

g := 40      // the ground
r := 10      // the crank
l := 30      // the coupler
k := 20      // the rocker

param t: Angle over (0deg, 360deg)

in std.front {
  a := point
  d := point
  b := point hint((r, 0))
  c := point hint((33, 19))      // above the ground: the assembly mode carried
  fix((0, 0)) a
  fix((g, 0)) d

  ground := line(a, d)
  crank := line(a, b)
  coupler_bar := line(b, c)
  rocker := line(d, c)
  length(r) crank
  length(l) coupler_bar
  length(k) rocker
  ground angle(t) crank          // the driver
}

path := trace(c over t)
