// A four-bar linkage driven round, and the curve its coupler draws.
//
// Four rods pinned end to end, one of them the ground `ad`: the crank `ab` turns about `a`, the
// rocker `dc` swings to and fro about `d`, and the coupler `bc` between them both turns and
// slides, so a point on it draws a curve that is neither a circle nor a line: the coupler curve.
// The crank is the shortest rod and the shortest and longest together (10 + 40) are shorter than
// the other two (35 + 25) — Grashof's condition — so the crank turns all the way round.
//
// `t` is the crank's angle, declared as a number that runs a whole turn.  The one relation that
// reads it, `ground angle(t) crank`, holds for every value of `t`, and so does every other
// relation here: the drawing is a function of `t`.  It is drawn at the start, `t = 0`, and
// followed from there round the turn, carrying the one assembly mode the seed of `c` picks (the
// coupler above the ground; its mirror below is the other).  `coupler` is the coupler's midpoint
// as `t` runs, closed on itself after a turn.
//
// Diagnosed: DOF 0, and `1 DOF over t` — the crank, which is how a mechanism is counted.
// `solventc` says what drives the motion and how far it runs: `t: followed from 0deg to 360deg`.

unit mm
use std

g := 40      // the ground, a to d
r := 10      // the crank
l := 35      // the coupler
k := 25      // the rocker

param t: Angle over (0deg, 360deg)

in std.front {
  a := point
  d := point
  b := point hint((10, 0))
  c := point hint((40, 25))      // above the ground: the assembly mode carried
  fix((0, 0)) a
  fix((g, 0)) d

  ground := line(a, d)
  crank := line(a, b)
  coupler_bar := line(b, c)
  rocker := line(d, c)
  length(r) crank
  length(l) coupler_bar
  length(k) rocker
  ground angle(t) crank          // the driver: held for every t

  m := point
  m midpoint coupler_bar
}

// where the coupler's midpoint goes as the crank turns round
coupler := trace(m over t)
