// A cam sized against its curvature: the base circle is not stated but solved, so that the cam's
// tightest bend has the radius a designer states.
//
// A cam is ground to its profile, and a flat-faced follower presses on it along a line: where the
// cam bends tightly the contact stress is high, and where it would bend the other way no flat
// face can cut it at all.  So a designer states the least radius the cam may have, and asks how
// big its base circle must be.  For a flat face the answer runs through the law alone.  The cam
// bends with radius `base + s + s''` (`flat_cam.sv`), so its tightest bend is where `s + s''` is
// least, a property of the law, and the base circle just adds to it.
//
// The follower dwells, rises `h` on the cycloidal law over a third of a turn, dwells and returns
// (`dwell_cam.sv`'s law).  A fraction `x` through the rise, `s + s''` is least where its own rate
// is zero, `cos(360deg x) = -1/8` (the rise over 120°: `s''` a radian is `9 / 2π` times `h
// sin(360deg x)`), late in the rise, where the follower is slowing hardest:
//
//   t = 60deg + 120deg (1 - acos(-1/8) / 360deg) = 180deg - acos(-1/8) / 3 = 147.61deg
//
// A circle of the stated radius is held osculating the rise there, and the base circle `base`,
// an unknown, is what the solve answers: 15 + 0.5332·h = 21.398.  Edit `least`, `h` or the
// law's stations (and the instant with them), and the base is solved again.

use std
use std.place (offset)
use std.motion (spins, slides)

h := 12           // the follower's lift
least := 15       // the least radius the cam may bend at
param base: Length hint(20)   // the face's nearest approach to the cam's centre: solved

param t: Angle over (0deg, 360deg)          // the cam's turn

in std.front {
  o := point
  fix((0, 0)) o
  s0 := point
  s1 := point
  fix((0, 0)) s0
  fix((10, 0)) s1
  path := line(s0, s1)                    // the follower's line, through the cam's centre
}

cam := frame
cam spins(1) o
follower := frame
follower slides(h) path                 // the value `360deg` is the whole lift
// dwell to 60deg, rise to 180deg, dwell to 240deg, return to 360deg: each rise cycloidal
fix(value == 360deg * (min(max((t - 60deg) / 120deg, 0), 1)
                       - sin(360deg * min(max((t - 60deg) / 120deg, 0), 1)) / (2 * pi)
                       - min(max((t - 240deg) / 120deg, 0), 1)
                       + sin(360deg * min(max((t - 240deg) / 120deg, 0), 1)) / (2 * pi))) follower

// the follower's face, square to its line, `base` out where the turn starts
in std.front {
  f0 := point hint((20, -25))
  f1 := point hint((20, 25))
  f0 offset(dx: base, dy: -25) o
  f1 offset(dx: base, dy: 25) o
  flat := line(f0, f1)
}

low := envelope(flat over t in (0deg, 60deg), under: follower, seen_from: cam)
rise := envelope(flat over t in (60deg, 180deg), under: follower, seen_from: cam)
high := envelope(flat over t in (180deg, 240deg), under: follower, seen_from: cam)
return := envelope(flat over t in (240deg, 360deg), under: follower, seen_from: cam)

// the tightest bend, late in the rise, held to the least radius
in std.front {
  k := point hint((0, 0))
  wheel := circle(center: k) hint(r: 15)
  radius(least) wheel
  rise curvature(t == 180deg - acos(-1 / 8) / 3) wheel
}
