// A disc cam with dwells: the follower rests, rises, rests and returns, each rise on the
// cycloidal law, and the cam cut from that law.
//
// The follower of a machine's cam usually waits: it stands still at the bottom of its stroke
// while the cam turns a sixth of a turn, rises `h` over the next third, stands still again at the
// top for a sixth, and comes back down over the last third.  How it rises matters as much as how
// far: on the cycloidal law its lift a fraction `x` of the way through a rise is
//
//   s = h (x - sin(360deg x) / 2π)
//
// so its speed and its acceleration are both zero at either end, and it joins each dwell
// without a jolt.  The follower is driven by that law of the cam's turn `t`, written as one
// expression: `x` is the turn into the rise, `(t - 60deg) / 120deg`, held to [0, 1] by `min` and
// `max`, so before the rise it reads 0 and after it 1; the return is the same shape a half turn
// on, taken away.  Its slide carries it `h` out in a whole turn of its value, so the value
// `360deg` is the whole lift.
//
// Each of the four stretches is its own envelope, reading the law on its own side of the corners
// `min` and `max` turn: the pitch curve (the roller centre's path seen from the cam) and the
// profile (the roller's side facing the cam's centre).  On the dwells they are arcs about the
// cam's centre, of `base` and `base + h`; on the rise and the return the cycloidal curve.  Edit
// `h`, `base`, the roller or the law, and the cam is cut again.

use std (right_of, spins, slides)

base := 30    // the roller centre's nearest approach to the cam's centre
h := 12       // the follower's lift
rr := 8       // the roller

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

// the roller where the turn starts, at the bottom of its stroke
in std.front {
  c := point hint((30, 0))
  c right_of(d: base) o
  roller := circle(center: c) hint(r: 8)
  radius(rr) roller
}

// the pitch curve, stretch by stretch
pitch_low := envelope(c over t in (0deg, 60deg), under: follower, seen_from: cam)
pitch_rise := envelope(c over t in (60deg, 180deg), under: follower, seen_from: cam)
pitch_high := envelope(c over t in (180deg, 240deg), under: follower, seen_from: cam)
pitch_return := envelope(c over t in (240deg, 360deg), under: follower, seen_from: cam)

// the cam
low := envelope(roller over t in (0deg, 60deg), under: follower, seen_from: cam, side: near)
rise := envelope(roller over t in (60deg, 180deg), under: follower, seen_from: cam, side: near)
high := envelope(roller over t in (180deg, 240deg), under: follower, seen_from: cam, side: near)
return := envelope(roller over t in (240deg, 360deg), under: follower, seen_from: cam, side: near)
