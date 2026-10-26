// A cam and its follower, on the timeline: the cam cut from its follower's law, and the follower
// drawn riding on it as the cam turns.
//
// The follower slides on its line, its roller pressed on the cam; the cam turns at a steady
// rate `t`.  What the follower must do is stated as one law of `t`, a rise of `h` and a fall
// on the simple harmonic law, `180deg * (1 - cos(t))`: its slide carries it `h` out in a whole
// turn of its value, so the value `360deg` is the whole lift.  The cam is not drawn: it is cut
// from the law, the envelope of the roller as the follower rises and falls by it, seen from the
// turning cam (`generation/heart_cam.sv` and its neighbours do the same).
//
// What is drawn in the follower's frame stands where the law puts it at the instant the drawing
// stands at, so `riding`, the roller's centre drawn in the follower, and `pin`, a point of the
// cam drawn in the cam's frame, move as `t` runs, and the drawing is followed round the turn.
// Scrub `t` in the app: the roller rides out and back, and the cam's pin turns with it; turned
// back by the cam, the roller stands on the profile cut for it at every instant.
//
// Diagnosed: DOF 0; `solventc` says the drawing is followed from 0deg to 360deg.

unit mm
use std
use std.place (right_of)
use std.motion (spins, slides)

base := 25    // the roller centre's nearest approach to the cam's centre
h := 10       // the follower's lift
rr := 5       // the roller

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
fix(value == t) cam                     // the cam turns with `t`
follower := frame
follower slides(h) path                 // the value `360deg` is the whole lift
fix(value == 180deg * (1 - cos(t))) follower   // out and back once a turn

// the roller where the turn starts, at the bottom of its stroke, and a pin on the cam
in std.front {
  c := point hint((25, 0))
  c right_of(d: base) o
  roller := circle(center: c) hint(r: 5)
  radius(rr) roller
  p := point
  fix((0, 12)) p
}

pitch := envelope(c over t, under: follower, seen_from: cam)
profile := envelope(roller over t, under: follower, seen_from: cam, side: near)

// drawn where the instant puts them
riding := c in follower
pin := p in cam
