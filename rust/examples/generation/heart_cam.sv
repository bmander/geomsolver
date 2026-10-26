// A heart cam: a roller follower rising and falling at uniform speed, and the cam that does it.
//
// A heart cam turns steady rotation into a steady back-and-forth: the follower rises at one rate
// for half a turn and falls at the same rate for the other half.  It is how a bobbin winder
// lays thread evenly.  Nobody draws the cam.  It is what is left when the roller is carried
// round it: the envelope of the roller as the follower slides out along its line while the cam
// turns under it.
//
// The follower is driven by one law of the cam's turn `t`: its value climbs with `t` for half a
// turn and comes back down for the other half, `180deg - abs(180deg - t)`, and its slide carries
// it out `lift` in half a turn of that value.  Read seen from the turning cam, the follower's
// roller cuts the cam.  Each half of the profile is the envelope of the roller over its half of
// the turn (`side: near`, the side facing the cam's centre), and each half of the pitch curve,
// the path the roller's centre takes, is the envelope of the centre itself.  The law turns a
// corner at the half turn, and the pitch curve turns one with it, at the heart's point and its
// cleft, so the profile's halves part there: across the point the roller's own arc bridges them,
// and in the cleft they cross.  Each half reads the law on its own side of the corner.
//
// A circle osculates the rising half at 90° into the turn: its radius, which the solve works out,
// is how tightly the profile bends there (it must stay larger than a grinding wheel's).  Edit
// `lift`, `base` or the roller's radius, or the law, and the cam is cut again.

use std
use std.place (right_of)
use std.motion (spins, slides)

base := 25    // the roller centre's nearest approach to the cam's centre
lift := 15    // how far the follower rises
rr := 6       // the roller

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
follower slides(2 * lift) path                      // out `lift` in half a turn of its value...
fix(value == 180deg - abs(180deg - t)) follower     // ...which rises and falls uniformly

// the roller where the rise starts
in std.front {
  c := point hint((25, 0))
  c right_of(d: base) o
  roller := circle(center: c) hint(r: 6)
  radius(rr) roller
}

pitch_rise := envelope(c over t in (0deg, 180deg), under: follower, seen_from: cam)
pitch_fall := envelope(c over t in (180deg, 360deg), under: follower, seen_from: cam)
cam_rise := envelope(roller over t in (0deg, 180deg), under: follower, seen_from: cam, side: near)
cam_fall := envelope(roller over t in (180deg, 360deg), under: follower, seen_from: cam, side: near)

// how tightly the profile bends a quarter turn in
in std.front {
  k := point hint((0, -5))
  osc := circle(center: k) hint(r: 25)
  cam_rise curvature(t == 90) osc
}
