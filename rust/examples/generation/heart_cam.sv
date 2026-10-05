// A heart cam: a roller follower rising and falling at uniform speed, and the cam that does it.
//
// A heart cam turns steady rotation into a steady back-and-forth: the follower rises at one rate
// for half a turn and falls at the same rate for the other half.  It is how a bobbin winder
// lays thread evenly.  Nobody draws the cam.  It is what is left when the roller is carried
// round it: the envelope of the roller as the follower slides out along its line while the cam
// turns under it.
//
// `rise` is the follower sliding out `lift` over half a turn, seen from the turning cam; `fall`
// slides back in.  Each half of the profile is the envelope of the roller under one of them
// (`side: near`, the side facing the cam's centre), and each half of the pitch curve, the path
// the roller's centre takes, is the envelope of the centre itself.  The pitch curve's halves
// meet at the heart's point and its cleft, where it turns corners, so the profile's halves part
// there: across the point the roller's own arc bridges them, and in the cleft they cross.
//
// A circle osculates the rising half at 90° into the turn: its radius, which the solve works out,
// is how tightly the profile bends there (it must stay larger than a grinding wheel's).  Edit
// `lift`, `base` or the roller's radius and the cam is cut again.

use std

base := 25    // the roller centre's nearest approach to the cam's centre
lift := 15    // how far the follower rises
rr := 6       // the roller

in std.front {
  o := point
  fix(x == 0, y == 0) o
  s0 := point hint(x: 0, y: 0)
  s1 := point hint(x: 10, y: 0)
  fix(x == 0, y == 0) s0
  fix(x == 10, y == 0) s1
  path := line(s0, s1)                    // the follower's line, through the cam's centre
}

cam := motion(about: o, ratio: 1)
out := motion(along: path, advance: 2 * lift)     // out `lift` in half a turn...
back := motion(along: path, advance: -2 * lift)   // ...and back in at the same rate
rise := motion(out, relative_to: cam)
fall := motion(back, relative_to: cam)

// the roller where the rise starts, and where the fall would start if it began at no roll
in std.front {
  c_rise := point hint(x: 25, y: 0)
  c_fall := point hint(x: 55, y: 0)
  o distance(base, along: x) c_rise
  o distance(0, along: y) c_rise
  o distance(base + 2 * lift, along: x) c_fall
  o distance(0, along: y) c_fall
  roller := circle(center: c_rise) hint(r: 6)
  roller_f := circle(center: c_fall) hint(r: 6)
  radius(rr) roller
  radius(rr) roller_f
}

pitch_rise := envelope(c_rise, under: rise, from: 0deg, to: 180deg)
pitch_fall := envelope(c_fall, under: fall, from: 180deg, to: 360deg)
cam_rise := envelope(roller, under: rise, from: 0deg, to: 180deg, side: near)
cam_fall := envelope(roller_f, under: fall, from: 180deg, to: 360deg, side: near)

// how tightly the profile bends a quarter turn in
in std.front {
  k := point hint(x: 0, y: -5)
  osc := circle(center: k) hint(r: 25)
  cam_rise curvature(t == 90) osc
}
