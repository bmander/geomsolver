// A disc cam with an oscillating roller follower: the follower is an arm that swings about a
// pivot by a law of the cam's turn, and the cam is cut from that swing.
//
// Not every follower slides.  Valve gear and looms swing a rocker arm about a fixed pivot, its
// roller riding the cam, so what the law states is an angle: here the arm swings out `swing` and
// back once a turn on the simple harmonic law, `swing / 2 * (1 - cos(t))`.  The arm is a frame
// that turns about the pivot (`arm spins(1) pivot`), driven by that law, and the roller at its
// end swings on an arc about the pivot, not along a line through the cam's centre.  Seen from
// the turning cam, the roller's centre runs the pitch curve and its near side cuts the cam.  The
// law is smooth, so one envelope takes the whole turn.
//
// At rest the arm stands square to the line of centres, so the roller starts moving straight
// out from the cam.  Edit the pivot, the arm, the swing or the roller, and the cam is cut again.

use std
use std.place (right_of)
use std.motion (spins)

base := 30      // the roller centre's nearest approach to the cam's centre
arm_length := 40
swing := 20deg  // how far the arm swings out
rr := 7         // the roller

param t: Angle over (0deg, 360deg)          // the cam's turn

in std.front {
  o := point
  fix((0, 0)) o
  pivot := point
  fix((base, arm_length)) pivot           // square above where the roller starts
}

cam := frame
cam spins(1) o
arm := frame
arm spins(1) pivot                      // the arm turns about its pivot...
fix(value == swing / 2 * (1 - cos(t))) arm   // ...out and back once a turn

// the roller where the turn starts, at the arm's end
in std.front {
  c := point hint((30, 0))
  c right_of(d: base) o
  rocker := line(pivot, c)
  roller := circle(center: c) hint(r: 7)
  radius(rr) roller
}

pitch := envelope(c over t, under: arm, seen_from: cam)
profile := envelope(roller over t, under: arm, seen_from: cam, side: near)
