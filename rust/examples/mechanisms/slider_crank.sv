// A slider-crank: the mechanism of every piston engine, driven round.
//
// The crank `ab` turns about `a`, and the rod `bs` drives the slider `s` back and forth along
// the rail through `a`.  Twice a turn the crank and the rod stand in line — top and bottom dead
// centre, at `t = 0` and `t = 180deg` — and the slider stands still there for an instant; the
// drawing does not, since the crank still turns, so the motion is followed straight through both.
//
// `t` is the crank's angle, declared as a number that runs a whole turn, and `rail angle(t)
// crank` is the one relation that reads it.  Every relation holds for every `t`, so the slider
// stands at `r·cos t + sqrt(l² − r²·sin² t)` along the rail at each instant, which nothing here
// states.  `stroke` is the slider's path, the rail between the dead centres run back and forth:
// 2r long.  `rod_path` is the rod's midpoint: an oval, traced from the same drawing.
//
// Diagnosed: DOF 0, and `1 DOF over t` (the crank); `t: followed from 0deg to 360deg`.

unit mm
use std

r := 10      // the crank
l := 30      // the connecting rod

param t: Angle over (0deg, 360deg)

in std.front {
  a := point
  e := point
  b := point hint((r, 0))
  s := point hint((r + l, 0))
  fix((0, 0)) a
  fix((60, 0)) e
  rail := line(a, e)

  crank := line(a, b)
  rod := line(b, s)
  length(r) crank
  length(l) rod
  s coincident rail
  rail angle(t) crank            // the driver: held for every t

  m := point
  m midpoint rod
}

stroke := trace(s over t)
rod_path := trace(m over t)
