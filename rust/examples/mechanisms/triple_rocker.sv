// A four-bar whose crank cannot turn round, and where it stops.
//
// The rods are `four_bar.sv`'s in another proportion: the ground 40, the input 20, the coupler 25
// and the rocker 30.  Now the shortest and the longest together (20 + 40) are longer than the
// other two (25 + 30), so no rod turns all the way round — a triple rocker.  Driven by its input
// angle `t` from 0 toward a whole turn, the linkage swings until the coupler and the rocker stand
// in line, `|bd| = l + k`, and past that no drawing stands: the motion turns back.
//
// The drawing is followed from `t = 0` and the march stops there, at the closed form's
// `acos((r² + g² − (l + k)²) / (2·r·g)) = 129.838deg`, and says so: `t: followed from 0deg to
// 129.838deg, singular at t = 129.838deg: no drawing stands past it, the motion turns back`.
// `path`, the rocker pin's curve, ends where the march does.  `four_bar.sv` runs the whole turn.

unit mm
use std

g := 40      // the ground
r := 20      // the input
l := 25      // the coupler
k := 30      // the rocker

param t: Angle over (0deg, 360deg)

in std.front {
  a := point
  d := point
  b := point hint((r, 0))
  c := point hint((23, 25))      // above the ground: the assembly mode carried
  fix((0, 0)) a
  fix((g, 0)) d

  ground := line(a, d)
  input := line(a, b)
  coupler_bar := line(b, c)
  rocker := line(d, c)
  length(r) input
  length(l) coupler_bar
  length(k) rocker
  ground angle(t) input          // the driver
}

path := trace(c over t)
