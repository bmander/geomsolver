// A cylinder whose rim is rounded into a dome (brep-bench's fillet/cylinder-rim-to-dome): a
// cylinder of radius 5 standing 10 high, the circle round its top rounded to radius 5, the
// cylinder's own.  The ball rolled round the rim inside both faces is as wide as the cylinder, so
// its centre has nowhere to go but the axis, 5 below the top: the top shrinks to a point and the
// round is a hemisphere about (0, 0, 5), touching the side along the circle at half its height.
// The solid is the cylinder from 0 to 5 under that dome, its top the dome's pole:
// π5²·5 + (2/3)π5³ = 625π/3 = 654.498 mm³, and 125π of area in a cylinder, a sphere and a plane.
//
// Edit `r` and apply (⌘↵): under 5 the round is a ring whose face is a torus, at 5 the torus
// closes on the axis into the sphere, and over 5 no ball fits inside the cylinder: refused.
unit mm
use std
use std.rounds (round)

r := 5mm   // the rim's radius

in std.top {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 5)
  radius(5mm) rim
}
drum := solid(face(rim), from: 0mm, to: 10mm)
drum.near round(r: r) drum.rim
