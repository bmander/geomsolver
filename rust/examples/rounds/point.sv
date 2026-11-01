// A cylinder whose rim is chamfered to a point (brep-bench's chamfer/cylinder-rim-to-point): a
// cylinder of radius 5 standing 10 high, the circle round its top chamfered 5 on both faces, the
// cylinder's own radius.  The leg across the top stands 5 in from the side, which is the axis
// alone, so the top shrinks to its centre and the flat is a cone of half-angle 45° from the circle
// at half the height up to its apex at (0, 0, 10).  The solid is the cylinder from 0 to 5 under
// that cone: π5²·5 + π5²·5/3 = 500π/3 = 523.599 mm³, and 75π + 25√2π of area in a cylinder, a
// cone and a plane.
//
// Edit `d` and apply (⌘↵): under 5 the top keeps a disc, at 5 the disc closes into the cone's
// apex, and over 5 no point of the top stands that far in from the side: refused.
unit mm
use std
use std.rounds (chamfer)

d := 5mm   // the chamfer's distance on each face

in std.top {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 5)
  radius(5mm) rim
}
drum := solid(face(rim), from: 0mm, to: 10mm)
drum.near chamfer(d: d) drum.rim
