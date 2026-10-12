// A disc whose rims are rounded and chamfered in its own definition, with no solid to cut and no
// body: a region is a conjunction of bounds on its point, and a round or a chamfer where two of
// them meet is one more bound.
//
// `disc` is the points inside the cylinder `wall` and between the half-spaces `top` and `bottom`.
// `std.Rounded(wall, top, r: …)` is what a ball of that radius rolled inside both cannot reach
// excluded; `std.Chamfered(wall, bottom, la: …, lb: …)` is the flat across their corner, `la` along
// the wall and `lb` along the bottom. Every term here is about the one axis, so the region is a
// meridian turned: the round and the chamfer are built exactly in that meridian (lowered), and
// the solid is a B-rep that `solventc --step` writes.
//
// Edit the radius, the chamfer's legs or the disc's size and apply (⌘↵). Tilt the wall (fix `ze`
// at (0.2, 0, 1)) and its line is no longer square to the planes: the terms share no axis, so the
// region is read point by point instead, still rounded and chamfered.
unit mm
use std

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
ze := point hint((0, 0, 1))
fix((0, 0, 1)) ze
axis_line := line(c, ze)
tx := axis hint(dir: (1, 0, 0))
ty := axis hint(dir: (0, 1, 0))
fix(dir == (1, 0, 0), origin == (0, 0, 2)) tx
fix(dir == (0, 1, 0), origin == (0, 0, 2)) ty
mid := plane(u: tx, v: ty)

wall := std.Cylinder(axis_line, r: 10)
top := { q | q distance(<= 2mm, along: n) mid }
bottom := { q | q distance(>= -2mm, along: n) mid }

finished := { p |
  p inside wall
  p inside top
  p inside bottom
  p inside std.Rounded(wall, top, r: 1.5mm)
  p inside std.Chamfered(wall, bottom, la: 1mm, lb: 1mm)
}
disc := solid(finished)
