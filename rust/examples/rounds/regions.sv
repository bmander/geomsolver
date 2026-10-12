// Solids that are sets: each is the points `p` a body of bounds holds, and none is a sweep. Their
// terms share no axis, so no meridian folds them: the field reads each point by point, its
// surface the place where the largest of the terms' signed distances is nothing.
//
// - `egg` is a bound on a sum of measurements: the points whose distances from two foci add to at
//   most 10mm, an ellipsoid of revolution with semi-axes 5 and 4.
// - `bicylinder` is two cylinders crossing square, each unbounded along its line but bounded by
//   the other: Steinmetz's solid.
// - `cell` is the half of a ball nearer one centre than the other: a bound comparing two
//   measurements, a Voronoi cell.
//
// Orbit (right-drag) to see them. Move a focus, change a radius or move a centre and apply (⌘↵).
unit mm
use std

// the egg's foci, along x about the origin
f1 := point hint((-3, 0, 0))
fix((-3, 0, 0)) f1
f2 := point hint((3, 0, 0))
fix((3, 0, 0)) f2
foci := { p | distance(p, f1) + distance(p, f2) <= 10mm }
egg := solid(foci)

// two lines through (20, 0, 0), along x and along z
o := point hint((20, 0, 0))
fix((20, 0, 0)) o
ox := point hint((21, 0, 0))
fix((21, 0, 0)) ox
oz := point hint((20, 0, 1))
fix((20, 0, 1)) oz
along_x := line(o, ox)
along_z := line(o, oz)
a := std.Cylinder(along_x, r: 3)
b := std.Cylinder(along_z, r: 3)
crossing := { p | p inside a; p inside b }
bicylinder := solid(crossing)

// a ball about (40, 0, 0), and two centres either side of it
m := point hint((40, 0, 0))
fix((40, 0, 0)) m
near := point hint((38, 0, 0))
fix((38, 0, 0)) near
far := point hint((42, 0, 0))
fix((42, 0, 0)) far
ball := std.Sphere(m, r: 5)
nearer := { p | p inside ball; distance(p, near) <= distance(p, far) }
cell := solid(nearer)
