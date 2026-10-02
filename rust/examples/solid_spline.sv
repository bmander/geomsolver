// Solids bounded by splines: a cam plate whose working edge is a cubic spline, bored through, and
// a vase whose wall is a spline turned about its axis.
unit mm
use std

// The cam: a base, a spline from its end round to the back, and the back straight down.
a := point
b := point
c := point
d := point
e := point
fix(x == 0mm, y == 0mm) a
fix(x == 30mm, y == 0mm) b
fix(x == 36mm, y == 14mm) c
fix(x == 18mm, y == 28mm) d
fix(x == 0mm, y == 20mm) e
lobe := spline(b, c, d, e)
base := line(a, b)
back := line(e, a)
plate := solid(face(base, lobe, back), depth: 6mm)
axle := point
fix(x == 12mm, y == 9mm) axle
bore := radius(4mm) circle(center: axle) hint(r: 4mm)
drill := solid(face(bore), through: plate)
cam := solid(plate)
drill cut cam

// The vase: its axis upright, a foot, a spline wall and a rim.
foot := point
heel := point
belly := point
neck := point
lip := point
crown := point
fix(x == 60mm, y == 0mm) foot
fix(x == 72mm, y == 0mm) heel
fix(x == 80mm, y == 12mm) belly
fix(x == 62mm, y == 24mm) neck
fix(x == 70mm, y == 36mm) lip
fix(x == 60mm, y == 36mm) crown
sole := line(foot, heel)
wall := spline(heel, belly, neck, lip)
rim := line(lip, crown)
construction centerline spine := line(crown, foot)
vase := solid(face(sole, wall, rim, spine), about: spine)
