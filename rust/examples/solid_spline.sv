// Solids bounded by splines: a cam plate whose working edge is a cubic spline, bored through, and
// a vase whose wall is a spline turned about its axis.
unit mm
use std

// The cam: a base, a spline from its end round to the back, and the back straight down.
a := point hint(x: 0mm, y: 0mm)
b := point hint(x: 30mm, y: 0mm)
c := point hint(x: 36mm, y: 14mm)
d := point hint(x: 18mm, y: 28mm)
e := point hint(x: 0mm, y: 20mm)
ground a
ground b
ground c
ground d
ground e
lobe := spline(b, c, d, e)
base := line(a, b)
back := line(e, a)
plate := solid(face(base, lobe, back), depth: 6mm)
axle := point hint(x: 12mm, y: 9mm)
ground axle
bore := radius(4mm) circle(center: axle) hint(r: 4mm)
drill := solid(face(bore), through: plate)
cam := solid(plate)
drill cut cam

// The vase: its axis upright, a foot, a spline wall and a rim.
foot := point hint(x: 60mm, y: 0mm)
heel := point hint(x: 72mm, y: 0mm)
belly := point hint(x: 80mm, y: 12mm)
neck := point hint(x: 62mm, y: 24mm)
lip := point hint(x: 70mm, y: 36mm)
crown := point hint(x: 60mm, y: 36mm)
ground foot
ground heel
ground belly
ground neck
ground lip
ground crown
sole := line(foot, heel)
wall := spline(heel, belly, neck, lip)
rim := line(lip, crown)
construction centerline spine := line(crown, foot)
vase := solid(face(sole, wall, rim, spine), about: spine)
