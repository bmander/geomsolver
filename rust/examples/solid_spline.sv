// Solids bounded by splines: a cam plate whose working edge is a cubic spline, bored through, and
// a vase whose wall is a spline turned about its axis.
unit mm
use std

// The cam: a base, a spline from its end round to the back, and the back straight down.
point a hint(x: 0mm, y: 0mm)
point b hint(x: 30mm, y: 0mm)
point c hint(x: 36mm, y: 14mm)
point d hint(x: 18mm, y: 28mm)
point e hint(x: 0mm, y: 20mm)
ground a
ground b
ground c
ground d
ground e
spline lobe(b, c, d, e)
line base(a, b)
line back(e, a)
solid plate(face(base, lobe, back), depth: 6mm)
point axle hint(x: 12mm, y: 9mm)
ground axle
radius(4mm) circle bore(center: axle) hint(r: 4mm)
solid drill(face(bore), through: plate)
solid cam(plate)
drill cut cam

// The vase: its axis upright, a foot, a spline wall and a rim.
point foot hint(x: 60mm, y: 0mm)
point heel hint(x: 72mm, y: 0mm)
point belly hint(x: 80mm, y: 12mm)
point neck hint(x: 62mm, y: 24mm)
point lip hint(x: 70mm, y: 36mm)
point crown hint(x: 60mm, y: 36mm)
ground foot
ground heel
ground belly
ground neck
ground lip
ground crown
line sole(foot, heel)
spline wall(heel, belly, neck, lip)
line rim(lip, crown)
construction centerline line spine(crown, foot)
solid vase(face(sole, wall, rim, spine), about: spine)
