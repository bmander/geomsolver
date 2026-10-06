// Solids bounded by splines: a cam plate whose working edge is a cubic spline, bored through, and
// a vase whose wall is a spline turned about its axis.
unit mm
use std

// The cam: a base, a spline from its end round to the back, and the back straight down.
in std.front {
  a := point
  b := point
  c := point
  d := point
  e := point
  fix((0mm, 0mm)) a
  fix((30mm, 0mm)) b
  fix((36mm, 14mm)) c
  fix((18mm, 28mm)) d
  fix((0mm, 20mm)) e
  lobe := spline(b, c, d, e)
  base := line(a, b)
  back := line(e, a)
  plate := solid(face(base, lobe, back), depth: 6mm)
  axle := point
  fix((12mm, 9mm)) axle
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
  fix((60mm, 0mm)) foot
  fix((72mm, 0mm)) heel
  fix((80mm, 12mm)) belly
  fix((62mm, 24mm)) neck
  fix((70mm, 36mm)) lip
  fix((60mm, 36mm)) crown
  sole := line(foot, heel)
  wall := spline(heel, belly, neck, lip)
  rim := line(lip, crown)
  construction centerline spine := line(crown, foot)
}
vase := solid(face(sole, wall, rim, spine), about: spine)
