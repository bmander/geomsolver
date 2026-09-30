// A square duct turned a quarter-square as it runs: a square inlet lofted to a diamond outlet,
// each side joined to the side written in its place — a twisted ruled face, not a plane.
unit mm
use std
param length = 40mm
param half = 10mm
param reach = 12mm

plane end_plane(origin: std.origin, toward: std.front.toward, from: std.front, offset: -length)
in std.front {
  point c hint(x: 0mm, y: 0mm)
  c coincident std.origin
  point a0 hint(x: -half, y: -half)
  point a1 hint(x: half, y: -half)
  point a2 hint(x: half, y: half)
  point a3 hint(x: -half, y: half)
  ground a0
  ground a1
  ground a2
  ground a3
}
in end_plane {
  point b0 hint(x: 0mm, y: -reach)
  point b1 hint(x: reach, y: 0mm)
  point b2 hint(x: 0mm, y: reach)
  point b3 hint(x: -reach, y: 0mm)
  ground b0
  ground b1
  ground b2
  ground b3
}
plane plan(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  point entry hint(x: 0mm, y: 0mm)
  entry coincident std.origin
  point exit hint(x: 0mm, y: length)
  construction centerline line guide(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
solid body(face(a0, a1, a2, a3, -> close), face(b0, b1, b2, b3, -> close), along: guide)
