// A square duct turned a quarter-square as it runs: a square inlet lofted to a diamond outlet,
// each side joined to the side written in its place — a twisted ruled face, not a plane.
unit mm
use std
length := 40mm
half := 10mm
reach := 12mm

end_plane := plane(origin: std.origin, toward: std.front.toward, from: std.front, offset: -length)
in std.front {
  c := point hint(x: 0mm, y: 0mm)
  c coincident std.origin
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix(x == -half, y == -half) a0
  fix(x == half, y == -half) a1
  fix(x == half, y == half) a2
  fix(x == -half, y == half) a3
}
in end_plane {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix(x == 0mm, y == -reach) b0
  fix(x == reach, y == 0mm) b1
  fix(x == 0mm, y == reach) b2
  fix(x == -reach, y == 0mm) b3
}
plan := plane(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  entry := point hint(x: 0mm, y: 0mm)
  entry coincident std.origin
  exit := point hint(x: 0mm, y: length)
  construction centerline guide := line(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
body := solid(face(a0, a1, a2, a3, -> close), face(b0, b1, b2, b3, -> close), along: guide)
