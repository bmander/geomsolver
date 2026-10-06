// A square duct turned a quarter-square as it runs: a square inlet lofted to a diamond outlet,
// each side joined to the side written in its place — a twisted ruled face, not a plane.
unit mm
use std
length := 40mm
half := 10mm
reach := 12mm

end_plane := plane
fix(origin == (0mm, length, 0mm)) end_plane
fix(dir == (1, 0, 0)) end_plane.u
fix(dir == (0, 0, 1)) end_plane.v
in std.front {
  c := point hint((0mm, 0mm))
  c coincident std.origin
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((-half, -half)) a0
  fix((half, -half)) a1
  fix((half, half)) a2
  fix((-half, half)) a3
}
in end_plane {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, -reach)) b0
  fix((reach, 0mm)) b1
  fix((0mm, reach)) b2
  fix((-reach, 0mm)) b3
}
in std.top {
  entry := point hint((0mm, 0mm))
  entry coincident std.top.origin
  exit := point hint((0mm, length))
  construction centerline guide := line(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
body := solid(face(a0, a1, a2, a3, -> close), face(b0, b1, b2, b3, -> close), along: guide)
