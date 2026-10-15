// A square lofted to a triangle (#243 L8): two faces of different corner counts. `std.Ruled`
// pairs corners by their places in the walks, so the walks must be as long (a square of four to
// a triangle of three is refused, naming both). The triangle is given a fourth corner by naming
// a point on one of its edges — `m`, the middle of its left side — and walking through it: a
// statement, never inferred. Where the corner goes says how the loft twists.
// Closed form: the corners run linearly between parallel ends, so the prismatoid is exact:
// 30/6 · (400 + 4·287.5 + 200) = 8750 mm³ as drawn.
// Edit `length` or `side`, or move `m` along the edge (`t == 0.5`).
unit mm
use std

length := 30mm
side := 20mm

end_plane := plane
fix(origin == (0mm, length, 0mm)) end_plane
fix(dir == (1, 0, 0)) end_plane.u
fix(dir == (0, 0, 1)) end_plane.v
in std.front {
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((-side / 2, -side / 2)) a0
  fix((side / 2, -side / 2)) a1
  fix((side / 2, side / 2)) a2
  fix((-side / 2, side / 2)) a3
}
in end_plane {
  t0 := point
  t1 := point
  t2 := point
  m := point
  fix((-side / 2, -side / 2)) t0
  fix((side / 2, -side / 2)) t1
  fix((0mm, side / 2)) t2
  construction left := line(t2, t0)
  m coincident(t == 0.5) left
}
square := face(a0, a1, a2, a3, -> close)
triangle := face(t0, t1, t2, m, -> close)
body := solid(std.Ruled(square, triangle).f over s in (0, 1))
