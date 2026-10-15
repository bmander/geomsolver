// Two faces drawn apart, lofted (#243 L8): CAD's "loft these two sketches". A square inlet and a
// diamond outlet, each sketched in its own plane, are joined corner to corner by
// `std.Ruled(a, b, s:)`: a straight rail from each corner of one walk to the corner in the same
// place of the other, and the section where the plane through three rails at `s` cuts every
// rail, so the ends are the faces themselves.
// Correspondence is the walks' written order and nothing else. A connector is where a walk
// starts: `turned` walks the diamond from its second corner, pairing the square's first corner
// with `b1`, and the section twists a quarter as it runs.
// Closed forms: the corners run linearly between parallel ends, so the section's area is
// quadratic in `s` and the prismatoid is exact — body 40/6 · (400 + 4·464 + 288) = 40/6 · 1856,
// turned 40/6 · (400 + 4·52 + 288) mm³.
// Edit `length`, `side` or `reach`, or start a walk at another corner.
unit mm
use std

length := 40mm
side := 20mm
reach := 12mm

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
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, -reach)) b0
  fix((reach, 0mm)) b1
  fix((0mm, reach)) b2
  fix((-reach, 0mm)) b3
}
inlet := face(a0, a1, a2, a3, -> close)
outlet := face(b0, b1, b2, b3, -> close)
// the same diamond walked from its second corner: a connector
outlet_turned := face(b1, b2, b3, b0, -> close)
body := solid(std.Ruled(inlet, outlet).f over s in (0, 1))
turned := solid(std.Ruled(inlet, outlet_turned).f over s in (0, 1))
