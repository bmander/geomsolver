// Two faces drawn apart, lofted (#243 L8): CAD's "loft these two sketches". A square inlet and a
// diamond outlet, each sketched in its own plane, are joined corner to corner by
// `std.stations.Ruled(a, b, s:)`: a straight rail from each corner of one walk to the corner in the
// same place of the other, and the section where the plane through three rails at `s` cuts every
// rail, so the ends are the faces themselves.
// Correspondence is the walks' written order and nothing else. A connector is where a walk
// starts: `turned` walks the diamond from its second corner, pairing the square's first corner
// with `b1`, and the section twists a quarter as it runs.
// Closed forms: the corners run linearly between parallel ends, so the section's area is
// quadratic in `s` and the prismatoid is exact — body 40/6 · (400 + 4·464 + 288) = 40/6 · 1856,
// turned 40/6 · (400 + 4·52 + 288) mm³.
// `turned` is drawn a second time, `apart` along x, so the two are seen side by side.
// Edit `length`, `side`, `reach` or `apart`, or start a walk at another corner.
unit mm
use std
use std.stations

length := 40mm
side := 20mm
reach := 12mm
apart := 40mm

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
// the same two faces again, `apart` along x, for `turned`
end_plane2 := plane
fix(origin == (apart, length, 0mm)) end_plane2
fix(dir == (1, 0, 0)) end_plane2.u
fix(dir == (0, 0, 1)) end_plane2.v
in std.front {
  c0 := point
  c1 := point
  c2 := point
  c3 := point
  fix((apart - side / 2, -side / 2)) c0
  fix((apart + side / 2, -side / 2)) c1
  fix((apart + side / 2, side / 2)) c2
  fix((apart - side / 2, side / 2)) c3
}
in end_plane2 {
  d0 := point
  d1 := point
  d2 := point
  d3 := point
  fix((0mm, -reach)) d0
  fix((reach, 0mm)) d1
  fix((0mm, reach)) d2
  fix((-reach, 0mm)) d3
}
inlet := face(a0, a1, a2, a3, -> close)
outlet := face(b0, b1, b2, b3, -> close)
inlet2 := face(c0, c1, c2, c3, -> close)
// the same diamond walked from its second corner: a connector
outlet_turned := face(d1, d2, d3, d0, -> close)
param s: Scalar over (0, 1)
body := sweep(std.stations.Ruled(inlet, outlet).f over s)
turned := sweep(std.stations.Ruled(inlet2, outlet_turned).f over s)
