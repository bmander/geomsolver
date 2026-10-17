// L0 (#243): a loft between two faces drawn apart, as CAD's loft dialog takes them: a square inlet
// and a diamond outlet sketched in their own planes and joined corner to corner in written order
// (`solid_twist.sv`'s pair). The reducer and the duct draw one section that runs; this program is
// the case where the writer has two sketches and wants the word to pair them. Since L8 the word
// is `std.Ruled`: a straight rail from each corner of the inlet to the outlet's corner in the same
// place of its walk, the section where the plane through three of them at `s` cuts every rail.
// Closed form: the ruled pair's area is quadratic in s; prismatoid V = 40/6 · (400 + 4·A½ + 288),
// A½ the area of the mid section (the square and the diamond averaged corner by corner).
unit mm
use std
length := 40mm

end_plane := plane
fix(origin == (0mm, length, 0mm)) end_plane
fix(dir == (1, 0, 0)) end_plane.u
fix(dir == (0, 0, 1)) end_plane.v
in std.front {
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((-10mm, -10mm)) a0
  fix((10mm, -10mm)) a1
  fix((10mm, 10mm)) a2
  fix((-10mm, 10mm)) a3
}
in end_plane {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, -12mm)) b0
  fix((12mm, 0mm)) b1
  fix((0mm, 12mm)) b2
  fix((-12mm, 0mm)) b3
}
inlet := face(a0, a1, a2, a3, -> close)
outlet := face(b0, b1, b2, b3, -> close)
param s: Scalar over (0, 1)
body := sweep(std.Ruled(inlet, outlet).f over s)
