// A loft to a point (#243 L8): `std.Apex(a, tip, s:)` joins each corner of the face `a` to the
// point `tip` by a straight rail, the section where the plane through three of them at `s` cuts
// every rail. At `s = 1` the section is the point itself, read as the limit of the stations
// closing in on it: the solid ends in an apex, with no cap there.
// Closed form: a third of base by height, whatever it leans: 20² · 30 / 3 = 4000 mm³ as drawn.
// Edit `side` or `height`, or move the tip off the axis (`lean`).
unit mm
use std

side := 20mm
height := 30mm
lean := 3mm

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
tip := point
fix((lean, height, 2mm)) tip
base := face(a0, a1, a2, a3, -> close)
param s: Scalar over (0, 1)
pyramid := sweep(std.Apex(base, tip).f over s)
