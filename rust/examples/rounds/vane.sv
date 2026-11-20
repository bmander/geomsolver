// A vane filled into a disc whose crown is a sphere (#962's 7): a disc of radius 100 turned from a
// section whose top is an arc about (0, −797) through its rim at (100, 3), and a flat vane 4 thick
// standing on it from x = 20 to 90, unioned, its foot filled by a ball of 2.
//
// A plane meets a sphere in a circle about the line through the sphere's centre square to the
// plane, and the ball between them has its centre on a circle about that line too: each of the
// four fills is a ring, a torus's stretch, turned about the line from the crown's centre square
// to its side and kept between the planes of the two sides beside it. At each of the vane's
// corners two stretches meet along its upright edge, each one's cap in the plane the other's
// torus touches. Built exactly: 198988.300 mm³ and 22 faces, six of them tori (the long sides'
// stretches each one torus, the ends' two each, split where the crown's seam crosses them).
//
// Edit `r` and apply (⌘↵): every ball the crown holds is built, up to the one whose fill along
// the near end reaches over the crown's top (r = 20, the end's distance from the disc's axis).
unit mm
use std
use std.solids
use std.solids (union)
use std.rounds (round)

r := 2mm   // the fill's radius

in std.front {
  o := point
  fix((0, 0)) o
  rim0 := point
  fix((100, 0)) rim0
  rim1 := point
  fix((100, 3)) rim1
  c := point
  fix((0, -797)) c
  top := point hint((0, 9.2))
  fix(x == 0mm) top
  bottom := line(o, rim0)
  edge := line(rim0, rim1)
  crown := arc(center: c, start: rim1, end: top)
  spine := line(top, o)
  section := face(bottom, edge, crown, spine)
}
disc := solid(std.solids.Revolution(section, about: std.z))

in std.top {
  a := point
  b := point
  d := point
  e := point
  fix((20, -2)) a
  fix((90, -2)) b
  fix((90, 2)) d
  fix((20, 2)) e
  outline := face(a, b, d, e, -> close)
}
vane := solid(std.solids.Prism(outline, from: 1mm, to: 31mm))

part := solid(disc)
vane union part
part.vane round(r: r) part.disc
