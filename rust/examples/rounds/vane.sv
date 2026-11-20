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
// The export says "`part` holds 2 faces thinner than the export's 10.000 µm: the first, `far`,
// 763.758 nm across, beside `far` and `round`, near (19.9996, 2.0000, 10.9684) mm", and that is
// the set's own: at each near corner the side's fill meets the vane's upright edge at z = 10.9684,
// the end's at 11.0228, and between them the end's fill's cap goes on beside the vane's side in
// one plane, a face of its own that the end's ball closes onto the edge as a cusp, 0.741 µm across
// where the side's fill leaves the edge. On a flat disc both fills reach the edge at one height.
//
// Edit `r` and apply (⌘↵): every ball the crown holds is built — past r = 20, the near end's
// distance from the disc's axis, the end's fill runs over the crown's top.
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
