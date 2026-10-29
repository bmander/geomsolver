// A bar's flat end cutting the root of a tooth space: a bar pointing at a disc's centre, its end
// face 4 mm inside the disc's rim, slid along a line while the disc turns under it at the rate
// that rolls the line on a circle of radius 16 — a rack's motion, the bar its tooth. The bar's end
// is a cap of its prism, and it cuts: seen from the disc it stays 18 mm from the centre, so its
// envelope is the space's floor, a stretch of the 18 mm cylinder, and the rims where its sides
// meet it carry the walls (generating class, a prism's cap in the blank, #684 F). The bar's sides
// face along the line of the slide and touch nothing. Its profile tapers, 2.5 mm wide at the disc's
// bottom face and 3.5 at its top, so the space is no prism: each height of the disc loses the
// space its own slice of the bar sweeps, 183.21 mm³ in all (integrated slice by slice, the area
// at each height the arithmetic `rack.sv`'s space is read by).
unit mm
use std
use std.motion (spins, slides)

in std.top {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 22)
  radius(22mm) rim

  // the line the bar slides along, a pitch line tangent to a circle of radius 16
  s0 := point hint((16, 0))
  s1 := point hint((16, 10))
  fix((16, 0)) s0
  fix((16, 10)) s1
  pitch_line := line(s0, s1)
}
disc := solid(face(rim), depth: 6mm)

// the bar's section square to its length, standing through the disc's 6 mm with 3 mm to spare
// above and below, so its top and bottom never reach the disc
in std.side {
  b0 := point hint((-1, -9))
  b1 := point hint((1, -9))
  b2 := point hint((2, 3))
  b3 := point hint((-2, 3))
  fix((-1, -9)) b0
  fix((1, -9)) b1
  fix((2, 3)) b2
  fix((-2, 3)) b3
}
construction bar := solid(face(b0, b1, b2, b3, -> close), from: 18mm, to: 30mm)

turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 16mm) pitch_line
construction space := sweep(bar over t in (-60deg, 60deg), under: rack, seen_from: turn)
part := solid(disc)
space cut part
