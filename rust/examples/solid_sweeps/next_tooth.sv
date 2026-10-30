// A rack's next tooth: `bar_end.sv`'s bar drawn once and placed a quarter turn of the roll along
// the rack (16 mm a radian, 25.13 mm), as a rack's further teeth are, then rolled with the rack.
// Its flat end cuts the floor of a space as the bar's does (generating class, a placed prism's cap
// in the blank, #684 AC): the placement carries the bar's sections across. Rolled from -150 to
// -30 degrees, it stands a quarter turn of the disc on from where the bar stood at each roll from
// -60 to 60, so it cuts the bar's space turned a quarter turn round the disc: 183.21 mm³ again.
unit mm
use std
use std.motion (spins, slides)

in std.top {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 22)
  radius(22mm) rim

  // the line the rack slides along, a pitch line tangent to a circle of radius 16
  s0 := point hint((16, 0))
  s1 := point hint((16, 10))
  fix((16, 0)) s0
  fix((16, 10)) s1
  pitch_line := line(s0, s1)
}
disc := solid(face(rim), depth: 6mm)

// the tooth as drawn, tapering through the disc's 6 mm with 3 mm to spare above and below
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
// the next tooth, a quarter turn of the rack's roll along it
construction tooth := bar at(under: rack) 90deg
construction space := sweep(tooth over t in (-150deg, -30deg), under: rack, seen_from: turn)
part := solid(disc)
space cut part
