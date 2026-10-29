// An enclosure with rounded corners, open at its top, and a screw post standing on its floor.
//
// Each straight wall is moved in by `wall`; each corner's arc about the same centre, its radius
// less by `wall` (8 outside, 5.5 inside): the walls' inner faces go on tangent, as the outer
// ones do. Built exactly.
//
// The post is added to the shelled body by naming it first (`case := solid(enclosure)`): added
// to the enclosure itself, the shell would have hollowed the post too, since a shell acts on its
// body after the body's other terms. 22835 + 1245.625π mm³: the walls, and the post's 17.5 above
// the floor.
unit mm
use std
use std.rounds (shell)

wall := 2.5mm

in std.top {
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  p4 := point
  p5 := point
  p6 := point
  p7 := point
  fix((8, 0)) p0
  fix((72, 0)) p1
  fix((80, 8)) p2
  fix((80, 42)) p3
  fix((72, 50)) p4
  fix((8, 50)) p5
  fix((0, 42)) p6
  fix((0, 8)) p7
  c1 := point
  c2 := point
  c3 := point
  c4 := point
  fix((72, 8)) c1
  fix((72, 42)) c2
  fix((8, 42)) c3
  fix((8, 8)) c4
  south := line(p0, p1)
  se := arc(center: c1, from: p1, to: p2)
  east := line(p2, p3)
  ne := arc(center: c2, from: p3, to: p4)
  north := line(p4, p5)
  nw := arc(center: c3, from: p5, to: p6)
  west := line(p6, p7)
  sw := arc(center: c4, from: p7, to: p0)
  outline := face(south, se, east, ne, north, nw, west, sw)
  o := point
  fix((20, 25)) o
  pin := radius(3mm) circle(center: o)
}
enclosure := solid(outline, from: 0mm, to: 30mm)
enclosure.near shell(t: wall) enclosure

case := solid(enclosure)
post := solid(face(pin), from: 1mm, to: 20mm)
post union case
