// A rail (issue #66, rung 2): a rod half sunk along a plate, the ball rolled along both sides of it
// between the plate's top and the rod — a section with a curved side, its face a cylinder still,
// ending flush in the plate's ends.
unit mm
a := point
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 10)
d := point hint(x: 0, y: 10)
fix(x == 0, y == 0) a
fix(x == 60, y == 0) b
fix(x == 60, y == 10) c
fix(x == 0, y == 10) d
(ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
o := point hint(x: 30, y: 10)
fix(x == 30, y == 10) o
k := circle(center: o) hint(r: 6)
radius(6) k
plate := solid(face(ab, bc, cd, da), from: 0mm, to: 40mm)
rod := solid(face(k), from: 0mm, to: 40mm)
rail := solid(plate)
rod union rail
sides := fillet(rod, plate, r: 2mm)
sides union rail
o1 := point hint(x: 100, y: 0)
o2 := point hint(x: 108, y: 0)
