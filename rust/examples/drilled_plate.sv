// How large a lightening hole may be (#510 S2): an aluminium web built in along its root, loaded
// along its end and by its own weight, read in plane stress, with a round hole at its middle whose
// radius `r` is an unknown.  The deflection of the end's lower corner is a driving requirement:
// the face is meshed once in quadratic triangles, its nodes on the hole moving with its radius
// and those inside following the boundary, and the solve finds the radius at which the corner
// sinks `sag` — 19.23mm.  The hole's circle is seeded near where it will be solved, since its
// mesh is made where the drawing stands when the row is compiled.
unit mm
use std

L := 160mm
h := 60mm
t := 5mm
P := 2kN
sag := 0.6mm
alloy := material(E: 70GPa, nu: 0.33, rho: 2700kg / 1m^3, yield: 240MPa)
param r: Length hint(18mm)

in std.front {
  root_low := point
  end_low := point
  end_high := point
  root_high := point
  o := point
  fix((0mm, 0mm)) root_low
  fix((L, 0mm)) end_low
  fix((L, h)) end_high
  fix((0mm, h)) root_high
  fix((L / 2, h / 2)) o
  low := line(root_low, end_low)
  end := line(end_low, end_high)
  top := line(end_high, root_high)
  root := line(root_high, root_low)
  hole := radius(r) circle(center: o) hint(r: 18)
  plate := face(low, end, top, root, holes: hole)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
web := solid(plate, from: -t / 2, to: t / 2) of alloy

scenario hung {
  plane stress web
  fixed web.root
  load web.end P along: down
  accelerate web (0m / 1s^2, 0m / 1s^2, -9.81m / 1s^2)
  deflection(end_low, along: down) == sag
  claim safety(web) >= 1
}
