// A web lightened to a mass (#510): an aluminium web built in along its root, with a round hole at
// its middle whose radius `r` is an unknown, and `mass(110g) web` stated of the drawing — what the
// web is made of answers no load, so it is no scenario's.  The mass is a row over the web's face,
// meshed once and moved with the hole, `ρ t (L h − π r²)`: the hole the solve finds is
// √((L h − m / (ρ t)) / π), 21.50mm.  The scenario beside it reads the lightened web in plane
// stress under a load at its end, and claims how far the end sinks.
unit mm
use std

L := 160mm
h := 60mm
t := 5mm
P := 2kN
alloy := material(E: 70GPa, nu: 0.33, rho: 2700kg / 1m^3, yield: 240MPa)
param r: Length hint(21mm)

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
  hole := radius(r) circle(center: o) hint(r: 21)
  plate := face(low, end, top, root, holes: hole)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
web := solid(plate, from: -t / 2, to: t / 2) of alloy
mass(110g) web

scenario hung {
  plane stress web
  fixed web.root
  load web.end P along: down
  claim deflection(end_low, along: down) <= 1mm
}
