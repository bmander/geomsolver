// A portal frame sized by its sag (#510 S1): two posts built in at their feet and a beam across
// their heads, all one round tube of wall `t`, its outside diameter `d` the unknown.  A load hangs
// from the middle of the beam — a point the scenario names on the member, which cuts it into a
// joint there — and the frame carries its own weight, a body load that grows with the tube.  The
// middle's deflection is a driving requirement, so the solve finds the diameter at which it sinks
// `sag`.  The section is no affine function of `d` — a tube's area and second moments are
// `π (d² − (d − 2t)²) / 4` and `π (d⁴ − (d − 2t)⁴) / 64` — and each is read as a formula over
// `d`, its rate carried to `d` by the formula's own.
unit mm
use std

H := 3000mm
W := 4000mm
t := 8mm
P := 20kN
sag := 2mm
steel := material(E: 210GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 355MPa)
param d: Length hint(200mm)

in std.front {
  left_foot := point
  left_head := point
  right_head := point
  right_foot := point
  middle := point
  fix((0mm, 0mm)) left_foot
  fix((0mm, H)) left_head
  fix((W, H)) right_head
  fix((W, 0mm)) right_foot
  fix((W / 2, H)) middle
  left := line(left_foot, left_head)
  top := line(left_head, right_head)
  right := line(right_head, right_foot)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
inner := d - 2 * t
portal := beam(left, top, right, A: pi * (d^2 - inner^2) / 4, Iy: pi * (d^4 - inner^4) / 64,
               Iz: pi * (d^4 - inner^4) / 64, J: pi * (d^4 - inner^4) / 32,
               Sy: pi * (d^4 - inner^4) / (32 * d), Sz: pi * (d^4 - inner^4) / (32 * d)) of steel

scenario roof {
  fixed left_foot
  fixed right_foot
  load middle P along: down
  accelerate portal (0, 0, -9.81 * 1m / 1s^2)
  deflection(middle, along: down) == sag
  claim safety(portal) >= 2
}
