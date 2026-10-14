// D0 (#339): the contact line of two bevel gears on intersecting axes, probed in today's
// spelling. The target is `../spherical_contact.sv`.
//
// The body's algebraic rows are its surface of action: the contact line stays in the plane of
// action `P`, through the apex and tangent to both base cones. Its place in `P` is the PDE's: the
// line at the home phase is the Cauchy data, and condition (3) of #326 turns it about the apex.
//   - `P` is drawn through the pitch line (where the pitch cones touch), tangent to the pinion's
//     base cone, and claimed tangent to the gear's: a theorem (sin δbᵢ = sin δᵢ cos α, the
//     spherical involute's base cones).
//   - At phi1 the closed form places the contact line: in `P`, through the apex, turned
//     phi1 · sin δb1 from the pitch line. DOF 0.
// Closed form: the line turns about the apex at sin δb1 a radian of the pinion; the pinion sees
// it sweep the cone over the spherical involute of its base cone.
unit mm
use std (on_u)

z1 := 20
z2 := 40
alpha := 20deg
delta1 := atan(z1 / z2)
delta2 := 90deg - delta1
base1 := asin(sin(delta1) * cos(alpha))
base2 := asin(sin(delta2) * cos(alpha))
phi1 := 30deg

o := point
fix((0mm, 0mm, 0mm)) o
ax1 := axis
ax2 := axis
fix(dir == (1, 0, 0), origin == (0, 0, 0)) ax1
fix(dir == (0, 0, 1), origin == (0, 0, 0)) ax2
pitch := axis
fix(dir == (cos(delta1), 0, sin(delta1)), origin == (0, 0, 0)) pitch

// the plane of action: through the pitch line, tangent to the pinion's base cone
P := plane(u: pitch) hint(origin: (0, 0, 0))
P.origin level(std.x) o
P.v perpendicular P.u
P.normal angle(90deg - base1) ax1
claim P.normal angle(90deg + base2) ax2       // ...and to the gear's (its normal turned from it)

// the contact line at phi1, through the apex
in P {
  k := point
  ref := line(P.origin, k)
  k on_u(d: 10mm) P
  e := point
  g := line(P.origin, e)
  length(30mm) g
  ref angle(phi1 * sin(base1)) g
}
