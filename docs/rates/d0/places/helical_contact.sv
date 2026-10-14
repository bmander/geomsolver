// D0 (#339): the contact line of two helical gears on parallel axes, probed in today's spelling.
// The target is `../helical_contact.sv`.
//
// The body's algebraic rows are its surface of action: the contact point stays in the plane of
// action `P`, tangent to both base cylinders. Its place in `P` is the PDE's: the contact line at
// the home phase is the Cauchy data, and condition (3) of #326 moves it within `P`.
//   - `P` is drawn from the pinion's base cylinder and the transverse pressure angle, and claimed
//     tangent to the gear's base cylinder: a theorem (the centre distance is
//     (rb1 + rb2)/cos αt).
//   - At phi1 the closed form places the contact line: in `P`, at the base helix angle to the
//     axes, carried `rb1 · phi1` along the line of action from where it stood at the home (back
//     along `P.u`, for a pinion turning counter-clockwise about `std.z`). DOF 0.
// Closed form: the line translates along the line of action at rb1 a radian of the pinion; the
// pinion sees it sweep the involute helicoid of its base cylinder, lead angle 90° − βb.
unit mm
use std (on_u, on_v)

rb1 := 20mm
rb2 := 40mm
alpha := 25deg
centres := (rb1 + rb2) / cos(alpha)
beta := 15deg
phi1 := 20deg

c1 := point
c2 := point
fix((0mm, 0mm, 0mm)) c1
fix((centres, 0mm, 0mm)) c2
ax1 := axis
ax2 := axis
fix(dir == (0, 0, 1)) ax1
fix(dir == (0, 0, 1)) ax2
c1 coincident ax1
c2 coincident ax2

// the plane of action: along the axes, the transverse line of action at alpha to the line of
// centres, touching the pinion's base cylinder at its origin
P := plane hint(origin: (8, 18, 0))
P.v parallel std.z
P.u perpendicular std.z
P.origin coincident std.top
spoke := line(c1, P.origin)
spoke perpendicular P.u
length(rb1) spoke
centre_line := line(c1, c2)
P.u angle(90deg - alpha) centre_line
claim c2 distance(-rb2, along: n) P           // ...and the gear's

// the contact line at phi1
in P {
  m := point
  m on_u(d: -rb1 * phi1 / 1rad) P
  top := point
  top on_v(d: 10mm) P
  vref := line(P.origin, top)
  n := point
  g := line(m, n)
  length(30mm) g
  vref angle(beta) g
}
