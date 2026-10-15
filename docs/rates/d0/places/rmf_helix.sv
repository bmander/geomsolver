// D0 (#339): a rotation-minimising frame along a helix, probed in today's spelling. The target is
// `../rmf_helix.sv`.
//
// The body without its rate row is `Square`: the plane `f` square to the helix at `phi` — its
// origin on the helix, both its axes square to the tangent `tg`, and to each other. It leaves one
// freedom at every place, the frame's turn about the tangent, which the rate row
// `rate(f.u over phi) perpendicular f.v` supplies away from the home.
//   - At the home (phi = 0) the drawing holds the turn: `f.u` level, so along the radius (the
//     reverse of Frenet's normal). DOF 0; delete that line and it is DOF 1.
//   - At phi1 the closed form gives the turn: the frame has fallen behind Frenet's by
//     τ·(arc length) = phi1 · b/√(a² + b²), measured unsigned against the radius (an angle in
//     space is) with a bound for the side: `f.u` rises. DOF 0, at the frame the rate rows
//     integrate to (`experiments.py`, E4).
// The helix and its tangent are written out (`fix` reading `phi`) rather than through
// `std.OnHelix`, which drawn by itself stands square to the mirror helix (see README.md).
// Closed form: per turn the frame falls behind Frenet's by 2πb/√(a² + b²) (b the rise a radian).
unit mm
use std

a := 10mm
pitch := 5mm
b := pitch / (2 * pi)
lead := atan(b / a)
phi1 := 270deg

component Square(a: Length, b: Length, phi: Angle) {
  m := point
  foot := point
  fix((a * cos(phi), a * sin(phi), b * phi / 1rad)) m
  fix((0mm, 0mm, b * phi / 1rad)) foot
  rad := line(foot, m)
  tg := axis
  fix(dir == (-sin(phi) * cos(atan(b / a)), cos(phi) * cos(atan(b / a)), sin(atan(b / a)))) tg
  tg coincident m
  f := plane hint(origin: (a * cos(phi), a * sin(phi), b * phi / 1rad))
  f.origin coincident m
  f.u perpendicular tg
  f.v perpendicular tg
  f.u perpendicular f.v
}

// the home: the drawn instance, and what holds there alone
home := Square(a: a, b: b, phi: 0deg)
home.f.u perpendicular std.z

// the place phi1, at the closed form's turn
place := Square(a: a, b: b, phi: phi1)
place.f.u angle(phi1 * b / sqrt(a^2 + b^2)) place.rad
zp := point
fix((a * cos(phi1), a * sin(phi1), b * phi1 / 1rad + 10mm)) zp
place.m distance(>= 0, along: place.f.u) zp
