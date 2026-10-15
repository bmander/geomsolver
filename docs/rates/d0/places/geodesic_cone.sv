// D0 (#339): a geodesic on a cone, probed in today's spelling. The target is
// `../geodesic_cone.sv`.
//
// The body without its rate rows is `OnCone`: `p` on the cone (its generator `gen` at the half
// angle to the axis), the cone's normal `nl` at `p` (square to the generator, through the axis,
// as every normal of a surface of revolution is), and the tangent `tl`, 10 long, square to it.
// It leaves three freedoms at every place — where `p` is on the cone and which way `tl` points in
// the tangent plane — which the rate rows supply away from the home: `p` runs along `tl` at unit
// speed, and `tl` turns only towards the cone's normal.
//   - At the home (s = 0) the drawing holds them: `p` on the x–z plane, 40 from the apex, `tl`
//     horizontal. DOF 0; delete the three home lines and it is DOF 3.
//   - At s1 the closed form places `p` (at ρ = √(40² + s1²), turned ψ/sin α round the axis,
//     ψ = atan(s1/40)) and `tl` (at acos(s1/ρ) to the generator). DOF 0.
// Closed form: unrolled, the cone is a plane and the geodesic the straight line ρ cos ψ = 40.
unit mm
use std

half := 30deg
r0 := 40mm
s1 := 30mm
rho1 := sqrt(r0^2 + s1^2)
phi1 := atan(s1 / r0) / sin(half)

component OnCone(o: point, ax: axis, half: Angle, s: Length) {
  p := point hint((20, 0, 35))
  q := point hint((20, 10, 35))
  w := point hint((0, 0, 46))
  gen := line(o, p)
  tl := line(p, q)
  nl := line(p, w)
  gen angle(half) ax
  w coincident ax
  nl perpendicular gen
  length(10mm) tl
  tl perpendicular nl
}

o := point
fix((0mm, 0mm, 0mm)) o

// the home: the drawn instance, and what holds there alone
home := OnCone(o, std.z, half: half, s: 0mm)
home.p level(std.y) o
o distance(r0) home.p
home.tl perpendicular std.z

// the place s1, at the closed form
place := OnCone(o, std.z, half: half, s: s1)
o distance(rho1 * sin(half) * cos(phi1), along: std.x) place.p
o distance(rho1 * sin(half) * sin(phi1), along: std.y) place.p
place.tl angle(acos(s1 / rho1)) place.gen
