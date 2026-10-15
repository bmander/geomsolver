// D0 (#339): the contact line of two bevel gears on intersecting axes, in the proposed spelling.
// Does not parse: it stops at `initially`. D5's second closed form.
//
// `helical_contact.sv`'s element with the axes meeting at the apex: the surface of action is the
// plane of action through the apex, tangent to both base cones, and the contact line at the home
// phase is the pitch line. The relative motion turns about the pitch line, so v₁₂ = 0 along the
// whole home line: (1) says nothing anywhere on it, and the normal is again stated by the surface.
//
// Closed form: the line turns about the apex at sin δb1 a radian of the pinion (the plane of
// action rolls on the base cone); the pinion's flank is the cone over the spherical involute of
// its base cone.
unit mm
use std (on_u)

z1 := 20
z2 := 40
alpha := 20deg
delta1 := atan(z1 / z2)
base1 := asin(sin(delta1) * cos(alpha))

component Contact(P: plane, pinion: motion, mesh: motion, home: line, phi: Angle,
                  sigma: Scalar) {
  x := point
  x coincident P
  initially {
    x coincident(t == sigma) home
  }
  k := point
  nl := line(x, k)
  length(10mm) nl
  k coincident P
  nl perpendicular rate(x over sigma)
  rate(x over phi, relative_to: pinion) perpendicular nl
  rate(x over phi) perpendicular rate(x over sigma)
  claim nl reciprocal mesh
  X := point
  X coincident x under(pinion, at: -phi)
}

o := point
fix((0mm, 0mm, 0mm)) o
x1 := point
z2p := point
fix((10mm, 0mm, 0mm)) x1
fix((0mm, 0mm, 10mm)) z2p
shaft1 := line(o, x1)
shaft2 := line(o, z2p)
pinion := motion(about: shaft1)
gear := motion(about: shaft2, ratio: -z1 / z2)
mesh := motion(gear, relative_to: pinion)

// the plane of action, as `places/spherical_contact.sv` draws it; the home line is the pitch
// line from 30 to 50 from the apex
pitch := axis
fix(dir == (cos(delta1), 0, sin(delta1)), origin == (0, 0, 0)) pitch
P := plane(u: pitch) hint(origin: (0, 0, 0))
P.origin level(std.x) o
P.v perpendicular P.u
P.normal angle(90deg - base1) std.x
in P {
  i := point
  j := point
  i on_u(d: 30mm) P
  j on_u(d: 50mm) P
  home := line(i, j)
}

contact := Contact(P, pinion, mesh, home).x over (phi in (-30deg, 30deg), sigma in (0, 1))
pinion_flank := Contact(P, pinion, mesh, home).X over (phi in (-30deg, 30deg), sigma in (0, 1))
