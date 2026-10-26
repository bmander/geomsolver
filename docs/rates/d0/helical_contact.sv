// D0 (#339): the contact line of two helical gears on parallel axes, in the proposed spelling.
// Does not parse: it stops at `initially`. D5's first closed form.
//
// #326's contact element over two formals, the phase `phi` (marched) and the place `sigma` along
// the line (sampled). The surface of action is stated: here the plane of action `P`, tangent to
// both base cylinders. The contact line at the home phase is the Cauchy data. Then #326's
// conditions:
//   (2) the normal `nl` is square to the line: `nl perpendicular rate(x over sigma)`;
//   (3) the pinion sees the contact slide on its own flank: x's rate as the pinion's turn carries
//       it, `rate(x over phi, seen_from: pinion)` = x_φ − v₁(x), is square to the normal;
//   the gauge: the line moves square to itself.
// (1) is not stated. With parallel axes the relative motion turns about the pitch line, where
// v₁₂ = 0, and every contact line crosses it: there `nl reciprocal(seen_from: pinion) gear`
// (#326 R2) says nothing, so a normal stated by (1) and (2) loses rank once a line. The normal is
// stated by the surface instead — in `P` — and (1) holds as a consequence (claimed).
//
// `seen_from:` on a rate is proposed here; it is #326 R4's carried point, differentiated, with
// no carried normal needed. The pinion's flank is the element as the pinion sees it (R4's
// carried point, `X coincident(under: pinion, t == -phi) x`).
//
// Closed form: the line translates along the line of action at rb1 a radian; the pinion's flank
// is the involute helicoid of its base cylinder, base helix angle 15°.
unit mm
use std
use std.place (on_v)
use std.motion (reciprocal)

rb1 := 20mm
rb2 := 40mm
alpha := 25deg
centres := (rb1 + rb2) / cos(alpha)

component Contact(P: plane, pinion: frame, gear: frame, home: line, phi: Angle,
                  sigma: Scalar) {
  x := point
  x coincident P                                           // the surface of action
  initially {
    x coincident(t == sigma) home                          // the Cauchy data
  }
  k := point
  nl := line(x, k)
  length(10mm) nl
  k coincident P                                           // the normal lies in P...
  nl perpendicular rate(x over sigma)                      // ...square to the line: (2)
  rate(x over phi, seen_from: pinion) perpendicular nl     // (3)
  rate(x over phi) perpendicular rate(x over sigma)        // the gauge
  claim nl reciprocal(seen_from: pinion) gear              // (1), as a consequence
  X := point
  X coincident(under: pinion, t == -phi) x                 // the contact as the pinion sees it
}

c1 := point
c2 := point
t1 := point
t2 := point
fix((0mm, 0mm, 0mm)) c1
fix((centres, 0mm, 0mm)) c2
fix((0mm, 0mm, 10mm)) t1
fix((centres, 0mm, 10mm)) t2
shaft1 := line(c1, t1)
shaft2 := line(c2, t2)
pinion := frame
std.Turn(pinion, about: shaft1, ratio: 1)
gear := frame
std.Turn(gear, about: shaft2, ratio: -rb1 / rb2)     // the mesh: `gear` seen from `pinion`

// the plane of action, as `places/helical_contact.sv` draws it, and the line at the home phase
P := plane hint(origin: (8, 18, 0))
P.v parallel std.z
P.u perpendicular std.z
P.origin coincident std.top
spoke := line(c1, P.origin)
spoke perpendicular P.u
length(rb1) spoke
centre_line := line(c1, c2)
P.u angle(90deg - alpha) centre_line
in P {
  top := point
  top on_v(d: 10mm) P
  vref := line(P.origin, top)
  e := point
  home := line(P.origin, e)
  length(30mm) home
  vref angle(15deg) home
}

contact := trace(Contact(P, pinion, gear, home).x over (phi in (-30deg, 30deg), sigma in (0, 1)))
pinion_flank := trace(Contact(P, pinion, gear, home).X
                        over (phi in (-30deg, 30deg), sigma in (0, 1)))
