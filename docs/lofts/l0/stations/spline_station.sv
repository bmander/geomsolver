// L0 (#243): `OnSpline`'s station on a cubic Bézier guide at s = ½: the midpoint (12.5, 22.5),
// the plane square to the tangent (60, 40). DOF 0 (README, finding 5, #251): the tangent at `m`
// reads the place of the contact holding `m` on the curve, where `g tangent(t == s) tl` restated
// the point and left one dependency.
unit mm
use std
use lofts
in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((0mm, 20mm)) k1
  fix((20mm, 30mm)) k2
  fix((40mm, 30mm)) k3
  g := spline(k0, k1, k2, k3)
}
st := lofts.OnSpline(g, std.z, s: 0.5)
