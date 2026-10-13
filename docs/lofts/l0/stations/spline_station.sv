// L0 (#243): `OnSpline`'s station on a cubic Bézier guide at s = ½: the midpoint (12.5, 22.5),
// the plane square to the tangent (60, 40). One dependency (README, finding 5): the tangency
// pinned at `s` restates the point already on the curve.
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
