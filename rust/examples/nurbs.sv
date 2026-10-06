// Rational splines: a weight on each control point, and a cubic can be a circle.
//
// An ordinary spline is a polynomial in its parameter, and no polynomial is round: however its
// control points are placed, a cubic only approximates a quarter circle. Give each control point a
// weight and the curve becomes the ratio of two cubics, which is enough to be a conic exactly. A
// heavier point pulls the curve towards itself; for a quarter circle the two inner points weigh
// (1 + √2) / 3 each, against 1 at the ends.
//
// Weights are document data, like the knots: written after the control points, never solved for.
// So the curve stays a sum of its control points times fixed functions of its parameter, and every
// contact with it — a point on it, a line tangent to it, a circle of curvature — works unchanged.
unit mm
use std

r := 30mm
// the inner control points stand where the end tangents of a quarter circle meet a cubic's thirds
inner := (2 - sqrt(2)) * r
heavy := (1 + sqrt(2)) / 3

in std.front {
  // the weighted quarter: four control points, the inner two heavy
  o := point
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) o
  fix((r, 0mm)) k0
  fix((r, inner)) k1
  fix((inner, r)) k2
  fix((0mm, r)) k3
  quarter := spline(k0, k1, k2, k3) weights [1, heavy, heavy, 1]

  // a bead riding the curve, free to slide along it: wherever it goes it is `r` from the centre,
  // and the claim says so — proved, because the curve is the circle, not sampled near it
  bead := point hint((22mm, 20mm))
  bead coincident quarter
  claim bead distance(r) o

  // the same control points with no weights: the polynomial cubic swells past the circle, and the
  // same claim about a bead on it is refuted
  p := point
  m0 := point
  m1 := point
  m2 := point
  m3 := point
  fix((50mm, 0mm)) p
  fix((50mm + r, 0mm)) m0
  fix((50mm + r, inner)) m1
  fix((50mm + inner, r)) m2
  fix((50mm, r)) m3
  plain := spline(m0, m1, m2, m3)
  slider := point hint((72mm, 20mm))
  slider coincident plain
  claim slider distance(r) p

  // the dome: the weighted quarter and its two radii, turned about the upright one — a hemisphere,
  // ⅔ π r³ to the last digit, where the polynomial quarter would turn a little more
  floor := line(o, k0)
  construction centerline upright := line(k3, o)
}
dome := solid(face(floor, quarter, upright), about: upright)
