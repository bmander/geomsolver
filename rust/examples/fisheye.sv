// Maxwell's fish-eye: a lens whose refractive index falls off from its centre as
// `2 / (1 + r^2 / R^2)`. Light takes the path of least optical length (Fermat's principle), so a
// ray from `a` to `b` is the curve that makes the index integrated along it least — and in this
// lens every such ray is an arc of a circle, and every ray from one point meets again at the
// point opposite it (the lens images perfectly).
//
// Nothing states how long the ray is: its length is where the optical length is stationary in
// it too. Drag `b` and the ray bends to it, still a circle; the report calls it a minimum.
unit mm
use std

R := 50mm

in std.front {
  a := point
  b := point hint((40, 30))
  fix((-50mm, 0mm)) a
  ray := curve(a, b)
}
ray minimizes integral(2 / (1 + (p.x^2 + p.y^2) / R^2) over p)
