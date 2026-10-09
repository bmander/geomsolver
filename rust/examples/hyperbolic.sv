// The shortest path in the hyperbolic plane, drawn in Poincaré's half-plane: above the line
// `y = 0`, length is measured as `ds / y`, so distances grow without bound toward the line. The
// curve from `a` to `b` that makes that length least is an arc of a circle centred on the line —
// the plane's straight line — found here from the principle alone.
//
// Nothing states how long it is. Drag `b`: the arc swings round, always meeting the line square on.
unit mm
use std

in std.front {
  a := point
  b := point hint((60, 20))
  fix((-30mm, 20mm)) a
  path := curve(a, b)
}
path minimizes integral(10mm / p.y over p)
