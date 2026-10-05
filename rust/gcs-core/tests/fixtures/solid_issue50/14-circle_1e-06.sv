unit mm
use std
in std.front {
  fp := point
  fix(x == 0, y == 0) fp
  fc := circle(center: fp) hint(r: 1e-06)
  radius(1e-06mm) fc
}
f := face(fc)
result := solid(f, depth: 5mm)
