unit mm
use std
use std.solids
in std.front {
  fp := point
  fix((0, 0)) fp
  fc := circle(center: fp) hint(r: 1e-06)
  radius(1e-06mm) fc
}
f := face(fc)
result := solid(std.solids.Prism(f, from: -5mm, to: 0))
