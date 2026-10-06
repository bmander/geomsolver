unit mm
use std
in std.front {
  fp0 := point
  fix((1e-05, 0)) fp0
  fp1 := point
  fix((1.4e-05, 0)) fp1
  fp2 := point
  fix((1.4e-05, 6e-06)) fp2
  fp3 := point
  fix((1e-05, 6e-06)) fp3
  f := face(fp0, fp1, fp2, fp3, -> close)
  a := point
  fix((0, 0)) a
  b := point
  fix((0, 1e-05)) b
  ax := line(a,b)
}
result := solid(f, about: ax)
