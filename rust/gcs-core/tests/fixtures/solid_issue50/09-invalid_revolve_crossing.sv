unit mm
use std
in std.front {
  fp0 := point
  fix((0, 0)) fp0
  fp1 := point
  fix((10, 0)) fp1
  fp2 := point
  fix((10, 10)) fp2
  fp3 := point
  fix((0, 10)) fp3
  f := face(fp0, fp1, fp2, fp3, -> close)
  a := point
  fix((5, 0)) a
  b := point
  fix((5, 10)) b
  ax := line(a,b)
}
result := solid(f, about: ax)
