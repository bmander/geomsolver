unit mm
use std
in std.front {
  fp0 := point
  fix(x == 1e-05, y == 0) fp0
  fp1 := point
  fix(x == 1.4e-05, y == 0) fp1
  fp2 := point
  fix(x == 1.4e-05, y == 6e-06) fp2
  fp3 := point
  fix(x == 1e-05, y == 6e-06) fp3
  f := face(fp0, fp1, fp2, fp3, -> close)
  a := point
  fix(x == 0, y == 0) a
  b := point
  fix(x == 0, y == 1e-05) b
  ax := line(a,b)
}
result := solid(f, about: ax)
