unit mm
use std
in std.front {
  fp0 := point
  fix(x == 90, y == 100) fp0
  fp1 := point
  fix(x == 86, y == 100) fp1
  fp2 := point
  fix(x == 86, y == 106) fp2
  fp3 := point
  fix(x == 90, y == 106) fp3
  f := face(fp0, fp1, fp2, fp3, -> close)
  a := point
  fix(x == 100, y == 100) a
  b := point
  fix(x == 100, y == 110) b
  ax := line(a,b)
}
result := solid(f, about: ax, sweep: 90deg)
