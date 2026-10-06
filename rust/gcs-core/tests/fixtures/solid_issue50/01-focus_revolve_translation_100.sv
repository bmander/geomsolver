unit mm
use std
in std.front {
  fp0 := point
  fix((90, 100)) fp0
  fp1 := point
  fix((86, 100)) fp1
  fp2 := point
  fix((86, 106)) fp2
  fp3 := point
  fix((90, 106)) fp3
  f := face(fp0, fp1, fp2, fp3, -> close)
  a := point
  fix((100, 100)) a
  b := point
  fix((100, 110)) b
  ax := line(a,b)
}
result := solid(f, about: ax, sweep: 90deg)
