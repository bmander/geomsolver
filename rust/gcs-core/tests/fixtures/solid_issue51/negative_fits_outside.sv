unit mm
use std
in std.front {
  resultfp0 := point
  fix(x == 0, y == 0) resultfp0
  resultfp1 := point
  fix(x == 1, y == 0) resultfp1
  resultfp2 := point
  fix(x == 1, y == 1) resultfp2
  resultfp3 := point
  fix(x == 0, y == 1) resultfp3
  resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
  result := solid(resultf, from: -1mm, to: 0mm)
  otherfp0 := point
  fix(x == 2, y == 0) otherfp0
  otherfp1 := point
  fix(x == 3, y == 0) otherfp1
  otherfp2 := point
  fix(x == 3, y == 1) otherfp2
  otherfp3 := point
  fix(x == 2, y == 1) otherfp3
  otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
  other := solid(otherf, from: -1mm, to: 0mm)
  claim result fits(-2mm) other
  claim result inside other
}
