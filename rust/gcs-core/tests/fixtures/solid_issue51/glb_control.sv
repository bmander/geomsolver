unit mm
use std
in std.front {
  resultfp0 := point
  fix(x == 0, y == 0) resultfp0
  resultfp1 := point
  fix(x == 10, y == 0) resultfp1
  resultfp2 := point
  fix(x == 10, y == 10) resultfp2
  resultfp3 := point
  fix(x == 0, y == 10) resultfp3
}
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -5mm, to: 0mm)
