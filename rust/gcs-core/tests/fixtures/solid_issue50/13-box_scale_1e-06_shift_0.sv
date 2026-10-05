unit mm
use std
in std.front {
  resultfp0 := point
  fix(x == 0, y == 0) resultfp0
  resultfp1 := point
  fix(x == 1e-05, y == 0) resultfp1
  resultfp2 := point
  fix(x == 1e-05, y == 6e-06) resultfp2
  resultfp3 := point
  fix(x == 0, y == 6e-06) resultfp3
}
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -4e-06mm, to: 0mm)
