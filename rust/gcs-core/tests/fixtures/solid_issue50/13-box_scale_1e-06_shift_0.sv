unit mm
use std
in std.front {
  resultfp0 := point
  fix((0, 0)) resultfp0
  resultfp1 := point
  fix((1e-05, 0)) resultfp1
  resultfp2 := point
  fix((1e-05, 6e-06)) resultfp2
  resultfp3 := point
  fix((0, 6e-06)) resultfp3
}
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -4e-06mm, to: 0mm)
