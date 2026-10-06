unit mm
use std
in std.front {
  resultfp0 := point
  fix((0, 0)) resultfp0
  resultfp1 := point
  fix((10, 0)) resultfp1
  resultfp2 := point
  fix((10, 10)) resultfp2
  resultfp3 := point
  fix((0, 10)) resultfp3
}
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -5mm, to: 0mm)
