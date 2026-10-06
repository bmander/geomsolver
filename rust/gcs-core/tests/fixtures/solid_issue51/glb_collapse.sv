unit mm
use std
in std.front {
  resultfp0 := point
  fix((1000000000, 1000000000)) resultfp0
  resultfp1 := point
  fix((1000000010, 1000000000)) resultfp1
  resultfp2 := point
  fix((1000000010, 1000000010)) resultfp2
  resultfp3 := point
  fix((1000000000, 1000000010)) resultfp3
}
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -5mm, to: 0mm)
