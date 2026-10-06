unit mm
use std
in std.front {
  resultfp0 := point
  fix((0, 0)) resultfp0
  resultfp1 := point
  fix((10, 0)) resultfp1
  resultfp2 := point
  fix((10, 1)) resultfp2
  resultfp3 := point
  fix((0, 1)) resultfp3
  resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
  result := solid(resultf, from: 0mm, to: 1mm)
  otherfp0 := point
  fix((2, -3)) otherfp0
  otherfp1 := point
  fix((3, -3)) otherfp1
  otherfp2 := point
  fix((3, 7)) otherfp2
  otherfp3 := point
  fix((2, 7)) otherfp3
  otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
  other := solid(otherf, from: -0.5mm, to: 1.5mm)
  claim result clear(0.1mm) other
}
