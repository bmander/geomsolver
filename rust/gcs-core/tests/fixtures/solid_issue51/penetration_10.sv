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
  resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
  result := solid(resultf, from: -10mm, to: 0mm)
  otherfp0 := point
  fix((5, 0)) otherfp0
  otherfp1 := point
  fix((15, 0)) otherfp1
  otherfp2 := point
  fix((15, 10)) otherfp2
  otherfp3 := point
  fix((5, 10)) otherfp3
  otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
  other := solid(otherf, from: -10mm, to: 0mm)
  claim result clear(0mm) other
}
