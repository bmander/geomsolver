unit mm
use std
in std.front {
  resultfp0 := point
  fix((0, 0)) resultfp0
  resultfp1 := point
  fix((1, 0)) resultfp1
  resultfp2 := point
  fix((1, 1)) resultfp2
  resultfp3 := point
  fix((0, 1)) resultfp3
  resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
  result := solid(resultf, from: -1mm, to: 0mm)
  otherfp0 := point
  fix((10, 10)) otherfp0
  otherfp1 := point
  fix((11, 10)) otherfp1
  otherfp2 := point
  fix((11, 11)) otherfp2
  otherfp3 := point
  fix((10, 11)) otherfp3
  otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
  other := solid(otherf, from: -1mm, to: 0mm)
  irrelevant := point
  fix((0, 0)) irrelevant
  claim result inside other
}
