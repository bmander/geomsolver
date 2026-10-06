unit mm
use std
param reach: Length
in std.front {
  o := point
  fix((0, 0)) o
  c := circle(center:o) hint(r: 1)
  radius(1mm) c
  p := point hint((0.5, 0.8660254037844386))
  p coincident c
  o distance(reach,along:x) p
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
  fix((5, 0)) otherfp0
  otherfp1 := point
  fix((6, 0)) otherfp1
  otherfp2 := point
  fix((6, 1)) otherfp2
  otherfp3 := point
  fix((5, 1)) otherfp3
}
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -1mm, to: 0mm)
claim over reach in (2mm,3mm) { result clear(1mm) other }
