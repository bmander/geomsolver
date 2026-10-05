unit mm
param reach: Length
o := point
fix(x == 0, y == 0) o
c := circle(center:o) hint(r:1)
radius(1mm) c
p := point hint(x:0.5,y:0.8660254037844386)
p coincident c
o distance(reach,along:x) p
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
fix(x == 5, y == 0) otherfp0
otherfp1 := point
fix(x == 6, y == 0) otherfp1
otherfp2 := point
fix(x == 6, y == 1) otherfp2
otherfp3 := point
fix(x == 5, y == 1) otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -1mm, to: 0mm)
claim over reach in (2deg,3deg) { result clear(1mm) other }
