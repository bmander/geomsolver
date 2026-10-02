unit mm
resultfp0 := point
fix(x == 0, y == 0) resultfp0
resultfp1 := point
fix(x == 10, y == 0) resultfp1
resultfp2 := point
fix(x == 10, y == 10) resultfp2
resultfp3 := point
fix(x == 0, y == 10) resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -10mm, to: 0mm)
otherfp0 := point
fix(x == 5, y == 0) otherfp0
otherfp1 := point
fix(x == 15, y == 0) otherfp1
otherfp2 := point
fix(x == 15, y == 10) otherfp2
otherfp3 := point
fix(x == 5, y == 10) otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -10mm, to: 0mm)
claim result clear(0mm) other
