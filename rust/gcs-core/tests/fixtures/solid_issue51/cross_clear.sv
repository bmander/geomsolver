unit mm
resultfp0 := point
fix(x == 0, y == 0) resultfp0
resultfp1 := point
fix(x == 10, y == 0) resultfp1
resultfp2 := point
fix(x == 10, y == 1) resultfp2
resultfp3 := point
fix(x == 0, y == 1) resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: 0mm, to: 1mm)
otherfp0 := point
fix(x == 2, y == -3) otherfp0
otherfp1 := point
fix(x == 3, y == -3) otherfp1
otherfp2 := point
fix(x == 3, y == 7) otherfp2
otherfp3 := point
fix(x == 2, y == 7) otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -0.5mm, to: 1.5mm)
claim result clear(0.1mm) other
