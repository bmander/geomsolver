unit mm
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
fix(x == 10, y == 10) otherfp0
otherfp1 := point
fix(x == 11, y == 10) otherfp1
otherfp2 := point
fix(x == 11, y == 11) otherfp2
otherfp3 := point
fix(x == 10, y == 11) otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -1mm, to: 0mm)
irrelevant := point
fix(x == 1000000000, y == 1000000000) irrelevant
claim result inside other
