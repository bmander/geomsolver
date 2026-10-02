unit mm
resultfp0 := point
fix(x == 1000000, y == 1000000) resultfp0
resultfp1 := point
fix(x == 1000000.1, y == 1000000) resultfp1
resultfp2 := point
fix(x == 1000000.1, y == 1000000.06) resultfp2
resultfp3 := point
fix(x == 1000000, y == 1000000.06) resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -0.04mm, to: 0mm)
