unit mm
resultfp0 := point hint(x: 1000000, y: 1000000)
ground resultfp0
resultfp1 := point hint(x: 1000000.1, y: 1000000)
ground resultfp1
resultfp2 := point hint(x: 1000000.1, y: 1000000.06)
ground resultfp2
resultfp3 := point hint(x: 1000000, y: 1000000.06)
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -0.04mm, to: 0mm)
