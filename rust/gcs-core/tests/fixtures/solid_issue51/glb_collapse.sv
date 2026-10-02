unit mm
resultfp0 := point hint(x: 1000000000, y: 1000000000)
ground resultfp0
resultfp1 := point hint(x: 1000000010, y: 1000000000)
ground resultfp1
resultfp2 := point hint(x: 1000000010, y: 1000000010)
ground resultfp2
resultfp3 := point hint(x: 1000000000, y: 1000000010)
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -5mm, to: 0mm)
