unit mm
point resultfp0 hint(x: 1000000, y: 1000000)
ground resultfp0
point resultfp1 hint(x: 1000000.1, y: 1000000)
ground resultfp1
point resultfp2 hint(x: 1000000.1, y: 1000000.06)
ground resultfp2
point resultfp3 hint(x: 1000000, y: 1000000.06)
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -0.04mm, to: 0mm)
