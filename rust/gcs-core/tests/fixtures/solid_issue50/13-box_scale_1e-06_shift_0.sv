unit mm
point resultfp0 hint(x: 0, y: 0)
ground resultfp0
point resultfp1 hint(x: 1e-05, y: 0)
ground resultfp1
point resultfp2 hint(x: 1e-05, y: 6e-06)
ground resultfp2
point resultfp3 hint(x: 0, y: 6e-06)
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -4e-06mm, to: 0mm)
