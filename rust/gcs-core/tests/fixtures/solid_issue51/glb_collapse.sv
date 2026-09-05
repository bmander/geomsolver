unit mm
point resultfp0 hint(x: 1000000000, y: 1000000000)
ground resultfp0
point resultfp1 hint(x: 1000000010, y: 1000000000)
ground resultfp1
point resultfp2 hint(x: 1000000010, y: 1000000010)
ground resultfp2
point resultfp3 hint(x: 1000000000, y: 1000000010)
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -5mm, to: 0mm)
