unit mm
point o hint(x: 0, y: 0)
ground o
point q hint(x: 40, y: 0)
ground q
plane front(origin:o,toward:q)
point resultfp0 hint(x: 0, y: 0) in front
ground resultfp0
point resultfp1 hint(x: 10, y: 0) in front
ground resultfp1
point resultfp2 hint(x: 10, y: 10) in front
ground resultfp2
point resultfp3 hint(x: 0, y: 10) in front
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -1mm, to: 0mm)
view(result) in front
