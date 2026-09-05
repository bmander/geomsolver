unit mm
point resultfp0 hint(x: 0, y: 0)
ground resultfp0
point resultfp1 hint(x: 1, y: 0)
ground resultfp1
point resultfp2 hint(x: 1, y: 1)
ground resultfp2
point resultfp3 hint(x: 0, y: 1)
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -1mm, to: 0mm)
point otherfp0 hint(x: 2, y: 0)
ground otherfp0
point otherfp1 hint(x: 3, y: 0)
ground otherfp1
point otherfp2 hint(x: 3, y: 1)
ground otherfp2
point otherfp3 hint(x: 2, y: 1)
ground otherfp3
face otherf(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
solid other(otherf, from: -1mm, to: 0mm)
claim result fits(-2mm) other
claim result inside other
