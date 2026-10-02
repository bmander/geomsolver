unit mm
resultfp0 := point hint(x: 0, y: 0)
ground resultfp0
resultfp1 := point hint(x: 10, y: 0)
ground resultfp1
resultfp2 := point hint(x: 10, y: 1)
ground resultfp2
resultfp3 := point hint(x: 0, y: 1)
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: 0mm, to: 1mm)
otherfp0 := point hint(x: 2, y: -3)
ground otherfp0
otherfp1 := point hint(x: 3, y: -3)
ground otherfp1
otherfp2 := point hint(x: 3, y: 7)
ground otherfp2
otherfp3 := point hint(x: 2, y: 7)
ground otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -0.5mm, to: 1.5mm)
claim result clear(0.1mm) other
