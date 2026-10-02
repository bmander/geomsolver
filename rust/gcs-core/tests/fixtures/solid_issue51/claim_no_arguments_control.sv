unit mm
resultfp0 := point hint(x: 0, y: 0)
ground resultfp0
resultfp1 := point hint(x: 1, y: 0)
ground resultfp1
resultfp2 := point hint(x: 1, y: 1)
ground resultfp2
resultfp3 := point hint(x: 0, y: 1)
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -1mm, to: 0mm)
otherfp0 := point hint(x: 0, y: 0)
ground otherfp0
otherfp1 := point hint(x: 1, y: 0)
ground otherfp1
otherfp2 := point hint(x: 1, y: 1)
ground otherfp2
otherfp3 := point hint(x: 0, y: 1)
ground otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -1mm, to: 0mm)
claim result inside other
