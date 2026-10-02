unit mm
o := point hint(x: 0, y: 0)
ground o
c := circle(center:o) hint(r:1)
radius(1mm) c
p := point hint(x:0.5,y:0.8660254037844386)
p on c
o distance(reach,along:x) p
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
otherfp0 := point hint(x: 5, y: 0)
ground otherfp0
otherfp1 := point hint(x: 6, y: 0)
ground otherfp1
otherfp2 := point hint(x: 6, y: 1)
ground otherfp2
otherfp3 := point hint(x: 5, y: 1)
ground otherfp3
otherf := face(otherfp0, otherfp1, otherfp2, otherfp3, -> close)
other := solid(otherf, from: -1mm, to: 0mm)
claim over reach in (2mm,3mm) { result clear(1mm) other }
