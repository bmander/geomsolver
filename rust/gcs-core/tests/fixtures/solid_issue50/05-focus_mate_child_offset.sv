unit mm
o := point hint(x: 0, y: 0)
ground o
q := point hint(x: 40, y: 0)
ground q
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front)
child := plane(origin: o, toward: q, from: back, offset: 3mm)
stockfp0 := point hint(x: 0, y: 0) in front
ground stockfp0
stockfp1 := point hint(x: 10, y: 0) in front
ground stockfp1
stockfp2 := point hint(x: 10, y: 10) in front
ground stockfp2
stockfp3 := point hint(x: 0, y: 10) in front
ground stockfp3
stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
stock := solid(stockf, from: -6mm, to: 0mm)
placedfp0 := point hint(x: 0, y: 0) in back
ground placedfp0
placedfp1 := point hint(x: 5, y: 0) in back
ground placedfp1
placedfp2 := point hint(x: 5, y: 5) in back
ground placedfp2
placedfp3 := point hint(x: 0, y: 5) in back
ground placedfp3
placedf := face(placedfp0, placedfp1, placedfp2, placedfp3, -> close)
placed := solid(placedf, from: -2mm, to: 0mm)
placed.far against stock.near
resultfp0 := point hint(x: 0, y: 0) in child
ground resultfp0
resultfp1 := point hint(x: 2, y: 0) in child
ground resultfp1
resultfp2 := point hint(x: 2, y: 2) in child
ground resultfp2
resultfp3 := point hint(x: 0, y: 2) in child
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -1mm, to: 0mm)
