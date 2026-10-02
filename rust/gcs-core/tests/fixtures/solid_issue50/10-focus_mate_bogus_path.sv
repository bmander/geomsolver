unit mm
o := point hint(x: 0, y: 0)
ground o
q := point hint(x: 40, y: 0)
ground q
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front)
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
resultfp0 := point hint(x: 0, y: 0) in back
ground resultfp0
resultfp1 := point hint(x: 5, y: 0) in back
ground resultfp1
resultfp2 := point hint(x: 5, y: 5) in back
ground resultfp2
resultfp3 := point hint(x: 0, y: 5) in back
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -2mm, to: 0mm)
result.nonsense.far against stock.typo.near
