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
toolfp0 := point hint(x: 2, y: 2) in front
ground toolfp0
toolfp1 := point hint(x: 8, y: 2) in front
ground toolfp1
toolfp2 := point hint(x: 8, y: 8) in front
ground toolfp2
toolfp3 := point hint(x: 2, y: 8) in front
ground toolfp3
toolf := face(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
tool := solid(toolf, from: -3mm, to: 0mm)
body := solid(stock)
tool cut body
resultfp0 := point hint(x: 3, y: 3) in back
ground resultfp0
resultfp1 := point hint(x: 5, y: 3) in back
ground resultfp1
resultfp2 := point hint(x: 5, y: 5) in back
ground resultfp2
resultfp3 := point hint(x: 3, y: 5) in back
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -1mm, to: 0mm)
result.far against body.tool.far
