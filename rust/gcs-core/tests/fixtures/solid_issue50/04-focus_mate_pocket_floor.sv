unit mm
o := point
fix(x == 0, y == 0) o
q := point
fix(x == 40, y == 0) q
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front)
stockfp0 := point in front
fix(x == 0, y == 0) stockfp0
stockfp1 := point in front
fix(x == 10, y == 0) stockfp1
stockfp2 := point in front
fix(x == 10, y == 10) stockfp2
stockfp3 := point in front
fix(x == 0, y == 10) stockfp3
stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
stock := solid(stockf, from: -6mm, to: 0mm)
toolfp0 := point in front
fix(x == 2, y == 2) toolfp0
toolfp1 := point in front
fix(x == 8, y == 2) toolfp1
toolfp2 := point in front
fix(x == 8, y == 8) toolfp2
toolfp3 := point in front
fix(x == 2, y == 8) toolfp3
toolf := face(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
tool := solid(toolf, from: -3mm, to: 0mm)
body := solid(stock)
tool cut body
resultfp0 := point in back
fix(x == 3, y == 3) resultfp0
resultfp1 := point in back
fix(x == 5, y == 3) resultfp1
resultfp2 := point in back
fix(x == 5, y == 5) resultfp2
resultfp3 := point in back
fix(x == 3, y == 5) resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -1mm, to: 0mm)
result.far against body.tool.far
