unit mm
o := point
fix(x == 0, y == 0) o
q := point
fix(x == 40, y == 0) q
front := plane(origin: o, toward: q)
back := plane(origin: o, toward: q, from: front)
child := plane(origin: o, toward: q, from: back, offset: 3mm)
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
placedfp0 := point in back
fix(x == 0, y == 0) placedfp0
placedfp1 := point in back
fix(x == 5, y == 0) placedfp1
placedfp2 := point in back
fix(x == 5, y == 5) placedfp2
placedfp3 := point in back
fix(x == 0, y == 5) placedfp3
placedf := face(placedfp0, placedfp1, placedfp2, placedfp3, -> close)
placed := solid(placedf, from: -2mm, to: 0mm)
placed.far against stock.near
resultfp0 := point in child
fix(x == 0, y == 0) resultfp0
resultfp1 := point in child
fix(x == 2, y == 0) resultfp1
resultfp2 := point in child
fix(x == 2, y == 2) resultfp2
resultfp3 := point in child
fix(x == 0, y == 2) resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -1mm, to: 0mm)
