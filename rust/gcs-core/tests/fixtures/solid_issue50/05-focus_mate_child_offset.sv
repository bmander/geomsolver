unit mm
point o hint(x: 0, y: 0)
ground o
point q hint(x: 40, y: 0)
ground q
plane front(origin: o, toward: q)
plane back(origin: o, toward: q, from: front)
plane child(origin: o, toward: q, from: back, offset: 3mm)
point stockfp0 hint(x: 0, y: 0) in front
ground stockfp0
point stockfp1 hint(x: 10, y: 0) in front
ground stockfp1
point stockfp2 hint(x: 10, y: 10) in front
ground stockfp2
point stockfp3 hint(x: 0, y: 10) in front
ground stockfp3
face stockf(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
solid stock(stockf, from: -6mm, to: 0mm)
point placedfp0 hint(x: 0, y: 0) in back
ground placedfp0
point placedfp1 hint(x: 5, y: 0) in back
ground placedfp1
point placedfp2 hint(x: 5, y: 5) in back
ground placedfp2
point placedfp3 hint(x: 0, y: 5) in back
ground placedfp3
face placedf(placedfp0, placedfp1, placedfp2, placedfp3, -> close)
solid placed(placedf, from: -2mm, to: 0mm)
placed.far against stock.near
point resultfp0 hint(x: 0, y: 0) in child
ground resultfp0
point resultfp1 hint(x: 2, y: 0) in child
ground resultfp1
point resultfp2 hint(x: 2, y: 2) in child
ground resultfp2
point resultfp3 hint(x: 0, y: 2) in child
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -1mm, to: 0mm)
