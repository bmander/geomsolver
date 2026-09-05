unit mm
point o hint(x: 0, y: 0)
ground o
point q hint(x: 40, y: 0)
ground q
plane front(origin: o, toward: q)
plane back(origin: o, toward: q, from: front)
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
point toolfp0 hint(x: 2, y: 2) in front
ground toolfp0
point toolfp1 hint(x: 8, y: 2) in front
ground toolfp1
point toolfp2 hint(x: 8, y: 8) in front
ground toolfp2
point toolfp3 hint(x: 2, y: 8) in front
ground toolfp3
face toolf(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
solid tool(toolf, from: -3mm, to: 0mm)
solid body(stock)
tool through body
point resultfp0 hint(x: 3, y: 3) in back
ground resultfp0
point resultfp1 hint(x: 5, y: 3) in back
ground resultfp1
point resultfp2 hint(x: 5, y: 5) in back
ground resultfp2
point resultfp3 hint(x: 3, y: 5) in back
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -1mm, to: 0mm)
result.far against body.tool.far
