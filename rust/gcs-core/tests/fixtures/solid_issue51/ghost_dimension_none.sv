unit mm
point o hint(x: 0, y: 0)
ground o
point q hint(x: 40, y: 0)
ground q
plane front(origin:o,toward:q)
point stockfp0 hint(x: 0, y: 0)
ground stockfp0
point stockfp1 hint(x: 10, y: 0)
ground stockfp1
point stockfp2 hint(x: 10, y: 10)
ground stockfp2
point stockfp3 hint(x: 0, y: 10)
ground stockfp3
face stockf(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
solid stock(stockf, from: -1mm, to: 0mm)
point fp hint(x: 20, y: 20)
ground fp
circle fc(center: fp) hint(r: 2)
radius(2mm) fc
face f(fc)
solid tool(f,depth:1mm)
solid result(stock)
dimensions(result) in front
