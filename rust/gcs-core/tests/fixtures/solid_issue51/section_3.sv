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
point bossfp0 hint(x: 2, y: 2)
ground bossfp0
point bossfp1 hint(x: 5, y: 2)
ground bossfp1
point bossfp2 hint(x: 5, y: 5)
ground bossfp2
point bossfp3 hint(x: 2, y: 5)
ground bossfp3
face bossf(bossfp0, bossfp1, bossfp2, bossfp3, -> close)
solid boss(bossf, from: 1mm, to: 2mm)
solid result(stock)
boss on result
plane cut(origin:o,toward:q,from:front,offset:3mm)
section(result,at:cut) in front
