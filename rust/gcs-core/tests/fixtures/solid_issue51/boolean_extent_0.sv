unit mm
point stockfp0 hint(x: 0, y: 0)
ground stockfp0
point stockfp1 hint(x: 10, y: 0)
ground stockfp1
point stockfp2 hint(x: 10, y: 10)
ground stockfp2
point stockfp3 hint(x: 0, y: 10)
ground stockfp3
face stockf(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
solid stock(stockf, from: -10mm, to: 0mm)
point toolfp0 hint(x: 2, y: 2)
ground toolfp0
point toolfp1 hint(x: 8, y: 2)
ground toolfp1
point toolfp2 hint(x: 8, y: 8)
ground toolfp2
point toolfp3 hint(x: 2, y: 8)
ground toolfp3
face toolf(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
solid tool(toolf, from: -6mm, to: -4mm)
solid result(stock)
tool cut result
point irrelevant hint(x: 0, y: 0)
ground irrelevant
