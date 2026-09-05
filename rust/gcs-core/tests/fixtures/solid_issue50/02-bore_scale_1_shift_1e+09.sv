unit mm
point stockfp0 hint(x: 1000000000, y: 1000000000)
ground stockfp0
point stockfp1 hint(x: 1000000010, y: 1000000000)
ground stockfp1
point stockfp2 hint(x: 1000000010, y: 1000000010)
ground stockfp2
point stockfp3 hint(x: 1000000000, y: 1000000010)
ground stockfp3
face stockf(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
solid stock(stockf, from: -5mm, to: 0mm)
point fp hint(x: 1000000005, y: 1000000005)
ground fp
circle fc(center: fp) hint(r: 2)
radius(2mm) fc
face f(fc)
solid tool(f, depth: 5mm)
solid result(stock)
tool cut result
