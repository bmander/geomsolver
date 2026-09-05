unit mm
point stockfp0 hint(x: 0, y: 0)
ground stockfp0
point stockfp1 hint(x: 20, y: 0)
ground stockfp1
point stockfp2 hint(x: 20, y: 20)
ground stockfp2
point stockfp3 hint(x: 0, y: 20)
ground stockfp3
face stockf(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
solid stock(stockf, from: -20mm, to: 0mm)
point voidfp0 hint(x: 8, y: 8)
ground voidfp0
point voidfp1 hint(x: 12, y: 8)
ground voidfp1
point voidfp2 hint(x: 12, y: 12)
ground voidfp2
point voidfp3 hint(x: 8, y: 12)
ground voidfp3
face voidf(voidfp0, voidfp1, voidfp2, voidfp3, -> close)
solid void(voidf, from: -12mm, to: -8mm)
solid shell(stock)
void cut shell
point resultfp0 hint(x: 4, y: 4)
ground resultfp0
point resultfp1 hint(x: 16, y: 4)
ground resultfp1
point resultfp2 hint(x: 16, y: 16)
ground resultfp2
point resultfp3 hint(x: 4, y: 16)
ground resultfp3
face resultf(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
solid result(resultf, from: -16mm, to: -4mm)
claim result inside shell
