unit mm
stockfp0 := point hint(x: 0, y: 0)
ground stockfp0
stockfp1 := point hint(x: 20, y: 0)
ground stockfp1
stockfp2 := point hint(x: 20, y: 20)
ground stockfp2
stockfp3 := point hint(x: 0, y: 20)
ground stockfp3
stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
stock := solid(stockf, from: -20mm, to: 0mm)
voidfp0 := point hint(x: 8, y: 8)
ground voidfp0
voidfp1 := point hint(x: 12, y: 8)
ground voidfp1
voidfp2 := point hint(x: 12, y: 12)
ground voidfp2
voidfp3 := point hint(x: 8, y: 12)
ground voidfp3
voidf := face(voidfp0, voidfp1, voidfp2, voidfp3, -> close)
void := solid(voidf, from: -12mm, to: -8mm)
shell := solid(stock)
void cut shell
resultfp0 := point hint(x: 4, y: 4)
ground resultfp0
resultfp1 := point hint(x: 16, y: 4)
ground resultfp1
resultfp2 := point hint(x: 16, y: 16)
ground resultfp2
resultfp3 := point hint(x: 4, y: 16)
ground resultfp3
resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
result := solid(resultf, from: -16mm, to: -4mm)
claim result inside shell
