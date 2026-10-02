unit mm
stockfp0 := point hint(x: 0, y: 0)
ground stockfp0
stockfp1 := point hint(x: 10, y: 0)
ground stockfp1
stockfp2 := point hint(x: 10, y: 10)
ground stockfp2
stockfp3 := point hint(x: 0, y: 10)
ground stockfp3
stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
stock := solid(stockf, from: -10mm, to: 0mm)
toolfp0 := point hint(x: 2, y: 2)
ground toolfp0
toolfp1 := point hint(x: 8, y: 2)
ground toolfp1
toolfp2 := point hint(x: 8, y: 8)
ground toolfp2
toolfp3 := point hint(x: 2, y: 8)
ground toolfp3
toolf := face(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
tool := solid(toolf, from: -6mm, to: -4mm)
result := solid(stock)
tool cut result
irrelevant := point hint(x: 1000000000, y: 1000000000)
ground irrelevant
