unit mm
fp0 := point hint(x: 0, y: 0)
ground fp0
fp1 := point hint(x: 10, y: 0)
ground fp1
fp2 := point hint(x: 10, y: 10)
ground fp2
fp3 := point hint(x: 0, y: 10)
ground fp3
f := face(fp0, fp1, fp2, fp3, -> close)
result := solid(f, depth: -5mm)
