unit mm
fp0 := point hint(x: 90, y: 100)
ground fp0
fp1 := point hint(x: 86, y: 100)
ground fp1
fp2 := point hint(x: 86, y: 106)
ground fp2
fp3 := point hint(x: 90, y: 106)
ground fp3
f := face(fp0, fp1, fp2, fp3, -> close)
a := point hint(x: 100, y: 100)
ground a
b := point hint(x: 100, y: 110)
ground b
ax := line(a,b)
result := solid(f, about: ax, sweep: 90deg)
