unit mm
fp0 := point hint(x: 1e-05, y: 0)
ground fp0
fp1 := point hint(x: 1.4e-05, y: 0)
ground fp1
fp2 := point hint(x: 1.4e-05, y: 6e-06)
ground fp2
fp3 := point hint(x: 1e-05, y: 6e-06)
ground fp3
f := face(fp0, fp1, fp2, fp3, -> close)
a := point hint(x: 0, y: 0)
ground a
b := point hint(x: 0, y: 1e-05)
ground b
ax := line(a,b)
result := solid(f, about: ax)
