unit mm
fp0 := point
fix(x == 0, y == 0) fp0
fp1 := point
fix(x == 10, y == 0) fp1
fp2 := point
fix(x == 10, y == 10) fp2
fp3 := point
fix(x == 0, y == 10) fp3
f := face(fp0, fp1, fp2, fp3, -> close)
a := point
fix(x == 5, y == 0) a
b := point
fix(x == 5, y == 10) b
ax := line(a,b)
result := solid(f, about: ax)
