unit mm
point fp0 hint(x: 10, y: 0)
ground fp0
point fp1 hint(x: 14, y: 0)
ground fp1
point fp2 hint(x: 14, y: 6)
ground fp2
point fp3 hint(x: 10, y: 6)
ground fp3
face f(fp0, fp1, fp2, fp3, -> close)
point a hint(x: 0, y: 0)
ground a
point b hint(x: 0, y: 10)
ground b
line ax(a,b)
solid result(f, about: ax)
