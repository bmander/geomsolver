unit mm
point fp0 hint(x: 90, y: 100)
ground fp0
point fp1 hint(x: 86, y: 100)
ground fp1
point fp2 hint(x: 86, y: 106)
ground fp2
point fp3 hint(x: 90, y: 106)
ground fp3
face f(fp0, fp1, fp2, fp3, -> close)
point a hint(x: 100, y: 100)
ground a
point b hint(x: 100, y: 110)
ground b
line ax(a,b)
solid result(f, about: ax, sweep: 90deg)
