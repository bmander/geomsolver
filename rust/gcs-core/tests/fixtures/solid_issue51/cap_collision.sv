unit mm
point a hint(x: 0, y: 0)
ground a
point b hint(x: 10, y: 0)
ground b
point c hint(x: 10, y: 5)
ground c
point d hint(x: 0, y: 5)
ground d
line near(a,b)
line right(b,c)
line top(c,d)
line left(d,a)
face f(near,right,top,left)
solid result(f,depth: 2mm)
