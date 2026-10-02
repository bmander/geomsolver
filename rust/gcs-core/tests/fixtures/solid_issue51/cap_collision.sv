unit mm
a := point hint(x: 0, y: 0)
ground a
b := point hint(x: 10, y: 0)
ground b
c := point hint(x: 10, y: 5)
ground c
d := point hint(x: 0, y: 5)
ground d
near := line(a,b)
right := line(b,c)
top := line(c,d)
left := line(d,a)
f := face(near,right,top,left)
result := solid(f,depth: 2mm)
