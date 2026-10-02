unit mm
a := point
fix(x == 0, y == 0) a
b := point
fix(x == 10, y == 0) b
c := point
fix(x == 10, y == 5) c
d := point
fix(x == 0, y == 5) d
near := line(a,b)
right := line(b,c)
top := line(c,d)
left := line(d,a)
f := face(near,right,top,left)
result := solid(f,depth: 2mm)
