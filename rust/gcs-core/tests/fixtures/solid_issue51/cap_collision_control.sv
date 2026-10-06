unit mm
use std
in std.front {
  a := point
  fix((0, 0)) a
  b := point
  fix((10, 0)) b
  c := point
  fix((10, 5)) c
  d := point
  fix((0, 5)) d
  bottom := line(a,b)
  right := line(b,c)
  top := line(c,d)
  left := line(d,a)
}
f := face(bottom,right,top,left)
result := solid(f,depth: 2mm)
