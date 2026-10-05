unit mm
use std
in std.front {
  stockfp0 := point
  fix(x == 1000000000, y == 1000000000) stockfp0
  stockfp1 := point
  fix(x == 1000000010, y == 1000000000) stockfp1
  stockfp2 := point
  fix(x == 1000000010, y == 1000000010) stockfp2
  stockfp3 := point
  fix(x == 1000000000, y == 1000000010) stockfp3
  stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
  stock := solid(stockf, from: -5mm, to: 0mm)
  fp := point
  fix(x == 1000000005, y == 1000000005) fp
  fc := circle(center: fp) hint(r: 2)
  radius(2mm) fc
}
f := face(fc)
tool := solid(f, depth: 5mm)
result := solid(stock)
tool cut result
