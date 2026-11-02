unit mm
use std
use std.solids
in std.front {
  stockfp0 := point
  fix((1000000000, 1000000000)) stockfp0
  stockfp1 := point
  fix((1000000010, 1000000000)) stockfp1
  stockfp2 := point
  fix((1000000010, 1000000010)) stockfp2
  stockfp3 := point
  fix((1000000000, 1000000010)) stockfp3
  stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
  stock := solid(std.solids.Prism(stockf, from: -5mm, to: 0mm))
  fp := point
  fix((1000000005, 1000000005)) fp
  fc := circle(center: fp) hint(r: 2)
  radius(2mm) fc
}
f := face(fc)
tool := solid(std.solids.Prism(f, from: -5mm, to: 0))
result := solid(stock)
tool cut result
