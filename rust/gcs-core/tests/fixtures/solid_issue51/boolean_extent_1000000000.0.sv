unit mm
use std
in std.front {
  stockfp0 := point
  fix(x == 0, y == 0) stockfp0
  stockfp1 := point
  fix(x == 10, y == 0) stockfp1
  stockfp2 := point
  fix(x == 10, y == 10) stockfp2
  stockfp3 := point
  fix(x == 0, y == 10) stockfp3
  stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
  stock := solid(stockf, from: -10mm, to: 0mm)
  toolfp0 := point
  fix(x == 2, y == 2) toolfp0
  toolfp1 := point
  fix(x == 8, y == 2) toolfp1
  toolfp2 := point
  fix(x == 8, y == 8) toolfp2
  toolfp3 := point
  fix(x == 2, y == 8) toolfp3
  toolf := face(toolfp0, toolfp1, toolfp2, toolfp3, -> close)
  tool := solid(toolf, from: -6mm, to: -4mm)
  result := solid(stock)
  tool cut result
  irrelevant := point
  fix(x == 1000000000, y == 1000000000) irrelevant
}
