unit mm
use std
in std.front {
  stockfp0 := point
  fix(x == 0, y == 0) stockfp0
  stockfp1 := point
  fix(x == 20, y == 0) stockfp1
  stockfp2 := point
  fix(x == 20, y == 20) stockfp2
  stockfp3 := point
  fix(x == 0, y == 20) stockfp3
  stockf := face(stockfp0, stockfp1, stockfp2, stockfp3, -> close)
  stock := solid(stockf, from: -20mm, to: 0mm)
  voidfp0 := point
  fix(x == 8, y == 8) voidfp0
  voidfp1 := point
  fix(x == 12, y == 8) voidfp1
  voidfp2 := point
  fix(x == 12, y == 12) voidfp2
  voidfp3 := point
  fix(x == 8, y == 12) voidfp3
  voidf := face(voidfp0, voidfp1, voidfp2, voidfp3, -> close)
  void := solid(voidf, from: -12mm, to: -8mm)
  shell := solid(stock)
  void cut shell
  resultfp0 := point
  fix(x == 4, y == 4) resultfp0
  resultfp1 := point
  fix(x == 16, y == 4) resultfp1
  resultfp2 := point
  fix(x == 16, y == 16) resultfp2
  resultfp3 := point
  fix(x == 4, y == 16) resultfp3
  resultf := face(resultfp0, resultfp1, resultfp2, resultfp3, -> close)
  result := solid(resultf, from: -16mm, to: -4mm)
  claim result inside shell
}
