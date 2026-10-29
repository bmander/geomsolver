// A sealed float: a box shelled with every face closed, `shell(t: …) X`. What it keeps is what
// lies within `wall` of everything outside it, so inside stands a hollow no face reaches: the
// body is two shells, the box's faces and the hollow's.
//
// Built exactly: 30000 less the hollow's 47 × 27 × 17, 8427 mm³.
unit mm
use std
use std.rounds (shell)

wall := 1.5mm

in std.top {
  f0 := point
  f1 := point
  f2 := point
  f3 := point
  fix((0, 0)) f0
  fix((50, 0)) f1
  fix((50, 30)) f2
  fix((0, 30)) f3
  (fs := line(f0, f1)) -> (fe := line(f1, f2)) -> (fn := line(f2, f3)) -> (fw := line(f3, f0)) -> close
  outline := face(fs, fe, fn, fw)
}
float := solid(outline, from: 0mm, to: 20mm)

shell(t: wall) float
