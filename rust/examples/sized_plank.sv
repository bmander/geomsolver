// A plank sized by its sag (#510 S3): a steel plank drawn in plan, built in at its root and
// loaded at its free end across its own plane — so it bends out of the face it is swept from,
// which no reading in that plane has a move for, and is read as the solid it is.  Its width `w`
// is an unknown the sides' dimensions read, and the sag of the end's corner is a driving
// requirement: the plank is meshed once in quadratic tetrahedra, its face's mesh stacked through
// its thickness, its nodes move with the sides as `w` does, and the solve finds the width at which
// the corner sinks `sag`.  Euler–Bernoulli's `4 P L³ / (E t³ sag)` would say 30mm, a plate's
// `(1 − ν²)` of it: three times as wide as it is thick, and held at its root from narrowing, the
// plank is a little stiffer than a beam, and narrower.
unit mm
use std (vertical)

L := 300mm
t := 10mm
P := 50N
sag := 0.9mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param w: Length hint(30mm)

in std.top {
  root_low := point
  end_low := point
  end_high := point hint((300, 30))
  root_high := point hint((0, 30))
  fix((0mm, 0mm)) root_low
  fix((L, 0mm)) end_low
  end_low vertical end_high
  root_low vertical root_high
  end_low distance(w) end_high
  root_low distance(w) root_high
  outline := (low := line(root_low, end_low)) -> (end := line(end_low, end_high)) ->
             (high := line(end_high, root_high)) -> (root := line(root_high, root_low)) -> close
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
plank := solid(outline, from: 0mm, to: t) of steel

// what it is made of answers no load, so it is the drawing's, claimed of it
claim mass(<= 1kg) plank

scenario tip_load {
  fixed plank.root
  load plank.end P along: down
  deflection(end_low, along: down) == sag
}
