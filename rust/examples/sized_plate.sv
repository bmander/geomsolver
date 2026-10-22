// A plate sized by its sag (#510 S2): a steel plate built in along its root and loaded along its
// free end, read in plane stress over its face — the plate itself, not a beam drawn on its axis.
// Its depth `h` is an unknown the sides' dimensions read, and the deflection of the end's lower
// corner is a driving requirement: the region is meshed once in quadratic triangles where the
// drawing stands, its nodes move with the sides as `h` does, and the solve finds the depth at
// which the corner sinks `sag`.  Euler–Bernoulli's `(4 P L³ / (E t sag))^(1/3)` would say 75.6mm;
// the plate's shear and its built-in root, which a beam's line does not have, make it deeper.
unit mm
use std (vertical)

L := 300mm
t := 10mm
P := 4kN
sag := 0.5mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param h: Length hint(78mm)

in std.front {
  root_low := point
  end_low := point
  end_high := point hint((300, 78))
  root_high := point hint((0, 78))
  fix((0mm, 0mm)) root_low
  fix((L, 0mm)) end_low
  end_low vertical end_high
  root_low vertical root_high
  end_low distance(h) end_high
  root_low distance(h) root_high
  outline := (low := line(root_low, end_low)) -> (end := line(end_low, end_high)) ->
             (top := line(end_high, root_high)) -> (root := line(root_high, root_low)) -> close
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
plate := solid(outline, from: -t / 2, to: t / 2) of steel

scenario tip_load {
  plane stress plate
  fixed plate.root
  load plate.end P along: down
  deflection(end_low, along: down) == sag
  claim safety(plate) >= 1
}
