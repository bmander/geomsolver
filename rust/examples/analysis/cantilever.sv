// A cantilever: a steel bar built in at its root and loaded at its tip, the first case of the
// verification corpus (#510 P8).  Euler–Bernoulli's tip deflection is P L³ / (3 E I); a solid
// also shears, Timoshenko's P L / (κ G A) more, which at this slenderness (L/h = 20) is 0.2% of
// it, so the claims bracket the bending answer within 1% either way, the solid's inside.  The
// bar's first natural frequency is its bending in the breadth, the weaker way:
// 1.8751² / (2π L²) · √(E I / (ρ A)), 1.8751 the least root of 1 + cos x cosh x = 0.
unit mm
use std

L := 1000mm
h := 50mm
b := 25mm
P := 1kN
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

// the section's second moments: about the breadth (bending under the load) and about the depth
I := b * h^3 / 12
I_weak := h * b^3 / 12
bending := P * L^3 / (3 * steel.E * I)
first := 1.8751^2 / (2 * pi * L^2) * sqrt(steel.E * I_weak / (steel.rho * b * h))

// the bar's side in the front (x along it, z up), its neutral axis on x, its breadth swept across
// y; `end` is the middle of the tip
in std.front {
  a := point
  c := point
  d := point
  e := point
  fix((0mm, -h / 2)) a
  fix((L, -h / 2)) c
  fix((L, h / 2)) d
  fix((0mm, h / 2)) e
  side := (bottom := line(a, c)) -> (tip := line(c, d)) -> (top := line(d, e)) ->
          (root := line(e, a)) -> close
  end := point
  fix((L, 0mm)) end
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
beam := solid(side, from: -b / 2, to: b / 2) of steel

scenario tip_load {
  fixed beam.root
  load beam.tip P along: down
  claim deflection(end, along: down) >= 0.99 * bending
  claim deflection(end, along: down) <= 1.01 * bending
}

scenario free {
  fixed beam.root
  claim mode(beam, 1) >= 0.99 * first
  claim mode(beam, 1) <= 1.01 * first
  claim mass(beam) == steel.rho * L * b * h
}
