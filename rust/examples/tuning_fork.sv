// A tuning fork's in-plane modes (#510 S2): a steel fork, its stem built in, read in plane stress
// over the face it is cut from — its own modes in its own plane, `mode(fork, n)`, which a plate
// this thin has no others of to read.  The first is the whole fork rocking on its stem, well below
// a tine's; the second is the tines swinging against each other, which rings a little below the
// 764Hz of a tine built in at its root, since the base the tines stand on gives with them.
unit mm
use std (vertical)

L := 80mm
w := 6mm
gap := 8mm
base := 20mm
stem := 8mm
reach := 25mm
t := 6mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
// a tine built in at its root rings at Euler–Bernoulli's first root
tine := 1.8751^2 / (2 * pi * L^2) * sqrt(steel.E * w^2 / (12 * steel.rho))

in std.front {
  a := point
  b := point
  c := point
  d := point
  e := point hint((14, 100))
  f := point hint((4, 100))
  g := point
  h := point
  i := point hint((-4, 100))
  j := point hint((-14, 100))
  k := point
  l := point
  fix((-stem / 2, -reach)) a
  fix((stem / 2, -reach)) b
  fix((stem / 2, 0mm)) c
  fix((gap / 2 + w, 0mm)) d
  d vertical e
  d distance(base + L) e
  g vertical f
  g distance(L) f
  fix((gap / 2, base)) g
  fix((-gap / 2, base)) h
  h vertical i
  h distance(L) i
  k vertical j
  k distance(base + L) j
  fix((-gap / 2 - w, 0mm)) k
  fix((-stem / 2, 0mm)) l
  outline := (foot := line(a, b)) -> line(b, c) -> line(c, d) -> line(d, e) -> line(e, f) ->
             line(f, g) -> line(g, h) -> line(h, i) -> line(i, j) -> line(j, k) -> line(k, l) ->
             line(l, a) -> close
}
fork := solid(outline, from: -t / 2, to: t / 2) of steel

scenario ring {
  plane stress fork
  fixed fork.foot
  claim mode(fork, 1) <= 0.6 * tine
  claim mode(fork, 2) >= 0.85 * tine
  claim mode(fork, 2) <= tine
}
