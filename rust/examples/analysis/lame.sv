// Lamé's thick cylinder, the corpus's pressure vessel (#510 P8): a steel tube of bore `a` and
// outside `c` under an internal pressure `p`, its ends open.  With A = p a² / (c² − a²) the
// stresses are σr = A (1 − c²/r²) and σθ = A (1 + c²/r²), their sum the same everywhere, so the
// axial stress is nothing and the solution is exact in a tube of any length: the bore moves out
// u(a) = (A a / E) ((1 − ν) + (1 + ν) c²/a²), and the von Mises stress, A √(1 + 3 c⁴/r⁴), is
// greatest at the bore.  A quarter of the tube is drawn; its two cut faces are planes of
// symmetry, each held square to itself, and one point on the middle plane holds it along the
// axis.
unit mm
use std

a := 50mm
c := 100mm
depth := 20mm
p := 50MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

A := p * a^2 / (c^2 - a^2)
bore_moves := A * a / steel.E * ((1 - steel.nu) + (1 + steel.nu) * c^2 / a^2)
mises := A * sqrt(1 + 3 * c^4 / a^4)

in std.front {
  o := point
  fix((0mm, 0mm)) o
  m := point
  n := point
  q := point
  s := point
  fix((a, 0mm)) m
  fix((c, 0mm)) n
  fix((0mm, c)) q
  fix((0mm, a)) s
  cut_x := line(m, n)
  outside := arc(center: o, start: n, end: q) hint(r: c)
  cut_y := line(q, s)
  bore := arc(center: o, start: m, end: s) hint(r: a)
  section := face(cut_x, outside, cut_y, bore)
}
tube := solid(section, from: -depth / 2, to: depth / 2) of steel

scenario inflate {
  pinned tube.cut_x along: std.z
  pinned tube.cut_y along: std.x
  pinned m along: std.y
  pressure tube.bore p
  claim deflection(m, along: std.x) >= 0.99 * bore_moves
  claim deflection(m, along: std.x) <= 1.01 * bore_moves
  claim safety(tube) >= 0.97 * steel.yield / mises
  claim safety(tube) <= 1.03 * steel.yield / mises
}
