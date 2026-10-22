// A long pipe under pressure, in plane strain (#510 S2): a cast iron pipe of bore `a` and outside
// `c`, its length held, under an internal pressure `p`.  A quarter of its section is read in plane
// strain, `plane strain pipe` — a long prism held along its length, so `εz = 0` and the axial
// stress is `ν(σr + σθ) = 2νA` — and Lamé's solution is exact there: the bore moves out `(1 + ν) A
// a / E ((1 − 2ν) + c²/a²)`, and the von Mises stress is greatest at the bore.  The two cut faces
// are planes of symmetry, each held square to itself; the pressure pushes into the bore's wall.
unit mm
use std

a := 150mm
c := 200mm
length := 1000mm
p := 20MPa
iron := material(E: 170GPa, nu: 0.28, rho: 7100kg / 1m^3, yield: 300MPa)

A := p * a^2 / (c^2 - a^2)
bore_moves := (1 + iron.nu) * A * a / iron.E * ((1 - 2 * iron.nu) + c^2 / a^2)
hoop := A * (1 + c^2 / a^2)
axial := 2 * iron.nu * A
mises := sqrt(((-p - hoop)^2 + (hoop - axial)^2 + (axial + p)^2) / 2)

in std.front {
  o := point
  m := point
  n := point
  q := point
  s := point
  fix((0mm, 0mm)) o
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
pipe := solid(section, depth: length) of iron

scenario pressurised {
  plane strain pipe
  pinned pipe.cut_x along: std.z
  pinned pipe.cut_y along: std.x
  pressure pipe.bore p
  claim deflection(m, along: std.x) >= 0.99 * bore_moves
  claim deflection(m, along: std.x) <= 1.01 * bore_moves
  claim safety(pipe) >= 0.97 * iron.yield / mises
  claim safety(pipe) <= 1.03 * iron.yield / mises
}
