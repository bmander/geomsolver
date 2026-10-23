// A turned shaft (#510 S3): a stepped steel shaft, its shoulder rounded, pulled along its axis.  A
// solid of revolution is no prism, so it is read by refining its exact boundary and filling it in
// quadratic tetrahedra, its cylinders' and its round's nodes on them.  The round is what gives
// its factor of safety an answer: drawn sharp, the inside corner of the shoulder is three
// quarters of a turn of material, where the elastic stress is unbounded, and the shaft says to
// round it.  Rounded to a third of the neck's radius, the shoulder concentrates the stress in the
// neck by Peterson's Kt of about 1.6.  Its error estimated beside it is the largest of the
// shaft's answers: a stress is the least forgiving output, and the round is a few elements across.
unit mm
use std

R := 20mm
r := 12mm
rho := 4mm
shoulder := 60mm
L := 160mm
sigma := 60MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

// the half section in the front, x out from the axis and z along it
in std.front {
  o := point
  a := point
  b := point
  f := point
  c := point
  g := point
  h := point
  k := point
  fix((0mm, 0mm)) o
  fix((R, 0mm)) a
  fix((R, shoulder)) b
  fix((r + rho, shoulder)) f
  fix((r + rho, shoulder + rho)) c
  fix((r, shoulder + rho)) g
  fix((r, L)) h
  fix((0mm, L)) k
  base := line(o, a)
  barrel := line(a, b)
  step := line(b, f)
  round := arc(center: c, start: g, end: f) hint(r: rho)
  neck := line(g, h)
  head := line(h, k)
  spine := line(k, o)
  section := face(base, barrel, step, round, neck, head, spine)
}
shaft := solid(section, about: std.z) of steel

// what it is made of answers no load, so it is the drawing's, claimed of it
claim mass(>= steel.rho * pi * (R^2 * shoulder + r^2 * (L - shoulder))) shaft

scenario pull {
  pinned shaft.base along: std.z
  pinned o along: std.x
  pinned o along: std.y
  pinned a along: std.y
  pressure shaft.head -sigma
  claim deflection(k, along: std.z) >= 0mm
  claim safety(shaft) >= steel.yield / (2 * sigma)
}
