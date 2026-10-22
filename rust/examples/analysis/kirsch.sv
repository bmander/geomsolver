// Kirsch's plate with a hole, the corpus's stress concentration (#510 P8): a wide thin plate
// pulled at `sigma` with a round hole at its middle.  In an infinite plate the hoop stress at the
// hole's edge square to the pull is three times the pull, Kirsch's Kt = 3, and as nothing else
// acts there the von Mises stress is the same.  This plate is 20 holes wide, so Heywood's
// (2 + (1 − d/W)³) / (1 − d/W) puts the gross factor at 3.008.  A quarter is drawn: the two cut
// edges are lines of symmetry, each held square to itself, and a point holds the plate across
// its thickness.  The plate is thin, read in plane stress.
unit mm
use std

r := 10mm
W := 400mm
L := 800mm
t := 2mm
sigma := 50MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

ratio := 2 * r / W
kt := (2 + (1 - ratio)^3) / (1 - ratio)

in std.front {
  o := point
  fix((0mm, 0mm)) o
  m := point
  n := point
  k := point
  q := point
  s := point
  fix((r, 0mm)) m
  fix((L / 2, 0mm)) n
  fix((L / 2, W / 2)) k
  fix((0mm, W / 2)) q
  fix((0mm, r)) s
  cut_x := line(m, n)
  pulled := line(n, k)
  side := line(k, q)
  cut_y := line(q, s)
  hole := arc(center: o, start: m, end: s) hint(r: r)
  quarter := face(cut_x, pulled, side, cut_y, hole)
}
plate := solid(quarter, from: -t / 2, to: t / 2) of steel

scenario pull {
  plane stress plate
  pinned plate.cut_x along: std.z
  pinned plate.cut_y along: std.x
  pinned m along: std.y
  pressure plate.pulled -sigma
  claim safety(plate) >= 0.97 * steel.yield / (kt * sigma)
  claim safety(plate) <= 1.03 * steel.yield / (kt * sigma)
}
