// An elbow casting (#996): a base and a web under a bent pipe — two straight legs and a quarter
// turn, its bore cut the same way — a square flange on the vertical leg and a lugged one on the
// horizontal, a few bolt holes; then a cored hole, a cylinder from the base to the flange cut
// round the pipe (the cylinder less copies of the legs and the turn, then cut from the part), so
// the cutter's faces lie on the pipe's own cylinders and torus; then six rounds, one a statement.
// `bench casting` builds it a rung at a time: the part, with the cored hole, and each round more.
unit mm
use std
use std.solids
use std.solids (union, cut)
use std.rounds (round)
// the bend: the pipe's section, r 17 (its bore r 13), its centre 18 from the bend axis
bend := plane
fix(origin == (0mm, 0mm, 18mm)) bend
fix(dir == (1, 0, 0)) bend.u
fix(dir == (0, 1, 0)) bend.v
in bend {
  c := point
  fix((-18, 0)) c
  ocirc := circle(center: c) hint(r: 17)
  radius(17mm) ocirc
  prof := face(ocirc)
  sec := circle(center: c) hint(r: 13)
  radius(13mm) sec
  ax0 := point
  ax1 := point
  fix((0, 10)) ax0
  fix((0, -10)) ax1
  axis_l := line(ax0, ax1)
}
turn := solid(std.solids.Revolution(prof, about: axis_l, from: 0deg, to: 90deg))
in std.top {
  vc := point
  fix((-18, 0)) vc
  v := circle(center: vc) hint(r: 17)
  radius(17mm) v
  vb := circle(center: vc) hint(r: 13)
  radius(13mm) vb
}
leg_v := solid(std.solids.Prism(face(v), from: 18mm, to: 80mm))
in std.side {
  hc := point
  fix((0, 0)) hc
  h := circle(center: hc) hint(r: 17)
  radius(17mm) h
  hb := circle(center: hc) hint(r: 13)
  radius(13mm) hb
}
leg_h := solid(std.solids.Prism(face(h), from: 0mm, to: 50mm))
// the square flange on the vertical leg
in std.top {
  a := point
  b := point
  cc := point
  d := point
  fix((-60, -30)) a
  fix((0, -30)) b
  fix((0, 30)) cc
  fix((-60, 30)) d
  (s0 := line(a, b)) -> (k1 := round(r: 8mm)) (s1 := line(b, cc)) -> (k2 := round(r: 8mm)) (s2 := line(cc, d)) -> (k3 := round(r: 8mm)) (s3 := line(d, a)) -> (k0 := round(r: 8mm)) close
  fl_f := face(s0, k1, s1, k2, s2, k3, s3, k0)
  h0c := point
  fix((-50, -20)) h0c
  h0 := circle(center: h0c) hint(r: 3)
  radius(3mm) h0
  h1c := point
  fix((-10, 20)) h1c
  h1 := circle(center: h1c) hint(r: 3)
  radius(3mm) h1
}
flange := solid(std.solids.Prism(fl_f, from: 80mm, to: 90mm))
bolt0 := solid(std.solids.Prism(face(h0), from: 70mm, to: 100mm))
bolt1 := solid(std.solids.Prism(face(h1), from: 70mm, to: 100mm))
// the lugged flange on the horizontal leg
in std.side {
  l0 := point
  l1 := point
  l2 := point
  l3 := point
  fix((-25, -25)) l0
  fix((25, -25)) l1
  fix((25, 25)) l2
  fix((-25, 25)) l3
  lf := face(l0, l1, l2, l3, -> close)
  g0c := point
  fix((-17, -17)) g0c
  g0 := circle(center: g0c) hint(r: 3)
  radius(3mm) g0
}
lflange := solid(std.solids.Prism(lf, from: 50mm, to: 60mm))
lbolt := solid(std.solids.Prism(face(g0), from: 40mm, to: 70mm))
// the web under the bend, standing on a base
in std.front {
  w0 := point
  w1 := point
  w2 := point
  w3 := point
  fix((-50, -40)) w0
  fix((50, -40)) w1
  fix((50, 0)) w2
  fix((-30, 60)) w3
  wf := face(w0, w1, w2, w3, -> close)
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((-60, -50)) p0
  fix((60, -50)) p1
  fix((60, -40)) p2
  fix((-60, -40)) p3
  bf := face(p0, p1, p2, p3, -> close)
}
web := solid(std.solids.Prism(wf, from: -6mm, to: 6mm))
base := solid(std.solids.Prism(bf, from: -30mm, to: 30mm))
part := solid(base)
web union part
leg_v union part
turn union part
leg_h union part
flange union part
lflange union part
bturn := solid(std.solids.Revolution(face(sec), about: axis_l, from: 0deg, to: 90deg))
bleg_v := solid(std.solids.Prism(face(vb), from: 18mm, to: 90mm))
bleg_h := solid(std.solids.Prism(face(hb), from: 0mm, to: 60mm))
bturn cut part
bleg_v cut part
bleg_h cut part
bolt0 cut part
bolt1 cut part
lbolt cut part
// the cored hole: a cylinder from the base to the flange, cut round the pipe
in std.top {
  qc := point
  fix((-10, 0)) qc
  q := circle(center: qc) hint(r: 5.75)
  radius(5.75mm) q
}
cyl := solid(std.solids.Prism(face(q), from: -50mm, to: 90mm))
turn2 := solid(std.solids.Revolution(prof, about: axis_l, from: 0deg, to: 90deg))
leg_v2 := solid(std.solids.Prism(face(v), from: 18mm, to: 80mm))
leg_h2 := solid(std.solids.Prism(face(h), from: 0mm, to: 50mm))
core := solid(cyl)
turn2 cut core
leg_v2 cut core
leg_h2 cut core
core cut part
// the rounds, one a statement
r1 := part.leg_v round(r: 3mm) part.flange
r2 := part.web round(r: 3mm) part.base
r3 := part.leg_h round(r: 3mm) part.lflange
r4 := part.web round(r: 2mm) part.turn
r5 := part.web round(r: 2mm) part.leg_v
r6 := part.web round(r: 2mm) part.leg_h
