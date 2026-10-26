// L0 (#243): a two-rail loft. A 3 mm thick blade 50 mm long, its width held between two rails
// drawn in plan: 20 mm at the root, 8 mm at the tip. The section's edges reach the rails where
// the station's plane meets them, each reaching point drawn in the station's plane and in plan.
// Closed form: the width runs linearly, so V = 3 · 50 · (20 + 8)/2 = 2100 mm³.
unit mm
use std
use std.place (horizontal, vertical, above, below)
use lofts

component Blade(g: line, up: axis, plan: plane, port: line, starboard: line, t: Length, s: Scalar) {
  st := lofts.OnLine(g, up: up, s: s)
  l := point in st.n, plan
  r := point in st.n, plan
  l coincident port
  r coincident starboard
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a below(d: t / 2) l
    d above(d: t / 2) l
    b below(d: t / 2) r
    c above(d: t / 2) r
  }
  f := face(a, b, c, d, -> close)
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  tip := point
  fix((0mm, 50mm)) tip
  spine := line(o, tip)
  l0 := point
  l1 := point
  r0 := point
  r1 := point
  fix((-10mm, 0mm)) l0
  fix((-4mm, 50mm)) l1
  fix((10mm, 0mm)) r0
  fix((4mm, 50mm)) r1
  port_rail := line(l0, l1)
  starboard_rail := line(r0, r1)
}
blade := Blade(spine, std.z, std.top, port_rail, starboard_rail, t: 3mm, s: 0.5)
