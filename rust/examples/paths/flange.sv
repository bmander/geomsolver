// A pipe stub on an inclined axis, and a flange square to it at its end (#363 S6). A circle
// about a point standing in space takes the axis it stands square to, `circle(center: o, normal:
// ax)` — an axis or a line — and is drawn in the plane minted through `o` square to it: a circle
// in a plane, so it bounds a face and a face makes a solid. The flange is that face swept its
// thickness back from the stub's end (`depth:` runs behind a face); the stub is a family, a round
// section run along the axis line as a pipe is run along a chain. Neither writes a plane.
//
// A circle has no turn of its own: its plane's turn about the normal is a gauge, held square to
// the world's axis least along the normal and taken again where a solve leaves the normal near
// it, so the stub's axis may lean any way.
// Closed form: the stub is π·r²·length, the flange π·rf²·thick, standing from the stub's end back
// `thick` down it, its rim `rf` from the axis; edit `lean` and both stand the same.
// Edit `length`, `r`, `rf`, `thick` or `lean`.
unit mm
use std
use std.stations

length := 40mm
r := 6mm
rf := 15mm
thick := 4mm
lean := 30deg

// the stub's axis: from a held base, `length` long, leaning `lean` from `z` toward `x`
base := point hint((0, 0, 0))
fix((0mm, 0mm, 0mm)) base
top := point hint((length * sin(lean), 0, length * cos(lean)))
base distance(length * sin(lean), along: std.x) top
base level(std.y) top
base distance(length * cos(lean), along: std.z) top
ax := line(base, top)

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.stations.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

param s: Scalar over (0, 1)
stub := sweep(Tube(ax, std.y, r: r).f over s)

// the flange: a circle about the stub's top, square to its axis, swept back down it
rim := circle(center: top, normal: ax)
radius(rf) rim
flange := solid(face(rim), depth: thick)
