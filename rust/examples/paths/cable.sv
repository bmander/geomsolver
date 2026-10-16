// A cable run through space between two connectors (#363 S3). A spline over points in space is a
// curve traced in space, its body Σ Rᵢ(u)·Pᵢ over the control points where they stand: exact,
// a `path`, drawn in the workspace as an axis is. `spline(through: route, start:, end:)` over a
// chain of corners in space is the cubic through them, its ends running along the lines given —
// here the connectors' axes, each drawn the way the cable runs: out of the plug from behind it,
// and into the socket toward its back. Its second derivative is square to the tangent at each
// end, the least bending the tangents leave. `cable` is that spline, and `hose` a Bézier over the
// same four corners as control points, which it passes through only at its ends.
//
// A spline in space bounds no face; it is a guide, and a tube along it is a family.
// Closed form: the cable stands at corner k where its parameter is k, and its control points are
// the cubic's solved apart (`tests/path_examples.rs`); each tube sweeps its section's area times
// its curve's length, V = π·r²·L.
// Edit the corners, the connectors or `r`.
unit mm
use std

r := 2mm

// the corners the cable is clipped at
p0 := point hint((0, 0, 0))
p1 := point hint((30, 15, 10))
p2 := point hint((60, 0, 25))
p3 := point hint((80, -20, 50))
fix((0mm, 0mm, 0mm)) p0
fix((30mm, 15mm, 10mm)) p1
fix((60mm, 0mm, 25mm)) p2
fix((80mm, -20mm, 50mm)) p3
route := line(p0, p1) -> line(p1, p2) -> line(p2, p3)

// each connector's axis: the plug's from behind it to the first corner, the socket's from the
// last corner to its back
q0 := point hint((0, -10, 0))
fix((0mm, -10mm, 0mm)) q0
q3 := point hint((80, -20, 60))
fix((80mm, -20mm, 60mm)) q3
plug := line(q0, p0)
socket := line(p3, q3)

cable := spline(through: route, start: plug, end: socket)
hose := spline(p0, p1, p2, p3)

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

// the through-spline's parameter runs one a corner, the Bézier's from 0 to 1
cable_tube := sweep(Tube(cable, std.x, r: r).f over s in (0, 3))
hose_tube := sweep(Tube(hose, std.x, r: r).f over s in (0, 1))
