# Paths in space

A sweep runs a section along a guide, and a guide need not lie in a plane. A curve traced in space
is a `path` like a line or a spline; a spline over points in space is one; an arc over points in
space mints the plane through its centre and its ends, and a circle takes the axis it stands
square to; a chain may run across planes, and may walk an arc from its end. `std.On`'s frame is
integrated along the guide, so a section that is not round stays square to a twisted one, and
`std.Frenet` turns its plane toward the curve's centre of curvature instead. Issue #363.

Read them in this order:

| File | What it shows |
|---|---|
| `coil.sv` | **A curve traced in space is a path.** A helix written by its three ordinates is handed to `std.On`, which stands a plane square to it where `u` is the family's `s` and turns it only as the guide makes it. A square wire wound three turns. |
| `frenet.sv` | **Frenet's frame, and the frame that does not twist.** `g curvature(at: m) c` holds the centre of curvature; `std.Frenet` turns its plane's `u` toward it, so a ribbon keeps facing the helix's axis. Beside it the same ribbon in `std.On`'s frame falls behind at the helix's torsion. |
| `bent_pipe.sv` | **A chain across planes.** A straight touches a bend in plan and a bend standing up, each in its own plane, so the five links are one chain. A round tube, and a bar whose frame is carried across every joint to an end cap the closed form places. |
| `routed_line.sv` | **Arcs over points in space.** Four straights and three bends routed through space with no plane written: each bend draws itself in the plane through its centre and its ends. |
| `s_bend.sv` | **A chain walks an arc from its end.** The right turn of an S-bend in plan, written as the arc from where it ends to where it starts, is entered at its end and walked back; `k3.start` still names the point written. `sense: cw` is the other way to say it. |
| `cable.sv` | **Splines over points in space.** A cable through four clip points between two connectors, `spline(through:)` with its ends along the connectors' axes, beside a Bézier hose over the same points. Both are curves traced in space and guides for a tube. |
| `flange.sv` | **A circle square to an axis.** `circle(center: o, normal: ax)` stands a flange square to a stub leaning any way; the circle's turn about the axis is a gauge the solver keeps clear of the axis. |

What the words are:

- **A curve traced in space as a path** — `g := Helix(…).p over u in (0, 1)` given to a `path`
  formal: `m coincident(t == s) g` pins the contact's place, the trace's own formal; `hint(at: m,
  along: g)` steps along it. A surface traced over two formals is no path.
- **`std.On(g, up:, s:)`** — the plane square to `g` where its parameter is `s`, its frame
  integrated from the family's home, where `n.u` is square to `up`; `up` is read there alone.
- **`g curvature(at: m) c`** — of a curve traced in space and a point: `c` the centre of the
  circle osculating `g` at `m`. **`std.Frenet(g, s:)`** — the plane with `u` toward it and `v` the
  binormal; refused where the curve has no curvature.
- **A spline over points in space** — `spline(a, b, c, d)`, or `spline(through: chain, start:,
  end:)`, with a control point or a corner no plane reaches: a curve traced in space. It bounds
  no face.
- **An arc over points in space** — drawn in the plane through its centre and its ends, the short
  way round (three points in a line are refused). **`circle(center: o, normal: ax)`** — a circle
  about a point in space, square to an axis or a line.
- **Chains** — a line touching an arc drawn in another plane is the tangency in space, so a chain
  may cross planes. A chain may enter an arc at its end, its both ends written
  (`(k := arc(center: c, start: f, end: e))` entered at `e`), or named there; in a plane,
  `arc(center: c, sense: cw)` turns clockwise from its start.

Each file's test is in `rust/gcs-core/tests/path_examples.rs`, its solids against the closed forms
in its comments; the gates of the work itself are `rust/gcs-core/tests/space_paths.rs`.
