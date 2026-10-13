// L0 (#243), the station probe with its plane seeded for a guide up the front plane: one literal
// seed cannot serve every guide (README, finding 7), so only `bottle.sv` uses this copy.
//
// A loft is a face of a component as one of its numbers runs (`solid(I.f over s in (0, 1))`).
// The words here only say where a station stands on its path. What the section is at that station
// (its shape, its twist, its scale, the rails it follows) is the writer's component, drawn in the
// station's plane.
use std (on_u, square)

// The plane square to the line `g` at the fraction `s` of the way from its start to its end: its
// origin there, its normal along `g`, and its `v` held along `up` (any direction square to `g`).
// Along a straight guide every station is the first one moved, so nothing turns.
component OnLine(g: line, up: axis, run: Length, s: Scalar) {
  m := point
  m coincident g
  g.p1 distance(run * s, along: g) m
  n := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 1, 0)))
  n.origin coincident m
  n square(up: up) g
}

// The plane square to the arc `g` at the fraction `s` of its turn: through the arc's axis, its `u`
// running out from the centre to the arc, its `v` along `up`, the normal of the plane the arc is
// drawn in. Square to the arc's tangent, and turned about the axis as the arc turns: along a guide
// in one plane this is the rotation-minimising frame.
component OnArc(g: arc, view: plane, up: axis, sweep: Angle, s: Scalar) {
  in view {
    m := point
    m coincident g
    out := line(g.center, m)
    (from := line(g.center, g.start)) angle(sweep * s, sense: ccw) out
  }
  n := plane
  n.origin coincident m
  n.u parallel out
  n.v parallel up
}

// The plane square to the spline `g` where its parameter is `s`: the tangent there drawn as a line
// from `m`, the plane square to it, its `v` along `up` (the normal of the plane `g` is drawn in, so
// again rotation-minimising for a guide in one plane).
component OnSpline(g: spline, view: plane, up: axis, s: Scalar) {
  in view {
    m := point
    m coincident(t == s) g
    q := point
    tl := line(m, q)
  }
  g tangent(at: m) tl
  // drawn a length, or `q` slides along it (finding 13: a direction has no word without one)
  distance(10mm) tl
  n := plane
  n.origin coincident m
  n square(up: up) tl
}

// A screw's station: the plane through the axis `ax` turned `360deg · turns · s` from `base.u` and
// risen `pitch · turns · s` along it. Its `u` runs out from the axis, its `v` along it. A section
// drawn in it is an axial section (a thread's): carried by the screw, it sweeps the thread.
// `base` is the plane square to the axis at the screw's start, its origin on the axis.
component Screwed(ax: axis, base: plane, pitch: Length, turns: Scalar, s: Scalar) {
  in base {
    k := point
    out := line(base.origin, k)
    distance(10mm) out
    r0 := point
    r0 on_u(d: 10mm) base
    ref := line(base.origin, r0)
    ref angle(360deg * turns * s, sense: ccw) out
  }
  n := plane
  n.u parallel out
  n.v parallel ax
  base.origin distance(pitch * turns * s, along: ax) n.origin
  n.origin coincident ax
}

// A helix's normal station: the plane square to the helix of radius `r` about `ax`, rising `pitch`
// a turn, at `turns · s` turns from `base.u`. A section drawn in it (a spring's wire) is carried
// along the helix, square to it at every station.
component OnHelix(ax: axis, base: plane, r: Length, pitch: Length, turns: Scalar, s: Scalar) {
  sc := Screwed(ax, base, pitch: pitch, turns: turns, s: s)
  in sc.n {
    m := point
    m on_u(d: r) sc.n
  }
  n := plane
  n.origin coincident m
  n.u parallel sc.n.u
  n.v perpendicular n.u
  // the helix leans from square to the axis by its lead angle
  n.v angle(atan(pitch / (2 * pi * r))) sc.n.v
}
