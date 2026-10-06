// The standard library: what most drawings start from, written once.
//
// `use std` brings it in — from the library compiled into the core, so it is there in the browser
// as in the terminal; a `std.sv` beside a document would win over it, as any module does.

// The standard axes and planes, which every document that says `use std` has — as a CAD part
// has its origin planes — so the workspace can offer them as places to draw.  The axes are held
// outright, each through the world origin: x right, y away from the front's viewer, z up, and
// `back` the x axis reversed.  Each plane is two of them, and so stands where they meet, at the
// world origin: front is x right and z up (a document's 2D drawing, `in std.front`), top x right
// and y away (looked at from above), side y right and z up (looked at from +x), and up the front
// turned a quarter, z right and x to the left.  `origin` is a point drawn in the front plane at
// its origin.
component StandardDatums() {
  x := axis
  fix(dir == (1, 0, 0), origin == (0, 0, 0)) x
  y := axis
  fix(dir == (0, 1, 0), origin == (0, 0, 0)) y
  z := axis
  fix(dir == (0, 0, 1), origin == (0, 0, 0)) z
  back := axis
  fix(dir == (-1, 0, 0), origin == (0, 0, 0)) back
  front := plane(u: x, v: z)
  top := plane(u: x, v: y)
  side := plane(u: y, v: z)
  up := plane(u: z, v: back)
  origin := point in front
  fix((0, 0)) origin
}

// Axes turned within a plane: the plane through `o` whose u runs toward `t` and whose v is a
// quarter turn on, in the plane `o` and `t` are drawn in — what a part standing at an angle is
// measured against (`p distance(d, along: u) axes.axes`) and seeded in (`hint(at: axes.axes,
// x: 3, y: 4)`).  Draw it in that plane: `axes := std.Turned(o, t) in std.front`; `axes.u` is
// the line from `o` to `t`.  The two lines share `o`, so the plane stands there unsaid.
component Turned(o: point, t: point) {
  construction u := line(o, t)
  private q := point hint(at: o, toward: t, turn: 90deg)
  private construction v := line(o, q)
  v perpendicular u
  v equal u
  axes := plane(u: u, v: v)
}

// An axis-aligned rectangle about a supplied center. The public loop is a face boundary;
// the diagonal only constrains the center and stays private construction geometry.
component CenteredRectangle(center: point, w: Length, h: Length) {
  a := point hint((center.x - w / 2, center.y - h / 2))
  b := point hint((center.x + w / 2, center.y - h / 2))
  c := point hint((center.x + w / 2, center.y + h / 2))
  d := point hint((center.x - w / 2, center.y + h / 2))
  loop := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
         horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  distance(w) ab
  distance(h) bc
  private construction diagonal := line(a, c)
  center midpoint diagonal
}

// An ellipse, as a curve: the point at eccentric angle `u` on the ellipse of semi-axes `a` and
// `b` about the centre `c`, its major axis turned `turn` from the `x` of the plane `c` is drawn
// in.  A computed point, so every contact is exact to third order: `p coincident e` holds a
// point to the rim, `e tangent l` a line to it, `e curvature k` makes `k` the rim's osculating
// circle.
//
//   use std
//   in std.front {
//     o := point hint((0, 0))
//   }
//   e := std.Ellipse(o, a: 40, b: 25, tilt: 0deg).p over u in (0, 360)
//
// The axes and the tilt are formals: leave one free and a dimension that reads the rim sizes it
// (issue #47, item 4 — this replaces the entity kind the language once had, whose rim, tangent
// and curvature were three kernels of their own).
component Ellipse(c: point, a: Length, b: Length, tilt: Angle, u: Angle) {
  p := point(x: c.x + a * cos(u) * cos(tilt) - b * sin(u) * sin(tilt), y: c.y + a * cos(u) * sin(tilt) + b * sin(u) * cos(tilt))
}

// A regular polygon: `n` vertices on a circle of radius `r` about `c`, the first at `phase`
// counter-clockwise from the line `ref`'s direction, so the figure turns with whatever `ref`
// belongs to — a part's axis, a crank arm.  Every vertex is `r` from the centre, each chord is
// as long as the next, and the first edge is turned a fixed angle from `ref` (an edge lies
// `90° + 180°/n` past its own vertex's bearing): `2n` statements for `2n` coordinates, with the
// seeds walking the circle once so the winding is the one asked for.  Not "every edge at its
// own angle": with every direction stated and every vertex on the circle, alternate vertices
// can slide opposite ways along the circle to first order when `n` is even, and the diagnosis
// reads that flex as a dependency.  The last chord's equality is left unstated — it is the
// theorem the others imply.  The vertices are `v[i]` and the edges `e[i]`, `e[i]` running from
// `v[i]` to `v[i + 1]`; a class on the instance dashes or hides the lot.
//
//   use std
//   pocket: Hex(c, axis, af: 11.1, phase: 0deg) class hidden
//   claim pocket.p.e[1] distance(11.1) pocket.p.e[4]      // across the flats
component Polygon(c: point, ref: line, n: Int, r: Length, phase: Angle) {
  cycle n as i {
    v := point hint(x: c.x + r * cos(atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x) + phase + i * 360deg / n),
                 y: c.y + r * sin(atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x) + phase + i * 360deg / n))
    c distance(r) v
    e := line(v, next.v)
    repeat 1 - min(i, 1) {
      ref angle(phase + 90deg + 180deg / n) e
    }
    repeat 1 - floor(i / (n - 1)) {
      e equal e[i + 1]
    }
  }
}

// A hexagon by its width across the flats — a nut, a bolt's head, the pocket either sits in —
// the first vertex at `phase` from `ref`, so `phase: 0deg` puts a corner along the reference
// and `phase: 30deg` a flat square to it.
component Hex(c: point, ref: line, af: Length, phase: Angle) {
  p := Polygon(c, ref, n: 6, r: af / (2 * cos(30deg)), phase: phase)
}
