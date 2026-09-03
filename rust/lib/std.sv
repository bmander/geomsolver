// The standard library: what most drawings start from, written once.
//
// `use std` brings it in — from the library compiled into the core, so it is there in the browser
// as in the terminal; a `std.sv` beside a document would win over it, as any module does.

// The three principal views of third-angle projection (§6.7), laid out on one sheet: the page is
// the **front** view, its origin at `o`; the **right** view is folded from the front's z-axis and
// drawn `right` along the page from `o`, turned so up stays up; the **top** view is folded up from
// the front's x-axis and drawn `up` the page from `o`.  Every origin is one corner of the object,
// as each view sees it, so the views need no projection between their origins — a drawing states
// its projections between the points it draws.  Ground `o` and the sheet is placed; the spacing
// is stated here, so dragging `o` moves the whole sheet.
//
//   use std
//   point O hint(x: 0, y: 0)
//   ground O
//   views: ThreeViews(O, right: 620, up: 620)
//   part: Something(views.right_origin) in views.right
//
// The views are `front`, `right` and `top`, reached as `views.right`; `right` the length and
// `right` the view do not meet, since a number is read in an expression and an entity in a
// reference.
component ThreeViews(o: point, right: Length, up: Length) {
  // each view's `toward` point sets which way it is turned on the page; a hand's breadth away
  point qf
  o distance(40, along: x) qf
  o distance(0, along: y) qf
  plane front(origin: o, toward: qf)

  point right_origin
  point qr
  o distance(right, along: x) right_origin
  o distance(0, along: y) right_origin
  right_origin distance(0, along: x) qr
  right_origin distance(40, along: down) qr
  plane right(origin: right_origin, toward: qr, from: front, fold: -90deg)

  point top_origin
  point qt
  o distance(0, along: x) top_origin
  o distance(up, along: y) top_origin
  top_origin distance(40, along: x) qt
  top_origin distance(0, along: y) qt
  plane top(origin: top_origin, toward: qt, from: front, fold: 0deg)
}

// An ellipse, as a curve: the point at eccentric angle `u` on the ellipse of semi-axes `a` and
// `b` standing on the datum `f` — its centre at `f.origin`, its major axis along the datum's
// bearing.  A computed point, so every contact is exact to third order: `p on e` holds a point
// to the rim, `e tangent l` a line to it, `e curvature k` makes `k` the rim's osculating circle.
//
//   use std
//   point o hint(x: 0, y: 0)
//   point q hint(x: 40, y: 0)
//   plane f(origin: o, toward: q)
//   curve e = Ellipse(f, a: 40, b: 25).p over u in (0, 360)
//
// The axes are formals: leave one free and a dimension that reads the rim sizes it (issue #47,
// item 4 — this replaces the entity kind the language once had, whose rim, tangent and
// curvature were three kernels of their own).
component Ellipse(f: plane, a: Length, b: Length, u: Angle) {
  point p = ( f.origin.x + a * cos(u) * cos(f.angle) - b * sin(u) * sin(f.angle),
              f.origin.y + a * cos(u) * sin(f.angle) + b * sin(u) * cos(f.angle) )
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
    point v hint(x: c.x + r * cos(atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x) + phase + i * 360deg / n),
                 y: c.y + r * sin(atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x) + phase + i * 360deg / n))
    c distance(r) v
    line e(v, next.v)
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
  p: Polygon(c, ref, n: 6, r: af / (2 * cos(30deg)), phase: phase)
}

// A point in a *tilted* frame, for the components above: `u` along the line `ax` from the point
// `org` on it, `v` across it, with `ac` the line across `ax` through `org` and `dir` the bearing
// of `ax`.  The seed — which is the point itself, worked out — is what says which side of each
// line it falls on, so the two distances are magnitudes and the coordinates keep their signs.
component Loc(org: point, ax: line, ac: line, dir: Angle, u: Length, v: Length) {
  point p hint(x: org.x + u * cos(dir) - v * sin(dir), y: org.y + u * sin(dir) + v * cos(dir))
  p distance(abs(v)) ax
  p distance(abs(u)) ac
}
