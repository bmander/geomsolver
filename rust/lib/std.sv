// The standard library: what most drawings start from, written once.
//
// `use std` brings it in — from the library compiled into the core, so it is there in the browser
// as in the terminal; a `std.sv` beside a document would win over it, as any module does.

// Two points at one height, and one above the other, in the view they are drawn in: the
// ordinate's zero along the view's up and along its right (§9.9).  A file says `use std
// (horizontal, vertical)` to write them bare; `horizontal l` of a line is the language's own.
a horizontal b := a level(up) b
a vertical b := a level(right) b

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

// A sphere of radius `r` about `center`, which may be drawn in a view or stand in space: the points
// `r` from the centre (§6.21), a set and nothing drawn.  A point on it is `p coincident ball`, a
// line touching it `l tangent ball`; two touching outside are `a.center distance(a.r + b.r)
// b.center`, the radius read by the instance's name, and a circle drawn in another view lying on
// it is `std.CircleOnSphere`.  Leave `r` unbound and it is an unknown of the drawing.  Not a
// solid: a ball is a half disc turned about its diameter.
//
//   use std
//   in std.front {
//     c := point hint((0, 0))
//   }
//   ball := std.Sphere(c, r: 12mm)
//   p := point hint((5, 10)) in std.side
//   p coincident ball
component Sphere(center: point, r: Length) := { p | p distance(r) center }

// A circle `k` drawn in `view` lying on the sphere `s` all the way round: the sphere's centre on
// the circle's axis, and one point of the circle — `q`, level with the centre in `view`, so it
// cannot slide round — at the sphere's radius from it.  Three equations, independent wherever
// the circle is off the sphere's centre.  The toe or heel circle of a bevel blank on its end
// sphere.
//
//   std.CircleOnSphere(k, ball, std.side)
component CircleOnSphere(k: circle, s: group, view: plane) {
  private q := point hint(at: k, bearing: 0deg) in view
  q coincident k
  k.center horizontal q
  private n := axis
  n perpendicular view
  k.center coincident n
  s.center coincident n
  q coincident s
}

// A cylinder of radius `r` about the line `about`: the points `r` from it (§6.21), a set and
// nothing drawn.  A point on it is `p coincident shaft`, a line touching it `l tangent shaft`.
// Leave `r` unbound and it is an unknown of the drawing.  (The line is not called `axis`: that
// is an element's word, and a body could not name it bare.)
//
//   shaft := std.Cylinder(ax, r: 8mm)
//   l tangent shaft
component Cylinder(about: line, r: Length) := { p | p distance(r) about }

// A cone about the line `about`, its apex the line's start and opening toward its end, `half` the
// angle between the axis and every generator: the points whose generator from the apex makes
// that angle with the axis (§6.21) — on the nappe the axis points into — a set and nothing drawn.
// A point on it is `p coincident k`, a line touching it `l tangent k`, two cones touching at a
// point `TangentCones`.  Across views the angle is the angle in space; with the point in the
// axis's own view it is the page's directed angle, so the point is on the generator
// counter-clockwise of the axis.  Leave `half` unbound (`half: hint(30deg)`) and it is an unknown
// of the drawing.
//
//   gc := std.Cone(gax, half: 60deg)
//   M coincident gc
component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(half) g
}

// Two cones `k1` and `k2` touching at `m` with one tangent plane there, `m` on each beside it
// (`m coincident k1`).  Each cone's tangent plane at `m` is the one through its generator square to
// its meridian plane, so the two are one when the plane through both generators stands square to
// both meridian planes: its normal, `n`, lies in each.  Two equations.  What a hypoid's pitch
// cones do at the mean point.
//
//   std.TangentCones(gc, pc, M)
component TangentCones(k1: group, k2: group, m: point) {
  private construction g1 := line(k1.about.p1, m)
  private construction g2 := line(k2.about.p1, m)
  private t := plane(u: g1, v: g2)
  private m1 := plane(u: k1.about, v: g1)
  private m2 := plane(u: k2.about, v: g2)
  private n := axis
  n perpendicular t
  n parallel m1
  n parallel m2
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
// belongs to — a part's axis, a crank arm.  A `ring`: every vertex is the first turned a step of
// `360°/n` about `c`, so the polygon is regular by construction and one vertex is solved for —
// `r` from the centre, its edge turned a fixed angle from `ref` (an edge lies `90° + 180°/n` past
// its own vertex's bearing).  The vertices are `v[i]` and the edges `e[i]`, `e[i]` running from
// `v[i]` to `v[i + 1]`; a class on the instance dashes or hides the lot.
//
//   use std
//   pocket: Hex(c, axis, af: 11.1, phase: 0deg) class hidden
//   claim pocket.p.e[1] distance(11.1) pocket.p.e[4]      // across the flats
component Polygon(c: point, ref: line, n: Int, r: Length, phase: Angle) {
  ring n about c {
    v := point hint(at: c, along: ref, turn: phase)
    c distance(r) v
    e := line(v, next.v)
  }
  ref angle(phase + 90deg + 180deg / n) e[0]
}

// A hexagon by its width across the flats — a nut, a bolt's head, the pocket either sits in —
// the first vertex at `phase` from `ref`, so `phase: 0deg` puts a corner along the reference
// and `phase: 30deg` a flat square to it.
component Hex(c: point, ref: line, af: Length, phase: Angle) {
  p := Polygon(c, ref, n: 6, r: af / sqrt(3), phase: phase)
}
