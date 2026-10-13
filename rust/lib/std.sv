// The standard library: what most drawings start from, written once.
//
// `use std` brings it in — from the library compiled into the core, so it is there in the browser
// as in the terminal; a `std.sv` beside a document would win over it, as any module does.

// Two points at one height, and one above the other, in the view they are drawn in: the
// ordinate's zero along the view's up and along its right (§9.9).  A file says `use std
// (horizontal, vertical)` to write them bare; `horizontal l` of a line is the language's own.
a horizontal b := a level(up) b
a vertical b := a level(right) b

// Where one point stands from another in the view both are drawn in (§9.9, #103): `b offset(dx:
// 30, dy: 12) a` — `b` 30 to the right of `a` and 12 above it, each a dimension; and the four
// steps along one of the view's axes, `b right_of(d: 30) a` — `b` 30 to the right of `a`, level
// with it — `left_of`, `above` and `below`.
b offset(dx, dy) a := {
  a distance(dx, along: x) b
  a distance(dy, along: y) b
}
b right_of(d) a := {
  a distance(d, along: right) b
  a level(up) b
}
b left_of(d) a := {
  a distance(d, along: left) b
  a level(up) b
}
b above(d) a := {
  a distance(d, along: up) b
  a level(right) b
}
b below(d) a := {
  a distance(d, along: down) b
  a level(right) b
}

// Where a point stands in a plane's own frame, from its origin (§9.9, #103): `p coords(du: 20,
// dv: 5) P` — 20 along `P.u` and 5 along its `v` — and on one of its axes, `p on_u(d: 20) P`,
// `p on_v(d: 5) P`.
p coords(du, dv) P := {
  p distance(du, along: u) P
  p distance(dv, along: v) P
}
p on_u(d) P := {
  p distance(d, along: u) P
  p level(v) P
}
p on_v(d) P := {
  p level(u) P
  p distance(d, along: v) P
}

// Two lines crossed at an angle and a distance apart — two shafts, the angle between them and
// their common perpendicular, in space: `gax skew(theta: 90deg, e: 20mm) pax`.
a skew(theta, e) b := {
  a angle(theta) b
  a distance(e) b
}

// A line the length of another, turned from it by an angle — `ax turned(theta: 180deg) t.ax`
// is `t.ax` reversed, `turned(theta: 0deg)` a copy carried across.
a turned(theta) b := {
  b angle(theta) a
  a equal b
}

// A rope hanging (#121): a curve `L` long that makes its height, integrated along it, least —
// `hangs(L: 150mm) rope` of a free curve `rope := curve(a, b)`.  The view's down is the way it
// falls.  What it comes to is the catenary, which nothing here says.
hangs(L) k := {
  length(L) k
  k minimizes integral(p.y over p)
}

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
component CircleOnSphere(k: circle, s: set, view: plane) {
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
// point `TangentCones`.  The angle is the angle in space wherever the point is drawn, as every
// set's body is read (§6.21): with the point in the axis's own view, on either generator there.
// Leave `half` unbound (`half: hint(30deg)`) and it is an unknown of the drawing.
//
//   gc := std.Cone(gax, half: 60deg)
//   M coincident gc
component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(half) g
}

// A cone about the line `about` through the point `q`, its radius growing `tan(lean)` per unit
// along `about` (a negative lean narrows it): `std.Cone`'s about the line `.spine`, from the apex
// on `about` where the generator through `q` meets it, toward where the cone opens — along
// `about` for a positive lean, against it for a negative one — so its inside is always the side
// nearer `about`.  The apex and the spine read nothing of the set's point, so they are the
// instance's (`.apex`, `.spine`), made once and placed by `q` (§6.21); a point on it, a line
// touching it, `inside`, `outside` and an offset read as a cone's.  `lean` is not 0: that is a
// cylinder (`std.Cylinder`); and `q` is off `about`.  A gear cutter's flanks: the cones through
// the pitch points leaning by the pressure angle.
//
//   inner := std.ConeThrough(cut_ax, lp, lean: 20deg)
//   p outside inner
component ConeThrough(about: line, q: point, lean: Angle) := { p |
  // Seed: the apex `about`'s length back from its start against the way the cone opens.
  apex := point hint(at: about.p1, along: about, by: -lean / abs(lean))
  private foot := point hint(at: about.p1)
  apex coincident about
  // the generator through `q` makes the lean with `about` where the cone opens along it, and its
  // supplement where it opens against it: one apex either way
  private construction through := line(apex, q)
  about angle(90deg - (90deg - abs(lean)) * lean / abs(lean)) through
  // the spine, from the apex to `q`'s foot on `about`: the way the cone opens, as long as the
  // cone is to `q`, whatever the drawing's size
  foot coincident about
  private construction across := line(foot, q)
  across perpendicular about
  construction spine := line(apex, foot)
  private construction g := line(apex, p)
  spine angle(abs(lean)) g
}

// The plane square to the line `along` at the ordinate `d` along it from `o`: the points `d` from
// `o` along `along` (§6.21), a set and nothing drawn.  Its inside is behind it, the side `along`
// points away from (`p inside top` is at most `d`).
//
//   top := std.Level(c, cut_ax, d: 2mm)
component Level(o: point, along: line, d: Length) := { p | o distance(d, along: along) p }

// The set `S` offset by `d`: the points `d` outside its surface (§6.21) — a sphere's, a
// cylinder's or a cone's grown by `d` — a set and nothing drawn.  `p inside std.Offset(S, d)` is
// `p inside(d) S`, within `S` grown by `d`; a point on it is `p coincident tip`, its foot on `S`
// found by the solve.  A cone's offset is a cone, its apex rounded by a ball where it grows.
//
//   tip := std.Offset(pitch, d: 2mm)
//   region := { p | p inside tip; p outside(4mm) pitch }
component Offset(S: set, d: Length) := { p | p distance(d, side: outside) S }

// Two cones `k1` and `k2` touching at `m` with one tangent plane there, `m` on each beside it
// (`m coincident k1`): every direction along one cone at `m` is along the other (`k1
// tangent(at: m) k2`, §6.21).  Two equations.  What a hypoid's pitch cones do at the mean point.
//
//   std.TangentCones(gc, pc, M)
component TangentCones(k1: set, k2: set, m: point) {
  k1 tangent(at: m) k2
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

// -- rounds and chamfers (#183) -----------------------------------------------------------------
//
// What a round or a chamfer means is said here, over the language's own words: faces as sets (a
// face is its patch, `q coincident a`; `a.surface` the whole surface it lies on; `p inside a` the
// surface's material side), the distance from a set, a bound on measurements, where an edge ends,
// the material's angle at an edge, words that repeat over faces and make solids.  An evaluator
// reads the sets (the field); a lowering may build an exact surface for a shape it recognises, and
// says nothing these do not.

// What a ball of radius `r` rolled along the convex edge of faces `a` and `b` cannot reach: the
// points within both faces' sides that no such ball within both covers; only where the point's
// nearest point of the faces' meeting, short of where the edge ends, is on the edge (so surfaces
// meeting where the body has no edge make nothing); so carried on past each end of the edge and
// cut off there by the faces it ends on, however they stand.
component Run(a: face, b: face, r: Length) := { p |
  p inside a
  p inside b
  angle(a, b, at: p) <= 180deg
  centres := { c | c inside a; c distance(>= r) a.surface; c inside b; c distance(>= r) b.surface }
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// The same at a concave edge, every side turned round: what the ball rolled outside both faces
// cannot reach, added.
component Fill(a: face, b: face, r: Length) := { p |
  p outside a
  p outside b
  angle(a, b, at: p) >= 180deg
  centres := { c | c outside a; c distance(>= r) a.surface; c outside b; c distance(>= r) b.surface }
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// Where three rounded convex edges meet at a corner of the body: what the ball touching all three
// faces cannot reach, within the three edges' extents.
component Corner(a: face, b: face, c: face, r: Length) := { p |
  p inside a
  p inside b
  p inside c
  angle(a, b, at: p) <= 180deg
  angle(b, c, at: p) <= 180deg
  angle(c, a, at: p) <= 180deg
  centres := { o | o inside a; o distance(>= r) a.surface; o inside b; o distance(>= r) b.surface
               o inside c; o distance(>= r) c.surface }
  vertex := { q | q coincident a; q coincident b; q coincident c }
  meeting := { q | q coincident a.surface; q coincident b.surface; q coincident c.surface }
  ab := { q | q coincident a; q coincident b }
  bc := { q | q coincident b; q coincident c }
  ca := { q | q coincident c; q coincident a }
  distance(p, centres) >= r
  distance(p, vertex) <= distance(p, meeting)
  p inside ab.ends
  p inside bc.ends
  p inside ca.ends
}

// `Corner` at a concave corner, every side turned round.
component CornerFill(a: face, b: face, c: face, r: Length) := { p |
  p outside a
  p outside b
  p outside c
  angle(a, b, at: p) >= 180deg
  angle(b, c, at: p) >= 180deg
  angle(c, a, at: p) >= 180deg
  centres := { o | o outside a; o distance(>= r) a.surface; o outside b; o distance(>= r) b.surface
               o outside c; o distance(>= r) c.surface }
  vertex := { q | q coincident a; q coincident b; q coincident c }
  meeting := { q | q coincident a.surface; q coincident b.surface; q coincident c.surface }
  ab := { q | q coincident a; q coincident b }
  bc := { q | q coincident b; q coincident c }
  ca := { q | q coincident c; q coincident a }
  distance(p, centres) >= r
  distance(p, vertex) <= distance(p, meeting)
  p inside ab.ends
  p inside bc.ends
  p inside ca.ends
}

// A chamfer's flat at a convex edge: the points within both faces nearer them than the flat
// through each face's point its leg from the other face — `la` on `a`, `lb` on `b` (#183 X1's
// finding 4).  Between planes the flat, about an axis a cone.
component Flat(a: face, b: face, la: Length, lb: Length) := { p |
  p inside a
  p inside b
  angle(a, b, at: p) <= 180deg
  distance(p, a.surface) / lb + distance(p, b.surface) / la <= 1
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// `Flat` at a concave edge, every side turned round: what the flat adds.
component FlatFill(a: face, b: face, la: Length, lb: Length) := { p |
  p outside a
  p outside b
  angle(a, b, at: p) >= 180deg
  distance(p, a.surface) / lb + distance(p, b.surface) / la <= 1
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// `A round(r: 3mm) B`: every edge where a face of `A` meets a face of `B` rounded by a ball of radius
// `r` — taken away where it is convex, filled where it is concave, read off the geometry — and
// where three such edges meet, the ball touching all three.  Faces that do not meet make
// nothing.  Its material acts on the body the faces are of, as made by the terms that do not
// read it.  `round(r) X` is every edge among `X`'s faces.
A round(r) B := {
  repeat a in A {
    repeat b in B if b != a {
      run := Run(a, b, r: r)
      taken := solid(run)
      taken cut A.body
      fill := Fill(a, b, r: r)
      filled := solid(fill)
      filled union A.body
      repeat c in A if c in B and c != a and c != b {
        corner := Corner(a, b, c, r: r)
        cornered := solid(corner)
        cornered cut A.body
        corner_fill := CornerFill(a, b, c, r: r)
        corner_filled := solid(corner_fill)
        corner_filled union A.body
      }
    }
  }
}
round(r) X := X round(r: r) X

// `A chamfer(d) B`: every edge where a face of `A` meets one of `B` cut by a flat `d` along each
// face, measured from the other; where three meet, their flats meet in a point.
A chamfer(d) B := {
  repeat a in A {
    repeat b in B if b != a {
      flat := Flat(a, b, la: d, lb: d)
      taken := solid(flat)
      taken cut A.body
      flat_fill := FlatFill(a, b, la: d, lb: d)
      filled := solid(flat_fill)
      filled union A.body
    }
  }
}
chamfer(d) X := X chamfer(d: d) X

// -- in a region's body ---------------------------------------------------------------------------
//
// A region is a conjunction, so every edge where two of its bounds meet is convex, and a round or
// a chamfer there is one more bound on its point: no solid, no body.  `a` and `b` are two of the
// sets the region is within, as it states them (`{ q | q outside toe }` for the side outside a
// toe it bounds).
//
//   disc := { p | p inside wall; p inside top; p inside bottom; p inside std.Rounded(wall, top, r: 2mm) }

// Within reach of a ball of radius `r` lying within both `a` and `b`.
component Rounded(a: set, b: set, r: Length) := { p |
  centres := { c | c distance(>= r, side: inside) a; c distance(>= r, side: inside) b }
  distance(p, centres) <= r
}

// Short of the flat through each set's point `la` (on `a`) and `lb` (on `b`) from the other's
// surface: the points whose distances from the two surfaces, each over the other's leg, sum to
// one or more — for a point within both, its distance from each surface is its distance from
// what is outside that set.  Between planes the flat, about an axis a cone; where a surface is
// curved in section it bows with it (on a sphere beside a cone, a parabola's arc turned).
// Multiplied through by both legs, so a leg of nothing cuts nothing.
component Chamfered(a: set, b: set, la: Length, lb: Length) := { p |
  out_a := { q | q outside a }
  out_b := { q | q outside b }
  la * distance(p, out_a) + lb * distance(p, out_b) >= la * lb
}
