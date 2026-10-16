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
  a distance(dx, along: u) b
  a distance(dy, along: v) b
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

// A plane square to a line or an axis, its `v` along a direction (#250): `n square(up: std.z) g`
// — `n`'s `v` along `up`, its `u` square to `up` and to `g`, so with `up` square to `g` its
// normal runs along `g`.  A plane's own axes are not held square to one another (§6.7), so it
// takes all three to say.  `up` square to `g` is the drawing's to keep: where it is not, the
// normal is `g`'s part square to `up`.
P square(up) l := {
  P.v parallel up
  P.u perpendicular l
  P.u perpendicular up
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
  length(w) ab
  length(h) bc
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
// point with one tangent plane there `k1 tangent(at: m) k2`.  The angle is the angle in space
// wherever the point is drawn, as every set's body is read (§6.21): with the point in the axis's
// own view, on either generator there.
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

// The half-space behind the plane square to the line `along` at the ordinate `d` along it from
// `o`: its surface is the points `d` from `o` along `along` (§6.21), a set and nothing drawn.  Its
// inside is behind that plane, the side `along` points away from (`p inside top` is at most `d`
// along); its points are not level with `o`, which `o level(along) p` says.  Through 0.84 it was
// `std.Level` (#270).
//
//   top := std.HalfSpace(c, cut_ax, d: 2mm)
component HalfSpace(o: point, along: line, d: Length) := { p | o distance(d, along: along) p }

// The set `S` offset by `d`: the points `d` outside its surface (§6.21) — a sphere's, a cylinder's
// or a cone's grown by `d` — a set and nothing drawn.  `p inside std.Offset(S, d)` is `p
// distance(<= d, side: outside) S`, within `S` grown by `d`; a point on it is `p coincident tip`,
// its foot on `S` found by the solve.  A cone's offset is a cone, its apex rounded by a ball where
// it grows.
//
//   tip := std.Offset(pitch, d: 2mm)
//   region := { p | p inside tip; p distance(<= 4mm, side: inside) pitch }
component Offset(S: set, d: Length) := { p | p distance(d, side: outside) S }

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
    repeat b in B {
      when b != a {
        run := Run(a, b, r: r)
        taken := solid(run)
        taken cut A.body
        fill := Fill(a, b, r: r)
        filled := solid(fill)
        filled union A.body
        repeat c in A {
          when c in B and c != a and c != b {
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
  }
}
round(r) X := X round(r: r) X

// `A chamfer(d) B`: every edge where a face of `A` meets one of `B` cut by a flat `d` along each
// face, measured from the other; where three meet, their flats meet in a point.
A chamfer(d) B := {
  repeat a in A {
    repeat b in B {
      when b != a {
        flat := Flat(a, b, la: d, lb: d)
        taken := solid(flat)
        taken cut A.body
        flat_fill := FlatFill(a, b, la: d, lb: d)
        filled := solid(flat_fill)
        filled union A.body
      }
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

// -- stations of a face family (#243 L3) --------------------------------------------------------
//
// A loft or a sweep is a face of a component as one of its numbers runs (§6.9):
// `solid(I(…).f over s in (0, 1))`.  These words only say where a station stands on its path: a
// plane `n` square to the path where its parameter is `s`, its `v` along `up`.  What the section
// is at that station — its shape, its twist, its scale, the rails it follows — is the writer's
// component, drawn `in st.n` and reading `s` (the laws below run a twist or a scale):
//
//   component Reducer(g: line, inlet: Length, outlet: Length, s: Scalar) {
//     st := std.OnLine(g, up: std.z, s: s)
//     sec := Section(st.n.origin, size: inlet + (outlet - inlet) * s) in st.n
//   }
//   body := solid(Reducer(guide, inlet: 24mm, outlet: 12mm).sec.profile over s in (0, 1))
//
// The edge `ab` at one station is `ab` at every other, so two sections correspond by being one.

// The plane square to the line `g` at the fraction `s` of the way from its start to its end: its
// origin there, its normal along `g`, and its `v` held along `up` (any direction square to `g`).
// Along a straight guide every station is the first one moved, so nothing turns.  It starts as it
// ends, read off the guide and `up`: `v` along `up`, and `u` that turned back a quarter about `g`.
component OnLine(g: line, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  n := plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))
  n.origin coincident m
  n square(up: up) g
}

// The plane square to the arc `g` at the fraction `s` of its turn: through the arc's axis, its `u`
// running out from the centre to the arc, its `v` along `up`, the normal of the plane the arc is
// drawn in.  Square to the arc's tangent, and turned about the axis as the arc turns: along a
// guide in one plane this is the rotation-minimising frame.
component OnArc(g: arc, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  out := line(g.center, m)
  n := plane
  n.origin coincident m
  n.u parallel out
  n.v parallel up
}

// The plane square to the spline `g` where its parameter is `s`: the tangent there drawn as a line
// from `m`, the plane square to it, its `v` along `up` (the normal of the plane `g` is drawn in,
// so again rotation-minimising for a guide in one plane).  The tangent is drawn 10 mm long, or its
// far end would slide along it, and starts ahead along `g`, so the plane starts facing the way
// the guide runs, `v` along `up` and not the mirror of it.
component OnSpline(g: spline, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  n := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  n.origin coincident m
  n square(up: up) tl
}

// The plane square to any path `g` — a line, an arc, a spline, a curve, or a curve traced in
// space (#363) — where its parameter is `s`: `OnSpline`'s place and tangent over a `path`, and the
// rotation-minimising frame along it, `Minimal`'s.  A line's and an arc's `s` is the fraction of
// its run (an arc's by angle), a spline's its own parameter, a traced curve's its formal.  Given a
// named open chain of lines and arcs, a family runs it link by link, `s` from 0 to the count of
// its links (#243 L4), the frame carried across each joint.  The frame is integrated along the
// running number from where its family stands drawn, its home, where `n.v` leans toward `up` and
// `n.u` is square to it; from there its axes turn only as the guide makes them (`n.u`'s rate has
// no part along `n.v`).  Over a guide in one plane with `up` its normal that is the frame holding
// `v` along `up`; along a twisted guide `up` is read at the home alone, so the plane stays square
// to the guide where an `up` held everywhere would lean it.  Give it the family's running number
// as `s`.
component On(g: path, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  n := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  n.origin coincident m
  n.u perpendicular tl
  n.v perpendicular tl
  n.u perpendicular n.v
  rate(n.u over s) perpendicular n.v
  initially {
    n.u perpendicular up
  }
}

// A screw's station (#243 L6): the frame of `base` turned `360deg · turns · s` about its normal
// through its origin — the screw's axis — and risen `pitch · turns · s` along it.  `n` is `base`
// so carried, square to the axis: a section drawn in it (an auger's flight, a helical gear's
// tooth) sweeps a twisted prism.  `axial` is the plane through the axis, its `u` running out along
// `n.u` and its `v` up the axis: a section drawn in it (a thread's) sweeps a thread.  Every
// direction is an ordinate in `base`'s frame, so nothing is held by a seed: the screw turns from
// `base.u` toward `base.v` as it rises, right-handed for a positive pitch, left for a negative.
component Helix(base: plane, pitch: Length, turns: Scalar, s: Scalar) {
  private o := point
  o coincident base.normal
  base.origin distance(pitch * turns * s, along: base.normal) o
  private a := point
  a coords(du: 10mm * cos(360deg * turns * s), dv: 10mm * sin(360deg * turns * s)) base
  base.origin distance(pitch * turns * s, along: base.normal) a
  private b := point
  b coords(du: -10mm * sin(360deg * turns * s), dv: 10mm * cos(360deg * turns * s)) base
  base.origin distance(pitch * turns * s, along: base.normal) b
  private c := point
  c coincident base.normal
  base.origin distance(pitch * turns * s + 10mm, along: base.normal) c
  private construction out := line(o, a)
  private construction across := line(o, b)
  private construction rise := line(o, c)
  n := plane(u: out, v: across)
  axial := plane(u: out, v: rise)
}

// A helix's normal station: the plane square to the helix of radius `r` about `base`'s normal
// (`Helix`'s screw), rising `pitch` a turn, at `turns · s` turns from `base.u`.  Its `u` runs out
// from the axis and its `v` up the axis leaned back by the helix's lead angle, so a section drawn
// in it (a spring's wire) is carried along the helix, square to it at every station.  Like
// `Helix`, every direction is an ordinate in the drawn `base`'s frame: against `Helix`'s axial
// plane, which moves, its `normal` would keep its sense only by continuity.
component OnHelix(base: plane, r: Length, pitch: Length, turns: Scalar, s: Scalar) {
  // the helix's tangent is (2πr·t̂ + pitch·ẑ) / lead over a turn, t̂ the way the screw turns
  private m := point
  m coords(du: r * cos(360deg * turns * s), dv: r * sin(360deg * turns * s)) base
  base.origin distance(pitch * turns * s, along: base.normal) m
  private a := point
  a coords(du: (r + 10mm) * cos(360deg * turns * s), dv: (r + 10mm) * sin(360deg * turns * s)) base
  base.origin distance(pitch * turns * s, along: base.normal) a
  private b := point
  b coords(du: r * cos(360deg * turns * s)
               + 10mm * pitch / hypot(2 * pi * r, pitch) * sin(360deg * turns * s),
           dv: r * sin(360deg * turns * s)
               - 10mm * pitch / hypot(2 * pi * r, pitch) * cos(360deg * turns * s)) base
  base.origin distance(pitch * turns * s + 10mm * 2 * pi * r / hypot(2 * pi * r, pitch),
                       along: base.normal) b
  private construction out := line(m, a)
  private construction lean := line(m, b)
  n := plane(u: out, v: lean)
}

// The rotation-minimising frame of a station (#339 D3, #243 L9): the plane `f` through `n`'s
// origin, square to the path as `n` is, whose axes turn only as the path makes them — `f.u`'s
// rate has no part along `f.v` (with both unit and square to the path, U′ ∥ T).  It is integrated
// along the running number from where its family stands drawn, its home, where `f.u` is square to
// `up` and `f.v` leans toward it; give it the family's running number as `s`.  Over a guide in
// one plane, with `up` that plane's normal, it is `On`'s frame; along a twisted path it turns
// against the path's Frenet frame at the path's torsion, where `OnHelix`'s turns with it.
component Minimal(n: plane, up: axis, s: Scalar) {
  f := plane(u: hint(dir: up, turn: -90deg, about: n.normal), v: hint(dir: up))
  f.origin coincident n.origin
  f.u perpendicular n.normal
  f.v perpendicular n.normal
  f.u perpendicular f.v
  rate(f.u over s) perpendicular f.v
  initially {
    f.u perpendicular up
  }
}

// Frenet's frame of a station on a curve traced in space (#363 S5): the plane square to `g`
// where its formal is `s`, `u` toward the centre of curvature there — the principal normal, its
// sense the centre's, no seed's — and `v` the binormal, so the plane faces the way the curve runs.
// Along a helix it is `OnHelix`'s plane, `u` the other way; it turns with the curve's torsion,
// where `Minimal`'s does not.  A curve with no curvature somewhere (a straight run, an
// inflection) has no centre there, and the station is refused where that happens.
component Frenet(g: path, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  c := point
  g curvature(at: m) c
  nl := line(m, c)
  n := plane(u: nl, v: hint(dir: tl, turn: -90deg, about: nl))
  n.origin coincident m
  n.v perpendicular tl
  n.v perpendicular n.u
}

// A station with no guide (#243 L7): the plane through the places at `s` on three rails — a loft
// that joins its sections in space directly.  Each rail runs through one corner of every
// section: a chain of lines, one a link a pair of neighbouring sections (the ruled loft, a family
// running the chains in step), or a spline through them (`spline(through: …)`, the smooth one).
// The section's corners stand where their rails meet the plane, each `point in st.n` and
// `coincident` its rail.  `u` runs from the first rail's place toward the second's and `v`
// toward the third's, so the plane faces the way its rails are handed in.
//
//   component Quad(ra: path, rb: path, rc: path, rd: path, s: Scalar) {
//     st := std.Across(ra, rb, rc, s: s)
//     a := point in st.n
//     a coincident ra
//     …
//     f := face(a, b, c, d, -> close)
//   }
component Across(a: path, b: path, c: path, s: Scalar) {
  private pa := point
  private pb := point
  private pc := point
  pa coincident(t == s) a
  pb coincident(t == s) b
  pc coincident(t == s) c
  private construction ab := line(pa, pb)
  private construction ac := line(pa, pc)
  n := plane(u: ab, v: ac)
}

// -- two faces drawn apart (#243 L8) -------------------------------------------------------------
//
// CAD's "loft these two sketches": the section at `s` joins corresponding corners of `a` and `b`
// by straight rails, corner `k` of each walk to corner `k` of the other, and stands where the
// plane through three of them at `s` cuts every rail (`Across`), so the ends are the faces
// themselves whatever their planes.  Correspondence is the walks' written order: a connector is
// where a walk starts (`face(b1, b2, b3, b0, -> close)` pairs `a`'s first corner with `b1`), and
// two walks of different lengths are made alike by naming a point on an edge in the shorter one.
// Nothing is inferred.  The first three corners must not stand in a line.
//
//   body := solid(std.Ruled(inlet, outlet).f over s in (0, 1))
component Ruled(a: face, b: face, s: Scalar) {
  cycle (ea, eb) in (a, b) {
    private construction rail := line(ea.p1, eb.p1)
    q := point in st.n
    q coincident rail
    in st.n {
      side := line(q, next.q)
    }
  }
  st := Across(rail[0], rail[1], rail[2], s: s)
  f := face(side)
}

// A loft to a point: `a`'s corners joined to `tip` by straight rails, the section at `s` where
// the plane through three of them at `s` cuts every rail.  At `s = 1` the section is the point:
// the solid ends in an apex there.
component Apex(a: face, tip: point, s: Scalar) {
  cycle e in a {
    private construction rail := line(e.p1, tip)
    q := point in st.n
    q coincident rail
    in st.n {
      side := line(q, next.q)
    }
  }
  st := Across(rail[0], rail[1], rail[2], s: s)
  f := face(side)
}

// A section held to its rails (#243 L8): the face `prof`, drawn once anywhere, carried whole into
// the plane `n` by the map that takes `from` to `a`, `to` to `b` and `third` to `c` — moved,
// turned, scaled and sheared, so that three rails give an affine map (`mapped`, #357; its edges
// keep their names under `f`).  In a family whose station holds `a`, `b` and `c` where three
// rails cross its plane, the section follows the rails.  `from`, `to` and `third` standing in a
// line is refused where the map is stated.  Written as the word, `f := prof mapped(from -> a,
// to -> b, third -> c) in n`, this is the same statement.
component Affine(prof: face, from: point, to: point, third: point, n: plane, a: point, b: point,
                 c: point) {
  f := prof mapped(from -> a, to -> b, third -> c) in n
}

// A section held to two rails: the similarity taking `from` to `a` and `to` to `b`, keeping the
// turn — moved, turned and scaled alike both ways, never stretched: two rails give a similarity,
// and an arc or a circle of `prof` stays one, its radius scaled.
component Similar(prof: face, from: point, to: point, n: plane, a: point, b: point) {
  f := prof mapped(from -> a, to -> b) in n
}

// -- laws of a station (#243 L5) ----------------------------------------------------------------
//
// A twist or a scale is a dimension of the section reading `s`, and any expression in it is a
// law: `twist * (3 * s^2 - 2 * s^3)`, `r * (1 + 0.2 * sin(180 * s))`.  A word takes numbers, not
// laws (Solvent has no function values), so these take a total — the turn or the scale at
// `s = 1` — and run it linearly from the section as drawn at `s = 0`; any other law is the same
// word given another fraction, `s: 3 * s^2 - 2 * s^3`.  A `when` cannot say a law: it is made
// once, for the whole run, and the station is one shape all along it; `min`, `max` and `abs` say
// a law in pieces.  A section drawn once is carried whole, turned and scaled by the similarity
// its frame is mapped by (#357), the law in where its frame's second point stands: `Twisted`'s
// turned point, or any point the station places (`lofts/tapered.sv`).  `scaled` and
// `scaled_radius` are for a section written as coordinates in the station's own plane.
//
//   component Duct(g: path, up: axis, prof: face, o: point, x: point, s: Scalar) {
//     st := std.On(g, up: up, s: s)
//     tw := std.Twisted(st.n, twist: 90deg, reach: 10mm, s: s)
//     sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
//   }

// The point `reach` along the plane `n`'s `u` from its origin, turned `twist · s` about the origin
// within it: where a section drawn once carries the second point of its frame, so the section
// turns as it runs — `sec := prof mapped(o -> n.origin, x -> tw.t) in n`, `o` and `x` the
// section's frame `reach` apart (#357).  At `s = 0` it stands on `n`'s own `u`, and the turn is
// the angle itself, so a twist past a half turn is a twist past a half turn.
component Twisted(n: plane, twist: Angle, reach: Length, s: Scalar) {
  in n {
    t := point
    t coords(du: reach * cos(twist * s), dv: reach * sin(twist * s)) n
  }
}

// Where a point stands in a plane's frame as its section is scaled about the origin: `(du, dv)`
// at `s = 0`, `scale` times that at `s = 1`, and run linearly between — `p coords` of a section
// drawn at its start's size.
p scaled(du, dv, scale, s) P := {
  p distance(du * (1 + (scale - 1) * s), along: u) P
  p distance(dv * (1 + (scale - 1) * s), along: v) P
}

// A round's radius scaled as `scaled` scales a point: `r` at `s = 0`, `scale` times it at `s = 1`.
scaled_radius(r, scale, s) k := radius(r * (1 + (scale - 1) * s)) k

// -- a frame's joint: turns, screws and slides (#383) --------------------------------------------
//
// A frame's motion is its joint: an axis `f.axis`, a ratio `f.ratio` (radians a radian of the
// shared roll) and an advance `f.advance` (along the axis a turn of the roll), held by relations
// like anything else.  These are the shapes a designer states them in.  Roll 0 is as drawn: a
// member that must stand turned there is drawn turned.

// `f` turns about the line `about`, `ratio` radians a radian of the roll, and screws `advance`
// along it a turn: a turn where the advance is left out, a screw where it is given.
component Turn(f: frame, about: line, ratio: Scalar, advance: Length = 0mm) {
  about coincident f.axis
  fix(ratio == ratio, advance == advance) f
}

// `f` turns in the view `on` about its point `about`: the axis through it square to the view.
component Spin(f: frame, about: point, on: plane, ratio: Scalar) {
  about coincident f.axis
  f.axis parallel on.normal
  fix(ratio == ratio, advance == 0mm) f
}

// `f` slides along the line `along`, `advance` a turn of the roll, turning not at all.
component Slide(f: frame, along: line, advance: Length) {
  along coincident f.axis
  fix(ratio == 0, advance == advance) f
}

// -- a motion's twist, read by rates (#326, #365) -------------------------------------------------
//
// A turn about a fixed line, a screw along one or a slide moves every point at a velocity, and so
// does one seen from another (`under: gear, seen_from: pinion`): its **twist**.  It has no words
// of its own: it is the rate of a point the frame carries.  `X coincident(under: m, t == 0deg) x`
// is `x` carried by the frame's pose at its roll 0, where it stands as drawn, and `rate(X over
// t)` is how fast `X` runs there, a length a radian of the roll — the twist at `x`.

// `l reciprocal(seen_from: g) m`: a force along the line `l` does no work on frame `m`'s motion
// seen from frame `g` — its velocity at a point of `l` is square to `l`, the same number at every
// point of it.  For the mesh of two members (`m` the gear, `g` the pinion) it is the meshing law
// of a line of action: the members move as fast along it (Phillips' spatial involute gearing).
// Seen from the world, `g` is a frame at rest.  Claimed (`claim l reciprocal(seen_from: g) m`),
// the point is carried all the same and only its rate is judged.
l reciprocal(seen_from) m := {
  moved := point
  moved coincident(under: m, seen_from: seen_from, t == 0deg) l.p1
  rate(moved over t) perpendicular l
}

// The instantaneous screw axis of a frame's motion, seen from `seen_from` where it is given (a
// frame left out is at rest: the world): `isa`, the axis every point of which `m` moves along it,
// and `pitch`, how fast — a length a radian of the roll, Ball's pitch times how fast the motion
// turns about the axis.  `s := std.Screw(isa, gear, seen_from: pinion)` with no pitch given solves
// it, and `s.pitch` reads it.  Two points of the axis carried: their rates along it place it, and
// the first's speed along it is the pitch.  A motion that turns nothing (a slide) has no such axis.
component Screw(isa: axis, m: frame, seen_from: frame, pitch: Length) {
  private origin := point
  fix((0, 0, 0)) origin
  private a := point
  private b := point
  a coincident isa
  b coincident isa
  origin level(isa) a
  a distance(10mm, along: isa) b
  private moved_a := point
  private moved_b := point
  moved_a coincident(under: m, seen_from: seen_from, t == 0deg) a
  moved_b coincident(under: m, seen_from: seen_from, t == 0deg) b
  rate(moved_a over t) parallel isa
  rate(moved_b over t) parallel isa
  rate(moved_a over t) distance(pitch, along: isa)
}
