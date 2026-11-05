// `std.sets`: regions as sets of points (§6.21, #783) — a ball, a cylinder, a cone, a half-space,
// a rod, a set grown and a set shrunk — each a set and nothing drawn; the sphere, the one surface
// mathematics names apart from the region it bounds; a segment; and a circle lying on a sphere.
// A region's surface is its boundary, `boundary(S)`: a point on a cylinder is `p in
// boundary(shaft)`, and a line touching it `l tangent boundary(shaft)`.

use std.place (horizontal)

// A ball of radius `r` about `center`, which may be drawn in a view or stand in space: the points
// within `r` of the centre (§6.21), a set and nothing drawn.  A point in it is `p in ball`, one
// half a width off it `p distance(w / 2) ball`; its surface is the sphere, `boundary(ball)`.
// Leave `r` unbound and it is an unknown of the drawing.
//
//   ball := std.sets.Ball(c, r: 12mm)
//   region := { p | p in ball; p not in std.sets.Ball(c, r: 5mm) }
component Ball(center: point, r: Length) := { p | p distance(<= r) center }

// A sphere of radius `r` about `center`, which may be drawn in a view or stand in space: the points
// `r` from the centre (§6.21), a set and nothing drawn — `boundary(std.sets.Ball(center, r))`.  A
// point on it is `p in ball`, a line touching it `l tangent ball`; two touching outside are
// `a.center distance(a.r + b.r) b.center`, the radius read by the instance's name, and a circle
// drawn in another view lying on it is `k on_sphere(view: v) ball`.  Leave `r` unbound and it is
// an unknown of the drawing.  Not a solid: a ball is a half disc turned about its diameter.
//
//   use std
//   use std.sets
//   in std.front {
//     c := point hint((0, 0))
//   }
//   ball := std.sets.Sphere(c, r: 12mm)
//   p := point hint((5, 10)) in std.side
//   p in ball
component Sphere(center: point, r: Length) := { p | p distance(r) center }

// `k on_sphere(view: v) s`: a circle `k` drawn in `v` lying on the sphere `s` all the way round:
// the sphere's centre on the circle's axis, and one point of the circle — `q`, level with the
// centre in `v`, so it cannot slide round — at the sphere's radius from it.  Three equations,
// independent wherever the circle is off the sphere's centre.  The toe or heel circle of a bevel
// blank on its end sphere.  A fact about the circle and the sphere, so a word (through 0.175 the
// component `std.CircleOnSphere(k, s, view)`).
//
//   k on_sphere(view: std.side) ball
k on_sphere(view) s := {
  q := point hint(at: k, bearing: 0deg) in view
  q coincident k
  k.center horizontal q
  n := axis
  n perpendicular view
  k.center coincident n
  s.center coincident n
  q in s
}

// A cylinder of radius `r` about the line `about`: the points within `r` of it (§6.21), a set and
// nothing drawn.  A point on it is `p in boundary(shaft)`, a line touching it `l tangent
// boundary(shaft)`.  Leave `r` unbound and it is an unknown of the drawing.  (The line is not
// called `axis`: that is an element's word, and a body could not name it bare.)
//
//   shaft := std.sets.Cylinder(ax, r: 8mm)
//   l tangent boundary(shaft)
component Cylinder(about: line, r: Length) := { p | p distance(<= r) about }

// A cone about the line `about`, its apex the line's start and opening toward its end, `half` the
// angle between the axis and every generator: the points whose generator from the apex makes at
// most that angle with the axis (§6.21) — within the nappe the axis points into — a set and nothing
// drawn.  A point on it is `p in boundary(k)`, a line touching it `l tangent boundary(k)`, two
// cones touching at a point with one tangent plane there `boundary(k1) tangent(at: m)
// boundary(k2)`.  The angle is the angle in space wherever the point is drawn, as every set's body
// is read (§6.21): with the point in the axis's own view, on either generator there.
// Leave `half` unbound (`half: hint(30deg)`) and it is an unknown of the drawing.
//
//   gc := std.sets.Cone(gax, half: 60deg)
//   M in boundary(gc)
component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(<= half) g
}

// A cone about the line `about` through the point `q`, its radius growing `tan(lean)` per unit
// along `about` (a negative lean narrows it): `std.sets.Cone`'s about the line `.spine`, from the
// apex on `about` where the generator through `q` meets it, toward where the cone opens — along
// `about` for a positive lean, against it for a negative one — so its inside is always the side
// nearer `about`.  The apex and the spine read nothing of the set's point, so they are the
// instance's (`.apex`, `.spine`), made once and placed by `q` (§6.21); a point in it, on it, a
// line touching it and a set grown from it read as a cone's.  `lean` is not 0: that is a
// cylinder (`std.sets.Cylinder`); and `q` is off `about`.  A gear cutter's flanks: the cones
// through the pitch points leaning by the pressure angle.
//
//   inner := std.sets.ConeThrough(cut_ax, lp, lean: 20deg)
//   p not in inner
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
  spine angle(<= abs(lean)) g
}

// The half-space behind the plane square to the line `along` at the ordinate `d` along it from
// `o`: the points at most `d` from `o` along `along` (§6.21), a set and nothing drawn — behind
// that plane, the side `along` points away from.  Its surface, `boundary(top)`, is the plane;
// its points are not level with `o`, which `o level(along) p` says.  Through 0.84 it was
// `std.Level` (#270).
//
//   top := std.sets.HalfSpace(c, cut_ax, d: 2mm)
component HalfSpace(o: point, along: line, d: Length) := { p | o distance(<= d, along: along) p }

// A rod: the points within `r` of the line `about`, between its two ends — a cylinder cut off
// square at each end of its axis's segment, so a bar is one call.
//
//   rod := solid(std.sets.Rod(ax, r: 5mm))
component Rod(about: line, r: Length) := { p |
  p distance(<= r) about
  about.p1 distance(>= 0mm, along: about) p
  about.p2 distance(<= 0mm, along: about) p
}

// The set `S` grown by `d`: the points within `d` of it (§6.21) — a ball's, a cylinder's or a
// cone's, `S ⊕ B_d`, the distance from a set being nothing inside it.  A cone's grown is a cone,
// its apex rounded by a ball.
//
//   tip := std.sets.Grown(pitch, d: 2mm)
//   region := { p | p in tip; p distance(<= 4mm) { q | q not in pitch } }
component Grown(S: set, d: Length) := { p | p distance(<= d) S }

// The set `S` shrunk by `d`: the points at least `d` from everything not in it — what is left of
// `S` once `d` is taken off every side.
//
//   back := { q | q not in std.sets.Shrunk(pitch, d: 4mm) }
component Shrunk(S: set, d: Length) := { p | p distance(>= d) { q | q not in S } }

// The segment from `a` to `b`: the points on the line through them, from one to the other — where
// `line(a, b)` as a set, `p coincident l`, is the whole line.
//
//   p in std.sets.Segment(m, n)
component Segment(a: point, b: point) := { p |
  private construction l := line(a, b)
  p coincident l
  a distance(>= 0mm, along: l) p
  b distance(<= 0mm, along: l) p
}
