// `std.sets`: surfaces as sets of points (§6.21) — a sphere, a cylinder, a cone, a half-space and
// a set offset — each a set and nothing drawn, and a circle lying on a sphere.

use std.place (horizontal)

// A sphere of radius `r` about `center`, which may be drawn in a view or stand in space: the points
// `r` from the centre (§6.21), a set and nothing drawn.  A point on it is `p coincident ball`, a
// line touching it `l tangent ball`; two touching outside are `a.center distance(a.r + b.r)
// b.center`, the radius read by the instance's name, and a circle drawn in another view lying on
// it is `k on_sphere(view: v) ball`.  Leave `r` unbound and it is an unknown of the drawing.  Not a
// solid: a ball is a half disc turned about its diameter.
//
//   use std
//   use std.sets
//   in std.front {
//     c := point hint((0, 0))
//   }
//   ball := std.sets.Sphere(c, r: 12mm)
//   p := point hint((5, 10)) in std.side
//   p coincident ball
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
  q coincident s
}

// A cylinder of radius `r` about the line `about`: the points `r` from it (§6.21), a set and
// nothing drawn.  A point on it is `p coincident shaft`, a line touching it `l tangent shaft`.
// Leave `r` unbound and it is an unknown of the drawing.  (The line is not called `axis`: that
// is an element's word, and a body could not name it bare.)
//
//   shaft := std.sets.Cylinder(ax, r: 8mm)
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
//   gc := std.sets.Cone(gax, half: 60deg)
//   M coincident gc
component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(half) g
}

// A cone about the line `about` through the point `q`, its radius growing `tan(lean)` per unit
// along `about` (a negative lean narrows it): `std.sets.Cone`'s about the line `.spine`, from the
// apex on `about` where the generator through `q` meets it, toward where the cone opens — along
// `about` for a positive lean, against it for a negative one — so its inside is always the side
// nearer `about`.  The apex and the spine read nothing of the set's point, so they are the
// instance's (`.apex`, `.spine`), made once and placed by `q` (§6.21); a point on it, a line
// touching it, `inside`, `outside` and an offset read as a cone's.  `lean` is not 0: that is a
// cylinder (`std.sets.Cylinder`); and `q` is off `about`.  A gear cutter's flanks: the cones
// through the pitch points leaning by the pressure angle.
//
//   inner := std.sets.ConeThrough(cut_ax, lp, lean: 20deg)
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
//   top := std.sets.HalfSpace(c, cut_ax, d: 2mm)
component HalfSpace(o: point, along: line, d: Length) := { p | o distance(d, along: along) p }

// The set `S` offset by `d`: the points `d` outside its surface (§6.21) — a sphere's, a cylinder's
// or a cone's grown by `d` — a set and nothing drawn.  `p inside std.sets.Offset(S, d)` is `p
// distance(<= d, side: outside) S`, within `S` grown by `d`; a point on it is `p coincident tip`,
// its foot on `S` found by the solve.  A cone's offset is a cone, its apex rounded by a ball where
// it grows.
//
//   tip := std.sets.Offset(pitch, d: 2mm)
//   region := { p | p inside tip; p distance(<= 4mm, side: inside) pitch }
component Offset(S: set, d: Length) := { p | p distance(d, side: outside) S }
