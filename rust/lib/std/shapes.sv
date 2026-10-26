// `std.shapes`: figures drawn in a plane — a rectangle about its centre, an ellipse, a regular
// polygon and a hexagon by its flats — and the shape a rope hangs in.

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


// An ellipse, as a curve: the point at eccentric angle `u` on the ellipse of semi-axes `a` and
// `b` about the centre `c`, its major axis turned `turn` from the `x` of the plane `c` is drawn
// in.  A computed point, so every contact is exact to third order: `p coincident e` holds a
// point to the rim, `e tangent l` a line to it, `e curvature k` makes `k` the rim's osculating
// circle.
//
//   use std
//   use std.shapes
//   in std.front {
//     o := point hint((0, 0))
//   }
//   e := trace(std.shapes.Ellipse(o, a: 40, b: 25, tilt: 0deg).p over u in (0, 360))
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

// A rope hanging (#121): a curve `L` long that makes its height, integrated along it, least —
// `hangs(L: 150mm) rope` of a free curve `rope := curve(a, b)`.  The view's down is the way it
// falls.  What it comes to is the catenary, which nothing here says.
hangs(L) k := {
  length(L) k
  k minimizes integral(p.y over p)
}
