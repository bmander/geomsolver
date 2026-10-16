// D0 (#339): a geodesic on a cone, in the proposed spelling. Does not parse: it stops at `rate`.
//
// A curve on a cone whose curvature lies along the cone's normal. Second order, said as first
// order through geometry the body draws: `p` on the cone, the cone's normal `nl` at `p` (square
// to the generator, through the axis), and the tangent `tl` square to it. The rate rows:
//   - `p` runs along `tl` at unit speed (three rows in space);
//   - `tl` turns only within the plane of `tl` and `nl`: the rate of its far end is parallel to
//     that plane (one row). With `tl` 10 long and touching the cone, that is all a geodesic says.
// `p`'s rate is held by four rows — three rate rows and `p` on the cone differentiated — of rank
// three: the tangency already implies the fourth. The index check is a rank, not a count.
// `w`, where the normal meets the axis, is fixed by algebraic rows alone at every place.
//
// Closed form: unrolled, the cone is a plane and the geodesic a straight line. From 40 along the
// generator, starting square to it: ρ = √(40² + s²), turned atan(s/40)/sin α round the axis.
unit mm
use std

component Geodesic(o: point, ax: axis, half: Angle, s: Length) {
  p := point hint((20, 0, 35))
  q := point hint((20, 10, 35))
  w := point hint((0, 0, 46))
  gen := line(o, p)
  tl := line(p, q)
  nl := line(p, w)
  gen angle(half) ax                         // p on the cone
  w coincident ax
  nl perpendicular gen                       // nl: the cone's normal at p
  length(10mm) tl
  tl perpendicular nl                        // tl touches the cone
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)      // unit speed
  O := plane(u: tl, v: nl)
  rate(q over s) parallel O                  // tl turns only towards the normal: a geodesic
}

o := point
fix((0mm, 0mm, 0mm)) o
g := Geodesic(o, std.z, half: 30deg, s: 0mm)
g.p level(std.y) o                           // the home: on the x–z plane...
o distance(40mm) g.p                         // ...40 from the apex...
g.tl perpendicular std.z                     // ...setting off level
path := trace(g.p over s in (-60mm, 60mm))
