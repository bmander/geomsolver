// A geodesic on a cone: the path a filament takes when it is wound onto a cone under tension and
// does not slip — it bends only toward the surface's normal, never along the surface. In space.
//
// The body draws the point `p` on the cone (its generator at the half angle to the axis), the
// cone's normal `nl` at `p` (square to the generator, through the axis), the tangent `tl` square to
// it and the binormal `bl` square to both, all in the surface but `nl`. Its rate rows (#339 D2):
//
//   - `rate(p over s) parallel tl`, `rate(p over s) distance(1, along: tl)`: the point runs along
//     the tangent at unit speed;
//   - `rate(q over s) perpendicular bl`: the tangent's tip turns only toward the normal — the
//     tangent never turns within the surface. That is all a geodesic says.
//
// In space a `parallel` is two rows, and `p` on the cone already implies one of them: thirteen
// rows for twelve rates, of rank twelve. The index check is a rank, read at every place.
//
// A body traced in space is written in place, so `Started` says where it sets off, `initially`:
// on the x–z plane, `r0` from the apex, square to the axis. A point held on the curve at a stated
// height is where the closed form is that high, and the line along its tangent there is the closed
// form's.
//
// Closed form: unrolled, the cone is a plane and the geodesic a straight line. From r0 along the
// generator, setting off square to it, at arc length s: ρ = √(r0² + s²) from the apex, turned
// atan(s/r0)/sin α round the axis.
unit mm
use std

half := 30deg
r0 := 40mm

component Geodesic(o: point, top: point, half: Angle, s: Length) {
  ax := line(o, top)
  p := point hint((20, 0, 35))
  q := point hint((20, 10, 35))
  w := point hint((0, 0, 46))
  m := point hint((25, 0, 44))
  gen := line(o, p)
  tl := line(p, q)
  nl := line(p, w)
  bl := line(p, m)
  gen angle(half) ax                         // p on the cone
  w coincident ax
  nl perpendicular gen                       // nl: the cone's normal at p
  length(10mm) tl
  tl perpendicular nl                        // tl touches the cone
  length(10mm) bl
  bl perpendicular tl
  bl perpendicular nl                        // bl: the other way along the surface
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)      // unit speed
  rate(q over s) perpendicular bl            // the tangent turns only toward the normal
}

component Started(o: point, top: point, ya: line, half: Angle, r0: Length, s: Length) {
  g := Geodesic(o, top, half: half, s: s)
  initially {
    o distance(0mm, along: ya) g.p           // on the x–z plane...
    o distance(r0) g.p                       // ...r0 from the apex...
    g.tl perpendicular g.ax                  // ...setting off level
  }
}

o := point hint((0, 0, 0))
fix((0, 0, 0)) o
top := point hint((0, 0, 10))
fix((0, 0, 10)) top
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
ya := line(o, y1)
zl := line(o, top)
path := trace(Started(o, top, ya, half: half, r0: r0, s: 0mm).g.p over s in (-60mm, 60mm))

h := point hint((7, 24, 43))
h coincident path
o distance(r0 * 1.25 * cos(half), along: zl) h   // held where ρ = 50: at s = 30
h2 := point hint((0, 30, 50))
th := line(h, h2)
path tangent(at: h) th
h distance(10mm) h2
