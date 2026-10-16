// Wavefronts in Maxwell's fish-eye: light from a point source `a` spreading through a lens whose
// refractive index falls off from its centre as n = 2 / (1 + r² / R²) — the eikonal equation in
// a graded medium, |∇s| = n, marched along the optical length `s` by the method of lines (#339
// D4, #378). `../fisheye.sv` draws one ray of the same lens as a free curve, the path of least
// optical length; here are the fronts square to every such ray.
//
// The body runs over `s`, the optical length from the source, marched, and `u`, an angle round
// the front, sampled. `nl`, placed at every node, is the front's normal: in the lens's plane
// (the front plane, x right and z up, `fisheye.sv`'s) and square to `rate(p over u)`. Then, as
// `front.sv`, a law and a gauge, but the law's speed reads where the front is:
//
//   - the law, `rate(p over s) distance(1 / n, along: nl)`: `p` moves along `nl` at the speed
//     light has there, 1/n a unit of optical length, with n read at `p`'s own coordinates — a
//     number the body places at each step, held to the expression by a row of its own (#378);
//   - the gauge, `rate(p over s) parallel nl`: square to the front, so each node runs along a ray.
//
// The source is `a`, at (−R, 0) on the lens's rim. A front is no front at the source itself,
// where it is a point and has no normal, so `initially` starts the march a little way out, at
// s = 2: the front there, by the closed form below, a circle of radius R tan(s/R) about
// (−R / cos(s/R), 0). The seeds pick the outward normal. The arc u in (−120°, 120°) holds the
// rays that leave `a` within 120° of the lens's centre — `fisheye.sv`'s, which leaves straight up
// along the rim, among them; the rest run out through infinity before they return.
//
// The lens is the sphere of radius R seen through a stereographic projection (its metric is
// exactly n|dx|), so the front at `s` is the sphere's circle at distance s from `a`, and
// projected it is a circle again: cos(s/R)(x² + z²) + 2R x + R² cos(s/R) = 0. The rays are the
// sphere's great circles through `a`, and every one meets again at the point opposite — here
// a's conjugate, (R, 0), at s = πR = 157.08: the lens images `a` perfectly. The fronts close on
// it, so the march is refused just short of it, where a node's area |x_s × x_u| falls to a
// hundredth of its home's. `solventc` says where.
unit mm
use std

R := 50mm

component Wave(c: point, x1: point, y1: point, z1: point, R: Length, s: Length, u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  p := point hint((-R + 2mm * cos(u), 0mm, 2mm * sin(u)))
  k := point hint((-R + 12mm * cos(u), 0mm, 12mm * sin(u)))
  nl := line(p, k)
  length(10mm) nl
  c distance(0mm, along: ya) k               // the normal in the lens's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over s) parallel nl                 // the gauge: along the ray
  // the law: at 1/n, n = 2 / (1 + r²/R²) read at p, r its distance from the centre
  rate(p over s) distance((1 + ((p.x - c.x)^2 + (p.y - c.y)^2 + (p.z - c.z)^2) / R^2) / 2,
                          along: nl)
  initially {
    // the Cauchy data: the front at the interval's start, a circle by the closed form
    c distance(-R / cos(s / R * 1rad) + R * tan(s / R * 1rad) * cos(u), along: xa) p
    c distance(0mm, along: ya) p
    c distance(R * tan(s / R * 1rad) * sin(u), along: za) p
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1

// the source, and its conjugate, where the fronts refocus
a := point hint((-50, 0, 0))
fix((-50, 0, 0)) a
image := point hint((50, 0, 0))
fix((50, 0, 0)) image

// run past the conjugate point
wave := trace(Wave(c, x1, y1, z1, R: R).p over (s in (2mm, 160mm), u in (-120deg, 120deg)))
