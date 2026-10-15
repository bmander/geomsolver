// Geodesic circles: a front moving on a surface at unit speed, square to itself within the
// surface — on a sphere until it focuses at the antipode, and on a cone, where unrolled it is a
// circle growing about the apex. The first-order PDE of a front confined to a surface, marched
// along how far it has moved by the method of lines (#339 D4).
//
// Each body runs over `d`, how far the front has moved, marched, and `u`, an angle round its
// starting circle, sampled. Its point `p` is held to the surface by a row over `p` alone (on the
// sphere `R` from its centre, on the cone a generator at the half angle to the axis), and `nl`,
// placed at every node, is the front's normal *within the surface*: in its tangent plane and
// square to `rate(p over u)`. Then, as `front.sv`:
//
//   - the law, `rate(p over d) distance(1, along: nl)`: `p` moves along `nl` at unit speed;
//   - the gauge, `rate(p over d) parallel nl`: square to the front, so no node slides along it.
//
// The surface's row, differentiated, is one more row on `p`'s rates than they need: four rows of
// rank three, as `geodesic.sv`'s. `initially` is the Cauchy data, a circle of radius `r1` about
// the axis, its two ordinates across it; the surface's row gives the third. The seeds are
// numbers that pick each root: `p` above the centre, `nl`'s end toward the south pole on the
// sphere and up the axis on the cone, so both fronts run away from the north.
//
// On the sphere (radius R) the circle of radius 15 stands at colatitude θ₀ = 30°, and the front is
// the circle at colatitude θ₀ + d/R: every point runs down its meridian, a great circle, and the
// fronts close in on the antipode, where they focus, at d = R(π − θ₀) = 78.54. The surface is run
// there and is refused just short of it, where a node's area |x_d × x_u| = R sin(θ₀ + d/R) falls
// to a hundredth of its home's, at R(π − θ₀ − asin(sin θ₀ / 100)) = 78.39 for the closed form.
// `solventc` says where.
//
// On the cone (half angle α = 30°) the circle of radius 15 is 30 from the apex, and the front is
// the circle 30 + d from it: every point runs up its generator. Unrolled, the cone is a plane and
// the fronts are circles about the apex — geodesics on a cone are straight lines unrolled, as
// `geodesic.sv`'s.
//
// Closed forms: on the sphere x(d, u) = R (sin θ cos u, sin θ sin u, cos θ), θ = θ₀ + d/R; on the
// cone x(d, u) = ρ (sin α cos u, sin α sin u, cos α), ρ = 30 + d.
unit mm
use std

R := 30mm
r1 := 15mm
half := 30deg

component OnSphere(c: point, x1: point, y1: point, R: Length, r1: Length, d: Length, u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  p := point hint((0, 0, 30))
  k := point hint((0, 0, -30))
  rad := line(c, p)
  nl := line(p, k)
  c distance(R) p                            // p on the sphere
  length(10mm) nl
  nl perpendicular rad                       // the normal in the sphere's tangent plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(1, along: nl)      // the law: at unit speed
  initially {
    c distance(r1 * cos(u), along: xa) p     // the Cauchy data: a circle about the axis
    c distance(r1 * sin(u), along: ya) p
  }
}

component OnCone(o: point, x1: point, y1: point, top: point, half: Angle, r1: Length, d: Length,
                 u: Angle) {
  xa := line(o, x1)
  ya := line(o, y1)
  ax := line(o, top)
  p := point hint((0, 0, 30))
  k := point hint((0, 0, 200))
  w := point hint((0, 0, 50))
  gen := line(o, p)
  cn := line(p, w)
  nl := line(p, k)
  gen angle(half) ax                         // p on the cone
  w coincident ax
  cn perpendicular gen                       // cn: the cone's normal at p
  length(10mm) nl
  nl perpendicular cn                        // the front's normal in the tangent plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge
  rate(p over d) distance(1, along: nl)      // the law
  initially {
    o distance(r1 * cos(u), along: xa) p     // the Cauchy data: a circle about the axis
    o distance(r1 * sin(u), along: ya) p
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
top := point hint((0, 0, 10))
fix((0, 0, 10)) top

on_sphere := OnSphere(c, x1, y1, R: R, r1: r1).p over (d in (0mm, pi * R), u in (0deg, 360deg))
on_cone := OnCone(c, x1, y1, top, half: half, r1: r1).p over (d in (0mm, 30mm),
                                                            u in (0deg, 360deg))
