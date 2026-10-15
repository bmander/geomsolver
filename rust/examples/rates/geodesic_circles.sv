// Geodesic circles: a front moving on a surface at unit speed, square to itself within the
// surface — on a sphere until it focuses at the antipode, and on a cone, where unrolled it is a
// circle growing about its centre. The first-order PDE of a front confined to a surface, marched
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
// rank three, as `geodesic.sv`'s. `initially` is the Cauchy data, two coordinates of a circle;
// the surface's row gives the third.
//
// On the sphere (radius R, the front starting at colatitude θ₀) the front is the circle at
// colatitude θ₀ + d/R: every point runs down its meridian, a great circle, and the fronts close
// in on the antipode, where they focus, at d = R(π − θ₀) = 78.54. The surface is run there and
// is refused just short of it, where a node's area |x_d × x_u| = R sin(θ₀ + d/R) falls to a
// hundredth of its home's, at R(π − θ₀ − asin(sin θ₀ / 100)) = 78.39 for the closed form.
// `solventc` says where.
//
// On the cone (half angle α) the front sets off from a circle drawn on the cone about `P`, the
// point `rp` up a generator: unrolled, the cone is a plane, its generator through `P` a ray from
// the apex, and the starting circle is the circle of radius r0 about `P`'s image. Unrolled the
// front stays a circle about `P`, of radius r0 + d — geodesics on a cone are straight lines
// unrolled, as `geodesic.sv`'s. It is run while the circle stays clear of the apex.
//
// Closed forms: on the sphere x(d, u) = R (sin θ cos u, sin θ sin u, cos θ), θ = θ₀ + d/R. On the
// cone, unrolled with the apex at the origin and P at (rp, 0), the point at
// (rp + (r0 + d) cos u, (r0 + d) sin u): ρ its distance from the apex, its angle there turned
// 1/sin α about the axis.
unit mm
use std

R := 30mm
t0 := 30deg
half := 30deg
rp := 50mm
r0 := 10mm

component OnSphere(c: point, x1: point, y1: point, R: Length, t0: Angle, d: Length, u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  p := point hint((R * sin(t0) * cos(u), R * sin(t0) * sin(u), R * cos(t0)))
  k := point hint((R * sin(t0 + 15deg) * cos(u), R * sin(t0 + 15deg) * sin(u),
                   R * cos(t0 + 15deg)))
  rad := line(c, p)
  nl := line(p, k)
  c distance(R) p                            // p on the sphere
  length(10mm) nl
  nl perpendicular rad                       // the normal in the sphere's tangent plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(1, along: nl)      // the law: at unit speed
  initially {
    c distance(R * sin(t0) * cos(u), along: xa) p   // the Cauchy data: a circle of latitude
    c distance(R * sin(t0) * sin(u), along: ya) p
  }
}

component OnCone(o: point, x1: point, y1: point, top: point, half: Angle, rp: Length,
                 r0: Length, d: Length, u: Angle) {
  xa := line(o, x1)
  ya := line(o, y1)
  ax := line(o, top)
  // the starting circle unrolled, its point `rho` from the apex at `az` round the axis; and the
  // seed of the normal's end, on the circle half as large again
  rho := hypot(rp + r0 * cos(u), r0 * sin(u))
  az := atan2(r0 * sin(u), rp + r0 * cos(u)) / sin(half)
  rk := hypot(rp + 1.5 * r0 * cos(u), 1.5 * r0 * sin(u))
  ak := atan2(1.5 * r0 * sin(u), rp + 1.5 * r0 * cos(u)) / sin(half)
  p := point hint((rho * sin(half) * cos(az), rho * sin(half) * sin(az), rho * cos(half)))
  k := point hint((rk * sin(half) * cos(ak), rk * sin(half) * sin(ak), rk * cos(half)))
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
    o distance(rho * sin(half) * cos(az), along: xa) p   // the Cauchy data: a circle about P
    o distance(rho * sin(half) * sin(az), along: ya) p
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

on_sphere := OnSphere(c, x1, y1, R: R, t0: t0).p over (d in (0mm, pi * R), u in (0deg, 360deg))
on_cone := OnCone(c, x1, y1, top, half: half, rp: rp, r0: r0).p over (d in (0mm, 20mm),
                                                                     u in (0deg, 360deg))
