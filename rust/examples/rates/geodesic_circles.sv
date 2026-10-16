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
// rank three, as `geodesic.sv`'s. `initially` is the Cauchy data: the starting circle is `s`, a
// drawn point 15 from the axis, turned about the axis by `u` — a point the spin carries
// (#377), three rows, and the surface's row a fourth that agrees with them, `s` being on both
// surfaces. The seeds are numbers that pick each root: `p` above the centre, `nl`'s end toward
// the south pole on the sphere and up the axis on the cone, so both fronts run away from the
// north.
//
// On the sphere (radius R) the circle of radius 15 stands at colatitude θ₀ = 30°, and the front is
// the circle at colatitude θ₀ + d/R: every point runs down its meridian, a great circle, and the
// fronts close in on the antipode, where they focus, at d = R(π − θ₀) = 78.54. The surface is run
// there and is refused just short of it, where a node's area |x_d × x_u| = R sin(θ₀ + d/R) falls
// to a hundredth of its home's, at R(π − θ₀ − asin(sin θ₀ / 100)) = 78.39 for the closed form.
// `solventc` says where.
//
// On the cone (half angle α = 30°) the circle of radius 15 is 30 from the apex — the sphere's
// circle — and the front is the circle 30 + d from it: every point runs up its generator.
// Unrolled, the cone is a plane and the fronts are circles about the apex — geodesics on a cone
// are straight lines unrolled, as `geodesic.sv`'s.
//
// Closed forms: on the sphere x(d, u) = R (sin θ cos u, sin θ sin u, cos θ), θ = θ₀ + d/R; on the
// cone x(d, u) = ρ (sin α cos u, sin α sin u, cos α), ρ = 30 + d.
unit mm
use std

R := 30mm
r1 := 15mm
half := 30deg

component OnSphere(c: point, s: point, spin: frame, R: Length, d: Length, u: Angle) {
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
    p coincident(under: spin, t == u) s      // the Cauchy data: s turned about the axis
  }
}

component OnCone(o: point, top: point, s: point, spin: frame, half: Angle, d: Length,
                 u: Angle) {
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
    p coincident(under: spin, t == u) s      // the Cauchy data: s turned about the axis
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
top := point hint((0, 0, 10))
fix((0, 0, 10)) top
za := line(c, top)
spin := frame
std.Turn(spin, about: za, ratio: 1)      // a turn about the axis, a degree a degree

// the starting circle's point: r1 from the axis, on the sphere (and so on the cone)
s := point hint((15, 0, 26))
fix(x == r1, y == 0mm) s
c distance(R) s

on_sphere := OnSphere(c, s, spin, R: R).p over (d in (0mm, pi * R), u in (0deg, 360deg))
on_cone := OnCone(c, top, s, spin, half: half).p over (d in (0mm, 30mm), u in (0deg, 360deg))
