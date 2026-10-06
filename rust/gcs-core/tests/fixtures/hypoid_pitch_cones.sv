// A hypoid's pitch cones, laid out through the mean point M (docs/spiral-bevel-layout-plan.md,
// docs/spatial-constraints-plan.md P3).  The pitch plane P is the front plane; each axial plane
// stands square to it over its member's pitch generator, so each axis is drawn in the plane of
// its generator and P is the tangent plane of both pitch cones along their generators: the
// common pitch plane holds by construction and nothing states it.
//
// The design: 48 and 24 teeth at a 4 mm module give the mean pitch radii, the shafts are square
// and offset by 20 mm, and the gear's pitch angle is chosen at 60 degrees.  That last is the one
// condition the pitch cones need beyond the radii, the shaft angle and the offset: the gear's
// apex, the pinion's apex and the pinion's pitch angle are then three unknowns held by three
// equations (the pinion's radius, the shaft angle and the offset), and the gear cone by its own
// two (its radius and its pitch angle).
unit mm
use std
module := 4mm
Rg := 48 * module / 2
Rp := 24 * module / 2
E := 20mm

// the pitch plane, and M on it
in std.front {
  M := point
  fix((0, 0)) M
  O := point hint((110, 0))
  A := point hint((95, 18))
  gen_g := line(O, M)
  gen_p := line(A, M)
  horizontal gen_g
}

// the axial planes, square to P over the generators and through M: each stands on its
// generator, and its v runs along std.y; its origin is where M is along the generator
G := plane(u: gen_g, v: hint(dir: (0, 1, 0)))
G.v parallel std.y
M level(u) G
Q := plane(u: gen_p, v: hint(dir: (0, 1, 0)))
Q.v parallel std.y
M level(u) Q

// each axis in its axial plane, from its apex: the apex's image is on P and projects to the apex
// drawn in P; how long an axis is drawn says nothing about the cone
gax := line(hint((-110, 0)), hint((-50, 104))) in G
pax := line(hint((-97, 0)), hint((-28, -39))) in Q
gax.p1 coincident std.front
O project gax.p1
pax.p1 coincident std.front
A project pax.p1
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2

// the gear's pitch angle, the two pitch radii at M, and the shafts: square, and E apart
gen_g angle(60deg) gax
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
