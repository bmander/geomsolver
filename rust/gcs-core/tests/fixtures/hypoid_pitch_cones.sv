// A hypoid's pitch cones, laid out through the mean point M (docs/spiral-bevel-layout-plan.md,
// docs/spatial-constraints-plan.md P3).  The pitch plane P is stated; each axial view is folded
// square to it along its member's pitch generator, so each axis is drawn in the plane of its
// generator and P is the tangent plane of both pitch cones along their generators: the common
// pitch plane holds by construction and nothing states it.
//
// The design: 48 and 24 teeth at a 4 mm module give the mean pitch radii, the shafts are square
// and offset by 20 mm, and the gear's pitch angle is chosen at 60 degrees.  That last is the one
// condition the pitch cones need beyond the radii, the shaft angle and the offset: the gear's
// apex, the pinion's apex and the pinion's pitch angle are then three unknowns held by three
// equations (the pinion's radius, the shaft angle and the offset), and the gear cone by its own
// two (its radius and its pitch angle).
unit mm
param module = 4mm
param Rg = 48 * module / 2
param Rp = 24 * module / 2
param E = 20mm

// the pitch plane, and M on it
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane P(origin: o, toward: t)
point M hint(x: 0, y: 0) in P
ground M
point O hint(x: 110, y: 0) in P
point A hint(x: 95, y: 18) in P
line gen_g(O, M) in P
line gen_p(A, M) in P
horizontal gen_g

// the axial views, folded square to P along the generators
point og hint(x: 0, y: 200)
point tg hint(x: 40, y: 200)
plane G(origin: og, toward: tg, from: P, fold: along gen_g)
point oq hint(x: 0, y: -200)
point tq hint(x: 40, y: -200)
plane Q(origin: oq, toward: tq, from: P, fold: along gen_p)

// each axis in its axial view, from its apex: the apex's image is on the fold line (on P) and
// projects to the apex drawn in P; how long an axis is drawn says nothing about the cone
line gax(hint(x: -110, y: 200), hint(x: -50, y: 304)) in G
line pax(hint(x: -97, y: -200), hint(x: -28, y: -239)) in Q
gax.p1 on P
O project gax.p1
pax.p1 on P
A project pax.p1
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2

// the gear's pitch angle, the two pitch radii at M, and the shafts: square, and E apart
gen_g angle(60deg) gax
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
