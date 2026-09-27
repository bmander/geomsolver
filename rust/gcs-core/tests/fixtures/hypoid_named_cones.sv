// The hypoid of `hypoid_pitch_cones.sv`, spelled with its pitch cones named: the same
// design — 48 and 24 teeth at a 4 mm module, square shafts 20 mm apart, a 60 degree gear pitch
// angle — and the same pitch plane P through the mean point M.  There the common pitch plane held
// by construction, each axial view folded square to P along its generator; here each cone is an
// entity, and the contact is stated: M is on both, and the two touch there (`gc tangent(M) pc`).
//
// The gear's axial plane G is P folded square about its vertical axis, the line the gear's
// generator lies on, and the gear's apex is on P (`gax.p1 on P`): that makes P the gear cone's
// tangent plane at M.  The pinion's axial plane Q is solved, through M, its own turn held by
// drawing the axis level in it; that the pinion's apex is on P is not said, since two cones with
// one tangent plane at M have it there already.
unit mm
param module = 4mm
param Rg = 48 * module / 2
param Rp = 24 * module / 2
param E = 20mm

// the pitch plane, and M on it
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane P(origin: o, toward: t)
ground o
ground t
point M hint(x: 0, y: 0) in P
ground M

// the gear's axial plane, square to P about its vertical axis; the pinion's, solved, through M
point og hint(x: 0, y: 200)
point tg hint(x: 40, y: 200)
plane G(origin: og, toward: tg, from: P, fold: 90deg)
ground og
ground tg
point oq hint(x: 0, y: -200)
point tq hint(x: 40, y: -200)
plane Q(origin: oq, toward: tq, attitude: free, through: M) hint(u: (0.1618, -0.4935, -0.8546), v: (0.0918, 0.8698, -0.4847))

// each axis from its apex, drawn at a length that says nothing about the cone; the pinion's
// drawn level in its view, which is where the view's own turn is held
line gax(hint(x: 110.85, y: 200), hint(x: 50.85, y: 303.9)) in G
line pax(hint(x: -84.62, y: -248), hint(x: -4.62, y: -248)) in Q
gax.p1 on P
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2
horizontal pax

// the pitch cones: the gear's pitch angle, M on both, and the two touching there
cone gc(axis: gax) hint(half: 60deg)
cone pc(axis: pax) hint(half: 30deg)
angle(60deg) gc
M on gc
M on pc
gc tangent(M) pc

// the two pitch radii at M, and the shafts: square, and E apart
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
