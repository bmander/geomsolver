// A hypoid gear pair's pitch cones, laid out through the mean point M — the design step every
// hypoid starts from, and one no 2D sketch can state: the shafts are square and E apart, so the
// view that shows both axes in true shape has an attitude that depends on the answer.
//
// The pitch plane P holds M.  The gear's axial view G is P folded square about the vertical line
// the gear's generator lies on; the pinion's axial view Q is solved (`attitude: free, through: M`),
// its own turn held by drawing the pinion's axis level in it.  Each pitch cone is an entity about
// its axis: M is on both, and `gc tangent(M) pc` says they touch there with one tangent plane.
// The gear's apex is on P, which makes P the gear cone's tangent plane at M, and so the pinion's
// too — its apex comes out on P with nothing saying so.  Two pitch radii, the gear's pitch angle,
// the shaft angle and the offset settle the rest: 39 unknowns, 39 equations, DOF 0, the pinion's
// pitch angle γ = 29.56° and its offset angle in the pitch plane ε = 10.72°.
//
// Open the glass box (⌘B) and orbit: the two cones kiss at M on the pitch plane, their axes
// crossing square and E apart.  Set `E` to 0mm and the pair becomes a straight bevel, the axes
// meeting; edit `module`, the tooth counts `Ng` and `Np`, or the gear's 60° pitch angle, and
// the pinion's cone and its view follow.  The same hypoid can be built without naming the cones —
// each axial view folded `along` its pitch generator, square to P by construction — which is the
// spelling in the primer's §2.12 and `gcs-core/tests/fixtures/hypoid_pitch_cones.sv`; the tests
// solve both and find the same pair to 2e-11.

unit mm
param Ng = 48          // gear teeth
param Np = 24          // pinion teeth
param module = 4mm
param Rg = Ng * module / 2
param Rp = Np * module / 2
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
