// A hypoid gear pair's pitch cones, laid out through the mean point M — the design step every
// hypoid starts from, and one no 2D sketch can state: the shafts are square and E apart, so the
// view that shows both axes in true shape has an attitude that depends on the answer.
//
// The pitch plane P, the front plane, holds M.  The gear's axial plane G stands square to P on
// the vertical line the gear's generator lies on; the pinion's axial plane Q stands on two axes
// the solve turns, square to each other, with its origin at M — its own turn held by drawing the
// pinion's axis level in it.  Each pitch cone is an entity about
// its axis: M is on both, and `gc tangent(M) pc` says they touch there with one tangent plane.
// The gear's apex is on P, which makes P the gear cone's tangent plane at M, and so the pinion's
// too — its apex comes out on P with nothing saying so.  Two pitch radii, the gear's pitch angle,
// the shaft angle and the offset settle the rest: 37 unknowns, 37 equations, DOF 0, the pinion's
// pitch angle γ = 29.56° and its offset angle in the pitch plane ε = 10.72°.
//
// Open the glass box (⌘B) and orbit: the two cones kiss at M on the pitch plane, their axes
// crossing square and E apart.  Set `E` to 0mm and the pair becomes a straight bevel, the axes
// meeting; edit `module`, the tooth counts `Ng` and `Np`, or the gear's 60° pitch angle, and
// the pinion's cone and its plane follow.  The same hypoid can be built without naming the cones —
// each axial plane standing on its pitch generator, square to P by construction — which is the
// spelling in the primer's §2.12 and `gcs-core/tests/fixtures/hypoid_pitch_cones.sv`; the tests
// solve both and find the same pair to 2e-11.

unit mm
use std
Ng := 48          // gear teeth
Np := 24          // pinion teeth
module := 4mm
Rg := Ng * module / 2
Rp := Np * module / 2
E := 20mm

// the pitch plane P is the front plane, and M is on it
M := point in std.front
fix(x == 0, y == 0) M

// the gear's axial plane, square to P about its vertical axis; the pinion's, solved, through M
G := plane(u: std.z, v: std.y)
fix(x == 0, y == 0, z == 0) G
qu := axis hint(x: 0.1618, y: -0.4935, z: -0.8546)
qv := axis hint(x: 0.0918, y: 0.8698, z: -0.4847)
qu perpendicular qv
Q := plane(u: qu, v: qv)
M coincident Q.origin

// each axis from its apex, drawn at a length that says nothing about the cone; the pinion's
// drawn level in its plane, which is where the plane's own turn is held
gax := line(hint(x: 110.85, y: 0), hint(x: 50.85, y: 103.9)) in G
pax := line(hint(x: -84.62, y: -48), hint(x: -4.62, y: -48)) in Q
gax.p1 coincident std.front
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2
horizontal pax

// the pitch cones: the gear's pitch angle, M on both, and the two touching there
gc := cone(axis: gax) hint(half: 60deg)
pc := cone(axis: pax) hint(half: 30deg)
angle(60deg) gc
M coincident gc
M coincident pc
gc tangent(M) pc

// the two pitch radii at M, and the shafts: square, and E apart
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
