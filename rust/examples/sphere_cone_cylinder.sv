// A sphere, a cone and a cylinder: the three surfaces a relation can reach in space, each built
// about geometry drawn in a view and each owning one number — a radius, a half-angle, a radius.
// None is on the sheet; the glass box draws a sphere as three great circles, and a cone or a
// cylinder as two rings square to its axis and four rulings.
//
// Two views: the front (the page) and a side view folded square to it at the front's origin, so
// the side view is the plane the front view sees edge-on along its vertical axis.  Everything in
// the front view is grounded; what is drawn in the side view is placed by the surfaces.
//
// - A shaft runs square through the side view.  A line drawn there from a grounded end, 50 long,
//   is `tangent` to it: in space, their common perpendicular is the shaft's radius.
// - A ball is centred in the front view.  A point of the side view is `on` it — one equation for
//   two coordinates, so the point keeps one freedom: the circle the side view cuts from the ball.
// - Two cones: one about a vertical axis, its half-angle stated; the other about a level axis
//   whose apex may slide along it and whose half-angle is not stated.  A point M of the side view
//   is on both, and `k1 tangent(M) k2` says they touch there with one tangent plane — two
//   equations more.  Four equations, four unknowns (M's two, the second apex, its half-angle).
//
// 51 unknowns and 50 equations: DOF 1, and it is the point on the ball.  Open the glass box (⌘B)
// and orbit to see the two cones kiss at M; edit `angle(30deg) k1` and the second cone reopens
// and slides to keep touching; edit the shaft's radius and the tangent line swings.  Back on the
// sheet, drag `pb` and it runs round its circle on the ball.

unit mm
o := point hint(x: 0, y: 0)
t := point hint(x: 40, y: 0)
front := plane(origin: o, toward: t)
o2 := point hint(x: 150, y: 0)
t2 := point hint(x: 150, y: -40)
side := plane(origin: o2, toward: t2, from: front, fold: -90deg)   // turned so up is up
ground o
ground t
ground o2
ground t2

// a shaft square to the side view, and a line in that view touching it
ax := line(hint(x: -60, y: 15), hint(x: 20, y: 15)) in front
ground ax.p1
ground ax.p2
shaft := cylinder(axis: ax) hint(r: 8)
radius(8) shaft
l := line(hint(x: 180, y: -10), hint(x: 150, y: 35)) in side
ground l.p1
l.p1 distance(50) l.p2
shaft tangent l

// a ball centred in the front view, and a point of the side view on it
bc := point hint(x: -8, y: 38) in front
ground bc
ball := sphere(center: bc) hint(r: 12)
radius(12) ball
pb := point hint(x: 155, y: 45) in side
pb on ball

// two cones touching at a point
kax := line(hint(x: 25, y: 0), hint(x: 25, y: 70)) in front
ground kax.p1
ground kax.p2
k1 := cone(axis: kax) hint(half: 30deg)       // its apex is the axis's start; it opens toward the end
angle(30deg) k1
jax := line(hint(x: -42, y: 82), hint(x: 18, y: 82)) in front
ground jax.p2
horizontal jax
k2 := cone(axis: jax) hint(half: 38deg)
M := point hint(x: 125, y: 61) in side
M on k1
M on k2
k1 tangent(M) k2
