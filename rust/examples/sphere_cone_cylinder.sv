// A sphere, a cone and a cylinder: the three surfaces a relation can reach in space, each built
// about geometry drawn in a view and each owning one number — a radius, a half-angle, a radius.
// None is on the sheet; the glass box draws a sphere as three great circles, and a cone or a
// cylinder as two rings square to its axis and four rulings.
//
// Two planes: the front and the side, square to it at the origin, so the side plane is the one
// the front sees edge-on along its vertical axis.  Everything in the front plane is grounded;
// what is drawn in the side plane is placed by the surfaces.
//
// - A shaft runs square through the side view.  A line drawn there from a grounded end, 50 long,
//   is `tangent` to it: in space, their common perpendicular is the shaft's radius.
// - A ball is centred in the front view.  A point of the side view is `coincident` it — one
//   equation for two coordinates, so the point keeps one freedom: the circle the side view cuts
//   from the ball.
// - Two cones: one about a vertical axis, its half-angle stated; the other about a level axis
//   whose apex may slide along it and whose half-angle is not stated.  A point M of the side view
//   is on both, and `k1 tangent(M) k2` says they touch there with one tangent plane — two
//   equations more.  Four equations, four unknowns (M's two, the second apex, its half-angle).
//
// 45 unknowns and 44 equations: DOF 1, and it is the point on the ball.  Open the glass box (⌘B)
// and orbit to see the two cones kiss at M; edit `angle(30deg) k1` and the second cone reopens
// and slides to keep touching; edit the shaft's radius and the tangent line swings.  Back on the
// side plane, drag `pb` and it runs round its circle on the ball.

unit mm
use std

// a shaft square to the side plane, and a line in that plane touching it
in std.front {
  ax := line
  fix((-60, 15)) ax.p1
  fix((20, 15)) ax.p2
}
shaft := cylinder(axis: ax) hint(r: 8)
radius(8) shaft
in std.side {
  l := line(p2: hint((0, 35)))
  fix((30, -10)) l.p1
  l.p1 distance(50) l.p2
}
shaft tangent l

// a ball centred in the front plane, and a point of the side plane on it
in std.front {
  bc := point
  fix((-8, 38)) bc
  ball := sphere(center: bc) hint(r: 12)
  radius(12) ball
}
pb := point hint((5, 45)) in std.side
pb coincident ball

// two cones touching at a point
in std.front {
  kax := line
  fix((25, 0)) kax.p1
  fix((25, 70)) kax.p2
  jax := line(hint((-42, 82)))
  fix((18, 82)) jax.p2
  horizontal jax
}
k1 := cone(axis: kax) hint(half: 30deg)       // its apex is the axis's start; it opens toward the end
angle(30deg) k1
k2 := cone(axis: jax) hint(half: 38deg)
M := point hint((-25, 61)) in std.side
M coincident k1
M coincident k2
k1 tangent(M) k2
