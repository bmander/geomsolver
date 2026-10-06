// A sphere, a cone and a cylinder: three surfaces a relation can reach in space, each the
// library's (`std.Sphere`, `std.Cone`, `std.Cylinder`), built about geometry drawn in a view and
// each owning one number — a radius, a half-angle, a radius.  None is drawn: what stands on a
// sphere or a cylinder is a distance from its centre or its axis, and what stands on a cone an
// angle at its apex (`std.PointOnCone`).
//
// Two planes: the front and the side, square to it at the origin, so the side plane is the one
// the front sees edge-on along its vertical axis.  Everything in the front plane is grounded;
// what is drawn in the side plane is placed by the surfaces.
//
// - A shaft runs square through the side view.  A line drawn there from a grounded end, 50 long,
//   touches it: in space, their common perpendicular is the shaft's radius.
// - A ball is centred in the front view.  A point of the side view is the ball's radius from its
//   centre — one equation for two coordinates, so the point keeps one freedom: the circle the
//   side view cuts from the ball.
// - Two cones: one about a vertical axis, its half-angle stated; the other about a level axis
//   whose apex may slide along it and whose half-angle is not stated.  A point M of the side view
//   is on both, and `std.TangentCones(k1, k2, M)` says they touch there with one tangent plane —
//   two equations more.  Four equations, four unknowns (M's two, the second apex, its
//   half-angle).
//
// DOF 1, and it is the point on the ball.  Edit the first cone's `half` and the second cone
// reopens and slides to keep touching; edit the shaft's radius and the tangent line swings.  Back
// on the side plane, drag `pb` and it runs round its circle on the ball.

unit mm
use std

// a shaft square to the side plane, and a line in that plane touching it
in std.front {
  ax := line
  fix((-60, 15)) ax.p1
  fix((20, 15)) ax.p2
}
shaft := std.Cylinder(ax, r: 8)
in std.side {
  l := line(p2: hint((0, 35)))
  fix((30, -10)) l.p1
  l.p1 distance(50) l.p2
}
shaft.axis distance(shaft.r) l

// a ball centred in the front plane, and a point of the side plane on it
in std.front {
  bc := point
  fix((-8, 38)) bc
}
ball := std.Sphere(bc, r: 12)
pb := point hint((5, 45)) in std.side
pb distance(ball.r) ball.center

// two cones touching at a point
in std.front {
  kax := line
  fix((25, 0)) kax.p1
  fix((25, 70)) kax.p2
  jax := line(hint((-42, 82)))
  fix((18, 82)) jax.p2
  horizontal jax
}
k1 := std.Cone(kax, half: 30deg)       // its apex is the axis's start; it opens toward the end
k2 := std.Cone(jax, half: hint(38deg))
M := point hint((-25, 61)) in std.side
std.PointOnCone(M, k1)
std.PointOnCone(M, k2)
std.TangentCones(k1, k2, M)
