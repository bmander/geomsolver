// Step 3, a member's cones: a meridian parallel to the pitch generator at an
// offset square to it, from half to one and a half cone distances along it, its
// caps square to the axis and its spine on the axis. Revolved about the axis it
// is the tip, root or back cone.
use std

// `generator` runs from the apex to the mean point. The meridian stands `offset` off it
// along the square to it, turned `lean` from the direction toward the axis: 0deg for the
// root and the back, 180deg for the tip.
component ConeBoundary(generator: line, axis: line, offset: Length, lean: Angle) {
  private point near hint(x: (generator.p1.x + generator.p2.x) / 2,
                          y: (generator.p1.y + generator.p2.y) / 2)
  private point far hint(x: 1.5 * generator.p2.x - 0.5 * generator.p1.x,
                         y: 1.5 * generator.p2.y - 0.5 * generator.p1.y)
  private line span(near, far)
  near midpoint generator
  generator.p2 midpoint span
  // Each end stands on the square to the generator through it, which meets the axis.
  // Seeds: where the squares through the ends meet the axis, from its seeded direction.
  private point near_cross hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x) * ((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2)
      / (2 * ((generator.p2.x - generator.p1.x) * (axis.p2.x - axis.p1.x) + (generator.p2.y - generator.p1.y) * (axis.p2.y - axis.p1.y))),
    y: axis.p1.y + (axis.p2.y - axis.p1.y) * ((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2)
      / (2 * ((generator.p2.x - generator.p1.x) * (axis.p2.x - axis.p1.x) + (generator.p2.y - generator.p1.y) * (axis.p2.y - axis.p1.y))))
  private point far_cross hint(x: 3 * near_cross.x - 2 * axis.p1.x, y: 3 * near_cross.y - 2 * axis.p1.y)
  private line near_rib(near, near_cross)
  private line far_rib(far, far_cross)
  near_rib perpendicular generator
  far_rib perpendicular generator
  near_cross on axis
  far_cross on axis
  // Seeds: a step along each rib, toward the axis or away as `lean` says.
  private point p hint(x: near.x + (1 - lean / 90deg) * (near_cross.x - near.x) / 20,
                       y: near.y + (1 - lean / 90deg) * (near_cross.y - near.y) / 20)
  private point q hint(x: far.x + (1 - lean / 90deg) * (far_cross.x - far.x) / 20,
                       y: far.y + (1 - lean / 90deg) * (far_cross.y - far.y) / 20)
  // Seeds: the feet on the axis of the meridian's ends.
  private point a hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x) * ((p.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (p.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2),
    y: axis.p1.y + (axis.p2.y - axis.p1.y) * ((p.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (p.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2))
  private point b hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x) * ((q.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (q.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2),
    y: axis.p1.y + (axis.p2.y - axis.p1.y) * ((q.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (q.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2))
  private line near_lift(near, p)
  private line far_lift(far, q)
  near_rib angle(lean) near_lift
  far_rib angle(lean) far_lift
  distance(offset) near_lift
  distance(offset) far_lift
  a on axis
  b on axis
  private profile = line near_cap(a, p) -> line meridian(p, q) ->
                    line far_cap(q, b) -> line spine(b, a) -> close
  near_cap perpendicular axis
  far_cap perpendicular axis
  private construction solid carrier(profile, about: axis)
  surface wall(carrier, meridian)
}

preview {
  unit mm
  point mean hint(x: 50, y: 0)
  point foot hint(x: 40, y: 20)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  line generator(std.origin, mean)
  line axis(std.origin, foot)
  line to_foot(mean, foot)
  to_foot perpendicular axis
  generator angle(30deg) axis
  tip: ConeBoundary(generator, axis, offset: 2mm, lean: 180deg)
  root: ConeBoundary(generator, axis, offset: 2.5mm, lean: 0deg)
}
