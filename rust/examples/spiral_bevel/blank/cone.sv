// Step 3, a member's cones: a meridian parallel to the pitch generator at an
// offset square to it, over the generator's span from half to one and a half cone
// distances, its caps square to the axis and its spine on the axis. Revolved about
// the axis it is the tip, root or back cone.
use std

// `generator` runs from the apex to the mean point. `near` and `far` are half and one
// and a half of it from the apex, and each rib runs from one of them square to the
// generator to where it meets `axis`.
component ConeSpan(generator: line, axis: line) {
  point near hint(x: (generator.p1.x + generator.p2.x) / 2,
                  y: (generator.p1.y + generator.p2.y) / 2)
  point far hint(x: 1.5 * generator.p2.x - 0.5 * generator.p1.x,
                 y: 1.5 * generator.p2.y - 0.5 * generator.p1.y)
  private line extent(near, far)
  near midpoint generator
  generator.p2 midpoint extent
  // Seeds: where the square meets the axis, from the seeded directions,
  // `(g . g) / (2 g . a)` axes out.
  point near_cross hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x)
      * ((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2)
      / (2 * ((generator.p2.x - generator.p1.x) * (axis.p2.x - axis.p1.x)
            + (generator.p2.y - generator.p1.y) * (axis.p2.y - axis.p1.y))),
    y: axis.p1.y + (axis.p2.y - axis.p1.y)
      * ((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2)
      / (2 * ((generator.p2.x - generator.p1.x) * (axis.p2.x - axis.p1.x)
            + (generator.p2.y - generator.p1.y) * (axis.p2.y - axis.p1.y))))
  point far_cross hint(x: 3 * near_cross.x - 2 * axis.p1.x,
                       y: 3 * near_cross.y - 2 * axis.p1.y)
  line near_rib(near, near_cross)
  line far_rib(far, far_cross)
  near_rib perpendicular generator
  far_rib perpendicular generator
  near_cross on axis
  far_cross on axis
}

// The meridian stands `offset` off `generator` along each rib of its span, turned `lean`
// from the direction toward the axis: 0deg for the root and the back, 180deg for the tip.
component ConeBoundary(generator: line, axis: line, offset: Length, lean: Angle) {
  private span: ConeSpan(generator, axis)
  // Seeds: a step along each rib, toward the axis or away as `lean` says, and the feet of
  // the ends on the axis.
  private point p hint(
    x: span.near.x + (1 - lean / 90deg) * (span.near_cross.x - span.near.x) / 20,
    y: span.near.y + (1 - lean / 90deg) * (span.near_cross.y - span.near.y) / 20)
  private point q hint(
    x: span.far.x + (1 - lean / 90deg) * (span.far_cross.x - span.far.x) / 20,
    y: span.far.y + (1 - lean / 90deg) * (span.far_cross.y - span.far.y) / 20)
  private point a hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x)
      * ((p.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (p.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2),
    y: axis.p1.y + (axis.p2.y - axis.p1.y)
      * ((p.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (p.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2))
  private point b hint(
    x: axis.p1.x + (axis.p2.x - axis.p1.x)
      * ((q.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (q.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2),
    y: axis.p1.y + (axis.p2.y - axis.p1.y)
      * ((q.x - axis.p1.x) * (axis.p2.x - axis.p1.x) + (q.y - axis.p1.y) * (axis.p2.y - axis.p1.y))
      / ((axis.p2.x - axis.p1.x)^2 + (axis.p2.y - axis.p1.y)^2))
  private line near_lift(span.near, p)
  private line far_lift(span.far, q)
  span.near_rib angle(lean) near_lift
  span.far_rib angle(lean) far_lift
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
