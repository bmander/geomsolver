// Step 3, a member's cones: a meridian parallel to the pitch generator at an
// offset square to it, over the generator's span from half to one and a half cone
// distances, its caps square to the axis and its spine on the axis. Revolved about
// the axis it is the tip, root or back cone.
use std

// `generator` runs from the apex to the mean point. `near` and `far` are half and one
// and a half of it from the apex, and each rib runs from one of them square to the
// generator to where it meets `axis`.
component ConeSpan(generator: line, axis: line) {
  near := point hint(x: (generator.p1.x + generator.p2.x) / 2,
                  y: (generator.p1.y + generator.p2.y) / 2)
  far := point hint(x: 1.5 * generator.p2.x - 0.5 * generator.p1.x,
                 y: 1.5 * generator.p2.y - 0.5 * generator.p1.y)
  private extent := line(near, far)
  near midpoint generator
  generator.p2 midpoint extent
  // Seeds, rough: on the axis at its far end.
  near_cross := point hint(at: axis.p2)
  far_cross := point hint(at: axis.p2)
  near_rib := line(near, near_cross)
  far_rib := line(far, far_cross)
  near_rib perpendicular generator
  far_rib perpendicular generator
  near_cross on axis
  far_cross on axis
}

// The meridian stands `offset` off `generator` along each rib of its span, turned `lean`
// from the direction toward the axis: 0deg for the root and the back, 180deg for the tip.
// Its ends, `p` and `q`, are public for the end relief (blank/ends.sv).
component ConeBoundary(generator: line, axis: line, offset: Length, lean: Angle) {
  private span := ConeSpan(generator, axis)
  // Seeds, rough: a step along each rib, toward the axis or away as `lean` says, and
  // where each rib meets the axis for the feet of the ends.
  p := point hint(
    x: span.near.x + (1 - lean / 90deg) * (span.near_cross.x - span.near.x) / 20,
    y: span.near.y + (1 - lean / 90deg) * (span.near_cross.y - span.near.y) / 20)
  q := point hint(
    x: span.far.x + (1 - lean / 90deg) * (span.far_cross.x - span.far.x) / 20,
    y: span.far.y + (1 - lean / 90deg) * (span.far_cross.y - span.far.y) / 20)
  private a := point hint(at: span.near_cross)
  private b := point hint(at: span.far_cross)
  private near_lift := line(span.near, p)
  private far_lift := line(span.far, q)
  span.near_rib angle(lean) near_lift
  span.far_rib angle(lean) far_lift
  distance(offset) near_lift
  distance(offset) far_lift
  a on axis
  b on axis
  private profile := (near_cap := line(a, p)) -> (meridian := line(p, q)) ->
                    (far_cap := line(q, b)) -> (spine := line(b, a)) -> close
  near_cap perpendicular axis
  far_cap perpendicular axis
  private construction carrier := solid(profile, about: axis)
  wall := surface(carrier, meridian)
}

preview {
  unit mm
  mean := point hint(x: 50, y: 0)
  foot := point hint(x: 40, y: 20)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  generator := line(std.origin, mean)
  axis := line(std.origin, foot)
  to_foot := line(mean, foot)
  to_foot perpendicular axis
  generator angle(30deg) axis
  tip := ConeBoundary(generator, axis, offset: 2mm, lean: 180deg)
  root := ConeBoundary(generator, axis, offset: 2.5mm, lean: 0deg)
}
