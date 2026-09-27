// Step 3, a member's cones: a meridian parallel to the pitch generator at an
// offset square to it, from half to one and a half cone distances along it, its
// caps square to the axis and its spine on the axis. Revolved about the axis it
// is the tip, root or back cone.
use std

// `generator` runs from the apex to the mean point; `side` is the side of it the
// meridian stands on, away from the axis for the tip and toward it otherwise.
component ConeBoundary(generator: line, axis: line, offset: Length, side: Side) {
  private point near hint(x: (generator.p1.x + generator.p2.x) / 2,
                          y: (generator.p1.y + generator.p2.y) / 2)
  private point far hint(x: 1.5 * generator.p2.x - 0.5 * generator.p1.x,
                         y: 1.5 * generator.p2.y - 0.5 * generator.p1.y)
  private line span(near, far)
  near midpoint generator
  generator.p2 midpoint span
  // Each end stands on the square to the generator through it, which meets the axis.
  private point near_cross hint(at: axis.p2)
  private point far_cross hint(at: axis.p2)
  private line near_rib(near, near_cross)
  private line far_rib(far, far_cross)
  near_rib perpendicular generator
  far_rib perpendicular generator
  near_cross on axis
  far_cross on axis
  private point p hint(at: near)
  private point q hint(at: far)
  private point a hint(at: near_cross)
  private point b hint(at: far_cross)
  p on near_rib
  q on far_rib
  p distance(offset, side: side) generator
  q distance(offset, side: side) generator
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
  tip: ConeBoundary(generator, axis, offset: 2mm, side: right)
  root: ConeBoundary(generator, axis, offset: 2.5mm, side: left)
}
