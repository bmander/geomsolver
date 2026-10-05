// Step 3, a member's cones: a meridian parallel to the pitch generator, a stated offset off it,
// spanning half to one and a half cone distances, its caps square to the axis and its spine on
// the axis. Revolved, it is the tip, root or back cone.
use std

// Half and one and a half of `generator` (apex -> mean point) from the apex, `near` and `far`,
// each with a rib square to the generator to where it meets `axis`.
component ConeSpan(generator: line, axis: line) {
  near := point hint(at: generator.p1, toward: generator.p2, by: 0.5)
  far := point hint(at: generator.p1, toward: generator.p2, by: 1.5)
  private extent := line(near, far)
  near midpoint generator
  generator.p2 midpoint extent
  // Seeds, rough: at the axis's far end.
  near_cross := point hint(at: axis.p2)
  far_cross := point hint(at: axis.p2)
  near_rib := line(near, near_cross)
  far_rib := line(far, far_cross)
  near_rib perpendicular generator
  far_rib perpendicular generator
  near_cross coincident axis
  far_cross coincident axis
}

// The meridian stands `offset` off `generator` along each rib, turned `lean` from the direction
// toward the axis: 0deg for the root and the back, 180deg for the tip. Its ends `p` and `q` are
// public for the end relief (blank/ends.sv).
component ConeBoundary(generator: line, axis: line, offset: Length, lean: Angle) {
  private span := ConeSpan(generator, axis)
  // Seeds, rough: a step along each rib turned `lean`, as the meridian stands, and the feet of
  // the caps where the ribs meet the axis.
  p := point hint(at: span.near, toward: span.near_cross, by: 0.05, turn: lean)
  q := point hint(at: span.far, toward: span.far_cross, by: 0.05, turn: lean)
  private a := point hint(at: span.near_cross)
  private b := point hint(at: span.far_cross)
  private near_lift := line(span.near, p)
  private far_lift := line(span.far, q)
  span.near_rib angle(lean) near_lift
  span.far_rib angle(lean) far_lift
  distance(offset) near_lift
  distance(offset) far_lift
  a coincident axis
  b coincident axis
  private profile := (near_cap := line(a, p)) -> (meridian := line(p, q)) ->
                     (far_cap := line(q, b)) -> (spine := line(b, a)) -> close
  near_cap perpendicular axis
  far_cap perpendicular axis
  private construction carrier := solid(profile, about: axis)
  wall := surface(carrier, meridian)
}

preview {
  unit mm
  in std.front {
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
}
