// Step 3, a member's ends: the face width centred on the mean point along the
// pitch generator, and the toe and heel spheres about the apex through its ends.
// A sphere's poles stand square to the generator, and its meridian runs from
// pole to pole through the rim, so it turns about its diameter clear of the cones.
use std

// The face width along `generator` (apex -> mean point), centred on the mean point.
component FaceSpan(generator: line, width: Length) {
  // Seeds only: the generator's seeded direction, scaled to half the width.
  point toe hint(x: generator.p2.x - width / 2 * (generator.p2.x - generator.p1.x)
      / sqrt((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2),
    y: generator.p2.y - width / 2 * (generator.p2.y - generator.p1.y)
      / sqrt((generator.p2.x - generator.p1.x)^2 + (generator.p2.y - generator.p1.y)^2))
  point heel hint(x: 2 * generator.p2.x - toe.x, y: 2 * generator.p2.y - toe.y)
  line span(toe, heel)
  generator.p2 midpoint span
  distance(width) span
  generator angle(0deg) span
}

// The sphere about `apex` through `rim`, a point of `generator` (apex -> mean point).
component SphericalBoundary(apex: point, generator: line, rim: point) {
  // Seeds: the rim turned a quarter either way about the apex.
  private point bottom hint(x: apex.x + rim.y - apex.y, y: apex.y - rim.x + apex.x)
  private point top hint(x: apex.x - rim.y + apex.y, y: apex.y + rim.x - apex.x)
  private arc meridian(center: apex, start: bottom, end: top)
  private line diameter(top, bottom)
  apex on diameter
  generator angle(90deg, sense: cw) diameter
  rim on meridian
  private face profile(meridian, diameter)
  private construction solid carrier(profile, about: diameter)
  surface wall(carrier, meridian)
}

preview {
  unit mm
  point mean hint(x: 50, y: 0)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  line generator(std.origin, mean)
  span: FaceSpan(generator, width: 10mm)
  toe: SphericalBoundary(std.origin, generator, span.toe)
  heel: SphericalBoundary(std.origin, generator, span.heel)
}
