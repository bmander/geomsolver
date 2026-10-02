// Step 3, a member's ends: the face width centred on the mean point along the
// pitch generator, and the toe and heel spheres about the apex through its ends.
// A sphere's poles stand square to the generator, and its meridian runs from
// pole to pole through the rim, so it turns about its diameter clear of the cones.
use std

// The face width along `generator` (apex -> mean point), centred on the mean point.
component FaceSpan(generator: line, width: Length) {
  // Seeds only, rough: a tenth of the generator back from the mean point.
  toe := point hint(x: generator.p2.x - (generator.p2.x - generator.p1.x) / 10,
                 y: generator.p2.y - (generator.p2.y - generator.p1.y) / 10)
  heel := point hint(x: 2 * generator.p2.x - toe.x, y: 2 * generator.p2.y - toe.y)
  span := line(toe, heel)
  generator.p2 midpoint span
  distance(width) span
  generator angle(0deg) span
}

// The sphere about `apex` through `rim`, a point of `generator` (apex -> mean point).
component SphericalBoundary(apex: point, generator: line, rim: point) {
  // Seeds: the rim turned a quarter either way about the apex.
  private bottom := point hint(x: apex.x + rim.y - apex.y, y: apex.y - rim.x + apex.x)
  private top := point hint(x: apex.x - rim.y + apex.y, y: apex.y + rim.x - apex.x)
  private meridian := arc(center: apex, start: bottom, end: top)
  private diameter := line(top, bottom)
  apex on diameter
  generator angle(90deg, sense: cw) diameter
  rim on meridian
  private profile := face(meridian, diameter)
  private construction carrier := solid(profile, about: diameter)
  wall := surface(carrier, meridian)
}

preview {
  unit mm
  mean := point hint(x: 50, y: 0)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  generator := line(std.origin, mean)
  span := FaceSpan(generator, width: 10mm)
  toe := SphericalBoundary(std.origin, generator, span.toe)
  heel := SphericalBoundary(std.origin, generator, span.heel)
}
