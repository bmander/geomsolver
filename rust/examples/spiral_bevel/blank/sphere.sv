// Step 3, a member's ends: the face width centred on the mean point along the pitch generator
// (`FaceSpan`), which the blank's end spheres pass through (blank/member.sv).
use std

// The face width along `generator` (apex -> mean point), centred on the mean point: the toe
// toward the apex, the heel away from it, each on the generator, wherever it is drawn.
component FaceSpan(generator: line, width: Length) {
  // Seeds only, rough: a tenth of the generator either side of the mean point.
  toe := point hint(at: generator.p2, toward: generator.p1, by: 0.1)
  heel := point hint(at: generator.p2, toward: generator.p1, by: -0.1)
  toe coincident generator
  heel coincident generator
  toe distance(width / 2, along: generator) generator.p2
  generator.p2 distance(width / 2, along: generator) heel
}

preview {
  unit mm
  in std.front {
    mean := point
    fix((50, 0)) mean
    generator := line(std.origin, mean)
  }
  span := FaceSpan(generator, width: 10mm)
}
