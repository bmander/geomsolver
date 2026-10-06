// Generation by motion, in the plane: a rack cuts a pinion's tooth, and the tooth cuts its mate.
//
// Nothing here states an involute.  `tooth` is the envelope of the rack's straight flank as the
// rack rolls on the pinion's pitch circle; `mate` is the envelope of `tooth` itself as the
// pinion and the gear turn together.  Each is a curve of the drawing, so the solve sees it: the
// flank's foot on the pitch line is this drawing's one freedom — drag `f0` up and down the pitch
// line and the tooth thickens or thins, and its mate follows.
//
// Where the two are drawn in mesh (roll 0) a circle osculates the mate, and the claim at the
// bottom asks whether its centre must lie on the gear's base circle — the evolute of an
// involute, and Euler–Savary's centre of curvature on the common normal.  The diagnosis judges
// it a theorem: true at every pose the freedom reaches, and implied rather than stated.

use std

r1 := 20                    // the pinion's pitch radius
r2 := 30                    // the gear's
alpha := 20deg              // the rack's pressure angle

in std.front {
  o1 := point
  o2 := point hint((50, 0))
  fix((0, 0)) o1
  o1 distance(r1 + r2, along: x) o2
  o1 distance(0, along: y) o2

  // the rack's pitch line, tangent to the pinion's pitch circle at the pitch point
  p0 := point hint((20, 0))
  p1 := point hint((20, 10))
  o1 distance(r1, along: x) p0
  o1 distance(0, along: y) p0
  pitch_line := vertical line(p0, p1)
  p0 distance(10) p1

  // the rack's flank, drawn where it stands at roll 0: its foot on the pitch line, at the
  // pressure angle from the radius
  f0 := point hint((20, 2))
  f1 := point hint((29.4, 5.4))
  radial := line(o1, p0)
  flank := line(f0, f1)
  f0 coincident pitch_line
  f0 distance(10) f1
  radial angle(alpha) flank
}

// the pinion and the gear turn together at the ratio of their pitch radii; the rack rolls on
// the pinion's pitch circle
pinion := motion(about: o1, ratio: 1)
gear := motion(about: o2, ratio: -r1 / r2)
rack := motion(along: pitch_line, advance: 2 * pi * r1)
cutting := motion(rack, relative_to: pinion)
meshing := motion(pinion, relative_to: gear)

tooth := envelope(flank, under: cutting, from: -40deg, to: 40deg)
mate := envelope(tooth, under: meshing, from: -15deg, to: 15deg)

// the mate's curvature where the two mesh, and where its centre must lie
in std.front {
  k := point hint((23.5, -9.6))
  osc := circle(center: k) hint(r: 6)
  mate curvature(t == 0) osc
  base2 := circle(center: o2) hint(r: 28.2)
  radius(r2 * cos(alpha)) base2
  claim k coincident base2
}
