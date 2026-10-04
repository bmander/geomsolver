// Conjugate cams: a lobe of any shape, turning against a partner it cuts for itself.
//
// Two shafts turn at equal speeds in opposite senses, and a lobe on the first is to stay in
// contact with a cam on the second all the way round, so the pair drives without slipping or
// separating.  The second cam is generated, not designed: it is the envelope of the lobe carried
// by the first shaft's turn as seen from the second (`motion(a, relative_to: b)`).
//
// The lobe is an ellipse written as a formula curve (a computed point, traced as its angle runs)
// set off its shaft's centre, so nothing about the pair is symmetric or standard.  At every roll
// the two touch with their common normal through the pitch point, halfway between the shafts:
// the law of gearing, which nothing in the document states.
//
// Edit the lobe's semi-axes `a` and `b`, or how far its centre `off` sits from the shaft, and
// the partner is cut again.

a := 14
b := 9
off := 3

component Lobe(c: point, a: Length, b: Length, u: Angle) {
  p := point(x: c.x + a * cos(u), y: c.y + b * sin(u))
}

o1 := point
o2 := point hint(x: 40, y: 0)
fix(x == 0, y == 0) o1
fix(x == 40, y == 0) o2
lc := point hint(x: 3, y: 0)
o1 distance(off, along: x) lc
o1 distance(0, along: y) lc

lobe := Lobe(lc, a: a, b: b).p over u in (0, 360)
shaft_a := motion(about: o1, ratio: 1)
shaft_b := motion(about: o2, ratio: -1)
seen := motion(shaft_a, relative_to: shaft_b)
partner := envelope(lobe, under: seen, from: 0deg, to: 360deg)
