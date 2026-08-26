// The Warren truss with one member more than it needs — over-constrained, and consistent.
//
// The extra bar runs from the first bottom node to the third, and its length is exactly what the
// two bays between them already force: `2 * span`.  So the structure is rigid twice over.  There
// is nothing to solve differently and nothing to report as a conflict — the matching simply finds
// more equations than unknowns, which is the amber reading rather than the red one.

param bays = 6
param span = 20
param height = 15

param web = hypot(span / 2, height)

// bays + 1 nodes along the bottom, and one above the middle of each bay
repeat bays + 1 as i {
  point b at (i * span, 0)
}
repeat bays as i {
  point t at ((i + 0.5) * span, height)
}

// the bottom chord, and the two web members that hang the top node off this bay
repeat bays as i {
  line chord(b[i], b[i + 1])
  line rise(b[i], t[i])
  line fall(t[i], b[i + 1])

  distance(b[i], b[i + 1]) == span
  distance(b[i], t[i]) == web
  distance(t[i], b[i + 1]) == web
}

// the top chord runs between neighbouring top nodes, so there is one fewer of it
repeat bays - 1 as i {
  line upper(t[i], t[i + 1])
  distance(t[i], t[i + 1]) == span
}

horizontal(chord[0])
ground(b[0])

// what the two bays it spans already say
distance(b[0], b[2]) == 2 * span
