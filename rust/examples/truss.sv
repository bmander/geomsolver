// A Warren truss — the triangulated girder under a footbridge or a roof.
//
// Every member is given a length, and that is what makes the frame rigid once one joint is
// pinned down and one chord is levelled.  Drag it anywhere and it keeps its shape exactly.
//
// The lengths are *stated* rather than measured off a picture, and a Warren truss needs only two
// of them: a panel of the bottom chord is one bay, and every diagonal rises `height` over half a
// bay, so it is `hypot(span / 2, height)`.  Writing those two formulas says what a Warren truss
// *is*.  Writing out the thirty-odd numbers they come to would say only what this particular one
// happened to measure — and would stop being true the moment the span changed.

param bays = 8
param span = 20
param height = 15

param web = hypot(span / 2, height)

// bays + 1 nodes along the bottom, and one above the middle of each bay
repeat bays + 1 as i {
  point b hint(x: i * span, y: 0)
}
repeat bays as i {
  point t hint(x: (i + 0.5) * span, y: height)
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
