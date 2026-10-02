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

bays := 50
span := 20
height := 15

web := hypot(span / 2, height)

// bays + 1 nodes along the bottom, and one above the middle of each bay
repeat bays + 1 as i {
  b := point hint(x: i * span, y: 0)
}
repeat bays as i {
  t := point hint(x: (i + 0.5) * span, y: height)
}

// the bottom chord, and the two web members that hang the top node off this bay
repeat bays as i {
  chord := line(b[i], b[i + 1])
  rise := line(b[i], t[i])
  fall := line(t[i], b[i + 1])

  b[i] distance(span) b[i + 1]
  b[i] distance(web) t[i]
  t[i] distance(web) b[i + 1]
}

// the top chord runs between neighbouring top nodes, so there is one fewer of it
repeat bays - 1 as i {
  upper := line(t[i], t[i + 1])
  t[i] distance(span) t[i + 1]
}

horizontal chord[0]
ground b[0]
