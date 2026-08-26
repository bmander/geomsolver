// The Warren truss with nothing holding it down — a rigid body with three degrees of freedom.
//
// Every member is still dimensioned, so the shape is entirely determined; what is missing is the
// *gauge*.  No node is grounded and no chord is levelled, so the whole thing may be slid in two
// directions and turned in one, and the diagnosis should say exactly that: three degrees of
// freedom whose null space is the rigid motions, not an under-constrained figure.  Drag it and
// it moves as one piece.

param bays = 8
param span = 20
param height = 15

param web = hypot(span / 2, height)

// bays + 1 nodes along the bottom, and one above the middle of each bay
repeat bays + 1 as i {
  point b hint at (i * span, 0)
}
repeat bays as i {
  point t hint at ((i + 0.5) * span, height)
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

