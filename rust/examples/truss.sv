// A Warren truss: a bottom chord, a top chord half a bay in from each end, and a zig-zag web.
//
// Every member is dimensioned, which is what makes the whole thing rigid once one node is
// grounded and one chord is levelled — and the dimensions are *stated* rather than measured.  A
// Warren truss has only two member lengths in it: a chord panel is `span`, and every web member
// runs from a chord node to a top node half a bay along and `height` up, so it is
// `hypot(span / 2, height)`.  Writing the two formulas says what the truss is; writing the
// thirty-odd numbers they come to would say only what this one measured.

param bays = 8
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
