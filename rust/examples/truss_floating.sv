// The same truss with nothing holding it down.
//
// Every member still has its length, so the *shape* is completely determined — but nothing says
// where the truss is or which way it faces.  It can slide two ways and turn one: three degrees
// of freedom, and they are exactly the three ways a rigid object moves without bending.
//
// Telling that apart from a genuinely floppy drawing is the whole point of the case.  Both would
// be reported as having freedom left; only one of them is a *shape* that is still unresolved.
// Drag this one and the whole frame travels as a single piece.

bays := 8
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
  chord := distance(span) line(b[i], b[i + 1])
  rise := distance(web) line(b[i], t[i])
  fall := distance(web) line(t[i], b[i + 1])
}

// the top chord runs between neighbouring top nodes, so there is one fewer of it
repeat bays - 1 as i {
  upper := distance(span) line(t[i], t[i + 1])
}
