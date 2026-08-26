// The same truss with one more member than it needs.
//
// The extra bar spans two bays, and it is given exactly the length those two bays already force.
// So the frame is held rigid twice over: more is being said than there are unknowns to pin down,
// but nothing said is *wrong*.
//
// That difference matters, and it is why this file and `truss_conflict.sv` are separate cases.
// Saying something twice consistently is a note — you could delete the extra bar and lose
// nothing.  Saying two things that cannot both hold is an error, and the drawing does not
// exist.

param bays = 6
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

horizontal(chord[0])
ground(b[0])

// what the two bays it spans already say
distance(b[0], b[2]) == 2 * span
