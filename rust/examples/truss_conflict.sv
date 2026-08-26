// The same truss with a member that cannot exist.
//
// The bar from `b0` to `b3` is given a length of 999, where the three bays it runs alongside come
// to 60 altogether.  No arrangement of the joints satisfies that, so this drawing has no
// solution.
//
// The case is about which statements get blamed.  It would be easy — and useless — to report
// that the truss as a whole does not work.  What is reported instead is the offending bar
// together with the run of members whose lengths it contradicts, and nothing else.

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

// three bays apart, and told to be 999
distance(b[0], b[3]) == 999
