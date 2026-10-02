// The same truss with a member that cannot exist.
//
// The bar from `b0` to `b3` is given a length of 999, where the three bays it runs alongside come
// to 60 altogether.  No arrangement of the joints satisfies that, so this drawing has no
// solution.
//
// The case is about which statements get blamed.  It would be easy — and useless — to report
// that the truss as a whole does not work.  What is reported instead is the offending bar
// together with the run of members whose lengths it contradicts, and nothing else.

bays := 6
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
fix(x == 0, y == 0) b[0]

// three bays apart, and told to be 999
b[0] distance(999) b[3]
