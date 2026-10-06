// The same truss with a member that cannot exist.
//
// The bar from `b0` to `b3` is given a length of 999, where the three bays it runs alongside come
// to 60 altogether.  No arrangement of the joints satisfies that, so this drawing has no
// solution.
//
// The case is about which statements get blamed.  It would be easy — and useless — to report
// that the truss as a whole does not work.  What is reported instead is the offending bar
// together with the run of members whose lengths it contradicts, and nothing else.

use std

bays := 6
span := 20
height := 15

web := hypot(span / 2, height)

// bays + 1 nodes along the bottom, and one above the middle of each bay
in std.front {
  repeat bays + 1 as i {
    b := point hint((i * span, 0))
  }
  repeat bays as i {
    t := point hint(((i + 0.5) * span, height))
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

  horizontal chord[0]
  fix((0, 0)) b[0]

  // three bays apart, and told to be 999
  b[0] distance(999) b[3]
}
