// The view the crown is drawn in. The pitch plane P is `std.top`, the gear apex O at its origin;
// N, the normal section, stands square to P on the trace normal C -> M through the mean point M,
// so it needs no angle of its own (crown/). The pitch cones and the blanks stand in space.
use std

// A view standing square to `parent` on `hinge`, a line drawn in it: its u is the hinge, so it
// stands on it, and its v is square to the parent, down from it as seeded. Its origin is the
// parent's, seen in it: on the fold line, where the parent's origin projects.
component FoldedView(parent: plane, hinge: line) {
  view := plane(u: hinge, v: hint(dir: (0, 0, -1)))
  parent perpendicular view.v
  parent.origin project view.origin
}

preview {
  unit mm
  in std.top {
    a := point
    b := point
    fix((0, 0)) a
    fix((40, 20)) b
    hinge := line(a, b)
  }
  folded := FoldedView(std.top, hinge)
}
