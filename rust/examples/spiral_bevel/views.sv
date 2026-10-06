// The views the layout is drawn in. The pitch plane P is `std.top`, the gear apex O at its origin,
// so the front plane is the view along P's v; every other view stands square to P on a line
// through the mean point M, so it needs no angle of its own:
//   G, the gear's axial view:    along O -> M, the gear apex to M  (pitch/gear.sv)
//   Q, the pinion's axial view:  along M -> A, the pinion apex     (pitch/pinion.sv)
//   N, the normal section:       along C -> M, the trace normal    (crown/)
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
