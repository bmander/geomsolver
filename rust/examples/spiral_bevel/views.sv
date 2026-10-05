// The views the layout is drawn in. The pitch plane P stands square to the front plane along
// its u; every other view stands square to P on a line through the mean point M, so it needs
// no angle of its own:
//   G, the gear's axial view:    along O -> M, the gear apex to M  (pitch/gear.sv)
//   Q, the pinion's axial view:  along M -> A, the pinion apex     (pitch/pinion.sv)
//   N, the normal section:       along C -> M, the trace normal    (crown/)
use std

// The pitch plane: square to `front` along its `u`, through its origin, so the gear apex is at
// both origins and the front plane is the view along the pitch plane's v. `down` is the side
// of it its normal does not point, for a view standing on it (`FoldedView`). `span` is kept for
// the callers that name it.
component PitchView(front: plane, span: Length) {
  private up := ray hint(x: 0, y: 1, z: 0)
  up perpendicular front
  view := plane(u: front.u, v: up)
  front.origin coincident view.origin
  down := ray hint(x: 0, y: 0, z: -1)
  down perpendicular view
}

// A view standing square to `parent` on `hinge`, a line drawn in it: its u runs along the hinge
// and its v along `up`, square to the parent. Its origin is the parent's, seen in it: on the
// fold line, where the parent's origin projects.
component FoldedView(parent: plane, hinge: line, up: ray, span: Length) {
  view := plane(u: hinge, v: up)
  parent coincident view.origin
  parent.origin project view.origin
  hinge.p1 coincident view
}

preview {
  unit mm
  pitch := PitchView(std.front, span: 40mm)
  a := point hint(x: 0, y: 0) in pitch.view
  b := point hint(x: 40, y: 20) in pitch.view
  a coincident pitch.view.origin
  a distance(40mm, along: right) b
  a distance(20mm, along: up) b
  hinge := line(a, b) in pitch.view
  folded := FoldedView(pitch.view, hinge, pitch.down, span: 40mm)
}
