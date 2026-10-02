// The views the layout is drawn in. The pitch plane P is the page; every other view folds
// square to it about a line through the mean point M, so it needs no angle of its own:
//   G, the gear's axial view:    along O -> M, the gear apex to M  (pitch/gear.sv)
//   Q, the pinion's axial view:  along M -> A, the pinion apex     (pitch/pinion.sv)
//   N, the normal section:       along C -> M, the trace normal    (crown/)
use std

// The pitch plane: the page, the gear apex at its datum origin, the datum `span` long.
component PitchView(front: plane, span: Length) {
  private origin := point hint(x: 0, y: 0)
  private toward := point hint(x: span, y: 0)
  origin coincident front.origin
  toward distance(span, along: u) front
  toward distance(0mm, along: v) front
  view := plane(origin: origin, toward: toward, from: front, fold: 0deg)
}

// A view folded square to `parent` about `hinge`, a line drawn in it; its u runs along the
// hinge. Its datum is held where drawn, `span` long: about the size of what it shows, which is
// how far a turn of the view is taken to move it.
component FoldedView(parent: plane, hinge: line, span: Length) {
  private origin := point hint(x: 0, y: 0)
  private toward := point hint(x: span, y: 0)
  view := plane(origin: origin, toward: toward, from: parent, fold: along hinge)
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
  folded := FoldedView(pitch.view, hinge, span: 40mm)
}
