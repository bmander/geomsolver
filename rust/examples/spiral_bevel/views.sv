// The views the layout is drawn in. The pitch plane is the page folded flat to
// the crown; every other view is folded square to it about a line through the
// mean point M, so it stands through M and needs no angle of its own:
//   G, gear axial:      along O -> M          (pitch/gear.sv)
//   Q, pinion axial:    along M -> the pinion apex (pitch/pinion.sv)
//   N, normal section:  along C -> M, the tooth-trace normal (crown/)
use std

// The pitch plane: the page folded up about its x axis, the gear apex at its
// datum origin. Its datum stands where the page's does, `span` long.
component PitchView(front: plane, span: Length) {
  private point origin hint(x: 0, y: 0)
  private point toward hint(x: span, y: 0)
  origin coincident front.origin
  toward distance(span, along: u) front
  toward distance(0mm, along: v) front
  plane view(origin: origin, toward: toward, from: front, fold: 0deg)
}

// A view folded square to `parent` about `hinge`, a line drawn in it: its u
// runs along the hinge, and the solve places it. Its datum is held where drawn,
// on the page's origin and `span` long: about the size of what is drawn in it,
// which is how far a turn of the view is taken to move its content.
component FoldedView(parent: plane, hinge: line, span: Length) {
  private point origin hint(x: 0, y: 0)
  private point toward hint(x: span, y: 0)
  plane view(origin: origin, toward: toward, from: parent, fold: along hinge)
}

preview {
  unit mm
  pitch: PitchView(std.front, span: 40mm)
  point a hint(x: 0, y: 0) in pitch.view
  point b hint(x: 40, y: 20) in pitch.view
  a coincident pitch.view.origin
  a distance(40mm, along: right) b
  a distance(20mm, along: up) b
  line hinge(a, b) in pitch.view
  folded: FoldedView(pitch.view, hinge, span: 40mm)
}
