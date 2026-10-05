// Step 3, the end relief: where a member's tip cone meets its toe or heel sphere, the corner
// chamfered `size` each way. In the member's axial view the chamfer is a line across the
// corner; revolved about the axis it is a cone band, a blank face as exact as the cones and
// spheres, on which every tooth's tip land and flanks end. The ring cut from the blank is the
// chamfer's triangle with a point beyond the corner, its other two sides outside the blank, so
// it removes the corner and nothing else.
use std
use blank.sphere
use blank.cone

// The chamfer at one end: `size` along the cone distance from the corner on the `tip` cone (a
// ConeBoundary), and `size` down from it on the sphere. `apex` is the member's apex, `rim` the
// sphere's point on the pitch generator and `rim_in` the point `size` from it toward the tooth;
// `cone_line` is the generator the cones are drawn on (apex -> mean point, or its mirror across
// the axis for the gear). Each point meets a line and a circle about the apex whose other
// crossing is across the apex, so the seeds need only start on the tooth's side of it.
component EndChamfer(apex: point, rim: point, rim_in: point, cone_line: line, tip: group,
                     axis: line, size: Length) {
  private tip_line := line(tip.p, tip.q)
  private to_rim := line(apex, rim)
  private to_rim_in := line(apex, rim_in)
  // On the tip: the corner, as far from the apex as the rim, and the chamfer's end, as far as
  // `rim_in`.
  corner := point hint(at: tip.p, toward: tip.q, by: 0.5)
  along_tip := point hint(at: corner)
  corner coincident tip_line
  along_tip coincident tip_line
  private to_corner := line(apex, corner)
  private to_along := line(apex, along_tip)
  to_corner equal to_rim
  to_along equal to_rim_in
  // On the sphere: the chamfer's end on the line `below` the tip, through `mark`, `size` from
  // the corner toward its `foot` on the cone line.
  private foot := point hint(at: cone_line.p2)
  private mark := point hint(at: corner, toward: foot, by: 0.5)
  private level := point hint(at: tip.q)
  foot coincident cone_line
  private drop := line(corner, foot)
  drop perpendicular cone_line
  private down := line(corner, mark)
  down angle(0deg) drop
  distance(size) down
  private below := line(mark, level)
  below parallel tip_line
  distance(size) below
  down_end := point hint(at: mark)
  down_end coincident below
  private to_down := line(apex, down_end)
  to_down equal to_rim
  // The ring's section: the chamfer extended its own length past each end, and the point as far
  // beyond the corner as the chamfer's middle is short of it. Midpoints place them, so they need
  // no seed.
  private tip_out := point
  private end_out := point
  private middle := point
  private beyond := point
  private tip_leg := line(tip_out, down_end)
  private end_leg := line(along_tip, end_out)
  private chord := line(along_tip, down_end)
  private across := line(beyond, middle)
  along_tip midpoint tip_leg
  down_end midpoint end_leg
  middle midpoint chord
  corner midpoint across
  private section := face(tip_out, end_out, beyond, -> close)
  construction ring := solid(section, about: axis)
}

// A member's ends relieved: both rings cut from its body.
component EndCut(body: solid, toe: solid, heel: solid) {
  toe cut body
  heel cut body
}

// The two chamfers in a member's axial view; blank/member.sv's preview cuts them from a blank.
preview {
  unit mm
  mean := point hint(x: 50, y: 0)
  foot := point hint(x: 40, y: 20)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  generator := line(std.origin, mean)
  axis := line(std.origin, foot)
  to_foot := line(mean, foot)
  to_foot perpendicular axis
  generator angle(30deg) axis
  span := blank.sphere.FaceSpan(generator, width: 10mm)
  inner := blank.sphere.FaceSpan(generator, width: 9mm)
  toe := blank.sphere.SphericalBoundary(std.origin, generator, span.toe)
  heel := blank.sphere.SphericalBoundary(std.origin, generator, span.heel)
  tip := blank.cone.ConeBoundary(generator, axis, offset: 2mm, lean: 180deg)
  toe_end := EndChamfer(std.origin, span.toe, inner.toe, generator, tip, axis, size: 0.5mm)
  heel_end := EndChamfer(std.origin, span.heel, inner.heel, generator, tip, axis, size: 0.5mm)
}
