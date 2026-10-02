// Step 3, the end relief: where a member's tip cone meets its toe or its heel sphere, the
// corner chamfered `size` each way, so every tooth's tip is relieved at both its ends. In
// the member's axial view the chamfer is a line across the corner; revolved about the axis
// it is a cone band. The ring cut from the blank is that line's triangle with the point
// beyond the corner, its other two sides standing outside the blank (above the tip cone,
// and inside the toe's or outside the heel's sphere), so it removes the corner and nothing
// else. It is a blank face, exact as the cones and spheres are, and every tooth's tip land
// and flanks end on it where they reach the corner.
use std
use blank.sphere
use blank.cone

// The chamfer at one end: `size` along the cone distance from the corner and `size` down
// from the tip cone. `apex` is the member's apex (the spheres' centre), `rim` the sphere's
// point on the pitch generator and `rim_in` the point `size` from it toward the tooth (a
// face span `size` narrower at each end), `cone_line` the generator the cones are drawn on
// (apex -> mean point, or its mirror across the axis for the gear), and `tip_near` and
// `tip_far` the tip cone's meridian ends (half and one and a half cone distances along
// `cone_line`). Every point is placed without a choice of root a seed would have to make: each
// meets a line and a circle about the apex whose other crossing is across the apex, and
// the drop toward the generator is turned 0deg from the corner's square to it. The seeds
// only start the solve near.
component EndChamfer(apex: point, rim: point, rim_in: point, cone_line: line, tip_near: point,
                     tip_far: point, axis: line, size: Length) {
  private to_rim := line(apex, rim)
  private to_rim_in := line(apex, rim_in)
  private tip := line(tip_near, tip_far)
  // Seeds: along the tip meridian as far as the sphere is from the apex along the pitch
  // generator (the meridian's ends are half and one and a half of `cone_line` from it).
  corner := point hint(
    x: tip_near.x + (tip_far.x - tip_near.x)
      * (sqrt((rim.x - apex.x)^2 + (rim.y - apex.y)^2)
         / sqrt((cone_line.p2.x - apex.x)^2 + (cone_line.p2.y - apex.y)^2) - 0.5),
    y: tip_near.y + (tip_far.y - tip_near.y)
      * (sqrt((rim.x - apex.x)^2 + (rim.y - apex.y)^2)
         / sqrt((cone_line.p2.x - apex.x)^2 + (cone_line.p2.y - apex.y)^2) - 0.5))
  corner on tip
  private to_corner := line(apex, corner)
  to_corner equal to_rim
  // The chamfer's end on the tip: as far from the apex as `rim_in`.
  along_tip := point hint(
    x: corner.x + (tip_far.x - tip_near.x)
      * (sqrt((rim_in.x - apex.x)^2 + (rim_in.y - apex.y)^2)
         - sqrt((rim.x - apex.x)^2 + (rim.y - apex.y)^2))
      / sqrt((tip_far.x - tip_near.x)^2 + (tip_far.y - tip_near.y)^2),
    y: corner.y + (tip_far.y - tip_near.y)
      * (sqrt((rim_in.x - apex.x)^2 + (rim_in.y - apex.y)^2)
         - sqrt((rim.x - apex.x)^2 + (rim.y - apex.y)^2))
      / sqrt((tip_far.x - tip_near.x)^2 + (tip_far.y - tip_near.y)^2))
  along_tip on tip
  private to_along := line(apex, along_tip)
  to_along equal to_rim_in
  // Its end on the sphere: where the line `size` below the tip, toward the generator,
  // meets it. `foot` is the corner's on the generator, and `mark` stands `size` toward it.
  private foot := point hint(
    x: corner.x - (tip_near.x - (apex.x + cone_line.p2.x) / 2),
    y: corner.y - (tip_near.y - (apex.y + cone_line.p2.y) / 2))
  private mark := point hint(
    x: corner.x + size * (foot.x - corner.x)
      / sqrt((foot.x - corner.x)^2 + (foot.y - corner.y)^2),
    y: corner.y + size * (foot.y - corner.y)
      / sqrt((foot.x - corner.x)^2 + (foot.y - corner.y)^2))
  private level := point hint(
    x: mark.x + size * (tip_far.x - tip_near.x)
      / sqrt((tip_far.x - tip_near.x)^2 + (tip_far.y - tip_near.y)^2),
    y: mark.y + size * (tip_far.y - tip_near.y)
      / sqrt((tip_far.x - tip_near.x)^2 + (tip_far.y - tip_near.y)^2))
  foot on cone_line
  private drop := line(corner, foot)
  drop perpendicular cone_line
  private down := line(corner, mark)
  down angle(0deg) drop
  distance(size) down
  private below := line(mark, level)
  below parallel tip
  distance(size) below
  down_end := point hint(at: mark)
  down_end on below
  private to_down := line(apex, down_end)
  to_down equal to_rim
  // The ring's section: the chamfer extended its own length past each end, and the point
  // as far beyond the corner as the chamfer's middle is short of it.
  private tip_out := point hint(x: 2 * along_tip.x - down_end.x, y: 2 * along_tip.y - down_end.y)
  private end_out := point hint(x: 2 * down_end.x - along_tip.x, y: 2 * down_end.y - along_tip.y)
  private middle := point hint(x: (along_tip.x + down_end.x) / 2, y: (along_tip.y + down_end.y) / 2)
  private beyond := point hint(x: 2 * corner.x - middle.x, y: 2 * corner.y - middle.y)
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
  span := FaceSpan(generator, width: 10mm)
  toe := SphericalBoundary(std.origin, generator, span.toe)
  heel := SphericalBoundary(std.origin, generator, span.heel)
  tip := ConeBoundary(generator, axis, offset: 2mm, lean: 180deg)
  inner := FaceSpan(generator, width: 9mm)
  toe_end := EndChamfer(std.origin, span.toe, inner.toe, generator, tip.p, tip.q, axis,
    size: 0.5mm)
  heel_end := EndChamfer(std.origin, span.heel, inner.heel, generator, tip.p, tip.q, axis,
    size: 0.5mm)
}
