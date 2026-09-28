// Step 3, a member's blank: the heel sphere within the tip cone, less the toe
// sphere and the back cone. The five limits are drawn in the member's axial view,
// about its apex; the root cone bounds only the tooth regions the checks declare.
use std
use blank.sphere
use blank.cone

// `generator` runs from the apex to the mean point; `cone` is the generator the
// cones are drawn on (the gear's lies opposite M across its axis).
component MemberLimits(generator: line, cone: line, axis: line, design: group,
                       normal_module: Length) {
  span: FaceSpan(generator, width: design.face_width)
  toe: SphericalBoundary(generator.p1, generator, span.toe)
  heel: SphericalBoundary(generator.p1, generator, span.heel)
  tip: ConeBoundary(cone, axis, offset: design.addendum * normal_module, lean: 180deg)
  root: ConeBoundary(cone, axis, offset: design.dedendum * normal_module, lean: 0deg)
  back: ConeBoundary(cone, axis, offset: design.back * normal_module, lean: 0deg)
}

// The blank term on a body its member declares over the heel, `solid body(design.heel)`:
// `design` names the limits' solids (heel, toe, tip, root, back).
component MemberBlank(body: solid, design: group) {
  design.tip bound body
  design.toe cut body
  design.back cut body
}

preview {
  unit mm
  group proportions(face_width: 10mm, addendum: 1, dedendum: 1.25, back: 4)
  point mean hint(x: 50, y: 0)
  point foot hint(x: 40, y: 20)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  line generator(std.origin, mean)
  line axis(std.origin, foot)
  line to_foot(mean, foot)
  to_foot perpendicular axis
  generator angle(30deg) axis
  limits: MemberLimits(generator, generator, axis, proportions, normal_module: 2mm)
  group solids(heel: limits.heel.wall.solid, toe: limits.toe.wall.solid,
    tip: limits.tip.wall.solid, root: limits.root.wall.solid, back: limits.back.wall.solid)
  solid body(solids.heel)
  blank: MemberBlank(body, solids)
}
