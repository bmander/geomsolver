// Step 3, a member's blank: the heel sphere within the tip cone, less the toe sphere and the
// back cone. Its limits are drawn in the member's axial view, about its apex; the root cone
// bounds only the tooth regions the checks declare (verification.sv). Where the design
// relieves the ends, the tip's toe and heel corners are chamfered too (blank/ends.sv).
use std
use blank.sphere
use blank.cone
use blank.ends

// `generator` runs from the apex to the mean point; `cone` is the generator the cones are drawn
// on (the gear's lies across its axis from M).
component MemberLimits(generator: line, cone: line, axis: line, design: group,
                       normal_module: Length) {
  span := blank.sphere.FaceSpan(generator, width: design.face_width)
  toe := blank.sphere.SphericalBoundary(generator.p1, generator, span.toe)
  heel := blank.sphere.SphericalBoundary(generator.p1, generator, span.heel)
  tip := blank.cone.ConeBoundary(cone, axis, offset: design.addendum * normal_module,
    lean: 180deg)
  root := blank.cone.ConeBoundary(cone, axis, offset: design.dedendum * normal_module,
    lean: 0deg)
  back := blank.cone.ConeBoundary(cone, axis, offset: design.back * normal_module, lean: 0deg)
  repeat design.ends_relieved {
    // the chamfers' ends on the tip: the face width less the relief at each end
    relieved_span := blank.sphere.FaceSpan(generator,
      width: design.face_width - 2 * design.end_relief)
    toe_end := blank.ends.EndChamfer(generator.p1, span.toe, relieved_span.toe, cone, tip, axis,
      size: design.end_relief)
    heel_end := blank.ends.EndChamfer(generator.p1, span.heel, relieved_span.heel, cone, tip,
      axis, size: design.end_relief)
  }
}

// The blank's term on a `body` its member declares over the heel; `design` names the limits'
// solids (heel, toe, tip, back).
component MemberBlank(body: solid, design: group) {
  design.tip bound body
  design.toe cut body
  design.back cut body
}

preview {
  unit mm
  proportions := group(face_width: 10mm, addendum: 1, dedendum: 1.25, back: 4,
    end_relief: 0.2mm, ends_relieved: 1)
  mean := point hint(x: 50, y: 0)
  foot := point hint(x: 40, y: 20)
  std.origin distance(50mm, along: right) mean
  std.origin distance(0mm, along: up) mean
  generator := line(std.origin, mean)
  axis := line(std.origin, foot)
  to_foot := line(mean, foot)
  to_foot perpendicular axis
  generator angle(30deg) axis
  limits := MemberLimits(generator, generator, axis, proportions, normal_module: 2mm)
  solids := group(heel: limits.heel.wall.solid, toe: limits.toe.wall.solid,
    tip: limits.tip.wall.solid, back: limits.back.wall.solid)
  body := solid(solids.heel)
  blank := MemberBlank(body, solids)
  ends := blank.ends.EndCut(body, limits.toe_end[0].ring, limits.heel_end[0].ring)
}
