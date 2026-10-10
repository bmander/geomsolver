// Step 3, a member's limits drawn for the checks (verification.sv): the toe and heel spheres and
// the tip, root and back cones as revolved sections in the member's axial view, about its apex,
// each with its named wall. The exported members are cut from the same limits stated as a
// region (blank/member.sv); the root cone bounds only the tooth regions the checks declare.
use std
use blank.sphere
use blank.cone

// `generator` runs from the apex to the mean point; `cone` is the generator the cones are drawn
// on (the gear's lies across its axis from M).
component MemberLimits(generator: line, cone: line, ax: line, design: group,
                       normal_module: Length) {
  span := blank.sphere.FaceSpan(generator, width: design.face_width)
  toe := blank.sphere.SphericalBoundary(generator.p1, generator, span.toe)
  heel := blank.sphere.SphericalBoundary(generator.p1, generator, span.heel)
  tip := blank.cone.ConeBoundary(cone, ax, offset: design.addendum * normal_module,
    lean: 180deg)
  root := blank.cone.ConeBoundary(cone, ax, offset: design.dedendum * normal_module,
    lean: 0deg)
  back := blank.cone.ConeBoundary(cone, ax, offset: design.back * normal_module, lean: 0deg)
}

preview {
  unit mm
  proportions := {face_width: 10mm, addendum: 1, dedendum: 1.25, back: 4}
  in std.front {
    mean := point
    foot := point hint((40, 20))
    fix((50, 0)) mean
    generator := line(std.origin, mean)
    ax := line(std.origin, foot)
    to_foot := line(mean, foot)
    to_foot perpendicular ax
    generator angle(30deg) ax
    limits := MemberLimits(generator, generator, ax, proportions, normal_module: 2mm)
  }
}
