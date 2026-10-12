// Step 3, a member's blank: the points within its tip cone and its heel sphere and outside its
// toe sphere and its back cone, one region and its solid (§6.21). The cones are the pitch cone
// offset by the addendum and by the back depth (`std.Offset`, `outside(d)`); the spheres are
// about its apex through the face span's ends. Their sizes are the layout's, solved. Where the
// design relieves the ends, the tip's toe and heel corners are chamfered too (blank/ends.sv).
// Each set the region's body uses names the blank's face it bounds: a member built from the
// blank has faces `material.tip`, `.heel`, `.toe` and `.back`, which the checks read.
use std
use blank.sphere
use blank.ends

// The blank of a member: `generator` runs from the apex to the mean point; `cone` is the
// generator the end chamfers are drawn on (the gear's lies across its axis from M); `ax` is the
// member's axis from its apex, and `view` its axial view.
component MemberBlank(generator: line, cone: line, ax: line, view: plane, design: group,
                      normal_module: Length) {
  span := blank.sphere.FaceSpan(generator, width: design.face_width) in view
  toe := std.Sphere(generator.p1, r: hint(design.cone_distance))
  heel := std.Sphere(generator.p1, r: hint(design.cone_distance))
  span.toe coincident toe
  span.heel coincident heel
  // The pitch cone, about the member's axis; its half is the drawing's to solve.
  pitch := std.Cone(ax)
  generator.p2 coincident pitch
  tip := std.Offset(pitch, d: design.addendum * normal_module)
  back := { q | q outside(design.back * normal_module) pitch }
  region := { p | p inside tip; p inside heel; p outside toe; p inside back }
  material := solid(region)
  repeat design.ends_relieved {
    // the chamfers' ends on the tip: the face width less the relief at each end
    relieved_span := blank.sphere.FaceSpan(generator,
      width: design.face_width - 2 * design.end_relief) in view
    toe_end := blank.ends.EndChamfer(generator.p1, span.toe, relieved_span.toe, cone, tip, ax,
      size: design.end_relief) in view
    heel_end := blank.ends.EndChamfer(generator.p1, span.heel, relieved_span.heel, cone, tip,
      ax, size: design.end_relief) in view
  }
}

preview {
  unit mm
  proportions := {face_width: 10mm, cone_distance: 50mm, addendum: 1, back: 4,
    end_relief: 0.2mm, ends_relieved: 1}
  in std.front {
    mean := point
    foot := point hint((40, 20))
    fix((50, 0)) mean
    generator := line(std.origin, mean)
    ax := line(std.origin, foot)
    to_foot := line(mean, foot)
    to_foot perpendicular ax
    generator angle(30deg) ax
  }
  member := MemberBlank(generator, generator, ax, std.front, proportions, normal_module: 2mm)
  body := solid(member.material)
  ends := blank.ends.EndCut(body, member.toe_end[0].ring, member.heel_end[0].ring)
}
