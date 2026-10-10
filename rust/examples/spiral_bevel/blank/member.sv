// Step 3, a member's blank: the points within its tip cone and its heel sphere and outside its
// toe sphere and its back cone, one region and its solid (§6.21). The cones are the pitch cone
// offset by the addendum and by the back depth, drawn in the member's axial view to where they
// meet its axis; the spheres are about its apex through the face span's ends. Their sizes are
// the layout's, solved. Where the design relieves the ends, the tip's toe and heel corners are
// chamfered too (blank/ends.sv). The checks draw the same limits as revolved sections
// (blank/limits.sv), for their named walls.
use std
use blank.sphere
use blank.ends

// A cone of the blank about `ax`: its meridian parallel to `cone` (apex -> mean point), `offset`
// off it at the mean point, turned `lean` from the direction toward the axis (0deg for the back,
// 180deg for the tip). `p` is the meridian's point across from the mean point and `apex` where
// it meets the axis. The cone turns about a line from the apex to a point held in space on the
// axis, so its angle is read in space, unsigned; its half is the drawing's to solve.
component OffsetCone(cone: line, ax: line, view: plane, offset: Length, lean: Angle,
                     normal_module: Length) {
  in view {
    // Seeds, rough: the rib's foot at the axis's far end, the meridian's point a step along the
    // rib turned `lean`, the apex at the axis's start.
    foot := point hint(at: ax.p2)
    p := point hint(at: cone.p2, toward: foot, by: 0.05, turn: lean)
    apex := point hint(at: ax.p1)
    private rib := line(cone.p2, foot)
    private lift := line(cone.p2, p)
    private meridian := line(apex, p)
  }
  rib perpendicular cone
  foot coincident ax
  rib angle(lean) lift
  distance(offset) lift
  meridian parallel cone
  apex coincident ax
  private far := point hint(at: ax.p2)
  far coincident ax
  apex distance(normal_module, along: ax) far
  private about := line(apex, far)
  nappe := std.Cone(about, half: hint(45deg))
  p coincident nappe
}

// The blank of a member: `generator` runs from the apex to the mean point; `cone` is the
// generator the cones are drawn on (the gear's lies across its axis from M); `view` is the
// member's axial view.
component MemberBlank(generator: line, cone: line, ax: line, view: plane, design: group,
                      normal_module: Length) {
  span := blank.sphere.FaceSpan(generator, width: design.face_width) in view
  toe := std.Sphere(generator.p1, r: hint(design.cone_distance))
  heel := std.Sphere(generator.p1, r: hint(design.cone_distance))
  span.toe coincident toe
  span.heel coincident heel
  tip := OffsetCone(cone, ax, view, offset: design.addendum * normal_module, lean: 180deg,
    normal_module: normal_module)
  back := OffsetCone(cone, ax, view, offset: design.back * normal_module, lean: 0deg,
    normal_module: normal_module)
  region := { p | p inside tip.nappe; p inside heel; p outside toe; p outside back.nappe }
  material := solid(region)
  repeat design.ends_relieved {
    // the chamfers' ends on the tip: the face width less the relief at each end; the tip's
    // meridian from its apex through `p`
    relieved_span := blank.sphere.FaceSpan(generator,
      width: design.face_width - 2 * design.end_relief) in view
    tip_line := {p: tip.apex, q: tip.p}
    toe_end := blank.ends.EndChamfer(generator.p1, span.toe, relieved_span.toe, cone, tip_line,
      ax, size: design.end_relief) in view
    heel_end := blank.ends.EndChamfer(generator.p1, span.heel, relieved_span.heel, cone,
      tip_line, ax, size: design.end_relief) in view
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
