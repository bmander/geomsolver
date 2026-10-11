// X0 (#183): the spiral bevel's member blank (`spiral_bevel/blank/member.sv`) with its end relief
// said inside the region's body (L5, #145 F5). Does not parse yet: `A chamfer(d) B` between two
// of a region's bounding sets is proposed.
//
// Today the relief is `blank/ends.sv`: an `EndChamfer` component (44 statement lines) that draws
// the chamfer's section in the axial view, seeds eight points so the corner lands on the right
// branch, and revolves a construction ring; an `EndCut` that cuts both rings from the body; and a
// `repeat design.ends_relieved` switch over `min(ceil(end_relief / 1mm), 1)`, here and again in
// `members.sv`. All of that becomes the two chamfer lines below, and a relief of zero is the
// identity, so the switch goes.
use std
use blank.sphere

component MemberBlank(generator: line, cone: line, ax: line, view: plane, design: group,
                      normal_module: Length) {
  span := blank.sphere.FaceSpan(generator, width: design.face_width) in view
  toe := std.Sphere(generator.p1, r: hint(design.cone_distance))
  heel := std.Sphere(generator.p1, r: hint(design.cone_distance))
  span.toe coincident toe
  span.heel coincident heel
  private far := point hint(at: ax.p2)
  far coincident ax
  ax.p1 distance(normal_module, along: ax) far
  private about := line(ax.p1, far)
  pitch := std.Cone(about)
  generator.p2 coincident pitch
  tip := std.Offset(pitch, d: design.addendum * normal_module)
  region := { p |
    p inside tip
    p inside heel
    p outside toe
    p outside(design.back * normal_module) pitch
    // the tip's corners at both ends, `end_relief` each way
    tip chamfer(design.end_relief) toe
    tip chamfer(design.end_relief) heel
  }
  material := solid(region)
}

// What X0 found here, recorded in docs/fillets/README.md (finding 7):
//
// "`end_relief` each way" is not one measure. Today's chamfer is `size` along the tip cone from
// the corner, and on the sphere the point `size` below the tip cone (its distance from the tip's
// surface), not `size` from the corner. A chamfer whose legs are each the distance from the edge
// puts the sphere's leg a little further round, by the factor 1 / sin of the angle the tip makes
// with the sphere there, and `gear_cells.rs`'s recorded volumes move. Either the word says how a
// leg is measured, or the fixtures are re-recorded under the word's one measure.
