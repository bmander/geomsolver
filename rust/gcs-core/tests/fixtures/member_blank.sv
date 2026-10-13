// A member's blank, as the spiral bevel's step 3 drew it when the pair was modules (it is now
// `spiral_bevel/hypoid.sv`'s `Blank`), kept for the tests that read a blank on its own
// (functional_solids/document.rs, fillet_meridian.rs). A member's blank: the points within its tip
// cone and its heel sphere and outside its toe sphere and its back cone, one region and its solid
// (§6.21). The cones are the pitch cone offset by the addendum and by the back depth (`std.Offset`,
// `distance(<= d, side: inside)`); the spheres are about its apex through the face span's ends
// (`FaceSpan`, the face width on the pitch generator centred on the mean point). Their sizes are
// the layout's, solved. The end relief chamfers the tip's toe and heel corners `end_relief` each
// way, each leg measured from the other face (`std.Chamfered`): on the sphere the flat bows with
// it, a parabola's arc turned about the axis, built exactly; a relief of nothing cuts nothing. Each
// set the region's body uses names the blank's face it bounds: a member built from the blank has
// faces `material.tip`, `.heel`, `.toe` and `.back`, which the checks read.
use std

// The face width along `generator` (apex -> mean point), centred on the mean point: the toe
// toward the apex, the heel away from it, each on the generator, wherever it is drawn.
component FaceSpan(generator: line, width: Length) {
  // Seeds only, rough: a tenth of the generator either side of the mean point.
  toe := point hint(at: generator.p2, toward: generator.p1, by: 0.1)
  heel := point hint(at: generator.p2, toward: generator.p1, by: -0.1)
  toe coincident generator
  heel coincident generator
  toe distance(width / 2, along: generator) generator.p2
  generator.p2 distance(width / 2, along: generator) heel
}

// The blank of a member: `generator` runs from the apex to the mean point; `ax` is the member's
// axis from its apex.
component MemberBlank(generator: line, ax: line, design: group, normal_module: Length) {
  span := FaceSpan(generator, width: design.face_width)
  toe := std.Sphere(generator.p1, r: hint(design.cone_distance))
  heel := std.Sphere(generator.p1, r: hint(design.cone_distance))
  span.toe coincident toe
  span.heel coincident heel
  // The pitch cone, about the member's axis; its half is the drawing's to solve.
  pitch := std.Cone(ax)
  generator.p2 coincident pitch
  tip := std.Offset(pitch, d: design.addendum * normal_module)
  back := { q | q distance(<= design.back * normal_module, side: inside) pitch }
  // the toe's side of the blank, a bound of its own for the toe's chamfer
  beyond_toe := { q | q outside toe }
  relief := design.end_relief
  region := { p | p inside tip; p inside heel; p outside toe; p inside back
                  p inside std.Chamfered(tip, beyond_toe, la: relief, lb: relief)
                  p inside std.Chamfered(tip, heel, la: relief, lb: relief) }
  material := solid(region)
}

preview {
  unit mm
  proportions := {face_width: 10mm, cone_distance: 50mm, addendum: 1, back: 4,
    end_relief: 0.2mm}
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
  member := MemberBlank(generator, ax, proportions, normal_module: 2mm)
}
