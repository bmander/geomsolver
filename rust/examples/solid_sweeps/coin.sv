// A coin turned a quarter turn about a line through its rim, half sunk in a slab: a disc of radius
// 2 mm and 0.5 mm thick, standing on edge with its rim touching the upright axis, swung a quarter
// turn about it. The turn moves each point square to the plane through it and the axis, so the
// coin's rim touches along its top and bottom lines, where its normal stands upright, and its two
// sharp circles join them round their fans, down to where they touch the axis: there the contact
// meets the axis, a point that does not move, where the sheets it carries round close on a pole.
// Read on the whole coin, each piece of contact is carried exactly round the turn on the surface
// its meridian turns into (a plane, a cone, a ball), the coin at both ends of the turn beside them,
// every meeting of two of them given by construction, and what the coin turned back holds trimmed
// away (the contact's road, docs/generating-sweeps.md, #880 Q1b). The slab's top is the coin's
// middle plane, which the turn keeps, so it loses half of what the coin sweeps: at each height and
// radius the coin holds one arc of the circle about the axis, which the turn lengthens by its
// quarter turn — integrated apart from the build, 46.547001 mm³ swept, of which the slab loses
// 23.273501 of its 300, leaving 276.726499 mm³.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (turns)

radius := 2mm        // the coin: its radius and thickness
thick := 0.5mm
side := 10mm         // the slab: its side, and its depth under the coin's middle plane
depth := 3mm

in std.top {
  private s0 := point
  private s1 := point
  private s2 := point
  private s3 := point
  s0 coords(du: -side / 2, dv: -side / 2) std.top
  s1 coords(du: side / 2, dv: -side / 2) std.top
  s2 coords(du: side / 2, dv: side / 2) std.top
  s3 coords(du: -side / 2, dv: side / 2) std.top
  private e0 := line(s0, s1)
  private e1 := line(s1, s2)
  private e2 := line(s2, s3)
  private e3 := line(s3, s0)
}
slab := solid(std.solids.Prism(face(e0, e1, e2, e3), from: -depth, to: 0mm))

// the upright axis the coin turns about
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
}

// the coin, drawn in the upright plane through the axis that its face lies along
private out := axis
fix(dir == (1, 0, 0)) out
private lift := axis
fix(dir == (0, 0, 1)) lift
holder := plane(u: out, v: lift)
fix(origin == (0, 0, 0)) holder
in holder {
  private c := point
  c coords(du: radius, dv: 0mm) holder
  private rim := circle(center: c) hint(r: 2)
  radius(radius) rim
}
construction coin := solid(std.solids.Prism(face(rim), from: -thick, to: 0mm))

// the quarter turn about the upright axis through the coin's rim
swing := frame
swing turns(1) ax
construction swept := solid({ p | some t in (0deg, 90deg): p in coin at(under: swing) t })
swept cut slab
