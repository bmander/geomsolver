// X3 (#183): each feature the rounds and chamfers in `std.sv` need, used where nothing is
// rounded or chamfered. Does not parse. A feature that has no use outside fillets is a fillet
// feature in disguise, and does not go in the language (docs/fillets/README.md, X3).
unit mm
use std

// [F1] A face is a set: its patch, and its surface. A bolt hole's centre on a plate's top face,
// a bracket standing on it, a dimension to it.
hole.center coincident plate.top
bracket.base.p1 coincident plate.top
p distance(12mm) plate.top.surface

// [F2] Distance from a set bounded by several conditions: a curve (two surfaces met) or a region.
// A weld's reach from the edge it runs along; a part's clearance envelope (a body grown by a gap,
// the Minkowski sum); a pocket's tool path, the pocket shrunk by the cutter's radius.
bead := { p | p distance(<= 6mm) { q | q coincident frame.web; q coincident frame.flange } }
keep_out := std.Offset({ p | p inside engine.block; p inside engine.head }, d: 5mm)
path := { c | c inside pocket; c distance(>= 3mm) pocket.walls.surface }

// [F3] A bound on an expression of measurements in a set's body. An ellipse by its foci; the
// points nearer one bore than another (a Voronoi cell: where a split line between two bosses
// falls); a cable that reaches from an anchor round a pulley.
oval := { p | distance(p, f1) + distance(p, f2) <= 2 * a }
nearer := { p | distance(p, bore_a.axis) <= distance(p, bore_b.axis) }
reach := { p | distance(p, anchor) + distance(p, pulley) <= cable }

// [F4] Repetition over a body's faces. A drain hole in every face of a box; a claim per face; a
// class on every face of a part for a sheet.
repeat f in box {
  claim f clear(2mm) wall
}

// [F5] A word whose body makes solids and puts them in a body's terms, and the body a face is
// of. The library's own features written in Solvent: a bore at a point of a face, a counterbore,
// a slot.
bore(d, depth) f at(p) := {
  hole := solid(face(circle(center: p) hint(r: d / 2)), depth: depth)
  hole cut f.body
}

// [F6] A term that reads a body acts on the body as made by the terms that make what it reads
// (X0's ordering rule, X4): drill through whatever the body has at that face, the boss included;
// a sheet's dimension of the finished face.

// [F7] The faces an edge ends on. A weld bead along an edge, stopped where the edge stops; a
// sheet's callout of an edge's length between the faces it runs from and to.
web_edge := { q | q coincident frame.web; q coincident frame.flange }
bead := { p | p distance(<= 6mm) web_edge; p inside web_edge.ends }

// [F8] Repetition with a condition. Every face that faces up; every edge between two named
// groups that also meets a third.
repeat f in box if angle(f.surface, std.z) < 1deg {
  f class top_face
}

// [F9] The material's angle at an edge: under a half turn the edge is convex, over it concave.
// The sheet's rule for drawing an edge (a crease where faces meet at an angle, never a seam where
// they meet tangent); a claim that an edge is an outside corner a deburring tool can reach.
claim angle(part.web, part.flange) < 180deg
