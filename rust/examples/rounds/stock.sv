// A stock block with a step across one end and a pocket in its top, its drawing's general note
// "all edges R1.5" (#962's fixture 150): every edge of the part rounded 1.5.
//
// `round(r: r) part` is every edge among the part's faces. But where edges of both turns meet at
// a vertex — the step's floor meets its wall in a concave edge between two convex edges of the
// block's side, each upright of the pocket is concave between two convex edges of the top — a
// ball rolled along every edge leaves a smooth surface only where the edge alone in its turn is
// rounded first and the others are rounded on the body it leaves: the side's round then goes on
// round the step's fill, and the top's rim round each upright's fill, a ring about it. Each round
// reads the body as its other terms leave it, so the note is two statements: the features' own
// edges first (the step's floor into its wall, the pocket's uprights and floor, all concave), then
// every edge on a solid of the body they leave (`whole := solid(part)`): `round(r: r) whole`
// pairs every face of it with every other, the fills' faces and their ends on the side and the
// top among them. Written as one `round(r: r) part`, the side's rounds and the step's fill each
// read the sharp block and touch along a line at the step's corners, which the export refuses,
// naming the vertex and the edge to round first.
//
// Built exactly: 50 deep, the stepped profile rounded at its corners (filled at the reflex one),
// each side's rim round it — runs, the corners' balls, and a torus of 3 about each fill's axis —
// less the pocket's rounded void and its rim. Edit a number and apply (⌘↵).
unit mm
use std
use std.solids (cut)
use std.rounds (round)

r := 1.5mm    // all edges

in std.top {
  outline := step(u: -40mm, v: -25mm) line -> step(u: 40mm) line -> step(v: 25mm) line ->
    step(u: -40mm) line -> close
  notch := step(u: 20mm, v: -26mm) line -> step(u: 41mm) line -> step(v: 26mm) line ->
    step(u: 20mm) line -> close
  opening := step(u: -30mm, v: -12mm) line -> step(u: 0mm) line -> step(v: 12mm) line ->
    step(u: -30mm) line -> close
}
block := solid(std.solids.Prism(outline, from: 0mm, to: 20mm))
shoulder := solid(std.solids.Prism(notch, from: 10mm, to: 21mm))
pocket := solid(std.solids.Prism(opening, from: 12mm, to: 21mm))
part := solid(block)
shoulder cut part
pocket cut part

// the features' own edges, each alone in its turn where it meets the part's
round(r: r) part.shoulder
round(r: r) part.pocket

// every edge, on the part as those leave it
whole := solid(part)
round(r: r) whole
