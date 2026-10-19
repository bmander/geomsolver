// Rounds composed by name. A disc's far rim is chamfered by a statement named `bevel`, and the two
// edges the chamfer makes are rounded by a statement that names it: `plate.bevel round(r: rr)
// plate`, every edge where a face the bevel gave the plate meets another of the plate's.
//
// A round reads the body as its other terms leave it, and as the rounds leave it whose faces it
// names by their statements' names — so this one reads the plate chamfered, and rounds edges
// the chamfer made. Which line comes first says nothing: the round is written above the chamfer
// here, and the name orders them. Rounds of one body that name nothing of each other each read
// the body before either, as ever.
//
// Built exactly: 55100.165 mm³ — the disc, less the chamfer's ring (Pappus), less the two slivers
// a 1 mm ball cannot reach at the chamfer's 135° edges. Edit a number and apply (⌘↵).
unit mm
use std (round, chamfer)

R := 30mm    // the disc's radius
d := 4mm     // the chamfer's legs
rr := 1mm    // the round over the chamfer's edges

in std.front {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 30)
  radius(R) rim
}
plate := solid(face(rim), depth: 20mm)

plate.bevel round(r: rr) plate
bevel := plate.rim chamfer(d: d) plate.far
