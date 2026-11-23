// A part bought in, as its maker's STEP file gives it (#993 stage 2): `drilled_block.step` beside
// this document is a 40 × 30 × 20 mm block bored through (r 6 mm), its corner at the origin.  The
// document names the file and states only what the designer changes over it: a 5 mm hole drilled
// through, 10 mm in from two edges.  Nothing of the block is drawn again.
unit mm
use drilled_block
use std
use std.solids (cut)
use std.measures (volume)

block := solid(drilled_block)

in std.top {
  c := point hint((10, 10))
  fix((10, 10)) c
  hole := circle(center: c) hint(r: 2.5)
  radius(2.5mm) hole
}
drill := solid(std.solids.Prism(face(hole), from: -5mm, to: 25mm))
drill cut block

// the block bought in, less the bore and the hole: under 21.4 cm³
claim volume(<= 21400 * 1mm^3) block
