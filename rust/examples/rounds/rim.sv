// A box rounded over its rounded corners (#879): its four upright edges rounded 5, then its top
// rim rounded 1 and its bottom rim chamfered 1, each all the way round — along the four straight
// edges and round the arcs the corners' round left, one tangent run.
//
// A round reads the body as its other terms leave it, and not as another round of it does: the
// rounds of one body that name nothing of each other each read it before either. Written beside
// the corners' round, `box.near round(r: rr) box` would run each top edge to the box's sharp
// corner, and the two would only cross there. The rim is to follow the corners, so it is rounded
// on a solid of the rounded box: `part := solid(box)` is the box as its rounds leave it, and its
// faces are `part.box`'s — the four sides, the two caps, and the four the corners' round made
// (`part.box.taken.round`). `part.box.near round(r: rr) part.box` pairs the top with every one of
// them: a run along each straight edge and a quarter ring round each arc, glued where a side
// goes on tangent into a corner's round. Which line comes first says nothing: `part` is what
// names `box`.
//
// (A round named, `vert := f round(r: rv) g`, puts its faces under the name, and `box.near
// round(r: rr) box.vert` rounds the four arcs alone: `rounds/composed.sv`.)
//
// Built exactly: 12185.690 mm³ — the box less its corners, less the rim's round along 110 of
// straight edge and once round an arc of 5 (Pappus), less the chamfer's the same. Edit a number
// and apply (⌘↵).
unit mm
use std
use std.solids
use std.rounds (round, chamfer)

w := 50mm    // the box, across
d := 25mm    // and deep
h := 10mm    // and tall
rv := 5mm    // the upright edges' round
rr := 1mm    // the top rim's round
c := 1mm     // the bottom rim's chamfer

in std.top {
  loop := step(u: w, v: 0mm) (ab := line) -> step(v: d) (bc := line) ->
          step(u: 0mm) (cd := line) -> step(v: 0mm) (da := line) -> close
}
box := solid(std.solids.Prism(loop, from: 0mm, to: h))

// the upright edges: where two sides, both along z, meet
repeat f in box {
  repeat g in box {
    when f parallel std.z and g parallel std.z and f != g {
      f round(r: rv) g
    }
  }
}

// the rims, over the box those rounds leave
part := solid(box)
part.box.near round(r: rr) part.box
part.box.far chamfer(d: c) part.box
