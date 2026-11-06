// A bull-nose mill plunged at a lean into a block, stopping inside it: radius 2 mm and 3 mm tall,
// its bottom corner rounded to 0.5 mm, its axis leaning 30° from upright, pushed straight down 8 mm
// from clear above the block until wholly under the block's top. The round touches the push along
// the half of its outer contact that lies on it, ending where the contact crosses the seam into the
// side's two lines whose normal is square to the push; carried down, the round's contact is the
// sheet over its outline seen along the push, and each of those lines the plane it is tangent along
// — and the sheet and each plane touch along the path of the point where the contact crosses the
// seam, which the kernel is given, not left to find (the closed class, #810). The flat end, the side
// and the top rim close it as a drum's do. Every line down the push meets the mill once or not at
// all, so the block loses each line's depth to the mill's underside where it stops: the flux of
// `(0, 0, z)` out of its faces turned down, its volume less that out of those turned up.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

lean := 30deg        // how far the mill's axis leans from upright

in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: -8mm, dv: -8mm) std.top
  b1 coords(du: 8mm, dv: -8mm) std.top
  b2 coords(du: 8mm, dv: 8mm) std.top
  b3 coords(du: -8mm, dv: 8mm) std.top
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(std.solids.Prism(face(s0, s1, s2, s3), from: -8mm, to: 0mm))

// the mill, drawn in a plane through its axis, the middle of its flat end 4 mm above the block's top
private across := axis
fix(dir == (cos(lean), 0, -sin(lean))) across
private along := axis
fix(dir == (sin(lean), 0, cos(lean))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, 4)) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: 1.5mm, dv: 0mm) bench
  e2 coords(du: 2mm, dv: 3mm) bench
  e3 coords(du: 0mm, dv: 3mm) bench
  private k := point
  k coords(du: 1.5mm, dv: 0.5mm) bench
  private e4 := point
  e4 coords(du: 2mm, dv: 0.5mm) bench
  private bottom := line(e0, e1)
  private corner := arc(center: k, start: e1, end: e4) hint(r: 0.5)
  radius(0.5mm) corner
  private side := line(e4, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction mill := solid(std.solids.Revolution(face(bottom, corner, side, top, spindle), about: spindle))

// the push: 8 mm straight down, the flat end's middle ending 4 mm under the block's top
private down := axis
fix(dir == (0, 0, -1)) down
ram := frame
ram slides(8mm) down
construction bore := solid({ p | some t in (0deg, 360deg): p in mill at(under: ram) t })
bore cut block
