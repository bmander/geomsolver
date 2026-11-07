// A V groove cut with a pointed tool raked along its feed: a V-bit, a cone ground to a 60° point
// on a round shank, its axis leaning 20° from upright toward where it goes, its point a millimetre
// into a block's top, slid along the block from clear of one end to clear of the other. The slide
// is not square to the bit's axis, so no plane it keeps stands square to the bit; it moves every
// point of the bit the same way, and the groove's wall is the bit's characteristic slid. On the
// cone that is two of its rulings, those whose normals lie square to the slide, and they meet at
// the point, where the cone meets its own axis and every normal of the cone round the apex is the
// bit's: the walk runs up one, turns at the apex and runs down the other. The walls are two planes
// meeting in a sharp edge along the apex's path, the groove's root (docs/generating-sweeps.md, the
// constant-twist class, an apex on the tool's axis, #721 G5). Each plane holds the slide and
// touches the cone, so it leans from upright by `asin(sin 30° / cos 20°)`: the block loses
// `depth² tan(asin(sin 30° / cos 20°))` along its 20, 12.569 mm³ of its 2000.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

length := 20mm       // the block: its length along the slide, its width across and its height
width := 10mm
height := 10mm
depth := 1mm         // the bit: how deep its point reaches into the block, its point's half
half := 30deg        // angle, its shank's radius and length, and how far its axis leans from
shank := 3mm         // upright toward where it goes
reach := 6mm
lean := 20deg

in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: 0mm, dv: -width / 2) std.top
  b1 coords(du: length, dv: -width / 2) std.top
  b2 coords(du: length, dv: width / 2) std.top
  b3 coords(du: 0mm, dv: width / 2) std.top
  private near_side := line(b0, b1)
  private far_end := line(b1, b2)
  private far_side := line(b2, b3)
  private near_end := line(b3, b0)
}
block := solid(std.solids.Prism(face(near_side, far_end, far_side, near_end), from: 0mm, to: height))

// the bit, drawn in a plane through its axis, its point down a little before the block's near
// end, its axis leaning `lean` from upright along the slide
private out := axis
fix(dir == (0, 1, 0)) out
private along := axis
fix(dir == (sin(lean), 0, cos(lean))) along
bench := plane(u: out, v: along)
fix(origin == (-4mm, 0, height - depth)) bench
in bench {
  private tip := point
  private lip := point
  private top := point
  private back := point
  tip coords(du: 0mm, dv: 0mm) bench
  lip coords(du: shank, dv: shank / tan(half)) bench
  top coords(du: shank, dv: reach + shank / tan(half)) bench
  back coords(du: 0mm, dv: reach + shank / tan(half)) bench
  private cone := line(tip, lip)
  private side := line(lip, top)
  private end := line(top, back)
  private spindle := line(back, tip)
}
construction bit := solid(std.solids.Revolution(face(cone, side, end, spindle), about: spindle))

// the slide along the block, 36 mm over the roll, from clear before it to clear past it
private way := axis
fix(dir == (1, 0, 0)) way
feed := frame
feed slides(36mm) way
construction groove := solid({ p | some t in (0deg, 360deg): p in bit at(under: feed) t })
groove cut block
