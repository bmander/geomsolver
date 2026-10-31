// A slot cut by an end mill: a flat-bottomed cutter standing upright, slid across a block from
// clear of one end to clear of the other. A slide moves every point of the cutter the same way at
// every time, so the slot's walls are where the cutter's side moves along itself, the two lines up
// it square to the slide; and its flat end, which the slide keeps in its own plane, touches all
// over, so it sweeps a strip of that plane, the slot's floor, from where it stands farthest one
// side of the slide to where it stands farthest the other: its diameter square to the slide
// (docs/generating-sweeps.md, the constant-twist class, a flat face the motion keeps in its own
// plane, #684 O). The walls meet the floor in sharp edges along the paths of that diameter's ends.
// The block loses the slot's section, `2 radius × (height − floor)`, along its length:
// 4 × 3 × 20 = 240 mm³ of its 2000, leaving 1760 mm³.
unit mm
use std
use std.place (coords)
use std.motion (slides)

length := 20mm       // the block: its length along the slide, its width across and its height
width := 10mm
height := 10mm
radius := 2mm        // the cutter: its radius, how high its flat end stands, and how far it
floor := 7mm         // reaches above that
flute := 8mm

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
block := solid(face(near_side, far_end, far_side, near_end), from: 0mm, to: height)

// the cutter, drawn in a plane through its axis, which stands upright a little before the block
private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (-5mm, 0, floor)) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: radius, dv: 0mm) bench
  e2 coords(du: radius, dv: flute) bench
  e3 coords(du: 0mm, dv: flute) bench
  private bottom := line(e0, e1)
  private side := line(e1, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction mill := solid(face(bottom, side, top, spindle), about: spindle)

// the slide along the block, 30 mm over the roll, from clear before it to clear past it
private way := axis
fix(dir == (1, 0, 0)) way
feed := frame
feed slides(30mm) way
construction slot := sweep(mill over t in (0deg, 360deg), under: feed)
slot cut block
