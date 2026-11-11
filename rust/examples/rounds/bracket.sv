// An L bracket: a plate 6 thick standing on a foot 6 thick, the edge where the foot meets the
// plate filled with a round of 3. Two slots run along the foot, each a bar 3.4 wide with a
// round end at either end, and the one nearer the plate reaches 2.7 into the plate's base, past
// the edge (#881).
//
// The slots cut the edge in three, and each run ends on a slot's square side: the round is
// carried on past each end and cut off there, so it stops flush with the slot's walls.  Within
// the ends of every run at once is within none of them — the runs either side of a slot end on
// its two sides, facing each other — so a point is read against the run nearest it, the one the
// round beside it is built along.
//
// Drawn on the top plane, the plate rising 60 from it. Orbit (right-drag) to see the round end
// at the slots. Edit `r` or the slots' `start` and apply (⌘↵): with the slots starting at 7.5
// the runs end on the slots' round ends instead, kept outside them.
unit mm
use std
use std.place (coords)
use std.solids (union, cut)
use std.rounds (round)

r := 3mm        // the round where the foot meets the plate
start := 5mm    // where each slot's round end stands, along the foot

component Block(x0: Length, x1: Length, y0: Length, y1: Length, z0: Length, z1: Length) {
  in std.top {
    loop := step(u: x1, v: y0) (ab := line) -> step(v: y1) (bc := line) ->
            step(u: x0) (cd := line) -> step(v: y0) (da := line) -> close
  }
  body := solid(std.solids.Prism(loop, from: z0, to: z1))
}

component Post(x: Length, y: Length, d: Length, z0: Length, z1: Length) {
  in std.top {
    c := point
    c coords(du: x, dv: y) std.top
    k := radius(d / 2) circle(center: c)
  }
  body := solid(std.solids.Prism(face(k), from: z0, to: z1))
}

component Bracket(r: Length, start: Length) {
  plate := Block(x0: 0mm, x1: 6mm, y0: -20mm, y1: 20mm, z0: 0mm, z1: 60mm)
  foot := Block(x0: 5mm, x1: 28mm, y0: -20mm, y1: 20mm, z0: 0mm, z1: 6mm)
  body := solid(plate.body)
  foot.body union body
  // a slot along the foot, through it, its near end reaching into the plate's base
  repeat sy in [-12mm, 12mm] {
    mid := Block(x0: start, x1: 22mm, y0: sy - 1.7mm, y1: sy + 1.7mm, z0: -1mm, z1: 7mm)
    near := Post(x: start, y: sy, d: 3.4mm, z0: -1mm, z1: 7mm)
    far := Post(x: 22mm, y: sy, d: 3.4mm, z0: -1mm, z1: 7mm)
    mid.body cut body
    near.body cut body
    far.body cut body
  }
  body.foot round(r: r) body.plate
}

bracket := Bracket(r: r, start: start)
