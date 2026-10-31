// A groove cut across a crease: a rectangular bar leaning across the turn, drafted 2° both ways
// from the plane its section is drawn in, as a tool forged or cast from a parting line is, and
// turned about the block's axis from clear of one end to clear of the other. Each side folds at
// that plane in a convex crease, its two halves leaning in from it. A turn moves each ruling of a
// half as one affine field moves it, so each touches at one point a ruling: the inner side's
// halves, facing the axis, touch along a line each, a little either side of the plane through the
// axis, and the crease carries the contact from the one to the other round its fan, every normal
// between the halves' (docs/generating-sweeps.md, the constant-twist class, a prism drafted both
// ways, #684 AG). The groove's wall is the three turned, nearest the axis where the crease
// crosses the block's middle. The block loses what the bar's shadow covers, each point at radius
// r and height z where some point of the bar stands, read slice by slice.
unit mm
use std
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 10mm        // the bar: how far its inner side stands from the axis, how deep and wide its
depth := 3mm         // section is, how far it stands either side of its plane, how far it leans
width := 4mm         // across the turn and how far its sides are drafted
length := 4mm
lean := 20deg
draft := 2deg

// the bar, drawn in a plane through x leaning `lean` across the turn
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos(lean), -sin(lean))) along
holder := plane(u: across, v: along)
fix(origin == (0, 0, 0)) holder
in holder {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: reach, dv: -width / 2) holder
  b1 coords(du: reach + depth, dv: -width / 2) holder
  b2 coords(du: reach + depth, dv: width / 2) holder
  b3 coords(du: reach, dv: width / 2) holder
  private lower := line(b0, b1)
  private outer_side := line(b1, b2)
  private upper := line(b2, b3)
  private inner_side := line(b3, b0)
}
construction bar := solid(face(lower, outer_side, upper, inner_side), from: -length, to: length, draft: draft)

// the block: a ring about z in common with a wedge `sector` round from x
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: inner, dv: -half) std.front
  q1 coords(du: outer, dv: -half) std.front
  q2 coords(du: outer, dv: half) std.front
  q3 coords(du: inner, dv: half) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
block := solid(face(wall_in, underside, wall_out, topside), about: ax)
in std.top {
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 2 * outer, dv: 0mm) std.top
  w2 coords(du: 2 * outer * cos(sector), dv: 2 * outer * sin(sector)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(face(wa, wb, wc), from: -2 * half, to: 2 * half)
wedge common block

// the turn about the block's axis, and the groove it sweeps
swing := frame
swing turns(1) ax
construction groove := sweep(bar over t in (-140deg, 140deg), under: swing)
groove cut block
