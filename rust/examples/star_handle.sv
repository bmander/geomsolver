// A radiator valve's star handle (#576): four arms with straight sides, a rounded tip on each and
// a fillet where each arm meets the next, extruded with a draft.  The outline is a polygon of
// eight corners with every corner rounded, and it is written as that: one arm and the notch
// after it, turned four times, each corner a joint `-> round(r: …)`.
//
// The round mints its arc, a tangency at each end and its branch: the arc lies inside its corner,
// on the side the walk turns toward — the tips convex, the notches concave — so no seed and no
// `ccw`/`cw` says which of the four tangent circles is meant, and the only hints are the corners'.
// Each corner stays a point of the drawing, on both its lines, and the spans are measured over
// the rounds: across the tips' rounded ends and across the fillets, each at the arc's middle.
unit mm
use std

span := 74.2       // across the tips' rounded ends
waist := 25.8      // across the fillets
tip_r := 4
fillet_r := 3
height := 12
draft := 2deg

in std.front {
  hub := point
  fix((0, 0)) hub
  ring 4 about hub {
    tip := point hint((53, 0))
    notch := point hint((9, 9))
    // each side the same length, so the arm is symmetric about its own centre line
    line(notch, tip) equal line(notch, next.tip)
    // the spans, over the rounds: each arc's middle
    tip_end := point
    tip_end coincident(t == 0.5) tip_round
    hub distance(span / 2) tip_end
    waist_end := point
    waist_end coincident(t == 0.5) fillet
    hub distance(waist / 2) waist_end
    // the arm and the notch after it; the last round, at the brace, rounds the next arm's tip
    (side := line(tip, notch)) -> (fillet := round(r: fillet_r))
      (next_side := line(notch, next.tip)) -> (tip_round := round(r: tip_r))
  }
  hub level(v) tip[0]
  outline := face(side[0], fillet[0], next_side[0], tip_round[0], side[1], fillet[1],
    next_side[1], tip_round[1], side[2], fillet[2], next_side[2], tip_round[2], side[3],
    fillet[3], next_side[3], tip_round[3])
}
handle := solid(outline, depth: height, draft: draft)
