// X0 (#183): the spiral bevel's crown tooth as a region about the cutter's axis, its two top edges
// rounded (#145's `Tooth`, L5). Does not parse yet.
//
// Today the tooth is `crown/section.sv`'s `RackSection` (30 statement lines: a six-link chain
// with two tangent arcs in the normal view, eight seeded points so the flanks start on the right
// branch, and a bound so the rounds cannot meet in a point) plus `crown/rounding.sv`'s
// `TipRounding` (9), revolved about the cutter's axis. As a region it is four bounds and two
// rounds, and nothing is drawn.
use std

// A crown tooth: beyond `a` and within `b` (each grown by `grow`, the backlash), above `lo` and
// below `hi`, its two top edges rounded to `r`.
component Tooth(a: group, b: group, lo: group, hi: group, grow: Length, r: Length) := { p |
  // A round names the sets the body bounds by. #145 wrote `p outside(grow) a` and `p inside(grow)
  // b`, so the surfaces the body bounds by are `a` shrunk and `b` grown, and those are what a
  // round must name: `a round(r) hi` would name a surface the body never states (finding 8).
  // `std.Offset` only grows (a negative distance is refused), so the shrunk flank is written out.
  // With no backlash (`fixtures::gear::design`) `grow` is 0, and 0.64 refuses a distance of zero
  // from a set: an offset of zero must be the identity, as a round of zero is (finding 10).
  private fa := { q | q distance(grow, side: inside) a }
  private fb := std.Offset(b, d: grow)
  p outside fa
  p inside fb
  p outside lo
  p inside hi
  fa round(r) hi
  fb round(r) hi
}

// As `crown/tooth.sv` would call it, after #145's F3 (`std.ConeThrough`, `std.HalfSpace`):
//
//   inner := std.ConeThrough(cut_ax, ahead.end, lean: pressure + shift)
//   outer := std.ConeThrough(cut_ax, behind.end, lean: shift - pressure)
//   tooth_tip := std.HalfSpace(C, cut_ax, dedendum * mn)
//   tooth_base := std.HalfSpace(C, cut_ax, -base * mn)
//   tooth := solid(Tooth(inner, outer, tooth_base, tooth_tip, grow: backlash / 4,
//                        r: rounding * mn))
//
// The land bound (`length(>= 0.01 * design.module) tip`) has no place to stand: there is no
// drawn tip line. Whether two rounds may meet and consume the tip is the definition's question
// (finding 9): rounds that meet take the face between them, and the tooth comes to a full round.
