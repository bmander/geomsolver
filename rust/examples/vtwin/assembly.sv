// A V-twin oscillating-cylinder air engine, in two views, written as modules (§14.4, §6.7).
//
// Two printed cylinders rock on bolts through one printed plate, their pistons' rods sharing a
// crank pin; the rocking lines each cylinder's port up with an intake port while the piston is
// driven and an exhaust port while it returns, so there are no valves.  A plenum inside the
// plate feeds both intakes from one boss on top, where a brass 1/4" NPT coupling takes the air
// line's quick-release plug and a rotary barrel throttle sits across the passage.  Hardware:
// 5/16" steel rod and two 608 bearings for the crankshaft, a 1/4" clevis pin and hairpin cotter
// for the crank pin, a 1/4"-20 hex bolt, spring and nylon-insert nut for each pivot, an O-ring a
// piston, three on the throttle, #8-32 set screws in trapped nuts on the disc and the flywheel.
//
// The dimension table is `components.dims`, whose numbers each part reads as constants of its own
// definition.  Each part is a definition, a function of what it is mated to, with its own three
// views (`components.cylinder`, `components.piston`, `components.disc`, `components.flywheel`, `components.throttle`,
// and the plate, `components.frame`): this sheet draws the plate in all of them, since it stands
// still, and the moving parts in the plane of swing only; each part's own sheet
// (`cylinder.svd`, `piston.svd`, …) draws it upright in all three with the
// dimensions a printer needs.  Each part's file has a `preview` of its own, so its dimensions are
// drawn there and not here: this document draws only the numbers it states.  The side view is
// what the assembly adds beyond its parts, with every height projected from the view along the
// axis.
//
// **Each moving part is drawn in its own frame, and its pose is a function of the crank angle.**
// `t` is that angle, a parameter the drawing runs over (§6.5): the crank's arm stands at `t` from
// the reference, the disc's frame turns with it, and each bank's two statements place its
// cylinder's frame at the pivot and its piston's at the crown.  Every other relation holds for
// every `t`, so the drawing is a function of `t`: drawn at `t = 0`, the pin straight up, and
// followed round the turn.  In the app a timeline scrubs it and dragging the pin turns it, rocking
// both cylinders and moving both pistons in both views; the timeline reads the instant shown.
// The assembly's sheet shows it at `t = 180deg`.  One lever angle in the table (`throttle`) turns
// the throttle.  The two banks are one part instanced twice, which is why their
// ports come out rotated rather than mirrored — the engine is one bank turned a quarter turn,
// and the drawing cannot say otherwise.

unit mm
use std
use std.place (above, left_of)
use components.dims
use components.parts
use components.frame
use components.crank
use components.bank
use components.side_view

param t: Angle over (0deg, 360deg)   // the crank angle

// the front plane is the view along the crank axis, where the V is; the side view is the side
// plane, its origin the crank axis on the plate's front face
in std.front {
  O := point
  fix((0, 0)) O
  up := point hint((0, 40))
  up above(d: 40) O
  ref := line(O, up)
}

// every part is drawn in the plane its operands are drawn in: the plate, the crank and both banks
// in the view along the crank axis, where `O` and the pin are
plate := components.frame.Frame(O, ref)
crank := components.crank.Crank(O, ref, theta: t)
bankR := components.bank.Bank(crank.pin, plate.pivots.r, z: components.dims.zB)
bankL := components.bank.Bank(crank.pin, plate.pivots.l, z: components.dims.zA)
// **the plate's side view is asked for, not drawn** (§6.11) — the part is a solid, so the
// assembly's side view of it is a reading of that solid and cannot disagree with the front view
// about how thick the plate is or how far the bearing boss stands off it

// what the *assembly* adds beyond its parts — the crank train along the shaft, a pivot bolt's
// stack, the two cylinders edge on — is still drawn, being hardware no part designs.  Its
// ordinates are measured from the plate's front face, which stands half a thickness in front of
// the plate's own zero: the plate is sectioned on its mid-plane (`components.frame`), and a solid's
// derived view stands where its plane's origin is
so := point hint((-components.dims.tp / 2, 0)) in std.side
so left_of(d: components.dims.tp / 2) std.side.origin  // the same height: the crank axis

// the two views agree: every height the side view shows is the front view's, and it is given the
// points whose heights it shows — the pin, a pivot, and each cylinder's highest and lowest
// corners, outside and in the bore.  Bank R (bank B, the thicker) is nearest in the side view,
// and its bore a rod further from the plate than bank L's
side := components.side_view.SideView(so, crank.pin, plate.pivots.r, bankR.span_cw, bankL.span_ccw)
