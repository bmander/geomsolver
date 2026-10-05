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
// The dimension table is `components.dims`.  Each part is a component of its own with its own three
// views (`components.cylinder`, `components.piston`, `components.disc`, `components.flywheel`, `components.throttle`,
// and the plate, `components.frame`): this sheet draws the plate in all of them, since it stands
// still, and the moving parts in the plane of swing only; each part's own sheet
// (`cylinder.svd`, `piston.svd`, …) draws it upright in all three with the
// dimensions a printer needs — `class detail`, which this sheet leaves hidden.  The side view is
// what the assembly adds beyond its parts, with every height projected from the view along the
// axis.  The drawing has one degree of freedom and it is the crank angle: `crank.theta` is a
// formal the call leaves unbound (§5), so dragging the pin rocks both cylinders and moves both
// pistons in both views, and the arm's callout reads the angle it is at.  One lever angle in the table (`throttle`)
// turns the throttle.  The two banks are one component instanced twice, which is why their
// ports come out rotated rather than mirrored — the engine is one bank turned a quarter turn,
// and the drawing cannot say otherwise.

unit mm
use std
use components.dims
use components.parts
use components.frame
use components.crank
use components.bank
use components.side_view

// the front plane is the view along the crank axis, where the V is; the side view is the side
// plane, its origin the crank axis on the plate's front face
in std.front {
  O := point
  fix(x == 0, y == 0) O
  up := point hint(x: 0, y: 40)
  O distance(0, along: x) up
  O distance(40, along: y) up
  ref := line(O, up)
}

layout := {front: std.front, origin: O, axis: ref}
plate := components.frame.Frame(layout, dims: components.dims.vtwin_dims)
crank := components.crank.Crank(O, ref, dims: components.dims.vtwin_dims) in std.front
bankR := components.bank.Bank(crank.pin, plate.r.piv, fw: components.dims.fwB, dim: 1, dims: components.dims.vtwin_dims) in std.front
bankL := components.bank.Bank(crank.pin, plate.l.piv, fw: components.dims.fwA, dim: 0, dims: components.dims.vtwin_dims) in std.front
// **the plate's side view is asked for, not drawn** (§6.11) — the part is a solid, so the
// assembly's side view of it is a reading of that solid and cannot disagree with the front view
// about how thick the plate is or how far the bearing boss stands off it

// what the *assembly* adds beyond its parts — the crank train along the shaft, a pivot bolt's
// stack, the two cylinders edge on — is still drawn, being hardware no part designs.  Its
// ordinates are measured from the plate's front face, which stands half a thickness in front of
// the plate's own zero: the plate is sectioned on its mid-plane (`components.frame`), and a solid's
// derived view stands where its plane's origin is
so := point hint(x: -components.dims.tp / 2, y: 0) in std.side
std.side.origin distance(-components.dims.tp / 2, along: x) so
std.side.origin distance(0, along: y) so       // the same height: the crank axis
side := components.side_view.SideView(so, dims: components.dims.vtwin_dims) in std.side

// the two views agree: every height the side view shows is the front view's
crank.pin project side.pin_s             // the pin
plate.r.piv project side.pv              // a pivot
bankR.cyl.k_tl project side.cyB_top    // bank R (bank B, the thicker) is nearest in the side view
bankR.cyl.k_br project side.cyB_bot
bankL.cyl.k_tr project side.cyA_top
bankL.cyl.k_bl project side.cyA_bot
bankR.cyl.b_tl project side.boB_top    // and each bore's, which is a rod further from the plate on B
bankR.cyl.b_br project side.boB_bot
bankL.cyl.b_tr project side.boA_top
bankL.cyl.b_bl project side.boA_bot

// how it looks
