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
// The dimension table is `vtwin.dims`.  Each part is a component of its own with its own three
// views (`vtwin.cylinder`, `vtwin.piston`, `vtwin.disc`, `vtwin.flywheel`, `vtwin.throttle`,
// and the plate, `vtwin.frame`): this sheet draws the plate in all of them, since it stands
// still, and the moving parts in the plane of swing only; each part's own sheet
// (`vtwin_cylinder.sv`, `vtwin_piston.sv`, …) draws it upright in all three with the
// dimensions a printer needs — `class detail`, which this sheet leaves hidden.  The side view is
// what the assembly adds beyond its parts, with every height projected from the view along the
// axis.  The drawing has one degree of freedom and it is the crank angle: `crank.theta` is a
// free variable (§5), so dragging the pin rocks both cylinders and moves both pistons in both
// views, and the arm's callout reads the angle it is at.  One lever angle in the table (`throttle`)
// turns the throttle.  The two banks are one component instanced twice, which is why their
// ports come out rotated rather than mirrored — the engine is one bank turned a quarter turn,
// and the drawing cannot say otherwise.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.frame
use vtwin.crank
use vtwin.bank
use vtwin.side_view

// the page is the view along the crank axis, where the V is; the side view stands to its right,
// its origin the crank axis on the plate's front face
point O hint(x: 0, y: 0)
ground O
views: ThreeViews(O, right: 240, up: 150)
point up hint(x: 0, y: 40) in views.front
O distance(0, along: x) up
O distance(40, along: y) up
line ref(O, up) class gone

plate: Frame(views.front, views.right, views.top, O, ref, views.right_origin, views.top_origin,
             draw_side: 1, draw_top: 0)
crank: Crank(views.front, O, ref)
bankR: Bank(views.front, O, crank.pin, plate.r.piv, alpha: alphaR, fw: fwB, dim: 1)
bankL: Bank(views.front, O, crank.pin, plate.l.piv, alpha: alphaL, fw: fwA, dim: 0)
side: SideView(views.right_origin) in views.right

// the two views agree: every height the side view shows is the front view's
crank.pin project side.pin_s             // the pin
plate.r.piv project side.pv              // a pivot
bankR.cyl.k_tl.p project side.cyB_top    // bank R (bank B, the thicker) is nearest in the side view
bankR.cyl.k_br.p project side.cyB_bot
bankL.cyl.k_tr.p project side.cyA_top
bankL.cyl.k_bl.p project side.cyA_bot
bankR.cyl.b_tl.p project side.boB_top    // and each bore's, which is a rod further from the plate on B
bankR.cyl.b_br.p project side.boB_bot
bankL.cyl.b_tr.p project side.boA_top
bankL.cyl.b_bl.p project side.boA_bot

// how it looks
style .dimension { display: none }
style .shown { display: inline }
style .detail { display: none }
style .point { display: none }
style .plane { display: none }
style .gone { display: none }
style .phantom { dash: 6 3; width: 0.6; color: #888888; display: geometry }
style .thin { width: 0.6 }
style .axis { dash: 14 3 2 3; width: 0.5; color: #888888 }
style .hidden { dash: 4 3; width: 0.6 }
style .barrel { dash: 4 3; width: 0.6 }
style .lever { width: 1.4; color: #2a7ab0 }
