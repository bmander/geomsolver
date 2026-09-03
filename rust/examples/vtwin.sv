// A V-twin oscillating-cylinder air engine, in two views, written as modules (§14.4, §6.7).
//
// Two printed cylinders rock on studs through one printed plate, their pistons' rods sharing a
// crank pin; the rocking lines each cylinder's port up with an intake port while the piston is
// driven and an exhaust port while it returns, so there are no valves.  A plenum inside the
// plate feeds both intakes from one boss on top, where a brass 1/4" NPT coupling takes the air
// line's quick-release plug and a rotary barrel throttle sits across the passage.  Hardware:
// 5/16" steel rod and two 608 bearings for the crankshaft, 3/16" rod for the pin, 1/4"-20
// threaded rod, springs and nylon-insert nuts for the pivots, an O-ring a piston.
//
// The dimension table is `vtwin.dims`; the frame, the crank train and a bank are one component
// each; the side view is what the assembly adds beyond its parts, with every height projected
// from the view along the axis.  The drawing has one degree of freedom and it is the crank
// angle: `crank.theta` is a free variable (§5), so dragging the pin rocks both cylinders and
// moves both pistons in both views, and the arm's callout reads the angle it is at.  One lever
// angle in the table (`tau`) turns the throttle.  The two banks are one
// component instanced twice, which is why their ports come out rotated rather than mirrored —
// the engine is one bank turned a quarter turn, and the drawing cannot say otherwise.

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

plate: Frame(O, ref) in views.front
crank: Crank(O, ref) in views.front
bankR: Bank(O, crank.pin, crank.eye, plate.r.piv, alpha: alphaR, dim: 1) in views.front
bankL: Bank(O, crank.pin, crank.eye, plate.l.piv, alpha: alphaL, dim: 0) in views.front
side: SideView(views.right_origin) in views.right

// the two views agree: every height the side view shows is the front view's
plate.p3.p project side.ptop         // the plate's top edge
plate.p0.p project side.pbot         // and its bottom
crank.pin project side.pin_s         // the pin
plate.r.piv project side.pv          // a pivot
bankR.k_tl.p project side.cyB_top    // bank R (bank B, the thicker) is nearest in the side view
bankR.k_br.p project side.cyB_bot
bankL.k_tr.p project side.cyA_top
bankL.k_bl.p project side.cyA_bot
plate.tb.p project side.T_s          // the throttle barrel
plate.tip project side.tip_s         // and its lever's tip, which says how far open it is

// how it looks
style .dimension { display: none }
style .shown { display: inline }
style .point { display: none }
style .plane { display: none }
style .gone { display: none }
style .phantom { dash: 6 3; width: 0.6; color: #888888; display: geometry }
style .thin { width: 0.6 }
style .axis { dash: 14 3 2 3; width: 0.5; color: #888888 }
style .hidden { dash: 4 3; width: 0.6 }
style .lever { width: 1.4; color: #2a7ab0 }
