// A four-cylinder engine in three views, written as modules (§14.4).
//
// The dimension table is `engine.dims`; the reciprocating parts and the valvetrain are
// components in `engine.parts` and `engine.valvetrain`; and each view is one component in a
// module of its own — `EndSection`, `SideSection`, `PlanView` — drawn here in its plane.  The
// views are tied by projection, the draughtsman's way (§6.7): the end view owns every height and
// width, the side view every length along the crank axis, and the plan is placed almost wholly
// by the other two.  One crank angle in the table turns every piston in every view.

unit mm
use std
use engine.dims
use engine.block
use engine.head
use engine.crankshaft
use engine.conrod
use engine.end_view
use engine.side_view

// the three views, from the standard library: the page is the side view, the end view stands
// to the right of it turned so up stays up, and the plan is folded up above it
point O hint(x: 0, y: 0)
ground O
views: ThreeViews(O, right: 620, up: 620)

end: EndSection(views.right_origin) in views.right
side: SideSection(O) in views.front

// the castings, each one part in its three views (`engine.block`, `engine.head`): the block
// from the pan rail to the deck with its bores and main bearings, and the head standing on its
// gasket with the valves and the camshafts in their bearings
block: EngineBlock(views.right, views.front, views.top, views.right_origin, O, views.top_origin)
head: CylinderHead(views.right, views.front, views.top, views.right_origin, O, views.top_origin)

// the crankshaft, one part in both sections (`engine.crankshaft`): the throw of cylinder 1 and
// a ghost of 2 and 3's in the end section, the whole shaft in the side section, every pin's
// height carried across inside the part
crank: Crankshaft(views.right, views.front, views.right_origin, end.bore, O, draw_end: 1, draw_side: 1)

// the connecting rods, one part drawn in the views it shows in (`engine.conrod`): rod 1 in the
// end section and the side section both, with the shank's section A-A beside the plan; rods 2 to
// 4 in the side section only, the small end of each placed by the end-view image it shares —
// rod 1's for cylinder 4, and a ghosted rod a half turn on for cylinders 2 and 3
point secA in views.top
views.top_origin distance(back + 120mm, along: x) secA
views.top_origin distance(0, along: y) secA
rod1: ConRod(views.right, views.front, views.top, crank.t1[0].pin, end.bore, crank.pin_s[0], side.small[0], secA, draw_end: 1, draw_side: 1, draw_sec: 1)
rod2: ConRod(views.right, views.front, views.top, crank.t2[0].pin, end.bore, crank.pin_s[1], side.small[1], secA, draw_end: 0, draw_side: 1, draw_sec: 0)
rod3: ConRod(views.right, views.front, views.top, crank.t2[0].pin, end.bore, crank.pin_s[2], side.small[2], secA, draw_end: 0, draw_side: 1, draw_sec: 0)
rod4: ConRod(views.right, views.front, views.top, crank.t1[0].pin, end.bore, crank.pin_s[3], side.small[3], secA, draw_end: 0, draw_side: 1, draw_sec: 0)
ghost: Rod(crank.t2[0].pin, end.bore) in views.right class phantom
piston1: Piston(rod1.sm[0], pin: 1) in views.right
ghost.small project side.small[1]
ghost.small project side.small[2]
rod1.sm[0] project side.small[3]

// the timing drive, on the front of the engine in the end section and edge on in the side
drive: Drive(views.right_origin, head.cam_i, head.cam_e) in views.right
drive_s: DriveSide(O, head.cam) in views.front

// how it looks: the dimensions the sheet shows, and nothing else
style .dimension { display: none }
style .shown { display: inline }
style .point { display: none }
style .phantom { dash: 6 3; width: 0.6; color: #888888; display: geometry }
style .thin { width: 0.6 }
style .belt { width: 1.2; color: #2a7ab0 }
style .axis { dash: 14 3 2 3; width: 0.5; color: #888888 }
style .hidden { dash: 4 3; width: 0.6 }
style .plane { display: none }
