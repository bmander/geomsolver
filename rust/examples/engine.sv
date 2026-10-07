// A four-cylinder engine in three views, written as modules (§14.4).
//
// The dimension table is `engine.dims`; the reciprocating parts and the valvetrain are
// components in `engine.parts` and `engine.valvetrain`; and each view is one component in a
// module of its own — `EndSection`, `SideSection`, `PlanView` — drawn here in its plane.  The
// views are tied by projection, the draughtsman's way (§6.7): the end view owns every height and
// width, the side view every length along the crank axis, and the plan is placed almost wholly
// by the other two.  One crank angle in the table turns every piston in every view.

unit mm
use engine.parts
use std (horizontal)
use engine.dims
use engine.block
use engine.head
use engine.crankshaft
use engine.conrod
use engine.end_view
use engine.side_view

// the three views are the standard planes: the side view is the front plane, the end view
// the side plane, and the plan the top plane; where each is drawn on the sheet is `engine.svd`'s
O := point in std.front
fix((0, 0)) O

end := engine.end_view.EndSection(std.side.origin, dims: engine.dims.engine_dims) in std.side
side := engine.side_view.SideSection(O, dims: engine.dims.engine_dims) in std.front

// the castings, each one part in its three views (`engine.block`, `engine.head`): the block
// from the pan rail to the deck with its bores and main bearings, and the head standing on its
// gasket with the valves and the camshafts in their bearings
block := engine.block.EngineBlock(std.side, std.front, std.top, std.side.origin, O, std.top.origin, dims: engine.dims.engine_dims)
head := engine.head.CylinderHead(std.side, std.front, std.top, std.side.origin, O, std.top.origin, dims: engine.dims.engine_dims)

// the crankshaft, one part in both sections (`engine.crankshaft`): the throw of cylinder 1 and
// a ghost of 2 and 3's in the end section, the whole shaft in the side section, every pin's
// height carried across inside the part
crank := engine.crankshaft.Crankshaft(std.side, std.front, std.side.origin, end.bore, O, draw_end: 1, draw_side: 1, dims: engine.dims.engine_dims)

// the connecting rods, one part drawn in the views it shows in (`engine.conrod`): rod 1 in the
// end section and the side section both, with the shank's section A-A beside the plan; rods 2 to
// 4 in the side section only, the small end of each placed by the end-view image it shares —
// rod 1's for cylinder 4, and a ghosted rod a half turn on for cylinders 2 and 3
secA := point in std.top
std.top.origin distance(engine.dims.back + 120mm, along: x) secA
std.top.origin horizontal secA
rod1 := engine.conrod.ConRod(std.side, std.front, std.top, crank.t1[0].pin, end.bore, crank.pin_s[0], side.small[0], secA, draw_end: 1, draw_side: 1, draw_sec: 1, dims: engine.dims.engine_dims)
rod2 := engine.conrod.ConRod(std.side, std.front, std.top, crank.t2[0].pin, end.bore, crank.pin_s[1], side.small[1], secA, draw_end: 0, draw_side: 1, draw_sec: 0, dims: engine.dims.engine_dims)
rod3 := engine.conrod.ConRod(std.side, std.front, std.top, crank.t2[0].pin, end.bore, crank.pin_s[2], side.small[2], secA, draw_end: 0, draw_side: 1, draw_sec: 0, dims: engine.dims.engine_dims)
rod4 := engine.conrod.ConRod(std.side, std.front, std.top, crank.t1[0].pin, end.bore, crank.pin_s[3], side.small[3], secA, draw_end: 0, draw_side: 1, draw_sec: 0, dims: engine.dims.engine_dims)
ghost := engine.parts.Rod(crank.t2[0].pin, end.bore, dims: engine.dims.engine_dims) in std.side
piston1 := engine.parts.Piston(rod1.sm[0], pin: 1, dims: engine.dims.engine_dims) in std.side
ghost.small project side.small[1]
ghost.small project side.small[2]
rod1.sm[0] project side.small[3]

// the timing drive, on the front of the engine in the end section and edge on in the side
drive := engine.end_view.Drive(std.side.origin, head.cam_i, head.cam_e, dims: engine.dims.engine_dims) in std.side
drive_s := engine.side_view.DriveSide(O, head.cam, dims: engine.dims.engine_dims) in std.front

// how it looks: the dimensions the sheet shows, and nothing else
