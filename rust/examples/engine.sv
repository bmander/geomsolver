// A four-cylinder engine in three views, written as modules (§14.4).
//
// The dimension table is `engine.dims`, whose numbers each part reads as constants of its own
// definition; the reciprocating parts and the valvetrain are in `engine.parts` and
// `engine.valvetrain`; and what each view adds beyond its parts is one part in a module of its
// own — `EndSection`, `SideSection` — drawn here in its plane.  Every definition is a part (§8.2):
// a function of what it is given, so the assembly places each by what it is mated to, and nothing
// here reaches inside one.  The views are tied by projection, the draughtsman's way (§6.7): the
// end view owns every height and width, the side view every length along the crank axis, and the
// plan is placed almost wholly by the other two.  One crank angle turns every piston in every
// view.

unit mm
use engine.parts
use std (right_of)
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

end := engine.end_view.EndSection(std.side.origin) in std.side
side := engine.side_view.SideSection(O) in std.front

// the castings, each one part in its three views (`engine.block`, `engine.head`): the block
// from the pan rail to the deck with its bores and main bearings, and the head standing on its
// gasket with the valves and the camshafts in their bearings
block := engine.block.EngineBlock(std.side, std.front, std.top, std.side.origin, O, std.top.origin)
head := engine.head.CylinderHead(std.side, std.front, std.top, std.side.origin, O, std.top.origin)

// the camshafts, each a part riding in its journal in the head (`engine.head`), every lobe timed
// to its cylinder's valves by the cycle; and the valves, a pair at each station along the axis,
// before and behind each bore's, as far off their seats as the cycle puts them
camshaft_i := engine.head.Camshaft(head.j_i, head.vaxis_i, head.cam, intake: 1)
camshaft_e := engine.head.Camshaft(head.j_e, head.vaxis_e, head.cam, intake: 0)
valves1f := engine.head.Valves(head, cylinder: 1, aft: 0)
valves1r := engine.head.Valves(head, cylinder: 1, aft: 1)
valves2f := engine.head.Valves(head, cylinder: 2, aft: 0)
valves2r := engine.head.Valves(head, cylinder: 2, aft: 1)
valves3f := engine.head.Valves(head, cylinder: 3, aft: 0)
valves3r := engine.head.Valves(head, cylinder: 3, aft: 1)
valves4f := engine.head.Valves(head, cylinder: 4, aft: 0)
valves4r := engine.head.Valves(head, cylinder: 4, aft: 1)

// the crankshaft, a part in both sections (`engine.crankshaft`): the throw of cylinder 1 and a
// ghost of 2 and 3's in the end section, the whole shaft in the side section, every pin's height
// carried across inside the part, at the table's crank angle
crank := engine.crankshaft.Crankshaft(std.side.origin, end.bore, O, theta: engine.dims.theta)

// the connecting rods, each a part on its pin and in its bore (`engine.conrod`): it draws itself
// in every view its pin is drawn in, and makes the plane its shank's section A-A is cut in.  Which
// of those pictures the sheet shows, and how, is `engine.svd`'s: rod 1 in the end section, rod 2
// ghosted there a half turn on, every rod in the side section, and rod 1's section A-A
rod1 := engine.conrod.ConRod(crank.pin1, end.bore)
rod2 := engine.conrod.ConRod(crank.pin2, end.bore)
rod3 := engine.conrod.ConRod(crank.pin3, end.bore)
rod4 := engine.conrod.ConRod(crank.pin4, end.bore)

// the pistons, each on its rod's small end: cylinder 1's in the end section, its pin seen end on,
// and all four in the side section
piston1 := engine.parts.Piston(rod1.sm, pin: 1) in std.side
pistons1 := engine.parts.Piston(rod1.sm_s, pin: 0) in std.front
pistons2 := engine.parts.Piston(rod2.sm_s, pin: 0) in std.front
pistons3 := engine.parts.Piston(rod3.sm_s, pin: 0) in std.front
pistons4 := engine.parts.Piston(rod4.sm_s, pin: 0) in std.front

// the timing drive, on the front of the engine in the end section and edge on in the side
drive := engine.end_view.Drive(std.side.origin, head.cam_i, head.cam_e)
drive_s := engine.side_view.DriveSide(O, head.cam) in std.front

// how it looks: the dimensions the sheet shows, and nothing else
