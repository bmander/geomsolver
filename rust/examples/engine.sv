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
use engine.conrod
use engine.end_view
use engine.side_view
use engine.top_view

// the three views, from the standard library: the page is the side view, the end view stands
// to the right of it turned so up stays up, and the plan is folded up above it
point O hint(x: 0, y: 0)
ground O
views: ThreeViews(O, right: 620, up: 620)

end: EndSection(views.right_origin) in views.right
side: SideSection(O) in views.front
plan: PlanView(views.top_origin) in views.top

// the connecting rods, one part drawn in the views it shows in (`engine.conrod`): rod 1 in the
// end section and the side section both, with the shank's section A-A beside the plan; rods 2 to
// 4 in the side section only, the small end of each placed by the end-view image it shares —
// rod 1's for cylinder 4, and a ghosted rod a half turn on for cylinders 2 and 3
point secA in views.top
views.top_origin distance(back + 80mm, along: x) secA
views.top_origin distance(0, along: y) secA
rod1: ConRod(views.right, views.front, views.top, end.crank.t1.pin, end.bore, side.cyl[0].pin, side.cyl[0].small, secA, draw_end: 1, draw_side: 1, draw_sec: 1)
rod2: ConRod(views.right, views.front, views.top, end.crank.t2.pin, end.bore, side.cyl[1].pin, side.cyl[1].small, secA, draw_end: 0, draw_side: 1, draw_sec: 0)
rod3: ConRod(views.right, views.front, views.top, end.crank.t2.pin, end.bore, side.cyl[2].pin, side.cyl[2].small, secA, draw_end: 0, draw_side: 1, draw_sec: 0)
rod4: ConRod(views.right, views.front, views.top, end.crank.t1.pin, end.bore, side.cyl[3].pin, side.cyl[3].small, secA, draw_end: 0, draw_side: 1, draw_sec: 0)
ghost: Rod(end.crank.t2.pin, end.bore) in views.right class phantom
piston1: Piston(rod1.sm[0], pin: 1) in views.right

// heights, end view to side view
end.block.d_l project side.block.bfl
end.block.pr_l project side.block.rfl
end.block.sp_l.p project side.sump.sd
end.head.tl project side.block.htl
end.head.cam_i project side.block.cam
ghost.small project side.cyl[1].small
ghost.small project side.cyl[2].small
rod1.sm[0] project side.cyl[3].small
end.crank.t1.pin project side.cyl[0].pin
end.crank.t2.pin project side.cyl[1].pin
end.crank.t2.pin project side.cyl[2].pin
end.crank.t1.pin project side.cyl[3].pin

// lengths, side view to plan
side.block.bfl project plan.fl
side.block.bfr project plan.br
side.ax[0].p project plan.c[0]
side.ax[1].p project plan.c[1]
side.ax[2].p project plan.c[2]
side.ax[3].p project plan.c[3]

// widths, end view to plan
end.block.d_r project plan.fl
end.block.d_l project plan.br
end.head.cam_i project plan.ci
end.head.cam_e project plan.ce

// how it looks: the dimensions the sheet shows, and nothing else
style .dimension { display: none }
style .shown { display: inline }
style .point { display: none }
style .phantom { dash: 6 3; width: 0.6; color: #888888 }
style .thin { width: 0.6 }
style .belt { width: 1.2; color: #2a7ab0 }
style .axis { dash: 14 3 2 3; width: 0.5; color: #888888 }
style .hidden { dash: 4 3; width: 0.6 }
style .plane { display: none }
