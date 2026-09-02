// A four-cylinder engine in three views, written as modules (§14.4).
//
// The dimension table is `engine.dims`; the reciprocating parts and the valvetrain are
// components in `engine.parts` and `engine.valvetrain`; and each view is one component in a
// module of its own — `EndSection`, `SideSection`, `PlanView` — drawn here in its plane.  The
// views are tied by projection, the draughtsman's way (§6.7): the end view owns every height and
// width, the side view every length along the crank axis, and the plan is placed almost wholly
// by the other two.  One crank angle in the table turns every piston in every view.

unit mm
use engine.dims
use engine.end_view
use engine.side_view
use engine.top_view

// the three planes: the page is the side view; the end view is folded from it on the right,
// turned so up stays up; the plan is folded up from the x-axis and drawn above
point Of hint(x: 0, y: 0)
point qf hint(x: 40, y: 0)
plane front(origin: Of, toward: qf)
point Or hint(x: 620, y: 0)
point qr hint(x: 620, y: -40)
plane right(origin: Or, toward: qr, from: front, fold: -90deg)
point Ot hint(x: 0, y: 620)
point qt hint(x: 40, y: 620)
plane top(origin: Ot, toward: qt, from: front, fold: 0deg)
ground Of
ground qf
ground Or
ground qr
ground Ot
ground qt

end: EndSection(Or) in right
side: SideSection(Of) in front
plan: PlanView(Ot) in top

// heights, end view to side view
end.block.d_l project side.block.bfl
end.block.pr_l project side.block.rfl
end.block.sp_l.p project side.sump.sd
end.head.tl project side.block.htl
end.head.cam_i project side.block.cam
end.crank.r1.small project side.cyl[0].small
end.crank.r2.small project side.cyl[1].small
end.crank.r2.small project side.cyl[2].small
end.crank.r1.small project side.cyl[3].small
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
style .plane { display: none }
