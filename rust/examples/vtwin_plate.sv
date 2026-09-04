// The frame plate alone: the part `vtwin.frame` designs, with the dimensions a printer needs —
// the ports and the pivots by radius and bearing from the crank axis, since all four ports share
// one radius — **and the other two views are asked for, not drawn** (§6.11).  The assembly draws
// the same component with its dimensions off and the engine on it.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.frame

point O hint(x: 0, y: 0) in views.front
ground O
views: ThreeViews(O, right: 260, up: 220)
axes: Axes(O) in views.front
plate: Frame(views.front, O, axes.ax)

// the other two views, derived from the solid the section is a section of
view(plate.body) in views.right
view(plate.body) in views.top

// how it looks: the part's own dimensions, and nothing else
style .dimension { display: none }
style .detail { display: inline }
style .shown { display: inline }
style .point { display: none }
style .plane { display: none }
style .gone { display: none }
style .axis { dash: 14 3 2 3; width: 0.5; color: #888888 }
style .hidden { dash: 4 3; width: 0.6 }
style .phantom { dash: 6 3; width: 0.6; color: #888888; display: geometry }
style .thin { width: 0.6 }
style .lever { width: 1.4; color: #2a7ab0 }
style .barrel { dash: 4 3; width: 0.6 }
