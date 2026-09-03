// The flywheel alone: the part `vtwin.flywheel` designs, in three views, with the dimensions
// a printer needs.  It is drawn on no other sheet: in the assembly it stands behind the plate,
// where the side view draws its outline.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.flywheel

point O hint(x: 0, y: 0) in views.front
ground O
views: ThreeViews(O, right: 110, up: 100)
axes: Axes(O)
fw: Flywheel(views.front, views.right, views.top, O, axes.ax,
             o_s: views.right_origin, o_t: views.top_origin, draw_side: 1, draw_top: 1)

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
