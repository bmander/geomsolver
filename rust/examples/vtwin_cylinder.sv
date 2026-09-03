// The cylinder alone: the part `vtwin.cylinder` designs, upright, in three views, with the
// dimensions a printer needs.
//
// The assembly (`vtwin.sv`) draws this same component twice, each rocked to the crank and each
// showing only its section; this sheet draws it once, at rest, in all three views, and turns on
// the `detail` dimensions the assembly's sheet leaves hidden.  One definition, two drawings:
// edit the part and both follow, and every dimension here is judged as a claim about the same
// statements the engine runs on.  Bank A's cylinder is drawn; bank B's is the same part with
// its face wall `fwB` thick, which is one number below.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.cylinder

point O hint(x: 0, y: 0) in views.front
ground O
views: ThreeViews(O, right: 80, up: 95)
axes: Axes(O)
param fw = fwA          // the face wall: bank A's

cyl: Cylinder(views.front, views.right, views.top, O, axes.ax, axes.ac, dir: 90deg, fw: fw,
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
