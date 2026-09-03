// The cylinder alone: the part `vtwin.cylinder` designs, upright, with the dimensions a printer
// needs — **and the other two views are asked for, not drawn** (§6.11).
//
// The component is one section and the solid it is a section of.  This sheet says where to stand
// and looks: two `view` statements, and every depth in them is the solid's own, so the side view
// cannot disagree with the section about how thick the face wall is.  What used to be here was
// sixty lines of `Slab` and `Box` rectangles re-tied by `project`, with the depths kept in step
// by hand — which is the whole of issue #48, item 9.
//
// The assembly (`vtwin.sv`) draws this same component twice, each rocked to the crank and each
// showing only its section; this sheet draws it once, at rest, and turns on the `detail`
// dimensions the assembly's sheet leaves hidden.  One definition, two drawings: edit the part
// and both follow, and every dimension here is judged as a claim about the same statements the
// engine runs on.  Bank A's cylinder is drawn; bank B's is the same part with its face wall
// `fwB` thick, which is one number below.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.cylinder

point O hint(x: 0, y: 0) in views.front
ground O
views: ThreeViews(O, right: 80, up: 95)
axes: Axes(O) in views.front
param fw = fwA          // the face wall: bank A's

cyl: Cylinder(views.front, O, axes.ax, axes.ac, dir: 90deg, fw: fw)

// the other two views, derived from the solid the section is a section of
view(cyl.body) in views.right
view(cyl.body) in views.top

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
