// The crank disc alone: the part `vtwin.disc` designs, the pin at the top, with the dimensions a
// printer needs — **and the other two views are asked for, not drawn** (§6.11).

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.disc

point O hint(x: 0, y: 0) in views.front
ground O
views: ThreeViews(O, right: 75, up: 65)
axes: Axes(O)
// the pin is `R` up the axis
point pin hint(x: 0, y: R) in views.front
pin on axes.ax
O distance(R) pin
line arm(O, pin) class axis

disc: Disc(views.front, O, pin, arm, dir: 90deg)

// the other two views, derived from the solid the section is a section of
view(disc.body) in views.right
view(disc.body) in views.top

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
