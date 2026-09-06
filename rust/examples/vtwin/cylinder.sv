// The cylinder alone: the part `components.cylinder` designs, upright, with the dimensions a printer
// needs — **and the other two views are asked for, not drawn** (§6.11).
//
// The component is one section and the solid it is a section of.  This sheet says where to stand
// and looks: two `view` statements, and every depth in them is the solid's own, so the side view
// cannot disagree with the section about how thick the face wall is.  What used to be here was
// sixty lines of `Slab` and `Box` rectangles re-tied by `project`, with the depths kept in step
// by hand — which is the whole of issue #48, item 9.
//
// The assembly (`assembly.sv`) draws this same component twice, each rocked to the crank and each
// showing only its section; this sheet draws it once, at rest, and turns on the `detail`
// dimensions the assembly's sheet leaves hidden.  One definition, two drawings: edit the part
// and both follow, and every dimension here is judged as a claim about the same statements the
// engine runs on.  Bank A's cylinder is drawn; bank B's is the same part with its face wall
// `fwB` thick, which is one number below.

unit mm
use std
use components.dims
use components.parts
use components.cylinder

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O) in front
param fw = fwA          // the face wall: bank A's

cyl: Cylinder(front, O, axes.ax, axes.ac, dir: 90deg, fw: fw)

// Projections and dimensions: cylinder.svd
