// The flywheel alone: the part `vtwin.flywheel` designs, in three views, with the dimensions
// a printer needs.  It is drawn on no other sheet: in the assembly it stands behind the plate,
// where the side view draws its outline.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.flywheel

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O)
fw: Flywheel(front, O, axes.ax)

// Projections and dimensions: vtwin_flywheel.svd
