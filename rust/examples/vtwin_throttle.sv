// The throttle barrel alone: the part `vtwin.throttle` designs, full open, in three views,
// with the dimensions a printer needs — the assembly draws the same component in the inlet
// boss, turned to `throttle`.

unit mm
use std
use vtwin.dims
use vtwin.parts
use vtwin.throttle

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O) in front
thr: Throttle(front, O, axes.ax, phi: 0deg)

// Projections and dimensions: vtwin_throttle.svd
