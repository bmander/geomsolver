// The frame plate alone: the part `components.frame` designs, with the dimensions a printer needs —
// the ports and the pivots by radius and bearing from the crank axis, since all four ports share
// one radius — **and the other two views are asked for, not drawn** (§6.11).  The assembly draws
// the same component with its dimensions off and the engine on it.

unit mm
use std
use components.dims
use components.parts
use components.frame

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O) in front
plate: Frame(front, O, axes.ax)

// Projections and dimensions: plate.svd
