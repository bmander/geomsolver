// The crank disc alone: the part `components.disc` designs, the pin at the top, with the dimensions a
// printer needs — **and the other two views are asked for, not drawn** (§6.11).

unit mm
use std
use components.dims
use components.parts
use components.disc

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O)
// the pin is `R` up the axis
point pin hint(x: 0, y: R) in front
pin on axes.ax
O distance(R) pin
line arm(O, pin)

disc: Disc(front, O, pin, arm, dir: 90deg)

// Projections and dimensions: disc.svd
