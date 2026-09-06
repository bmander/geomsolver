// The piston and its rod alone: the part `components.piston` designs, upright, with the dimensions a
// printer needs — **and the other two views are asked for, not drawn** (§6.11).
//
// The component is one section and the solid it is a section of, so this sheet says where to
// stand and looks.  The piston is a turn about the rod's line, which is why the view from the
// crown is a disc without anything here or there saying so.

unit mm
use std
use components.dims
use components.parts
use components.piston

point O hint(x: 0, y: 0) in front
ground O
point datum hint(x: 1, y: 0)
ground datum
plane front(origin: O, toward: datum)
axes: Axes(O) in front
// the crown is the origin; the pin is `L` down the axis
point pin hint(x: 0, y: -L) in front
pin on axes.ax
O distance(L) pin

plane piston_axes(origin: O, toward: pin)
pis: Piston(piston_axes) in front

// Projections and dimensions: piston.svd
