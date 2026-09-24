// A sphere with a cylindrical hole straight through it. Change a number and apply (⌘↵) to see it
// rebuild; ⌘B shows it in the glass box. Keep `hole + offset` below `sphere_r`, or the hole
// breaks out of the side rather than passing through.
unit mm
use std

param sphere_r = 20mm   // the sphere's radius
param hole = 6mm        // the hole's radius
param offset = 5mm      // from the sphere's centre to the hole's axis

// The sphere: a half disc turned about its diameter, which stands on the upright axis.
private point bottom hint(x: 0, y: -sphere_r)
private point top hint(x: 0, y: sphere_r)
private line diameter(bottom, top)
std.origin midpoint diameter
vertical diameter
private arc meridian(center: std.origin, start: bottom, end: top) hint(r: sphere_r)
radius(sphere_r) meridian
construction solid ball(face(meridian, diameter), about: diameter)

// The hole: a circle on the page, level with the sphere's centre and `offset` to its right,
// run through the page from one side of the sphere to the other.
private point axis hint(x: offset, y: 0)
std.origin horizontal axis
std.origin distance(offset, along: right) axis
private circle bore_c(center: axis) hint(r: hole)
radius(hole) bore_c
construction solid bore(face(bore_c), from: -sphere_r - 1mm, to: sphere_r + 1mm)

solid part(ball)
bore cut part
