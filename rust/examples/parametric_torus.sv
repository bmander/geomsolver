// A torus: a circle of radius `tube` revolved about an upright axis `ring` away from its centre.
// Change either number and apply (⌘↵) to see it rebuild; ⌘B shows it in the glass box.
// Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std

param ring = 20mm       // from the axis to the tube's centre
param tube = 6mm        // the tube's radius

construction centerline line spine(std.origin, std.up.toward)
private point centre hint(x: ring, y: 0)
centre distance(ring, along: u) std.front
centre distance(0mm, along: v) std.front
private circle section(center: centre) hint(r: tube)
radius(tube) section
solid torus(face(section), about: spine)
