// A torus: a circle of radius `tube` revolved about an upright axis `ring` away from its centre.
// Change either number and apply (⌘↵) to see it rebuild; ⌘B shows it in the glass box.
// Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std

ring := 20mm       // from the axis to the tube's centre
tube := 6mm        // the tube's radius

construction centerline spine := line(std.origin, std.up.toward)
private centre := point hint(x: ring, y: 0)
std.origin horizontal centre
std.origin distance(ring, along: right) centre
private section := circle(center: centre) hint(r: tube)
radius(tube) section
torus := solid(face(section), about: spine)
