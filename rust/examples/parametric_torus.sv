// A torus: a circle of radius `tube` revolved about an upright axis `ring` away from its centre.
// Change either number and apply (⌘↵) to see it rebuild; ⌘B shows it in the glass box.
// Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std (horizontal)

ring := 20mm       // from the axis to the tube's centre
tube := 6mm        // the tube's radius

in std.front {
  construction centerline spine := line(std.origin, hint((0, 1)))
  fix((0, 1)) spine.p2
  private centre := point hint((ring, 0))
  std.origin horizontal centre
  std.origin distance(ring, along: right) centre
  private section := circle(center: centre) hint(r: tube)
  radius(tube) section
}
torus := solid(face(section), about: spine)
