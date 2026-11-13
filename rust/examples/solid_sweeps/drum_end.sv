// A drum's end screwed through a rod, as a cutter's flat end chases a coarse thread: a drum of
// radius 1.2 mm lying along a radius of the screw, its flat end 4 mm out from the rod's axis,
// screwed about the axis 6 mm a turn from clear below the rod to clear above. Each ring of the drum
// about its own axis meets the screw at one angle, and the drum's side touches where the ring's
// normal stands square to its velocity, its flat end along the line where the plane through the
// screw's axis meets it, and the end's rim joins the two round its fan: the contact reaches across
// the end through the point nearest the axis, a graph and not one curve. Read on the whole drum,
// each piece of contact is carried exactly along the helix, the drum at both ends of the roll
// beside them, every meeting of two of them given by construction, and what the drum screwed back
// holds trimmed away (the contact's road, docs/generating-sweeps.md, #880 Q1b). Unrolled about the
// axis, the screw moves each radius's section of the drum along the helix, and the roll passes the
// whole section at every height of the rod, so each height loses the same: at each radius, the
// section's width across the helix — integrated apart from the build, the rod's 471.24 mm³ less
// 69.37, leaving 401.86 mm³.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 6mm
near_end := 4mm      // the drum: its flat end's distance from the rod's axis, its length and radius
reach := 8mm
drum_r := 1.2mm
lead := 6mm          // how far the screw climbs a turn

in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private r0 := point
  private r1 := point
  private r2 := point
  private r3 := point
  fix((0, 0)) r0
  r1 coords(du: rod_r, dv: 0mm) std.front
  r2 coords(du: rod_r, dv: rod_l) std.front
  r3 coords(du: 0mm, dv: rod_l) std.front
  private rod_axis := line(r3, r0)
  private rod_end := line(r0, r1)
  private rod_wall := line(r1, r2)
  private rod_top := line(r2, r3)
  // the drum, drawn in the same plane through its axis, which runs out along the rod's foot
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords(du: near_end, dv: 0mm) std.front
  c1 coords(du: near_end + reach, dv: 0mm) std.front
  c2 coords(du: near_end + reach, dv: drum_r) std.front
  c3 coords(du: near_end, dv: drum_r) std.front
  private spindle := line(c0, c1)
  private far := line(c1, c2)
  private side := line(c2, c3)
  private near := line(c3, c0)
}
rod := solid(std.solids.Revolution(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis))
construction drum := solid(std.solids.Revolution(face(spindle, far, side, near), about: spindle))

// the screw about the rod's axis, from a quarter turn below the rod to 2.5 mm past its top: the
// drum, 2.4 mm across, clear of the rod at both ends
chase := frame
chase screws(lead) ax
construction thread := solid({ p | some t in (-90deg, 510deg): p in drum at(under: chase) t })
thread cut rod
