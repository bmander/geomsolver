// A bored mounting flange: one stepped radial section turned about its axis, then a bolt pattern.
unit mm
use std (horizontal)
use hardware
outer_radius := 32mm
bore_radius := 8mm
plate_depth := 6mm
hub_radius := 15mm
hub_height := 10mm
bolt_circle := 24mm
bolt_radius := 3mm
bolts := 6

// The half-section carries the bore, the plate and the hub standing proud of it.  Each run is
// one number, as the part is dimensioned: a radius from the axis or a height from the base,
// measured from the front's origin where the two meet, or the hub's height from where its run
// begins.  The ordinate a run does not name stays where the run began, so every corner follows.
in std.front {
  top := point hint((0mm, plate_depth + hub_height))
  construction centerline spindle := vertical line(std.origin, top)
  profile := step(u: outer_radius) (ab := line) -> step(v: plate_depth) (bc := line) ->
             step(u: hub_radius) (cd := line) -> step(dv: hub_height) (de := line) ->
             step(u: bore_radius) (ef := line) -> step(v: 0mm) (fa := line) -> close
  // where the axis meets the top of the plate
  deck := point hint((0mm, plate_depth))
}
top horizontal ef.p2
deck coincident spindle
deck horizontal bc.p2
flange := solid(profile, about: spindle)

// The bolt circle is drawn on the top of the plate: a plane square to the section over a
// reference running out along that face from the axis, standing at the axis, so it looks down
// on the face.  Its other axis runs the way `std.y` does, through its own origin (a plane's axes
// pass through its origin).  The pattern is measured in it, from its origin along its own `x`.
in std.front {
  construction centerline reference := line(deck, bc.p2)
}
square := axis hint(dir: (0, 1, 0))
square parallel std.y
plate_top := plane(u: reference, v: square)
deck coincident plate_top.origin
in plate_top {
  construction centerline across := horizontal line(plate_top.origin, hint((10mm, 0mm)))
  length(10mm) across
}
pattern := hardware.BoltPattern(flange, plate_top.origin, across, n: bolts, pitch_r: bolt_circle,
                                hole_r: bolt_radius, phase: 0deg) in plate_top
