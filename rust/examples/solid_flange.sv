// A bored mounting flange: one stepped radial section turned about its axis, then a bolt pattern.
unit mm
use std
use hardware
outer_radius := 32mm
bore_radius := 8mm
plate_depth := 6mm
hub_radius := 15mm
hub_height := 10mm
bolt_circle := 24mm
bolt_radius := 3mm
bolts := 6

// The half-section carries the bore, the plate and the hub standing proud of it.
in std.front {
  top := point hint((0mm, plate_depth + hub_height))
  construction centerline spindle := vertical line(std.origin, top)
  a := point hint((bore_radius, 0mm))
  b := point hint((outer_radius, 0mm))
  c := point hint((outer_radius, plate_depth))
  d := point hint((hub_radius, plate_depth))
  e := point hint((hub_radius, plate_depth + hub_height))
  f := point hint((bore_radius, plate_depth + hub_height))
  profile := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
            horizontal (cd := line(c, d)) -> vertical (de := line(d, e)) ->
            horizontal (ef := line(e, f)) -> vertical (fa := line(f, a)) -> close
  // where the axis meets the top of the plate
  deck := point hint((0mm, plate_depth))
}
a horizontal std.origin
top horizontal f
deck coincident spindle
deck horizontal c
a distance(bore_radius, side: right) spindle
b distance(outer_radius, side: right) spindle
d distance(hub_radius, side: right) spindle
distance(plate_depth) bc
distance(hub_height) de
flange := solid(profile, about: spindle)

// The bolt circle is drawn on the top of the plate: a plane square to the section over a
// reference running out along that face from the axis, standing at the axis, so it looks down
// on the face.  Its other axis runs the way `std.y` does, through its own origin (a plane's axes
// pass through its origin).  The pattern is measured in it, from its origin along its own `x`.
in std.front {
  construction centerline reference := line(deck, c)
}
square := axis hint(dir: (0, 1, 0))
square parallel std.y
plate_top := plane(u: reference, v: square)
deck coincident plate_top.origin
in plate_top {
  construction centerline across := horizontal line(plate_top.origin, hint((10mm, 0mm)))
  distance(10mm) across
}
pattern := hardware.BoltPattern(flange, plate_top.origin, across, n: bolts, pitch_r: bolt_circle,
                                hole_r: bolt_radius, phase: 0deg) in plate_top
