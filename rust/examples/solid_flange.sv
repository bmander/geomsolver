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
  top := point hint(x: 0mm, y: plate_depth + hub_height)
  construction centerline spindle := vertical line(std.origin, top)
  a := point hint(x: bore_radius, y: 0mm)
  b := point hint(x: outer_radius, y: 0mm)
  c := point hint(x: outer_radius, y: plate_depth)
  d := point hint(x: hub_radius, y: plate_depth)
  e := point hint(x: hub_radius, y: plate_depth + hub_height)
  f := point hint(x: bore_radius, y: plate_depth + hub_height)
  profile := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
            horizontal (cd := line(c, d)) -> vertical (de := line(d, e)) ->
            horizontal (ef := line(e, f)) -> vertical (fa := line(f, a)) -> close
  // where the axis meets the top of the plate
  deck := point hint(x: 0mm, y: plate_depth)
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

// The bolt circle is drawn on the top of the plate: a plane folded square to the section about
// a reference running out along that face from the axis, so it looks down on the face.
construction centerline reference := line(deck, c)
plate_top := plane(origin: deck, toward: c, from: std.front, fold: along reference)
pattern := hardware.BoltPattern(flange, deck, reference, n: bolts, pitch_r: bolt_circle,
                                hole_r: bolt_radius, phase: 0deg) in plate_top
