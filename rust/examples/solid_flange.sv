// A bored mounting flange: an annular profile, an added hub, and repeated through cuts.
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

rim := radius(outer_radius) circle(center: std.origin)
bore := radius(bore_radius) circle(center: std.origin)
hub_rim := radius(hub_radius) circle(center: std.origin)

// Holes belong to the profile here, so both extrusions already have the shaft bore.
plate_section := face(rim, holes: bore)
flange := solid(plate_section, depth: plate_depth)
hub := solid(face(hub_rim, holes: bore), from: 0mm, to: hub_height)
hub on flange

construction centerline reference := line(std.origin, std.front.toward)
pattern := hardware.BoltPattern(flange, std.origin, reference,
                       n: bolts, pitch_r: bolt_circle, hole_r: bolt_radius, phase: 0deg)
