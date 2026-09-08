// A bored mounting flange: an annular profile, an added hub, and repeated through cuts.
unit mm
use std
use hardware
param outer_radius = 32mm
param bore_radius = 8mm
param plate_depth = 6mm
param hub_radius = 15mm
param hub_height = 10mm
param bolt_circle = 24mm
param bolt_radius = 3mm
param bolts = 6

radius(outer_radius) circle rim(center: std.origin)
radius(bore_radius) circle bore(center: std.origin)
radius(hub_radius) circle hub_rim(center: std.origin)

// Holes belong to the profile here, so both extrusions already have the shaft bore.
face plate_section(rim, holes: bore)
solid plate(plate_section, depth: plate_depth)
solid hub(face(hub_rim, holes: bore), from: 0mm, to: hub_height)
solid body(plate)
hub on body

construction centerline line reference(std.origin, std.front.toward)
pattern: BoltPattern(body, std.origin, reference,
                     n: bolts, pitch_r: bolt_circle, hole_r: bolt_radius, phase: 0deg)
