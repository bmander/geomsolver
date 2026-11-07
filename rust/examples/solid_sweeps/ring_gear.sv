// A ring gear cut by the gear whose teeth are its spaces: an internal gear's tooth space is
// exactly the tooth of an external gear of its tooth count and module, so the ring is cut by that
// gear. The gear is `rack.sv`'s, a rack tooth rolled on a turning blank (the generating class),
// its space cut at each of 20 teeth a pitch apart; the gear is a body built of sweeps, and it cuts
// the ring as any solid does: built by its own road, then subtracted by the kernel's Boolean
// (issue #818). The ring's bore, radius 19, lies between the gear's root (18) and its rim (22), so
// the ring keeps its own annulus outside the gear and, inside the gear's rim, what each of its
// spaces covers there: 20 teeth, each the 6 mm thickness times the area the rack tooth's roll
// removes from the annulus between 19 and 22 (`rack_cut::rack_space`).  Module 2, 20° pressure
// angle; the gear's own bore lets its spaces be cut as one sector turned.
unit mm
use std
use std.solids (cut)
use std.motion (spins, slides)

in std.front {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 22)
  radius(22mm) rim                          // the gear's addendum circle
  hub := circle(center: o) hint(r: 6)
  radius(6mm) hub                           // the gear's bore

  // the pitch line, tangent to the pitch circle (radius 20) where the tooth cuts
  s0 := point hint((20, 0))
  s1 := point hint((20, 10))
  fix((20, 0)) s0
  fix((20, 10)) s1
  pitch_line := line(s0, s1)

  // the rack's tooth: its tip a module below the pitch line, π m / 2 thick at the pitch line,
  // its flanks 20° from the radius
  t0 := point hint((18, -0.84))
  t1 := point hint((18, 0.84))
  t2 := point hint((24, 3.03))
  t3 := point hint((24, -3.03))
  fix((18, -0.842856)) t0
  fix((18, 0.842856)) t1
  fix((24, 3.026666)) t2
  fix((24, -3.026666)) t3

  // the ring: its outside, and its bore, the tips of its teeth
  outer := circle(center: o) hint(r: 30)
  radius(30mm) outer
  bore := circle(center: o) hint(r: 19)
  radius(19mm) bore
}
construction blank := solid(std.solids.Prism(face(rim, holes: hub), from: -6mm, to: 0))
construction tooth := solid(std.solids.Prism(face(t0, t1, t2, t3, -> close), from: -8mm, to: 2mm))

// the blank turning about its centre, and the rack sliding a pitch circumference a turn
turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 20mm) pitch_line
construction space := solid({ p | some t in (-60deg, 60deg): p in tooth at(under: rack, seen_from: turn) t })

// the external gear: its space at every tooth
construction gear := solid(blank)
repeat 20 as i {
  construction indexed := space at(under: turn) i * 18deg
  indexed cut gear
}

// the ring gear: the ring less the gear
ring := solid(std.solids.Prism(face(outer, holes: bore), from: -6mm, to: 0))
gear cut ring
