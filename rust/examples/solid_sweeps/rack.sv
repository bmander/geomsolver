// A rack cutting one tooth space of a spur gear: a trapezoidal rack tooth, longer than the blank
// is thick, slid along a line tangent to the blank's pitch circle while the blank turns under it,
// at the rate that rolls the line on the circle. Seen from the blank, the tooth rolls round it, and
// everything it passes through is the space: its flanks are the involutes the tooth's sides
// generate (the generating class, docs/generating-sweeps.md, issue #61). Module 2, 20 teeth, 20°
// pressure angle, the tooth's tip kept above where the flank would undercut. A real gear is this
// space at every tooth: `generation/rack_cut_gear.sv`.
unit mm
use std
use std.motion (spins, slides)

in std.front {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 22)
  radius(22mm) rim                          // the addendum circle

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
}
blank := solid(face(rim), depth: 6mm)
construction tooth := solid(face(t0, t1, t2, t3, -> close), from: -8mm, to: 2mm)

// the blank turning about its centre, and the rack sliding a pitch circumference a turn
turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 20mm) pitch_line
construction space := sweep(tooth over t in (-60deg, 60deg), under: rack, seen_from: turn)
gear := solid(blank)
space cut gear
