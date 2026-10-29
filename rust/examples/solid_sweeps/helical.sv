// A drafted rack tooth cutting one tooth space of a helical gear: `rack.sv`'s tooth drawn in a
// plane leaned 15° about the radius from the blank's face, so it crosses the blank aslant as a
// helical rack's teeth do, and its sides drafted 2° from that plane, so the tooth thins as it goes
// deeper. The motion is the rack's, rolling the pitch line on the pitch circle, but it no longer
// keeps the tooth's plane, and the drafted sides lean out of it: a plane each, the tool's contact a
// line across each at every instant, read in closed form (the generating class with a drafted
// prism, docs/generating-sweeps.md, issue #684 G). Each section square to the blank's axis is cut
// by the tooth's section there, a quadrilateral, so the space is 71.84 mm³ where the undrafted
// tooth's is 83.26. Module 2, 20 teeth, 20° pressure angle in the tooth's own plane.
unit mm
use std
use std.motion (spins, slides)

// the tooth's plane: x across it as the front's, its v leaned 15° from up toward the viewer
lean := axis
fix(dir == (0, -0.258819045102521, 0.965925826289068)) lean
inclined := plane(u: std.x, v: lean)
fix(origin == (0, 0, 0)) inclined

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
}

// the rack's tooth as `rack.sv` draws it, in the leaned plane
in inclined {
  t0 := point hint((18, -0.84))
  t1 := point hint((18, 0.84))
  t2 := point hint((24, 3.03))
  t3 := point hint((24, -3.03))
  fix((18, -0.842856)) t0
  fix((18, 0.842856)) t1
  fix((24, 3.026666)) t2
  fix((24, -3.026666)) t3
}

// the blank 6 thick, 2 behind the front; the tooth from its plane to 12 behind it, past the blank
// either side, its sides drafted inward as it goes
blank := solid(face(rim), from: -8mm, to: -2mm)
construction tooth := solid(face(t0, t1, t2, t3, -> close), from: -12mm, to: 0mm, draft: 2deg)

// the blank turning about its centre, and the rack sliding a pitch circumference a turn
turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 20mm) pitch_line
construction space := sweep(tooth over t in (-60deg, 60deg), under: rack, seen_from: turn)
gear := solid(blank)
space cut gear
