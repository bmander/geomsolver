// A rack tooth drafted both ways from a parting plane through the middle of the blank, as a tool
// forged or cast from a parting line is: `helical.sv`'s tooth, leaned 15° across the blank, the
// blank moved so its middle is the tooth's plane and the tooth run 7 either side of it, drafted 2°
// from that plane on both sides, so it is thickest where it crosses the middle of the blank and
// thins toward either face. Each side folds where the draft parts, a convex crease across it, and
// every normal between its two halves' is the tooth's there: the crease's fan carries the contact
// from the one half's envelope to the other's, as a sharp edge's does (the generating class with a
// drafted prism, docs/generating-sweeps.md, issue #684 P). Each section square to the blank's axis
// is cut by the tooth's section there, the tooth's eight half-planes meeting in it, so the space is
// 79.70 mm³, where `helical.sv`'s tooth, drafted one way from the blank's face, cuts 71.84 and the
// undrafted tooth 83.26. Module 2, 20 teeth, 20° pressure angle in the tooth's own plane.
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

// the blank 6 thick, its middle on the front; the tooth 7 either side of its plane, past the blank
// either side, its sides drafted inward both ways from the plane
blank := solid(face(rim), from: -3mm, to: 3mm)
construction tooth := solid(face(t0, t1, t2, t3, -> close), from: -7mm, to: 7mm, draft: 2deg)

// the blank turning about its centre, and the rack sliding a pitch circumference a turn
turn := frame
turn spins(1) o
rack := frame
rack slides(2 * pi * 20mm) pitch_line
construction space := sweep(tooth over t in (-60deg, 60deg), under: rack, seen_from: turn)
gear := solid(blank)
space cut gear
