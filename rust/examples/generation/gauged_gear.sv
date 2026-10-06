// A rack-cut spur gear whose pressure angle nobody states: a gauge touches the cut flank in space,
// and the solve answers the angle that puts the flank there (issue #70).
//
// The gear is `rack_cut_gear.sv`'s, drawn in `std.front`: a rack whose tooth is a trapezoid,
// extruded through the blank, rolling on the pitch circle; each `indexed` copy of what it sweeps
// is cut from the blank.  Here the rack's flank is left free to lean: its tip corner `t1` is held,
// and its root corner `t2` slides up and down the line `rp + 2m` from the centre.
//
// `side` is the rack's flank as a surface — the prism's side `rack_flank` sweeps — and `flank` is
// what that side generates under the rolling motion.  The motion turns about the front's normal
// and slides in the front plane, so every section of the side moves alike, and the surface it
// cuts is one curve of the front plane (the tooth's involute, drawn on the gear's face) extruded
// through the gear.  The drawing holds it, so a point drawn anywhere can be put on it.
//
// The gauge is such a point.  It is drawn in a plane square to the front, standing on the line
// 1.9 mm above the gear's centre, measured 20.58 mm across and 3 mm deep into the face: a point
// off the front plane, which `gauge coincident flank` puts on the flank in space.  That is the one
// equation that fixes the free corner, so the rack's pressure angle comes out of the solve (20
// degrees, to the gauge's two places).  Drag the gauge across and the rack leans to follow.
//
// `solventc --step` and `--stl` build the gear exactly; the tooth space's boundary is that same
// envelope extruded (`brep::sweep::extruded`), not a traced sheet.

unit mm
use std

N := 20                     // teeth
m := 2mm                    // module
width := 6mm                // face width
bore_r := 6mm
rp := m * N / 2             // pitch radius
hw := pi * m / 4            // half the rack tooth's thickness at the pitch line

in std.front {
  o := point
  rim := circle(center: o) hint(r: 22)
  construction bore_c := circle(center: o) hint(r: 6)
  // the rack's pitch line, tangent to the pitch circle at the pitch point, and the gear's centre
  // line, which the rack's tooth is symmetrical about
  s0 := point hint((20, 0))
  s1 := point hint((20, 10))
  construction slide := vertical line(s0, s1)
  e := point hint((30, 0))
  construction centre_line := horizontal line(o, e)
  // the rack's tooth: its tip at the working depth, its back past the blank's rim
  t0 := point hint((18, -0.84))
  t1 := point hint((18, 0.84))
  t2 := point hint((24, 3.03))
  t3 := point hint((24, -3.03))
  rack_flank := line(t1, t2)
}

fix((0, 0)) o
radius(rp + m) rim
radius(bore_r) bore_c
o distance(rp, along: x) s0
o distance(0, along: y) s0
s0 distance(10) s1
o distance(rp + 3 * m) e
blank := solid(face(rim), depth: width)
construction bore := solid(face(bore_c), from: -width - 2mm, to: 2mm)

// the tip held at the working depth, a pressure angle of 20 degrees's thickness; the root corner
// free along its line, the other flank its mirror
o distance(rp - m, along: x) t1
o distance(0.842856, along: y) t1
o distance(rp + 2 * m, along: x) t2
t1 symmetry(centre_line) t0
t2 symmetry(centre_line) t3
tooth := face(t0, t1, rack_flank, t3, -> close)
construction rack_tooth := solid(tooth, from: -width - 2mm, to: 2mm)

turn := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * rp)
cutting := motion(rack, relative_to: turn)
construction space := solid(rack_tooth, under: cutting, from: -60deg, to: 60deg)

// the flank in space: the prism's side, and what it generates
side := surface(rack_tooth, edge: rack_flank)
flank := envelope(side, under: cutting, from: -30deg, to: 30deg)

// the gauge, in a plane square to the front through the line 1.9 above the centre (its u the
// front's x, its v the front's normal, into the face)
gauge_view := plane
fix(origin == (0, 0, 1.9)) gauge_view
fix(dir == (1, 0, 0)) gauge_view.u
fix(dir == (0, 1, 0)) gauge_view.v
gauge := point in gauge_view hint((20.6, 3))
gauge distance(20.58mm, along: u) gauge_view
gauge distance(3mm, along: v) gauge_view
gauge coincident flank

gear := solid(blank)
bore cut gear
repeat N as i {
  construction indexed := solid(space, under: turn, at: i * 360deg / N)
  indexed cut gear
}
