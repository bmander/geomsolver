// Turn a stepped radial section through a full revolution, then cut a V-belt groove.
unit mm
use std
bore_radius := 6mm
rim_radius := 28mm
hub_radius := 12mm
half_width := 9mm
hub_extension := 5mm
groove_depth := 6mm
groove_half_width := 5mm

// The half-section is an orthogonal contour, symmetric about the radial centerline.
construction centerline spindle := line(std.origin, std.up.toward)
construction centerline mid_axis := line(std.origin, std.front.toward)
a := point hint(x: bore_radius, y: -half_width - hub_extension)
b := point hint(x: hub_radius, y: -half_width - hub_extension)
c := point hint(x: hub_radius, y: -half_width)
d := point hint(x: rim_radius, y: -half_width)
e := point hint(x: rim_radius, y: half_width)
f := point hint(x: hub_radius, y: half_width)
g := point hint(x: hub_radius, y: half_width + hub_extension)
h := point hint(x: bore_radius, y: half_width + hub_extension)
profile := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
          horizontal (cd := line(c, d)) -> vertical (de := line(d, e)) ->
          horizontal (ef := line(e, f)) -> vertical (fg := line(f, g)) ->
          horizontal (gh := line(g, h)) -> vertical (ha := line(h, a)) -> close
a distance(bore_radius, side: right) spindle
b distance(hub_radius, side: right) spindle
d distance(rim_radius, side: right) spindle
distance(hub_extension) bc
distance(2 * half_width) de
bc equal fg
b vertical g
std.origin distance(half_width, side: left) cd
body := solid(profile, about: spindle)

// The triangular cutter extends outside the rim so the groove opens cleanly.
lo := point hint(x: rim_radius + 1mm, y: -groove_half_width)
root := point hint(x: rim_radius - groove_depth, y: 0mm)
hi := point hint(x: rim_radius + 1mm, y: groove_half_width)
root on mid_axis
root distance(groove_depth, side: left) de
lo distance(1mm, side: right) de
lo symmetry(mid_axis) hi
lo distance(2 * groove_half_width) hi
groove := solid(face(lo, root, hi, -> close), about: spindle)
groove cut body
