// Turn a stepped radial section through a full revolution, then cut a V-belt groove.
unit mm
use std (vertical)
bore_radius := 6mm
rim_radius := 28mm
hub_radius := 12mm
half_width := 9mm
hub_extension := 5mm
groove_depth := 6mm
groove_half_width := 5mm

// The half-section is an orthogonal contour, symmetric about the radial centerline.
in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  construction centerline mid_axis := line(std.origin, hint((1, 0)))
  fix((1, 0)) mid_axis.p2
  a := point hint((bore_radius, -half_width - hub_extension))
  b := point hint((hub_radius, -half_width - hub_extension))
  c := point hint((hub_radius, -half_width))
  d := point hint((rim_radius, -half_width))
  e := point hint((rim_radius, half_width))
  f := point hint((hub_radius, half_width))
  g := point hint((hub_radius, half_width + hub_extension))
  h := point hint((bore_radius, half_width + hub_extension))
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
  lo := point hint((rim_radius + 1mm, -groove_half_width))
  root := point hint((rim_radius - groove_depth, 0mm))
  hi := point hint((rim_radius + 1mm, groove_half_width))
  root coincident mid_axis
  root distance(groove_depth, side: left) de
  lo distance(1mm, side: right) de
  lo symmetry(mid_axis) hi
  lo distance(2 * groove_half_width) hi
}
groove := solid(face(lo, root, hi, -> close), about: spindle)
groove cut body
