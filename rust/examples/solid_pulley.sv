// Turn a stepped radial section through a full revolution, then cut a V-belt groove.
unit mm
use std
param bore_radius = 6mm
param rim_radius = 28mm
param hub_radius = 12mm
param half_width = 9mm
param hub_extension = 5mm
param groove_depth = 6mm
param groove_half_width = 5mm

// The half-section is an orthogonal contour, symmetric about the radial centerline.
construction centerline line spindle(std.origin, std.up.toward)
construction centerline line mid_axis(std.origin, std.front.toward)
point a hint(x: bore_radius, y: -half_width - hub_extension)
point b hint(x: hub_radius, y: -half_width - hub_extension)
point c hint(x: hub_radius, y: -half_width)
point d hint(x: rim_radius, y: -half_width)
point e hint(x: rim_radius, y: half_width)
point f hint(x: hub_radius, y: half_width)
point g hint(x: hub_radius, y: half_width + hub_extension)
point h hint(x: bore_radius, y: half_width + hub_extension)
profile = horizontal line ab(a, b) -> vertical line bc(b, c) ->
          horizontal line cd(c, d) -> vertical line de(d, e) ->
          horizontal line ef(e, f) -> vertical line fg(f, g) ->
          horizontal line gh(g, h) -> vertical line ha(h, a) -> close
a distance(bore_radius, side: right) spindle
b distance(hub_radius, side: right) spindle
d distance(rim_radius, side: right) spindle
distance(hub_extension) bc
distance(2 * half_width) de
bc equal fg
b vertical g
std.origin distance(half_width, side: left) cd
solid blank(profile, about: spindle)

// The triangular cutter extends outside the rim so the groove opens cleanly.
point lo hint(x: rim_radius + 1mm, y: -groove_half_width)
point root hint(x: rim_radius - groove_depth, y: 0mm)
point hi hint(x: rim_radius + 1mm, y: groove_half_width)
root on mid_axis
root distance(groove_depth, side: left) de
lo distance(1mm, side: right) de
lo symmetry(mid_axis) hi
lo distance(2 * groove_half_width) hi
solid groove(face(lo, root, hi, -> close), about: spindle)
solid body(blank)
groove cut body
