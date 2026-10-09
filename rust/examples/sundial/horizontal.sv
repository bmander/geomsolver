// A horizontal sundial, its hour lines found from the sun rather than from a table. The gnomon
// stands on the noon line, its edge — the style — pointing at the celestial pole, so it rises at
// the latitude. `sun.EquinoxSun` sets the sun round the style an hour at a time, and each hour
// line is where the plane through the style and that hour's sun meets the dial: a line from the
// centre, which is on the style, square to that plane's normal (`sky.normal[k]`).
//
// Nothing states the closed form, tan(θ) = sin(latitude) · tan(h), θ an hour line's angle from
// noon and h the hour's from noon; every line comes out at it. Six o'clock is the east–west line
// at every latitude, and five in the morning the five o'clock line carried on through the
// centre: an hour and the one twelve later share a plane.
//
// The dial reads from `hours` before noon to `hours` after. How far it need go is the longest
// day's: `first_light` and `last_light` are the shadow at its sunrise and sunset. The sun then is
// `tilt` north of the equator, `90deg - tilt` from the pole, and rises on the horizon — the
// dial's plane — so its shadow there points straight away from it, `90deg + tilt` from the
// style. At 47.6° that is 4:07 in the morning, just beyond the five o'clock line, the dial's
// first. Past the Arctic Circle the longest day has no sunrise, and the document no solution.
//
// Edit `latitude` and the hour lines close up toward noon nearer the equator and open out toward
// the pole, where the dial is a clock face; the first and last light swing with them. On the
// equator the style lies on the plate and every line folds onto noon: `polar.sv` is the dial for
// it. Open the glass box (⌘B) and orbit to see the sun's ring standing square to the style.
unit mm
use std (horizontal, vertical)
use sun
param latitude := 47.6deg
param hours := 7     // hours either side of noon the dial reads, five in the morning to seven
tilt := 23.44deg     // the earth's axial tilt: how far north of the equator the summer sun goes
base := 60mm         // the gnomon's foot, along the noon line from the centre

// the dial's centre, where the style meets it, is on the noon line: the top plane and the side
O := point in std.top, std.side
fix(y == 0) O
in std.top {
  rim := circle(center: O) hint(r: 100)
  radius(100mm) rim
  E := point hint((100, 0))
  construction east := line(O, E)
  O horizontal E
  E coincident rim
}

// the gnomon, in the side plane: the style from the centre to the tip T, rising at the latitude,
// and the ray toward the noon sun, square to it
F := point in std.top, std.side hint((0, 60))
in std.side {
  T := point hint(at: O, toward: F, turn: latitude)
  noon := line(O, F)
  upright := line(F, T)
  style := line(O, T)
  distance(base) noon
  F vertical T
  noon angle(latitude) style
  S := point hint(at: O, toward: F, turn: latitude + 90deg)
  construction noon_sun := line(O, S)
  noon_sun perpendicular style
  noon_sun equal noon
}
sky := sun.EquinoxSun(O, style, east, noon_sun, r: 40mm)

// the hour lines, `12 - hours` o'clock to `12 + hours`: each from the centre in the shadow plane,
// square to its normal, out to the rim on the side away from the sun
repeat 2 * hours + 1 as i {
  tip := point in std.top hint(at: rim, bearing: 90deg + (hours - i) * 15deg)
  tip coincident rim
  hour := line(O, tip)
  hour perpendicular sky.normal[12 - hours + i]
}

// the longest day's first and last light
dawn := point in std.top hint(at: rim, bearing: 210deg)
dusk := point in std.top hint(at: rim, bearing: 330deg)
dawn coincident rim
dusk coincident rim
first_light := line(O, dawn)
last_light := line(O, dusk)
first_light angle(90deg + tilt) style
last_light angle(90deg + tilt) style

// the plate and the gnomon on it
plate := solid(face(rim), depth: 6mm)
gnomon := solid(face(noon, upright, style), from: -1.5mm, to: 1.5mm)
