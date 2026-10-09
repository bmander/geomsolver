// The sun at the equinox, an hour at a time, about a sundial's style.
//
// The style points at the celestial pole, along the earth's axis. On the equinox the sun runs
// round the celestial equator — square to that axis, so to the style — one turn a day: seen from
// the style it is a clock hand. The plane square to the style through `at` is that equator,
// stood on two rays from `at`: `east`, due east, and `noon`, toward the noon sun (square to the
// style in the meridian). The sun's 24 hourly places are a ring in it, each a turn of the first,
// and the first is gauged to midnight, straight down `noon`'s line: `s[k]` is the sun at `k`
// o'clock. Nothing works out an angle; the ring turns the copies.
//
// The style's shadow at `k` o'clock lies in the plane through the style and `s[k]` — on every
// day of the year, not only the equinox: the seasons move the sun along the style, which is in
// the plane. That plane is never built. Its normal is square to the style and to `ray[k]`, so it
// lies in the equator a quarter turn from the sun — the sun six hours on — and each copy carries
// it as `normal[k]`: a line from the style is in the shadow plane exactly when it is square to
// `normal[k]`. One row an hour, where a plane and its two axes would be two dozen unknowns.
component EquinoxSun(at: point, style: line, east: line, noon: line, r: Length) {
  equator := plane(u: east, v: noon)
  in equator {
    ring 24 about equator.origin {
      s := point hint((0mm, -r))
      equator.origin distance(r) s
      construction ray := line(equator.origin, s)
      n := point hint((r, 0mm))
      equator.origin distance(r) n
      construction normal := line(equator.origin, n)
      normal perpendicular ray
    }
  }
  s[0] level(u) equator
}
