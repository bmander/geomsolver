// Laws of scale (#243 L5): a section's size is a dimension reading `s`. `p scaled(du:, dv:,
// scale:, s:) P` places a corner at `(du, dv)` scaled `1 + (scale − 1)·s` about the plane's
// origin, and `scaled_radius(r:, scale:, s:) k` a round's radius alike; any other law is the
// same word given another fraction, or any expression in `s` (`sin`, `max`).
// Closed forms, over the 40 mm guide:
//   taper: the 20 mm square shrinking to half while it turns a quarter (Cavalieri, each section
//     the untwisted one turned): a frustum, 40/3 · (400 + 100 + 200) mm³;
//   cone: the disc of radius 10 shrinking to 5: π · 40/3 · (100 + 50 + 25) mm³, its wall a cone;
//   swell: the disc swelling as 1 + 0.3·sin(180°·s): 4000π · (1 + 1.2/π + 0.045) mm³.
// Edit `length`, `half`, `r`, a scale or a twist.
unit mm
use std (scaled, scaled_radius, vertical)

length := 40mm
half := 10mm
r := 10mm

component Taper(g: line, up: axis, half: Length, twist: Angle, scale: Scalar, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a scaled(du: -half, dv: -half, scale: scale, s: s) tw.axes
    b scaled(du: half, dv: -half, scale: scale, s: s) tw.axes
    c scaled(du: half, dv: half, scale: scale, s: s) tw.axes
    d scaled(du: -half, dv: half, scale: scale, s: s) tw.axes
  }
  f := face(a, b, c, d, -> close)
}

component Cone(g: line, up: axis, r: Length, scale: Scalar, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    scaled_radius(r: r, scale: scale, s: s) k
  }
  f := face(k)
}

// the disc's law is a sine: `scaled_radius` given the fraction `sin(180 * s)`
component Swell(g: line, up: axis, r: Length, scale: Scalar, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    scaled_radius(r: r, scale: scale, s: sin(180 * s)) k
  }
  f := face(k)
}

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0, length))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
}
taper := solid(Taper(guide, std.z, half: half, twist: 90deg, scale: 0.5).f over s in (0, 1))
cone := solid(Cone(guide, std.z, r: r, scale: 0.5).f over s in (0, 1))
swell := solid(Swell(guide, std.z, r: r, scale: 1.3).f over s in (0, 1))
