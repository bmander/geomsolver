// D0 (#339): the clothoid, probed in today's spelling. The target is `../clothoid.sv`.
//
// The body without its rate rows is `Frame`: the point `p`, its tangent `tl` and its left normal
// `nl`, each 10 long and square. It leaves three freedoms at every place — where `p` is and which
// way `tl` points — which its three rate rows supply away from the home (`p` runs along `tl` at
// unit speed; `tl` turns at κ = s/A²).
//   - At the home (s = 0) the drawing holds them: `p` at the origin, `tl` along the x axis.
//     DOF 0; delete the two home lines and it is DOF 3.
//   - At s1 the closed form is drawn as today's computed curve (the Fresnel integrals as their
//     series, 13 terms each, exact to 1e-16 over the interval) and the frame is put on it. The
//     rate rows are claimed there through today's contacts: the curve runs along `tl`, and its
//     osculating circle has radius A²/s1 with its centre on `nl`. All theorems.
// Closed form: x(s) = ∫₀ˢ cos(σ²/2A²) dσ, y(s) = ∫₀ˢ sin(σ²/2A²) dσ, the tangent at s²/2A².
unit mm
use std

A := 50mm
s1 := 80mm

component Frame(s: Length) {
  p := point
  q := point
  k := point
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
}

component Closed(o: point, A: Length, s: Length) {
  p := point(x: o.x + A * (1 * (s / A)^1 - 0.025000000000000001 * (s / A)^5 + 0.00028935185185185184 * (s / A)^9 - 1.6693376068376068e-06 * (s / A)^13 + 5.6988941409897288e-09 * (s / A)^17 - 1.2814973597463678e-11 * (s / A)^21 + 2.0387457995964941e-14 * (s / A)^25 - 2.4142025857290806e-17 * (s / A)^29 + 2.2099708013302823e-20 * (s / A)^33 - 1.6103390841701844e-23 * (s / A)^37 + 9.5607423161580268e-27 * (s / A)^41 - 4.7136896941135051e-30 * (s / A)^45 + 1.9605519468639802e-33 * (s / A)^49), y: o.y + A * (0.16666666666666666 * (s / A)^3 - 0.002976190476190476 * (s / A)^7 + 2.3674242424242424e-05 * (s / A)^11 - 1.0333994708994709e-07 * (s / A)^15 + 2.8327836373340759e-10 * (s / A)^19 - 5.3184673032952017e-13 * (s / A)^23 + 7.2604907393037542e-16 * (s / A)^27 - 7.5281586006605745e-19 * (s / A)^31 + 6.1284904574705302e-22 * (s / A)^35 - 4.0204147175638884e-25 * (s / A)^39 + 2.1704896731034278e-28 * (s / A)^43 - 9.8111025085618747e-32 * (s / A)^47 + 3.7673351135817659e-35 * (s / A)^51))
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  x := point
  fix((10mm, 0mm)) x
  datum := line(o, x)

  // the home: the drawn instance, and what holds there alone
  home := Frame(s: 0mm)
  home.p coincident o
  datum angle(0deg) home.tl

  // the place s1, on the closed form
  place := Frame(s: s1)
  place.p coincident(t == s1) cl
  datum angle(s1^2 / (2 * A^2) * 1rad) place.tl     // where the closed form's tangent points
  m := point
  tc := line(place.p, m)
  length(10mm) tc
  cl tangent(at: place.p) tc
  osc := circle hint(r: 30)
  cl curvature(t == s1) osc
  claim tc parallel place.tl                         // rate rows 1–2: p runs along tl
  claim radius(A^2 / s1) osc                      // rate row 3: tl turns at s1/A²...
  claim osc.center coincident place.nl               // ...towards its left normal
}
cl := Closed(o, A: A).p over s in (0mm, 100mm)
