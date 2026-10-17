// D0 (#339): a rotation-minimising frame along a helix, in the proposed spelling. Does not
// parse: it stops at `rate`. #243 L9's item; D3's word.
//
// `Minimal` is the station word D3 adds to `std`: the plane `f` square to the path where it
// stands, its axes square, and one rate row — `f.u` turns only about the tangent (U′ ∥ T: with
// `f.u` and `f.v` unit and square to T, U′ · V = 0 leaves U′ along T). A parallel in space is
// two rows and one of them `f.u`'s unit length already implies, so the row is said as the one
// that is not. Its `initially` is the word's own initial data: at the home, `f.u` square to `up`.
// The word is handed the running formal (`s: phi`), as `std`'s station words are: scope is
// closed, so a rate names a formal of the body it is written in, and `rate(x over s)` with `s`
// bound to an expression in `phi` is the rate along `phi` over ds/dphi.
//
// The helix and its tangent are written out here (`std.OnHelix`, drawn by itself at a phase,
// stands square to the mirror helix: README.md). A tube whose section's centroid stays on the
// helix has volume A·L, whatever the frame does (D3's second gate).
//
// Closed form: the frame turns against Frenet's at −τ an arc length, τ = b/(a² + b²): per turn
// −2πb/√(a² + b²), −28.558° here.
unit mm
use std (on_u)

component Helix(a: Length, b: Length, phi: Angle) {
  m := point
  fix((a * cos(phi), a * sin(phi), b * phi / 1rad)) m
  tg := axis
  fix(dir == (-sin(phi) * cos(atan(b / a)), cos(phi) * cos(atan(b / a)), sin(atan(b / a)))) tg
  tg coincident m
}

component Minimal(m: point, tg: axis, up: axis, s: Angle) {
  f := plane
  f.origin coincident m
  f.u perpendicular tg
  f.v perpendicular tg
  f.u perpendicular f.v
  rate(f.u over s) perpendicular f.v          // rotation-minimising: U′ has no part along V
  initially {
    f.u perpendicular up
  }
}

component Tube(a: Length, b: Length, r: Length, up: axis, phi: Angle) {
  h := Helix(a: a, b: b, phi: phi)
  st := Minimal(h.m, h.tg, up, s: phi)         // a word sees no formal it is not handed
  in st.f {
    c := circle(center: st.f.origin)
    radius(r) c
    k := point
    k on_u(d: r) st.f                          // a mark on the rim: it shows the frame's turn
  }
  sec := face(c)
}

param phi: Angle over (0deg, 720deg)
tube := sweep(Tube(a: 10mm, b: 5mm / (2 * pi), r: 1.5mm, up: std.z).sec over phi)
