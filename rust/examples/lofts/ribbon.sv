// A flat ribbon wound on a helix, twice (#339 D3): once in `std.OnHelix`'s plane, which turns with
// the helix's Frenet frame, so the ribbon's flat stays facing the axis; once in `std.Minimal`'s,
// the rotation-minimising frame of the same station, which turns only as the helix makes it — so
// against the Frenet frame it falls behind by the helix's torsion, 2π·b/√(r² + b²) a turn
// (b = pitch/2π: 28.56° at r 10, pitch 5), and the ribbon twists about its own centre line.
// `Minimal` states that as a rate, `rate(f.u over s) perpendicular f.v`, and its start where the
// family is drawn, `initially { f.u perpendicular up }`: the family integrates it from there.
// Closed form: either way, a section square to its path with its centroid on it sweeps its area
// times the path's length: V = w·h · turns·√((2π·r)² + pitch²), 4·0.5 · 3·√((24π)² + 6²) mm³ as
// drawn.  Edit `r`, `pitch`, `turns`, `w` or `h`.
unit mm
use std (coords)

r := 12mm
pitch := 6mm
turns := 3
w := 4mm
h := 0.5mm

component Flat(n: plane, w: Length, h: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: -w / 2, dv: -h / 2) n
  b coords(du: w / 2, dv: -h / 2) n
  c coords(du: w / 2, dv: h / 2) n
  d coords(du: -w / 2, dv: h / 2) n
  f := face(a, b, c, d, -> close)
}

component Frenet(base: plane, r: Length, pitch: Length, turns: Scalar, w: Length, h: Length,
                 s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  sec := Flat(st.n, w: w, h: h)
}

component Unturned(base: plane, r: Length, pitch: Length, turns: Scalar, w: Length, h: Length,
                   s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  rm := std.Minimal(st.n, up: base.normal, s: s)
  sec := Flat(rm.f, w: w, h: h)
}

param s: Scalar over (0, 1)
facing := sweep(Frenet(std.top, r: r, pitch: pitch, turns: turns, w: w, h: h).sec.f
                over s)
twisting := sweep(Unturned(std.top, r: r, pitch: pitch, turns: turns, w: w, h: h).sec.f
                  over s)
