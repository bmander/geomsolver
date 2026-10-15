// A blade on two rails (#243 L8): a profile drawn once, held to two rails at every station.
// `sec := prof mapped(from -> l, to -> r) in st.n` (#357) carries the face `prof` whole into the
// station's plane by the similarity that takes its point `from` to `l` and `to` to `r` — moved,
// turned and scaled alike both ways, never stretched — each corner keeping its place in the
// frame `from`, `to` make (`std.Similar` says the same). The station stands square to the spine
// (`std.OnLine`), and `l` and `r` are where the port and starboard rails cross it, so the
// profile follows the rails: they close from 20 apart to 8, and the port rail rises 3 as the
// starboard falls 3, so the section narrows and turns.
// Closed form: the profile's area times its scale squared, the rails' spread over the profile's,
// along the spine: 60 · 50 / 20² · ∫((20 − 12s)² + (6s)²) ds = 1650 mm³ as drawn (1560 with the
// rails level).
// Edit `length`, `root`, `tip`, `rise` or the profile.
unit mm
use std

length := 50mm
root := 20mm
tip := 8mm
rise := 3mm

component Blade(g: line, up: axis, port: line, starboard: line, prof: face, from: point,
                to: point, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  l := point in st.n
  r := point in st.n
  l coincident port
  r coincident starboard
  sec := prof mapped(from -> l, to -> r) in st.n
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  end := point
  fix((0mm, length)) end
  spine := line(o, end)
}
l0 := point
l1 := point
r0 := point
r1 := point
fix((-root / 2, 0mm, 0mm)) l0
fix((-tip / 2, length, rise)) l1
fix((root / 2, 0mm, 0mm)) r0
fix((tip / 2, length, -rise)) r1
port := line(l0, l1)
starboard := line(r0, r1)

// the profile, drawn once off to the side: a 20 × 3 section, `p0` and `p1` the middles of its
// ends, which the rails carry
in std.front {
  p0 := point
  p1 := point
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((-10mm, 100mm)) p0
  fix((10mm, 100mm)) p1
  fix((-10mm, 98.5mm)) k0
  fix((10mm, 98.5mm)) k1
  fix((10mm, 101.5mm)) k2
  fix((-10mm, 101.5mm)) k3
}
profile := face(k0, k1, k2, k3, -> close)
blade := solid(Blade(spine, std.z, port, starboard, profile, p0, p1).sec over s in (0, 1))
