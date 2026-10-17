// A section on three rails (#243 L8): `sec := prof mapped(from -> pa, to -> pb, third -> pc) in
// st.n` (#357) carries the face `prof` whole into the station's plane by the affine map taking
// `from`, `to` and `third` to `pa`, `pb` and `pc` — moved, turned, scaled and sheared — each
// corner keeping its place in the frame those three make (`std.Affine` says the same). With no guide, the station is the plane through the
// three rails at `s` (`std.Across`), and `pa`, `pb` and `pc` are where it crosses them.
// Closed form: an affine map scales every area alike, so the section is the profile's area times
// the rails' triangle's over the frame's (36/50), quadratic in `s` between level ends:
// 40/6 · 0.72 · (200 + 4·146.875 + 87.5) = 4200 mm³ as drawn.
// Edit any rail's end, or the profile.
unit mm
use std

component Tri(ra: path, rb: path, rc: path, prof: face, from: point, to: point, third: point,
              s: Scalar) {
  st := std.Across(ra, rb, rc, s: s)
  pa := point in st.n
  pb := point in st.n
  pc := point in st.n
  pa coincident ra
  pb coincident rb
  pc coincident rc
  sec := prof mapped(from -> pa, to -> pb, third -> pc) in st.n
}

a0 := point
b0 := point
c0 := point
a1 := point
b1 := point
c1 := point
fix((0mm, 0mm, 0mm)) a0
fix((20mm, 0mm, 0mm)) b0
fix((0mm, 20mm, 0mm)) c0
fix((5mm, 5mm, 40mm)) a1
fix((15mm, 0mm, 40mm)) b1
fix((0mm, 25mm, 40mm)) c1
ra := line(a0, a1)
rb := line(b0, b1)
rc := line(c0, c1)

// the profile, a 6 mm square in the frame `o`, `x`, `y`, drawn off to the side
in std.front {
  o := point
  x := point
  y := point
  fix((100mm, 0mm)) o
  fix((110mm, 0mm)) x
  fix((100mm, 10mm)) y
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((102mm, 2mm)) k0
  fix((108mm, 2mm)) k1
  fix((108mm, 8mm)) k2
  fix((102mm, 8mm)) k3
  sq := line(k0, k1) -> line(k1, k2) -> line(k2, k3) -> line(k3, k0) -> close
}
prof := face(sq)
param s: Scalar over (0, 1)
body := sweep(Tri(ra, rb, rc, prof, o, x, y).sec over s)
