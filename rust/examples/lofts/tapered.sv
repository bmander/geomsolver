// Laws of scale (#243 L5): a section's size is a dimension reading `s`. Each section is drawn
// once, off to the side, and carried whole onto its station by the similarity taking its frame
// `o`, `x` to the station's origin and `std.Twisted`'s point (`sec := prof mapped(o ->
// st.n.origin, x -> tw.t) in st.n`, #357), which stands `reach` scaled `1 + (scale − 1)·s` from
// the origin, turned `twist · s`: the section is scaled and turned about its centre, a round's
// radius with it.
// Any other law is the same word given another fraction, or any expression in `s` (`sin`, `max`).
// Closed forms, over the 40 mm guide:
//   taper: the 20 mm square shrinking to half while it turns a quarter (Cavalieri, each section
//     the untwisted one turned): a frustum, 40/3 · (400 + 100 + 200) mm³;
//   cone: the disc of radius 10 shrinking to 5: π · 40/3 · (100 + 50 + 25) mm³, its wall a cone;
//   swell: the disc swelling as 1 + 0.3·sin(180°·s): 4000π · (1 + 1.2/π + 0.045) mm³.
// Edit `length`, `half`, `r`, `apart`, a scale or a twist, or a section.
unit mm
use std (vertical)

length := 40mm
half := 10mm
r := 10mm
apart := 40mm
reach := 10mm

// The section `prof`, its frame `o`, `x` a `reach` apart, carried to the station on the guide
// `g`, scaled `1 + (scale − 1)·s` and turned `twist · s`.
component Section(g: line, up: axis, prof: face, o: point, x: point, reach: Length, twist: Angle,
                  scale: Scalar, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, reach: reach * (1 + (scale - 1) * s), s: s)
  sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
}

// The same, its scale's law a sine: `1 + (scale − 1)·sin(180°·s)`.
component Swell(g: line, up: axis, prof: face, o: point, x: point, reach: Length, scale: Scalar,
                s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: 0deg, reach: reach * (1 + (scale - 1) * sin(180 * s)), s: s)
  sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
}

// The sections, drawn once in the front view below the guides: a square about `o1`, and a disc
// about `o2`.
in std.front {
  o1 := point
  x1 := point
  fix((0mm, -50mm)) o1
  fix((reach, -50mm)) x1
  a := point
  b := point
  c := point
  d := point
  fix((-half, -50mm - half)) a
  fix((half, -50mm - half)) b
  fix((half, -50mm + half)) c
  fix((-half, -50mm + half)) d
  o2 := point
  x2 := point
  fix((apart, -50mm)) o2
  fix((apart + reach, -50mm)) x2
  disc := circle(center: o2)
  radius(r) disc
}
square := face(a, b, c, d, -> close)
round := face(disc)

// Each solid on a guide of its own, `apart` from the last, so the three are seen side by side.
in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0, length))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
  entry2 := point
  fix((apart, 0mm)) entry2
  exit2 := point hint((apart, length))
  guide2 := line(entry2, exit2)
  vertical guide2
  length(length) guide2
  entry3 := point
  fix((2 * apart, 0mm)) entry3
  exit3 := point hint((2 * apart, length))
  guide3 := line(entry3, exit3)
  vertical guide3
  length(length) guide3
}
taper := solid(Section(guide, std.z, square, o1, x1, reach: reach, twist: 90deg,
                       scale: 0.5).sec over s in (0, 1))
cone := solid(Section(guide2, std.z, round, o2, x2, reach: reach, twist: 0deg,
                      scale: 0.5).sec over s in (0, 1))
swell := solid(Swell(guide3, std.z, round, o2, x2, reach: reach, scale: 1.3).sec
               over s in (0, 1))
