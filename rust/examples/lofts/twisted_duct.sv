// A twisted duct (#243 L5): a 20 mm square run 40 mm along a line and turned as it runs. The
// square is drawn once, off to the side, and carried whole onto each station by the similarity
// taking its frame `o`, `x` to the station's origin and `std.Twisted`'s point turned `twist · s`
// about it (`sec := square mapped(o -> st.n.origin, x -> tw.t) in st.n`, #357), so a turn of
// three and a half turns is one statement, and an eased turn is the same word given another
// fraction (`s: 3 * s^2 - 2 * s^3`).
// Closed form (Cavalieri): every section is the same square, so V = (2·half)² · length at any
// turn and under any law, 16000 mm³ as drawn — the duct, the auger and the eased one alike.
// Edit `length`, `half`, `apart` or either twist, or the square.
unit mm
use std (vertical)

length := 40mm
half := 10mm
apart := 40mm
reach := 10mm

// The square at station `s`, turned `twist · s` about the guide.
component TwistedSquare(g: line, up: axis, prof: face, o: point, x: point, reach: Length,
                        twist: Angle, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, reach: reach, s: s)
  sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
}

// The same square turned on an eased law: slow at both ends, fastest half way.
component EasedSquare(g: line, up: axis, prof: face, o: point, x: point, reach: Length,
                      twist: Angle, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, reach: reach, s: 3 * s^2 - 2 * s^3)
  sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
}

// The square, drawn once in the front view below the guides, its frame `o`, `x` at its centre.
in std.front {
  o := point
  x := point
  fix((0mm, -50mm)) o
  fix((reach, -50mm)) x
  a := point
  b := point
  c := point
  d := point
  fix((-half, -50mm - half)) a
  fix((half, -50mm - half)) b
  fix((half, -50mm + half)) c
  fix((-half, -50mm + half)) d
}
square := face(a, b, c, d, -> close)

// Each duct on a guide of its own, `apart` from the last, so the three are seen side by side.
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
// a quarter turn
duct := solid(TwistedSquare(guide, std.z, square, o, x, reach: reach, twist: 90deg).sec
              over s in (0, 1))
// three and a half turns: no pairing of two sections can say this
auger := solid(TwistedSquare(guide2, std.z, square, o, x, reach: reach, twist: 1260deg).sec
               over s in (0, 1))
// a quarter turn, eased in and out
eased := solid(EasedSquare(guide3, std.z, square, o, x, reach: reach, twist: 90deg).sec
               over s in (0, 1))
