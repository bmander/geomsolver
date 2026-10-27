// Three plates in three frames: two of one size, which are one part placed twice, and one of
// another.  An assembly in miniature — the parts a CAD system would list, and the instances of
// each.
//
// The colours the workspace shades them in, and the STEP assembly writes them in, are the
// drawing's (`plates.svd` beside this file), never this file's: a model says what the parts
// are, and how they look is presentation.  A part the drawing names no colour for keeps its
// palette colour, the same on every reading.

unit mm
use std
use std.place (offset)

// a plate drawn about its frame's origin, `w` by `d` and `h` thick
component Plate(w: Length, d: Length, h: Length) {
  in std.front {
    a := point hint((1, 1))
    b := point hint((w, 1))
    c := point hint((w, d))
    e := point hint((1, d))
    a offset(dx: 1mm, dy: 1mm) std.origin
    b offset(dx: w, dy: 1mm) std.origin
    c offset(dx: w, dy: d) std.origin
    e offset(dx: 1mm, dy: d) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: h)
  }
}

// two plates of one size, standing in frames turned from the world's
in (f := frame) { one := Plate(w: 30mm, d: 20mm, h: 5mm) }
fix(dir == (0, 1, 0)) f.u
fix(dir == (0, 0, 1)) f.v
fix(origin == (10, 20, 30)) f
in (g := frame) { two := Plate(w: 30mm, d: 20mm, h: 5mm) }
fix(dir == (0, 0, 1)) g.u
fix(dir == (1, 0, 0)) g.v
fix(origin == (-40, 5, 12)) g

// and one of another, lying under them
in (h := frame) { three := Plate(w: 40mm, d: 20mm, h: 5mm) }
fix(origin == (0, 0, -50)) h
