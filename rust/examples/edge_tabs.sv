// A plate with a tab on every edge, the tab written once.
//
// `outline` is a named chain (§6.6): four lines walked in order, closed back to the first.
// `repeat e in outline { … }` makes one copy of its body per edge of that chain, in the order
// the chain walks them, with `e` naming the copy's edge — so the tab below is stated once and
// drawn four times, and a fifth edge in the outline would get a fifth tab with nothing else
// edited.  The copies are an ordinary `repeat`'s: `tip[2]` is the third tab's apex.
unit mm
use std
width := 60mm
height := 40mm
rise := 12mm

in std.front {
  a := point
  b := point hint((60, 0))
  c := point hint((60, 40))
  d := point hint((0, 40))
  fix((0, 0)) a
  outline := distance(height) vertical (ad := line(a, d)) -> horizontal (dc := line(d, c)) ->
    vertical (cb := line(c, b)) -> distance(width) horizontal (ba := line(b, a)) -> close

  // The outline runs clockwise, so outside is on each edge's left.  The apex stands `rise`
  // off its edge, and the tab's two sides are equal — which puts the apex over the edge's middle.
  repeat e in outline {
    tip := point
    tip distance(rise, side: left) e
    up := line(e.p1, tip)
    down := line(tip, e.p2)
    up equal down
  }
}
