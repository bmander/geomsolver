// A plate with a tab on every edge, the tab written once.
//
// `outline` is a named chain (§6.6): four lines walked in order, closed back to the first.
// `repeat e in outline { … }` makes one copy of its body per edge of that chain, in the order
// the chain walks them, with `e` naming the copy's edge — so the tab below is stated once and
// drawn four times, and a fifth edge in the outline would get a fifth tab with nothing else
// edited.  The copies are an ordinary `repeat`'s: `tip[2]` is the third tab's apex.
unit mm
param width = 60mm
param height = 40mm
param rise = 12mm

point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
point d hint(x: 0, y: 40)
ground a
outline = line ad(a, d) -> line dc(d, c) -> line cb(c, b) -> line ba(b, a) -> close
vertical ad
horizontal dc
vertical cb
horizontal ba
distance(width) ba
distance(height) ad

// The outline runs clockwise, so outside is on each edge's left.  The apex stands `rise`
// off its edge, and the tab's two sides are equal — which puts the apex over the edge's middle.
repeat e in outline {
  point tip
  tip distance(rise, side: left) e
  line up(e.p1, tip)
  line down(tip, e.p2)
  up equal down
}
