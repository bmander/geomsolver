// What every view draws with.

use std
use vtwin.dims

// A point placed from `o` by two ordinates.
component At(o: point, dx: Length, dy: Length) {
  point p hint(x: o.x + dx, y: o.y + dy)
  o distance(dx, along: x) p
  o distance(dy, along: y) p
}

// An axis-aligned rectangle about `o`: `a` is its lower-left corner, offset from `o`.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  point a hint(x: o.x + x0, y: o.y + y0)
  point b hint(x: o.x + x1, y: o.y + y0)
  point c hint(x: o.x + x1, y: o.y + y1)
  point d hint(x: o.x + x0, y: o.y + y1)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(x0, along: x) a
  o distance(y0, along: y) a
  o distance(x1, along: x) b
  o distance(y0, along: y) b
  o distance(x1, along: x) c
  o distance(y1, along: y) c
  o distance(x0, along: x) d
  o distance(y1, along: y) d
}

// `Loc` — a point in a tilted frame — moved to `std` (§6.9): it is a general helper and was
// written out twice, here and in `engine.parts`, which is one drawing in two files the moment
// one of them is edited.


// A rectangle between `x0` and `x1` whose top and bottom are the heights of two points another
// view placed — the side view's reading of a part the front view designs.
component Slab(o: point, x0: Length, x1: Length, top: point, bottom: point) {
  point a hint(x: o.x + x0, y: bottom.y)
  point b hint(x: o.x + x1, y: bottom.y)
  point c hint(x: o.x + x1, y: top.y)
  point d hint(x: o.x + x0, y: top.y)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(x0, along: x) a
  bottom distance(0, along: y) a
  o distance(x1, along: x) b
  bottom distance(0, along: y) b
  o distance(x1, along: x) c
  top distance(0, along: y) c
  o distance(x0, along: x) d
  top distance(0, along: y) d
}

// The same the other way up: a rectangle between `y0` and `y1` whose left and right are the
// widths of two points another view placed — the top view's reading of the front's.
component Wide(o: point, y0: Length, y1: Length, left: point, right: point) {
  point a hint(x: left.x, y: o.y + y0)
  point b hint(x: right.x, y: o.y + y0)
  point c hint(x: right.x, y: o.y + y1)
  point d hint(x: left.x, y: o.y + y1)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  left distance(0, along: x) a
  o distance(y0, along: y) a
  right distance(0, along: x) b
  o distance(y0, along: y) b
  right distance(0, along: x) c
  o distance(y1, along: y) c
  left distance(0, along: x) d
  o distance(y1, along: y) d
}

// A part sheet's frame: the part's axis up the page through `o`, and a line across it to the
// left, which is what `Loc` writes a part in.
component Axes(o: point) {
  point up hint(x: o.x, y: o.y + 40mm)
  o distance(0, along: x) up
  o distance(40, along: y) up
  line ax(o, up) class axis
  point left hint(x: o.x - 10mm, y: o.y)
  o distance(10, along: left) left
  o distance(0, along: y) left
  line ac(o, left) class gone
}

// A set screw into the shaft: its clearance hole from the bore at `rin` out to the rim at
// `rout`, and the pocket the nut is trapped in, `nutin` out from the bore.  Written in the
// frame (`ax`, `ac`) the screw's axis lies along.  Drawn on the part's sheet only.
component Grub(o: point, ax: line, ac: line, dir: Angle, rin: Length, rout: Length) {
  h0: Loc(o, ax, ac, dir: dir, u: rin, v: grub / 2)
  h1: Loc(o, ax, ac, dir: dir, u: rout, v: grub / 2)
  h2: Loc(o, ax, ac, dir: dir, u: rin, v: -grub / 2)
  h3: Loc(o, ax, ac, dir: dir, u: rout, v: -grub / 2)
  line s0(h0.p, h1.p) class hidden detail
  line s1(h2.p, h3.p) class hidden detail
  n0: Loc(o, ax, ac, dir: dir, u: rin + nutin, v: nutaf / 2)
  n1: Loc(o, ax, ac, dir: dir, u: rin + nutin + nutT, v: nutaf / 2)
  n2: Loc(o, ax, ac, dir: dir, u: rin + nutin + nutT, v: -nutaf / 2)
  n3: Loc(o, ax, ac, dir: dir, u: rin + nutin, v: -nutaf / 2)
  line q0(n0.p, n1.p) class hidden detail
  line q1(n1.p, n2.p) class hidden detail
  line q2(n2.p, n3.p) class hidden detail
  line q3(n3.p, n0.p) class hidden detail
  // **the screw's hole is a solid; its nut's pocket is not, and that is a limit of the language
  // and not of the design.**  The hole is a turn of the half-section above about the screw's own
  // line, which lies in this plane — `about:` takes exactly such a line.  The pocket is a *hex*
  // prism about that same line, and neither sweep reaches it: `from:`/`to:` runs along the
  // plane's normal and `about:` turns, so nothing here sweeps a section *along* a line lying in
  // the plane.  So the pocket stays what it has always been, four hidden lines a printer reads,
  // and it is not part of the body; it comes back when a swept solid does (spec §17).
  a0: Loc(o, ax, ac, dir: dir, u: rin, v: 0mm)
  a1: Loc(o, ax, ac, dir: dir, u: rout, v: 0mm)
  line s_in(a0.p, h0.p) class gone
  line s_out(h1.p, a1.p) class gone
  line s_axis(a1.p, a0.p) class gone
  face bore_f(s_in, s0, s_out, s_axis)
  solid bore(bore_f, about: ax)
  claim h0.p distance(grub) h2.p class detail at (0, 4)
  claim n0.p distance(nutT) n1.p class detail at (0, 8)
  claim n0.p distance(nutaf) n3.p class detail at (0, -6)
}
