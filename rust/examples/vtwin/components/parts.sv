// What every view draws with.

use std
use components.dims

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
  profile = line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(x0, along: x) a
  o distance(y0, along: y) a
  o distance(x1, along: x) b
  o distance(y0, along: y) b
  o distance(x1, along: x) c
  o distance(y1, along: y) c
  o distance(x0, along: x) d
  o distance(y1, along: y) d
}

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

// A part sheet's datum: the part's axis up the page through `o`.
component Axes(o: point) {
  point up hint(x: o.x, y: o.y + 40mm)
  o distance(0, along: x) up
  o distance(40, along: y) up
  line ax(o, up)
  plane f(origin: o, toward: up)
}

// A set screw into the shaft: its clearance hole from the bore at `rin` out to the rim at
// `rout`, and the pocket the nut is trapped in, `nutin` out from the bore.  Written in the
// datum `f` along the screw's axis.  Drawn on the part's sheet only.
component Grub(f: plane, rin: Length, rout: Length, dims: group) {
  line ax(f.origin, f.toward)
  point h0 hint(x: f.origin.x + (rin) * f.c - (dims.grub / 2) * f.s,
                    y: f.origin.y + (rin) * f.s + (dims.grub / 2) * f.c)
  point h1 hint(x: f.origin.x + (rout) * f.c - (dims.grub / 2) * f.s,
                    y: f.origin.y + (rout) * f.s + (dims.grub / 2) * f.c)
  point h2 hint(x: f.origin.x + (rin) * f.c - (-dims.grub / 2) * f.s,
                    y: f.origin.y + (rin) * f.s + (-dims.grub / 2) * f.c)
  point h3 hint(x: f.origin.x + (rout) * f.c - (-dims.grub / 2) * f.s,
                    y: f.origin.y + (rout) * f.s + (-dims.grub / 2) * f.c)
  line s0(h0, h1)
  line s1(h2, h3)
  point n0 hint(x: f.origin.x + (rin + dims.nutin) * f.c - (dims.nutaf / 2) * f.s,
                    y: f.origin.y + (rin + dims.nutin) * f.s + (dims.nutaf / 2) * f.c)
  point n1 hint(x: f.origin.x + (rin + dims.nutin + dims.nutT) * f.c - (dims.nutaf / 2) * f.s,
                    y: f.origin.y + (rin + dims.nutin + dims.nutT) * f.s + (dims.nutaf / 2) * f.c)
  point n2 hint(x: f.origin.x + (rin + dims.nutin + dims.nutT) * f.c - (-dims.nutaf / 2) * f.s,
                    y: f.origin.y + (rin + dims.nutin + dims.nutT) * f.s + (-dims.nutaf / 2) * f.c)
  point n3 hint(x: f.origin.x + (rin + dims.nutin) * f.c - (-dims.nutaf / 2) * f.s,
                    y: f.origin.y + (rin + dims.nutin) * f.s + (-dims.nutaf / 2) * f.c)
  line q0(n0, n1)
  line q1(n1, n2)
  line q2(n2, n3)
  line q3(n3, n0)
  // The bore flanks are parallel and mirror each other across the screw axis.
  s0 parallel ax
  h0 symmetry(ax) h2
  h1 symmetry(ax) h3
  h0 distance(dims.grub / 2, side: left) ax
  h0 distance(rin, along: u) f
  h1 distance(rout, along: u) f
  // The nut pocket is a centered rectangle, dimensioned by the selected nut.
  q0 parallel ax
  q1 perpendicular ax
  q2 parallel ax
  q3 perpendicular ax
  distance(dims.nutT) q0
  distance(dims.nutaf) q1
  n0 distance(dims.nutaf / 2, side: left) ax
  n0 distance(rin + dims.nutin, along: u) f
  // **the screw's hole is a solid; its nut's pocket is not, and that is a limit of the language
  // and not of the design.**  The hole is a turn of the half-section above about the screw's own
  // line, which lies in this plane — `about:` takes exactly such a line.  The pocket is a *hex*
  // prism about that same line, and neither sweep reaches it: `from:`/`to:` runs along the
  // plane's normal and `about:` turns, so nothing here sweeps a section *along* a line lying in
  // the plane.  So the pocket stays what it has always been, four hidden lines a printer reads,
  // and it is not part of the body; it comes back when a swept solid does (spec §17).
  point a0 hint(x: f.origin.x + (rin) * f.c - (0mm) * f.s,
                    y: f.origin.y + (rin) * f.s + (0mm) * f.c)
  point a1 hint(x: f.origin.x + (rout) * f.c - (0mm) * f.s,
                    y: f.origin.y + (rout) * f.s + (0mm) * f.c)
  face bore_f(a0, h0, s0, a1, -> close)
  line entry(h0, h2)
  line exit(h1, h3)
  a0 midpoint entry
  a1 midpoint exit
  solid bore(bore_f, about: ax)
  claim h0 distance(dims.grub) h2
  claim n0 distance(dims.nutT) n1
  claim n0 distance(dims.nutaf) n3
}
