// What every view draws with.

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

// A point in a *tilted* frame: `u` along the line `ax` from the point `org` on it, `v` to the
// left of it, with `ac` the line across `ax` through `org`, directed to the left, and `dir` the
// bearing of `ax` for the seed.  Two signed point-to-line distances place it, so a rocking
// cylinder is written in its own coordinates and turns whole when the crank does.
component Loc(org: point, ax: line, ac: line, dir: Angle, u: Length, v: Length) {
  point p hint(x: org.x + u * cos(dir) - v * sin(dir), y: org.y + u * sin(dir) + v * cos(dir))
  p distance(v) ax
  p distance(-u) ac
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
