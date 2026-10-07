// What every view draws with.

use std (horizontal, vertical, above, offset)
use components.dims

// A point placed from `o` by two ordinates.
component At(o: point, dx: Length, dy: Length) {
  p := point hint((o.x + dx, o.y + dy))
  p offset(dx: dx, dy: dy) o
}

// An axis-aligned rectangle about `o`: `a` is its lower-left corner, offset from `o`.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point hint((o.x + x0, o.y + y0))
  b := point hint((o.x + x1, o.y + y0))
  c := point hint((o.x + x1, o.y + y1))
  d := point hint((o.x + x0, o.y + y1))
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  a offset(dx: x0, dy: y0) o
  b offset(dx: x1, dy: y0) o
  c offset(dx: x1, dy: y1) o
  d offset(dx: x0, dy: y1) o
}

// A rectangle between `x0` and `x1` whose top and bottom are the heights of two points another
// view placed — the side view's reading of a part the front view designs.
component Slab(o: point, x0: Length, x1: Length, top: point, bottom: point) {
  a := point hint((o.x + x0, bottom.y))
  b := point hint((o.x + x1, bottom.y))
  c := point hint((o.x + x1, top.y))
  d := point hint((o.x + x0, top.y))
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o distance(x0, along: x) a
  bottom horizontal a
  o distance(x1, along: x) b
  bottom horizontal b
  o distance(x1, along: x) c
  top horizontal c
  o distance(x0, along: x) d
  top horizontal d
}

// The same the other way up: a rectangle between `y0` and `y1` whose left and right are the
// widths of two points another view placed — the top view's reading of the front's.
component Wide(o: point, y0: Length, y1: Length, left: point, right: point) {
  a := point hint((left.x, o.y + y0))
  b := point hint((right.x, o.y + y0))
  c := point hint((right.x, o.y + y1))
  d := point hint((left.x, o.y + y1))
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  left vertical a
  o distance(y0, along: y) a
  right vertical b
  o distance(y0, along: y) b
  right vertical c
  o distance(y1, along: y) c
  left vertical d
  o distance(y1, along: y) d
}

// A part sheet's datum: the part's axis up the page through `o`.
component Axes(o: point) {
  up := point hint((o.x, o.y + 40mm))
  up above(d: 40) o
  ax := line(o, up)
  f := std.Turned(o, up)
}

// A set screw into the shaft: its clearance hole from the bore at `rin` out to the rim at
// `rout`, and the pocket the nut is trapped in, `nutin` out from the bore.  Written in the
// datum `f` along the screw's axis.  Drawn on the part's sheet only.
component Grub(f: group, rin: Length, rout: Length, dims: group) {
  ax := line(f.u.p1, f.u.p2)
  h0 := point hint(at: f.axes, (rin, dims.grub / 2))
  h1 := point hint(at: f.axes, (rout, dims.grub / 2))
  h2 := point hint(at: f.axes, (rin, -dims.grub / 2))
  h3 := point hint(at: f.axes, (rout, -dims.grub / 2))
  s0 := line(h0, h1)
  s1 := line(h2, h3)
  n0 := point hint(at: f.axes, (rin + dims.nutin, dims.nutaf / 2))
  n1 := point hint(at: f.axes, (rin + dims.nutin + dims.nutT, dims.nutaf / 2))
  n2 := point hint(at: f.axes, (rin + dims.nutin + dims.nutT, -dims.nutaf / 2))
  n3 := point hint(at: f.axes, (rin + dims.nutin, -dims.nutaf / 2))
  q0 := line(n0, n1)
  q1 := line(n1, n2)
  q2 := line(n2, n3)
  q3 := line(n3, n0)
  // The bore flanks are parallel and mirror each other across the screw axis.
  s0 parallel ax
  h0 symmetry(ax) h2
  h1 symmetry(ax) h3
  h0 distance(dims.grub / 2, side: left) ax
  h0 distance(rin, along: u) f.axes
  h1 distance(rout, along: u) f.axes
  // The nut pocket is a centered rectangle, dimensioned by the selected nut.
  q0 parallel ax
  q1 perpendicular ax
  q2 parallel ax
  q3 perpendicular ax
  distance(dims.nutT) q0
  distance(dims.nutaf) q1
  n0 distance(dims.nutaf / 2, side: left) ax
  n0 distance(rin + dims.nutin, along: u) f.axes
  // **the screw's hole is a solid; its nut's pocket is not, and that is a limit of the language
  // and not of the design.**  The hole is a turn of the half-section above about the screw's own
  // line, which lies in this plane — `about:` takes exactly such a line.  The pocket is a *hex*
  // prism about that same line, and neither sweep reaches it: `from:`/`to:` runs along the
  // plane's normal and `about:` turns, so nothing here sweeps a section *along* a line lying in
  // the plane.  So the pocket stays what it has always been, four hidden lines a printer reads,
  // and it is not part of the body; it comes back when a swept solid does (spec §17).
  a0 := point hint(at: f.axes, (rin, 0mm))
  a1 := point hint(at: f.axes, (rout, 0mm))
  bore_f := face(a0, h0, s0, a1, -> close)
  entry := line(h0, h2)
  exit := line(h1, h3)
  a0 midpoint entry
  a1 midpoint exit
  bore := solid(bore_f, about: ax)
  claim h0 distance(dims.grub) h2
  claim n0 distance(dims.nutT) n1
  claim n0 distance(dims.nutaf) n3
}
