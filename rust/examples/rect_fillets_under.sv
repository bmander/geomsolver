// The same rounded rectangle, with the width taken away.
//
// Every other statement still holds, so the shape is unchanged — but nothing now fixes how wide
// the rectangle is, and the right-hand end is free to slide.  That is one **degree of freedom**:
// one independent way the drawing can still move without breaking anything it was told.
//
// The app colours what is still free, so this is the case to look at to see that reading.  Drag
// the right-hand side and it goes; drag the left and the whole figure is already pinned.
//
// The outline is a chain, exactly as in `rect_fillets.sv`.  What this file changes is one
// *number*, and a chain says nothing about numbers — so the two documents differ by precisely
// the line that is missing.

use std

w := 100
h := 60
r := 10

// the straight runs, each between the two fillets it joins
in std.front {
  b1 := point hint(x: r, y: 0)
  b2 := point hint(x: w - r, y: 0)
  r1 := point hint(x: w, y: r)
  r2 := point hint(x: w, y: h - r)
  t1 := point hint(x: w - r, y: h)
  t2 := point hint(x: r, y: h)
  l1 := point hint(x: 0, y: h - r)
  l2 := point hint(x: 0, y: r)

  // the fillet centres; where each arc starts and ends is the chain's to say
  c_br := point hint(x: w - r, y: r)
  c_tr := point hint(x: w - r, y: h - r)
  c_tl := point hint(x: r, y: h - r)
  c_bl := point

  // round the outline, counter-clockwise from the bottom edge
  horizontal (bottom := line(b1, b2)) -> tangent
  (a_br := arc(center: c_br) hint(r: r)) -> tangent
  vertical (right := line(r1, r2)) -> tangent
  (a_tr := arc(center: c_tr) hint(r: r)) -> tangent
  horizontal (top := line(t1, t2)) -> tangent
  (a_tl := arc(center: c_tl) hint(r: r)) -> tangent
  vertical (left := line(l1, l2)) -> tangent
  (a_bl := arc(center: c_bl) hint(r: r)) -> tangent close

  // one radius, stated once and shared
  a_br equal a_tr equal a_tl equal a_bl
  radius(r) a_bl

  // and no width: this is the freedom the case is about
  t1 distance(h) b2

  fix(x == r, y == r) c_bl
}
