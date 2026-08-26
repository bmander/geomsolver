// The filleted rectangle with its width dimension taken away.
//
// Everything else still holds — the fillets are still tangent to the sides they run between and
// still equal — so the figure keeps its shape and the right-hand end simply slides: one degree of
// freedom, and a null space that points along it.  The case the under-constrained colouring is
// read on.
//
// The contour is a chain (§6.6), as it is in `rect_fillets.sv`: what this case takes away is one
// *dimension*, and a chain says nothing about numbers, so the two documents differ by exactly the
// line that is missing.

param w = 100
param h = 60
param r = 10

// the straight runs, each between the two fillets it joins
point b1 at (r, 0)
point b2 at (w - r, 0)
point r1 at (w, r)
point r2 at (w, h - r)
point t1 at (w - r, h)
point t2 at (r, h)
point l1 at (0, h - r)
point l2 at (0, r)

// the fillet centres; where each arc starts and ends is the chain's to say
point c_br at (w - r, r)
point c_tr at (w - r, h - r)
point c_tl at (r, h - r)
point c_bl at (r, r)

// round the outline, counter-clockwise from the bottom edge
horizontal line bottom(b1, b2) tangent
arc a_br(center: c_br, r: r) tangent
vertical line right(r1, r2) tangent
arc a_tr(center: c_tr, r: r) tangent
horizontal line top(t1, t2) tangent
arc a_tl(center: c_tl, r: r) tangent
vertical line left(l1, l2) tangent
arc a_bl(center: c_bl, r: r) tangent close

// one radius, stated once and shared
a_br equal a_tr equal a_tl equal a_bl
radius(a_bl) == r

// and no width: this is the freedom the case is about
distance(t1, b2) == h

ground(c_bl)
