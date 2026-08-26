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

param w = 100
param h = 60
param r = 10

// the straight runs, each between the two fillets it joins
point b1 hint at (r, 0)
point b2 hint at (w - r, 0)
point r1 hint at (w, r)
point r2 hint at (w, h - r)
point t1 hint at (w - r, h)
point t2 hint at (r, h)
point l1 hint at (0, h - r)
point l2 hint at (0, r)

// the fillet centres; where each arc starts and ends is the chain's to say
point c_br hint at (w - r, r)
point c_tr hint at (w - r, h - r)
point c_tl hint at (r, h - r)
point c_bl hint at (r, r)

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
