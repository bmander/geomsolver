// The same rounded rectangle, told two different widths.
//
// The distance from `l1` to `r2` is given twice — once as `w`, which is 100, and once as 50.
// Nothing can be both, so this drawing has no solution at all.
//
// What the case is really for is the *report*.  A solver that only said "failed" would leave you
// hunting through forty statements for the two that disagree.  This one names them, and names
// only them: the rest of the rectangle is perfectly satisfiable and is not blamed for the pair
// that is not.
//
// The outline is a chain, exactly as in `rect_fillets.sv`.  What this file adds is one *number*,
// and a chain says nothing about numbers — so the two documents differ by precisely the line
// that is extra.

param w = 100
param h = 60
param r = 10

// the straight runs, each between the two fillets it joins
point b1 hint(x: r, y: 0)
point b2 hint(x: w - r, y: 0)
point r1 hint(x: w, y: r)
point r2 hint(x: w, y: h - r)
point t1 hint(x: w - r, y: h)
point t2 hint(x: r, y: h)
point l1 hint(x: 0, y: h - r)
point l2 hint(x: 0, y: r)

// the fillet centres; where each arc starts and ends is the chain's to say
point c_br hint(x: w - r, y: r)
point c_tr hint(x: w - r, y: h - r)
point c_tl hint(x: r, y: h - r)
point c_bl hint(x: r, y: r)

// round the outline, counter-clockwise from the bottom edge
horizontal line bottom(b1, b2) tangent
arc a_br(center: c_br) hint(r: r) tangent
vertical line right(r1, r2) tangent
arc a_tr(center: c_tr) hint(r: r) tangent
horizontal line top(t1, t2) tangent
arc a_tl(center: c_tl) hint(r: r) tangent
vertical line left(l1, l2) tangent
arc a_bl(center: c_bl) hint(r: r) tangent close

// one radius, stated once and shared
a_br equal a_tr equal a_tl equal a_bl
radius(a_bl) == r

// the same width, twice, and the two numbers disagree
distance(l1, r2) == w
distance(l1, r2) == 50
distance(t1, b2) == h

ground(c_bl)
