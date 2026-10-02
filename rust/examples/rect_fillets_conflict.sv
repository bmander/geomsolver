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

w := 100
h := 60
r := 10

// the straight runs, each between the two fillets it joins
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

// the same width, twice, and the two numbers disagree
l1 distance(w) r2
l1 distance(50) r2
t1 distance(h) b2

fix(x == r, y == r) c_bl
