// The filleted rectangle with a second width dimension that contradicts the first.
//
// `distance(b1, b2)` is stated twice — once at `w - 2r`, once at 50 — and the two cannot both
// hold.  Structurally this is over-constrained and the matching finds it at once; what the case
// is for is the *reporting*: the conflict names both dimensions, and the minimal conflict set is
// the pair rather than the whole rectangle.

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

line bottom(b1, b2)
line right(r1, r2)
line top(t1, t2)
line left(l1, l2)

// the fillet centres, and the arcs that run counter-clockwise from one side to the next
point c_br at (w - r, r)
point c_tr at (w - r, h - r)
point c_tl at (r, h - r)
point c_bl at (r, r)

arc a_br(center: c_br, start: b2, end: r1, r: r)
arc a_tr(center: c_tr, start: r2, end: t1, r: r)
arc a_tl(center: c_tl, start: t2, end: l1, r: r)
arc a_bl(center: c_bl, start: l2, end: b1, r: r)

horizontal(bottom)
horizontal(top)
vertical(left)
vertical(right)

// each fillet meets the side it comes from and the side it goes to, tangentially at its own ends
tangent_arc_line(a_br, bottom, at: start)
tangent_arc_line(a_br, right,  at: end)
tangent_arc_line(a_tr, right,  at: start)
tangent_arc_line(a_tr, top,    at: end)
tangent_arc_line(a_tl, top,    at: start)
tangent_arc_line(a_tl, left,   at: end)
tangent_arc_line(a_bl, left,   at: start)
tangent_arc_line(a_bl, bottom, at: end)

// one radius, stated once and shared
equal_radius(a_br, a_tr)
equal_radius(a_br, a_tl)
equal_radius(a_br, a_bl)
radius(a_bl) == r

distance(b1, b2) == w - 2 * r
distance(b1, b2) == 50
distance(l1, l2) == h - 2 * r

ground(c_bl)
