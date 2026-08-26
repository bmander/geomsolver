// A rectangle with four equal fillets — the smallest drawing that is about *tangency*.
//
// Nothing here states where a corner arc goes.  Each arc is told only that it runs into the line
// before it and out into the line after it, tangentially at its own two ends, and that all four
// are the same size; the rectangle's sides are levelled, one width and one height are given, and
// one arc centre is grounded.  Everything else — twelve points, four radii — is the solver's.
//
// It is written as a **chain** (§6.6), which is how a contour reads: one element after the next,
// with the word between them saying how they meet.  The arcs name only their centres, because a
// joint threads the point its two elements share — so each fillet takes its start from the line
// it comes from and its end from the line it goes to, and `tangent` at a joint is stated *at*
// that point rather than as a bare tangency over a coincidence.  `close` seals the loop back onto
// the first link.  Every statement it stands for is an ordinary one; `tests/chain.rs` holds the
// two spellings to being one drawing.
//
// The two dimensions measure **across the rectangle**, so they read `w` and `h` rather than the
// tangent-point spans they used to.  The drawing has points there to measure between because the
// fillets put them there: `a_tl` and `a_tr` are both tangent to `top`, so `l1` and `r2` are level
// with each other and a width apart; `a_tr` and `a_br` are both tangent to `right`, so `t1` and
// `b2` share a vertical and are a height apart.

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

distance(l1, r2) == w
distance(t1, b2) == h

ground(c_bl)
