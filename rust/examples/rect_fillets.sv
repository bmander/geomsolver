// A rectangle with rounded corners — and nothing in this file says where the corners go.
//
// This is a drawing made of *statements* rather than of coordinates.  You do not place the
// geometry; you say what has to be true about it, and the solver finds positions that satisfy
// everything at once.  Drag any point on the canvas and the rest rearranges to keep every line
// below still true.
//
// Here that means: the four straight sides are level or plumb, each corner arc runs smoothly
// into the side before it and out into the side after it, all four arcs are the same size, and
// the overall width and height are given.  Where the twelve points sit and how big the arcs are
// is the solver's to work out.
//
// The outline is written as a **chain**: one element after the next, with `->` marking each
// corner where one runs into the next, and a word beside the marker saying how they meet
// there.  `-> tangent` means they run smoothly into one another with no crease; `-> close`
// joins the last back round to the first.  The arcs name only their centres, because a corner
// already knows where each one starts and ends — those are the points it shares with the
// sides on either side of it.
//
// Two things are worth knowing before reading on.  A `hint(…)` clause is a starting guess and
// the solver will move it; everything else is a number it must not.  And every constraint is a
// word standing before its one operand or between its two — `horizontal bottom`,
// `l1 distance(w) r2` — with whatever is not an operand in the parentheses on the word.

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
horizontal line bottom(b1, b2) -> tangent
arc a_br(center: c_br) hint(r: r) -> tangent
vertical line right(r1, r2) -> tangent
arc a_tr(center: c_tr) hint(r: r) -> tangent
horizontal line top(t1, t2) -> tangent
arc a_tl(center: c_tl) hint(r: r) -> tangent
vertical line left(l1, l2) -> tangent
arc a_bl(center: c_bl) hint(r: r) -> tangent close

// one radius, stated once and shared
a_br equal a_tr equal a_tl equal a_bl
radius(r) a_bl

l1 distance(w) r2
t1 distance(h) b2

ground c_bl
