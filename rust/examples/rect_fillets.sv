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
// The outline is written as a **chain**: one element after the next, with the word between them
// saying how they meet.  `tangent` means they run smoothly into one another with no crease;
// `close` joins the last back round to the first.  The arcs name only their centres, because a
// chain already knows where each one starts and ends — those are the points it shares with the
// sides on either side of it.
//
// Two spellings are worth knowing before reading on.  `hint at (…)` is a starting guess and the
// solver will move it; `==` states a number it must not.

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

distance(l1, r2) == w
distance(t1, b2) == h

ground(c_bl)
