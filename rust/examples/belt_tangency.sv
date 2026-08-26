// A belt over two pulleys, stated the way a draughtsman reaches for it — and a double root.
//
// Each end of the belt is held on its pulley, and the belt is tangent to each.  Said that way the
// pair is *rank-deficient at every solution*: a contact held on the circle can swim along the
// tangent line to first order, so the Jacobian reports two degrees of freedom the figure does not
// have — blocked at second order, where no rank tolerance can look.
//
// This is the case the second-order screen exists for: step along each surplus null direction,
// let the solver settle, and a motion that walks back is `shaky` rather than free, so the drawing
// is called rigid.  (The app no longer *writes* it this way — `tangent_line_circle_at` names the
// endpoint and is regular — but a document may still say it, and an old one does.)
//
// `side` is stated rather than left out: a document's omitted flag takes the registry default,
// where the Rust constructor reads it off the geometry, and here the geometry says -1.

point c1 at (0, 0)
point c2 at (50, 0)

circle k1(center: c1, r: 10)
circle k2(center: c2, r: 10)

point p at (0, 10)
point q at (50, 10)
line  belt(p, q)

radius(k1) == 10
radius(k2) == 10
point_on_circle(p, k1)
point_on_circle(q, k2)
tangent_line_circle(belt, k1, side: -1)
tangent_line_circle(belt, k2, side: -1)

ground(c1)
ground(c2)
