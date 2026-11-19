// `std.measures`: what a body is made of, and where (§6.25) — each word an equation of integrals
// over a solid, a face of one, a beam, a line a beam is drawn on or an instance's bodies, by the
// set's own measure (a volume, an area, a member's section along its length).  Stated, each holds
// its equation; claimed, it is judged on the solved drawing; given a bound (`claim mass(<= 2kg)
// arm`), its equation is that bound.  A number a word states may read an unknown the drawing
// declares, which its row then determines: `mass(M) boat` sizes nothing and says what it weighs.
use std

// The volume of a body or a frame's members: `volume(42cm^3) web`.
volume(v) B := {
  integral(1 over p in B) == v
}

// What a body or a frame's members weigh, each point by the density of what it is made of:
// `mass(10kg) arm`.
mass(m) B := {
  integral(density(p) over p in B) == m
}

// A face's area: `area(L * B) hull.water`.
area(a) F := {
  integral(1 over p in F) == a
}

// A face's second moment about a line, an axis or a plane: `moment(i: I, about: centreline)
// hull.water`, `∫ r² dA`, `r` how far each point of the face stands off it.
moment(i, about) F := {
  integral(distance(p, about)^2 over p in F) == i
}

// A point at the centroid of a body's volume or a face's area: `g centroid hull` — the point
// the body's first moment about is nothing along each axis of the world, each over the body's
// measure, so each says how far the point stands off the centroid along its axis.
g centroid B := {
  integral(distance(g, p, along: std.x) over p in B) / integral(1 over p in B) == 0
  integral(distance(g, p, along: std.y) over p in B) / integral(1 over p in B) == 0
  integral(distance(g, p, along: std.z) over p in B) / integral(1 over p in B) == 0
}

// A point at a body's centre of mass, each point weighed by its density: `g centerofmass hull`.
// Of an instance, of every body under it, however its bodies are grouped into components.
g centerofmass B := {
  (integral(distance(g, p, along: std.x) * density(p) over p in B)
    / integral(density(p) over p in B)) == 0
  (integral(distance(g, p, along: std.y) * density(p) over p in B)
    / integral(density(p) over p in B)) == 0
  (integral(distance(g, p, along: std.z) * density(p) over p in B)
    / integral(density(p) over p in B)) == 0
}
