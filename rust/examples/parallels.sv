// A chain of segments held in place by direction alone.
//
// Almost nothing here is positioned.  What holds the figure is that one segment is parallel to
// the grounded base, another is vertical, and a third is square to that one — statements about
// which way things *point*, never about where they are.  Four lengths do the rest.
//
// Directions are worth treating as their own kind of fact.  "Parallel to" and "square to" chain
// together transitively, so a great many segments can end up sharing one direction without any
// of them touching, and a drawing is often held together far more by that than by its
// coincidences.
//
// One degree of freedom is left over: nothing says where along the base the chain sits, so it
// slides.

point o hint at (0, 0)
point e hint at (40, 0)
line  base(o, e)

point a hint at (0, 15)
point b hint at (40, 15)
line  l2(a, b)

point c hint at (10, 15)
point d hint at (10, 35)
line  l3(c, d)

point f hint at (10, 35)
point g hint at (30, 30)
line  l4(f, g)

parallel(base, l2)
distance(o, a) == 15
vertical(l3)
coincident(c, a)
distance(c, d) == 20
distance(a, b) == 40
perpendicular(l3, l4)
coincident(f, d)
distance(f, g) == 20

ground(o)
ground(e)
