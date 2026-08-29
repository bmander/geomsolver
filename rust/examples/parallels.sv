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

point o hint(x: 0, y: 0)
point e hint(x: 40, y: 0)
line  base(o, e)

point a hint(x: 0, y: 15)
point b hint(x: 40, y: 15)
line  l2(a, b)

point c hint(x: 10, y: 15)
point d hint(x: 10, y: 35)
line  l3(c, d)

point f hint(x: 10, y: 35)
point g hint(x: 30, y: 30)
line  l4(f, g)

base parallel l2
o distance(15) a
vertical l3
c coincident a
c distance(20) d
a distance(40) b
l3 perpendicular l4
f coincident d
f distance(20) g

ground o
ground e
