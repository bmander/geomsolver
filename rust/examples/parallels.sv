// Parallels and perpendiculars: the case the direction classes are read on.
//
// Nothing here is dimensioned into place except four lengths.  What holds the figure is that
// `l2` is parallel to the grounded base, `l3` is vertical, and `l4` is perpendicular to `l3` —
// three statements about *directions*, which `cgraph` collects into classes rather than into
// edges.  The chain is joined by coincidences instead of by shared points, so the drawing also
// says whether a merge follows an alias.
//
// One degree of freedom is left: nothing fixes where the chain sits along the base.

point o at (0, 0)
point e at (40, 0)
line  base(o, e)

point a at (0, 15)
point b at (40, 15)
line  l2(a, b)

point c at (10, 15)
point d at (10, 35)
line  l3(c, d)

point f at (10, 35)
point g at (30, 30)
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
