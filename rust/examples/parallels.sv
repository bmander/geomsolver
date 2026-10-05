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

use std

in std.front {
  o := point
  e := point
  base := line(o, e)

  a := point hint(x: 0, y: 15)
  b := point hint(x: 40, y: 15)
  d := point hint(x: 10, y: 35)
  l2 := line(a, b)
  base parallel l2
  a distance(15, side: left) base
  a distance(40) b

  // l3 rises from l2's start, and l4 leaves l3's top square to it
  vertical (l3 := line(a, d)) -> perpendicular (l4 := line(d, hint(x: 30, y: 30)))
  distance(20) l3
  distance(20) l4

  fix(x == 0, y == 0) o
  fix(x == 40, y == 0) e
}
