// A rigid framework with no triangle anywhere in it.
//
// Six joints in two groups of three, with a bar from every joint of one group to every joint
// of the other.  That is nine bars — exactly `2 × 6 − 3`, the fewest that can hold six joints
// rigid in a plane.  Drag any of them and the whole thing keeps its shape.
//
// It is a hard case because the usual way to see that a framework is rigid is to find a triangle
// and grow outwards from it, adding joints pinned by two bars to what you have already.
// This framework has no triangle at all — every closed path in it is four bars or more — so
// there is nowhere to start, and the shape has to be solved in one piece rather than assembled
// from parts.
//
// The nine lengths are read off a drawing that works.  They have to be: pick nine numbers at
// random and there is generally no arrangement of six joints that achieves them.

point k0 hint(x: 0, y: 0)
point k1 hint(x: 30, y: 4)
point k2 hint(x: 58, y: -2)
point k3 hint(x: 6, y: 26)
point k4 hint(x: 34, y: 32)
point k5 hint(x: 62, y: 24)

line datum(k0, k3)

distance(k0, k3) == 26.683328
distance(k0, k4) == 46.690470
distance(k0, k5) == 66.483081
distance(k1, k3) == 32.557641
distance(k1, k4) == 28.284271
distance(k1, k5) == 37.735925
distance(k2, k3) == 59.059292
distance(k2, k4) == 41.617304
distance(k2, k5) == 26.305893

// the framework is rigid but free to move as a whole; these two settle where it sits
horizontal(datum)
ground(k0)
