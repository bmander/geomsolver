// Three lengths that cannot be a triangle.
//
// Three points, with a distance given between each pair: one, one, and ten.  By the counting
// that is exactly right — three pairs, three numbers, nothing left over and nothing missing —
// so the drawing ought to be completely determined.
//
// It is not, because no triangle has sides 1, 1 and 10.  The two short sides together cannot
// reach across the long one.  Counting statements tells you whether there is *enough*
// information; it can never tell you whether the information is *possible*.
//
// So the drawing reports as fully determined and the solve has nowhere to go, and holding those
// two facts at once without pretending either away is what the case is for.

point a hint(x: 0, y: 0)
point b hint(x: 10, y: 0)
point c hint(x: 5, y: 5)

line ab(a, b)

a distance(10) b
b distance(1) c
a distance(1) c
horizontal ab

ground a
