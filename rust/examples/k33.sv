// K3,3 as a bar framework: rigid, and with no triangle anywhere in it.
//
// Six nodes in two groups of three, and a bar from every node of one group to every node of the
// other — nine bars, which is exactly `2 × 6 - 3` and so minimally rigid.  What makes it the case
// the decomposition is read on is that it contains **no triangle at all**: the cluster vocabulary
// builds rigidity out of pairs and triples, and here there is no triple to start from, so no
// sequence of merges reaches the answer and the whole framework has to be taken as one core.
//
// The nine lengths are what the drawn positions come to.  They are stated rather than measured
// because a bar framework *is* its bar lengths — but they have to be a set some placement really
// achieves, so they are read off one, and nothing else about the numbers matters.

point k0 hint at (0, 0)
point k1 hint at (30, 4)
point k2 hint at (58, -2)
point k3 hint at (6, 26)
point k4 hint at (34, 32)
point k5 hint at (62, 24)

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
