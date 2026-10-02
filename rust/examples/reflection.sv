// The law of reflection: a ray leaves a mirror at the angle it arrived at.
//
// A source `s` and a target `t` stand above a flat mirror `m`, each at a stated height and
// station along it.  The ray from `s` strikes the mirror at `p` and goes on to `t`, and the one
// thing said about `p` — besides that it is on the mirror — is that the two angles there are
// equal:
//
//     incoming angle(m, outgoing) m
//
// reads "the angle from `incoming` to `m` is the angle from `m` to `outgoing`".  An angle is
// normally stated as a number in `angle`'s parentheses; stating it as another pair of lines
// instead makes the two equal, with no number for anyone to work out.  Both are read the way
// `angle` reads one — counter-clockwise from the first line's direction to the second's — so the
// equality is directed, and `sense: cw` would make the second the mirror image of the first.
//
// That places `p`, and nothing else is needed.  The last three lines *claim* the classical proof:
// reflect the source in the mirror, and the image, `p` and the target lie on one line.  The
// diagnosis judges the claim a theorem — true, and adding nothing the equal angles had not
// already said.  Drag the source or the target (or edit a number) and it stays one.

m1 := point
m2 := point hint(x: 100, y: 0)
m := line(m1, m2)
horizontal m
m1 distance(100) m2
fix(x == 0, y == 0) m1

// the source and the target, each a station along the mirror and a height above it
s := point hint(x: 10, y: 40)
t := point hint(x: 90, y: 20)
m1 distance(10, along: x) s
s distance(40, side: left) m
m1 distance(90, along: x) t
t distance(20, side: left) m

// the ray, and the one statement that places where it strikes
p := point hint(x: 50, y: 0)
p on m
incoming := line(s, p)
outgoing := line(p, t)
incoming angle(m, outgoing) m

// the image of the source in the mirror lies on the line from the strike to the target
image := point hint(x: 10, y: -40)
s symmetry(m) image
sight := line(image, t)
claim p on sight
