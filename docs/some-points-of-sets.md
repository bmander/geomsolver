# Some points of sets: the experiment

`some` binds points of sets ([0.212], [changelog](solvent-changelog.md),
[contract](contract/sets.md) "Some points of sets"): in a set's body `some q in A, r in B: p in X`
is the points `p` some point of each set puts in `X`. This note is the experiment the design bible
asks of new syntax (principle 1, "measure, do not assume"): the cases it was made for, written
before and after, what reading them costs, and what it leaves for later.

## Why not a hull operator

The bench describes `offset/tetrahedron-grown` as `conv{a, b, c, d} ⊕ B₁`. A word `conv` would
have been new grammar for one concept. A projection is not one concept: a sweep (`some t: p in X
at(under: m) t`) is already a point bound over an interval, so letting `some` bind a point of a set
gives the language one existential where it had several readings of one. The hull then needs no
word: a point is in the hull of a set where it lies on a segment between two points each on a
segment between points of the set (Carathéodory, in three dimensions), so the hull of two convex
sets is their **join**, the segments between them, and every hull of finitely many points is a join
of joins of segments.

## The cases

| case | before [0.212] | with `some` |
|---|---|---|
| a tetrahedron (`offset/tetrahedron-grown`'s body) | four planes through triples of corners, each turned by the order of its two lines, and the half-space behind each | `some q in Segment(a, b), r in Segment(c, d): p in Segment(q, r)` |
| a pyramid on a drawn face (`fillet/rectangular-pyramid-all-edges`'s body) | a prism cut by four half-spaces through the apex | `some q in base: p in Segment(apex, q)` |
| the square-to-octagon transition (`loft/square-to-octagon`) | a face family of eight ruled walls on Gauss–Lobatto stations, or the hull of twelve points no word said | the join of a square and an octagon, each written as one: joins nest |
| a capsule | `p distance(<= r) Segment(a, b)` | `some q in Segment(a, b): p distance(<= r) q`, the same set |

Each reads to its closed form, built exactly (`tests/joins.rs`): the tetrahedron 72 and 72√3, grown
by 1 to the bench's 249.53323018738044; the transition 33760 and 2256 + 624√26 + 160√22, ten faces;
the pyramid 960.

## What it costs

`joins::a_joins_field_is_measured` (ignored; `cargo test -p gcs-core --test core --
joins::a_joins_field_is_measured --ignored --nocapture`), the tetrahedron as a join and as its
four half-spaces, on this session's container (load 2.8):

| | field made | a point read | built exactly |
|---|---|---|---|
| a join | 0.31 ms | 0.032 µs | 1.7 ms (the kernel's hull) |
| half-spaces | 0.07 ms | 0.036 µs | 8.2 ms (four Booleans) |

A join's field is the hull's planes, so a point costs what the half-spaces cost; the hull is found
once where the field is made. Built exactly it is one node with no Boolean, five times faster.

## How it is read

A join is read where each set it is of is the hull of finitely many points — a point of the drawing,
a set of one point, a segment, a convex polygon drawn as a face, or another join — and its field is
the hull of all those points, by the kernel's quickhull on the exact orientation test. That is the
signed distance within and a lower bound on it without, as any common of half-spaces is; it is
exact, not approximated, because the join of convex sets *is* their hull.

The proposal before the experiment read joins by support functions (the field as the most the
point stands past the set's reach in any direction), which also reads curved operands — a disc's
join with a point is a cone, a square's with a circle the bench's `loft/square-to-round`. The
experiment took the narrower road first: every case above is a hull of points, and a hull of points
has an exact field and an exact boundary the kernel already builds. The support-function reading
is the next step for curved operands; until then a join of a set bounded by a curve is refused by
name.

## Taken for a spin

Is this a description of one solid or a language? After the cases above, the same syntax was
tried on solids it was not made for, each through `solventc` and each checked against an
independent closed form or hull (`tests/joins.rs`):

| written | read as | |
|---|---|---|
| `component Join(A: set, B: set) := { p \| some q in A, r in B: p in Segment(q, r) }` | a word: `solid(Join(ab, cd))` | 72, the tetrahedron |
| `Join(sides, axis)`, `sides` a square as a join | an octahedron | 1333.33, 8 faces |
| a square on its corners joined to a smaller square turned a quarter, 12 above | a prismatoid | 1696, 10 faces |
| a triangle joined to the same triangle moved aslant (each a segment joined to a point) | an oblique prism | 400 |
| `some q in box: p in Segment(peak, q)`, `box` a solid | a house: walls and roof in one | 2080, 9 faces |
| `some q in box: p distance(<= 2mm) q` | the box rounded (`Grown`'s set, so built exactly) | Steiner's, 26 faces |
| `some q in tet: p distance(<= 1mm) q`, `tet` a join | the bench's `conv{a,b,c,d} ⊕ B₁` as written | 249.53323018738044 |
| two joins, one `union` the other | the stella octangula | 108, 24 faces |
| the hull of a cube and four cap points, sets written in place across lines | a twelve-faced gem | 10157.379, checked by a hull computed apart |
| a pyramid whose apex stands `h` over a base `w` square | follows its dimensions | 1600; `--set h=24mm` 3200; `w=30mm` 3600 |
| a pyramid cut from its tetrahedron | a Boolean of joins | 54 |

The spin found three faults, each fixed: a quantifier broken across lines did not parse (it now
runs on past a line break after a binder's `,` or its `:`); a union of joins said with `or` had no
bounds (a join's set, turned round inside a union, is now bounded by its own terms); and a named
instance of a component binding points split its binders off to the instance (a body binding any
is now the set's whole). It found one limit of the language that predated `some`, since lifted ([0.213]): a component's
`set` formal took a named set, not one written in place, so `Join(std.sets.Segment(a, b), …)` was
E103 and every set a word was given needed a line of its own. A set literal or a call standing
whole as the argument is now the set an anonymous instance of it is, one inside another's alike:
`solid(Join(Join(std.sets.Segment(s0, s1), std.sets.Segment(s2, s3)), std.sets.Segment(top,
bottom)))` is the octahedron.

What it cannot say yet, and says why: a cone or the bench's square-to-round loft (a join of a set
bounded by a curve), the join of a solid that is not convex, a union said with `or` built exactly
(read by the field, as a union of balls is), and a continuous sweep of a join exported (read by
the field, as any body swept continuously is).

## What is refused, and left

- A relation that is neither the segment between bound points nor a ball about one: the field
  would be the least, over every bound point, of the relation's own field — a search in as many
  dimensions as the points bound (a segment one, a face two, a solid three), certified as the
  sweeps' one-dimensional search is (`interval::minimum`). Refused by name until that search exists.
- A join of a set that is no hull of finitely many points: a polygon turning both ways (the hull of
  its corners is another set), a set bounded by a curve (above).
- A `some` whose set's point is a drawn point: no row says which point of the set it would be.
- `not some` of points, and one `some` inside another (`some q in A, r in B:` binds both).
