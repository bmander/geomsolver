# Solids as sets (#737): experiments, decisions and measurements

This is the record #737's P0 asks for: what was written by hand with the words that existed,
what was missing or refused, how each of the epic's open questions was decided, and what the
rewritten corpus costs against today's. The rules themselves are in the contract files
([sets.md](contract/sets.md), [solids.md](contract/solids.md),
[generating.md](contract/generating.md)) and the primer's §1.14.

## P0 — what the language could not say before

Each was tried against `solventc` as it stood at `edd4d70`, over a 60 × 40 rectangle `sec`
in `std.front`.

| tried | what came back |
|---|---|
| `solid({ p \| p inside sec })` | E001 *expected a name*: `solid(…)` took no set written in place |
| `R := { p \| p inside sec }`, `solid(R)` | E040 *`coincident` does not relate a point to a face*: a drawn face was no set operand (`inside` is `coincident(<=)`) |
| `{ p \| p inside sec; p distance(<= 0mm, along: n) std.front; … }` | the same: the face had no reading at a region's point |
| the box as six `std.sets.HalfSpace`s along three drawn lines | builds (a region in general position: the `bounded` route) — but no face of it is named `near`, `far` or by an edge, and a revolution has no such spelling at all |
| a revolution as a set | nothing to write: no term read a point *turned* into a face's plane, and the only existential over a parameter was the developables' `coincident(t in (0, 1))`, a place on a curve |
| a union, `{ p \| p outside { q \| q outside A; q outside B } }` | E040 *where several things hold, and what is outside it is where any one fails: a choice of root* |
| `{ p \| p inside(t in (0deg, 90deg)) sec }` | E001: `inside` took nothing in its parentheses |

So the missing constructs were: a face as a set operand (P1), a set written in place in
`solid(…)`, the existential over a running parameter in a set's body (P2's revolution and all
of P4), and a meaning for the outside of a conjunction (P3).

## Decisions on the open questions

**The rule by which a set's bounds name the faces of its solid.** A section's sides are named by
the edges they are swept from; of two ordinates along its normal the greater bounds `near` and
the lesser `far`; a turn short of a whole one is capped `start` where its interval begins and
`end` where it ends. A hand-written set obeys it because the faces are named by what the set is
recognised as, and recognition reads the set's terms (`solid/defined.rs`) or its statements
(`flatten/said.rs`), never a word's name.

**A union of regions.** The outside of a conjunction is given its meaning — where any of its
statements fails — rather than adding syntax: `p outside S`, `S` a set no one bound of which
turns round, is the complement of what holds at `S`'s own point (`RegionShape::Outside`), so
`{ p | p outside { q | q outside a; q outside b } }` is `a` and `b` together. A union of whole
solids is built as the body term; a union of regions is read point by point.

**The existential over a parameter in a set body.** Written as readings already write a running
number: `over t in (a, b)` (the interval in place) or `over s` (a declared parameter's), in the
parentheses of `coincident` or `inside`. `p coincident(under: m, over t in (a, b)) sec` is what a
face occupies under a motion at some roll of the interval; `p inside(under: m, over t in (a, b))
tool` what a body does; with no `over` at all, the whole roll, bounded by the body it is cut
from; `p coincident(over s) Station(…).f` a face family. `under:` names a frame's motion, or a
line — the turn about it.

**`cut`, `union` and `common`.** They stay the body words, now with a meaning as sets: the body is
the points inside its stock or anything in union with it, outside everything cut from it and
inside everything in common with it — a formula, not an order of steps — and a different set is
written as the set it is (`{ p | p inside shell; p outside pocket }`), which builds as the body
term over the same solids. Defining the words themselves in `std` needs a primitive below them
(a body's set extended by statements written elsewhere); that is not done here.

## Measurements

The corpus (`rust/examples/**/*.sv`, 274 documents) was run through `solventc` before and after
the rewrite — its report, its STEP assembly and its STL — and each output hashed
(`scripts/solids_as_sets.py` rewrote it). The figures are in the pull request that landed this
and are restated here once the run is final.
