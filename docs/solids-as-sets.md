# Solids as sets (#737): experiments, decisions and measurements

This is the record #737's P0 asks for: what was written by hand with the words that existed,
what was missing or refused, how each of the epic's open questions was decided, and what the
rewritten corpus costs against today's. The rules themselves are in the contract files
([sets.md](contract/sets.md), [solids.md](contract/solids.md),
[generating.md](contract/generating.md)) and the primer's §1.14. #783's P0, which writes these sets
in the notation of sets, is recorded beside this one in [sets-in-notation.md](sets-in-notation.md).

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
(`flatten/said.rs`, a family's instance or a set written by hand alike, #744), never a word's name.

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

The corpus (`rust/examples/**/*.sv`, 274 documents) was run through `solventc` before the epic
(`edd4d70`, the old spellings) and after it (the rewritten corpus,
`scripts/solids_as_sets.py`): each document's report, its STEP assembly (`--step-assembly`) and its
STL (`--stl`), every output hashed and timed, the two runs in turn on one idle 4-core machine.

| output | identical bytes |
|---|---|
| report | 272 of 274 |
| STEP assembly | 272 of 274 |
| STL | 274 of 274 |

What differs, and why:

- `solid_sweeps/slot.sv`: a warning's line number, one further down for the `use std.solids` line.
- `engine.sv`: the report prints hidden statement keys (`camshaft_i.#1073742761.0.ghost.spine`),
  renumbered because the new module and the statements written in place take statement ids; the
  numbers are the same.
- `engine.sv` and `engine/crankshaft.sv`'s STEP: the crankshaft's ring gear, a generating cut,
  differs in the last digit of some coordinates (`-3.2338681033288323` against
  `-3.233868103328894`) — the same renumbering reaching the order the generating sweep reads its
  pieces in. Its STL is identical.

Every other document's solve, report and exports are the old spelling's to the bit: the
flattener writes a `std.solids` word out as the sweep it says (`flatten/said.rs`), so nothing is
added to the solve — no probe point, and no frame for a revolution (`under:` names its line).

**Cost.** 1260.4 s before, 1245.8 s after (−1.2%: the report 424.9 → 427.0 s, the STEP 469.3 →
471.1 s, the STL 366.2 → 347.8 s), within the run-to-run spread of these documents. The largest
regression of one document is `sized_conrod.sv`, 558.8 → 575.0 s (+2.9%), and the largest gain
`engine/conrod.sv`, 90.2 → 80.2 s; none is attributable to the rewrite beyond that spread. A set
written by hand that says a prism, a drafted prism, a revolution about a line or a prism through
everything is read by the flattener as the word is (#744), so it adds nothing to the solve either;
one only the elaborator recognises (a turn under a frame's motion, a number reading a formal under
arithmetic) adds the region solid's probe point (three fixed parameters) and is recognised from
its terms (`solid/defined.rs`). `tests/solids_as_sets.rs` holds each to the word's bytes. No
document of the corpus writes such a set by hand, so #744 leaves the comparison above as it was.

## What is not done

- **Two syntax-level recognisers remain** (#744's interim step): `flatten/said.rs`, which reads a
  set's statements and trees, and the parser's `swept_set`, which rewrites a set saying what a body
  occupies under a motion (`inside(under: …, over …)`) or a face over a running parameter
  (`coincident(over s)`) into a `sweep(…)` before anything is parsed. Moving the latter into
  `said` mints a set statement for each such solid, renumbering the statement ids the report
  prints in hidden keys, so it waits on a corpus comparison of its own.

- **Sketching on the faces of a revolution, a face family or a motion's sweep**, and on a drafted
  side, is still E040: the carriers a drawn point or plane stands on are written for a prism's caps
  and straight sides only (`flatten/faces.rs`). It is now said as the evaluator's limit, not a rule
  of the language.
- **A union of regions** (`outside` of what is outside several) is read point by point; the
  meridian fold takes no complement yet, so two coaxial balls' union is not exported exactly.
  A union of whole solids is the body term, exactly.
- **`cut`, `union` and `common`** keep their built-in statement form; their meaning is now the set
  their statements say, but they are not defined as words of `std` over a lower primitive.
