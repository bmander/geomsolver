# Solvent: a primer for agents

Solvent is the language a drawing in this project is written in. You do not place geometry: you
declare entities, state what must be true of them, and the solver finds coordinates satisfying
every statement at once. A sketch is a set of facts, not a sequence of commands, so reordering its
statements never changes its meaning.

**The one rule: a number inside a `hint(…)` clause is a seed, and every other number is not.** A
seed is where the solve starts, and the solver may move it. Every other number is something the
solver must satisfy and never rewrites, so deleting every seed leaves the set of solutions
unchanged. Your job is to state enough true things that exactly one drawing satisfies them, then
check that the diagnosis agrees (1.16).

| form | meaning |
|---|---|
| `p := point hint((30, 10))` | start the point here; the solver may move it |
| `a distance(40) b` | require the distance to be 40 |
| `fix((0, 0)) a` | hold `a`'s own numbers at these values |
| `w := 40` | a value: arithmetic done while elaborating, never an unknown |
| `param a: Length` | an unknown the solve answers for (1.6) |
| `claim vertical rail` | check a consequence without enforcing it (1.11) |

**A point stands in space unless it is drawn `in` a plane.** A 2D drawing says `use std` and puts
its geometry inside `in std.front { … }` (1.12). Values and solids may stand outside the block; a
`param` must.

- Models are `.sv` files: geometry, constraints, hints and claims only. Presentation — `class`,
  `style`, views, sections, callout placement — belongs in a
  [Solvent Drawing (`.svd`)](solvent-drawing.md). A dimension is a constraint in the model; its
  callout is the drawing's.
- This primer is what the implementation accepts, and every quoted report came from `solventc`.
  [`solvent-spec.md`](../solvent-spec.md) is normative but specifies constructs that do not parse
  yet (`hint` as a statement, `path`). Section 1's examples are fragments unless they show a
  report; section 2's are complete.

---

## 1. The language

### 1.1 Lexical

```
program     ::= statement*
statement   ::= one of the forms in 1.3, ended by a newline or ';'
comment     ::= '//' ... end of line  |  '/*' ... '*/'
identifier  ::= [A-Za-z_][A-Za-z0-9_]*
number      ::= decimal, exponent form allowed; `3 1/2` is a mixed fraction;
                a unit may follow: 80mm  45deg  0.5rad  6"  1' 6 3/16"   (1.6)
```

A newline ends a statement except inside brackets, after a trailing chain joint (1.7), and at a
`}` closing a block. `@` is shorthand for `coincident` (`p @ c`, `a @(t == 0.4) s`) and `~` for
`hint` (`p := point ~((0, 0))`); a solve writes seeds back inside `~(…)` as written.

### 1.2 Seeds, constraints and values

| written | kind | meaning |
|---|---|---|
| `hint((0, 0))`, `hint(r: 25)`, `hint(t: 0.4)` | seed | where the solve begins; the solver may move it |
| `distance(80)`, `angle(30deg)`, `t == 0.4`, `fix(r == 25) c` | constraint | must hold; never rewritten |
| `w := 100`, `param w := 100` | value | worked out while elaborating; never an unknown |

**Brackets say what a thing is made of; `hint(…)` says where its solve begins.** Write
`c := circle(center: o) hint(r: 25)`, never `circle(center: o, r: 25)`: the centre is structure,
the radius a guess.

**A held number is stated by `fix`, never by a seed**, and is its own seed: `a := point` beside
`fix((0, 0)) a` needs no `hint`. A `hint` keeps only what no `fix` holds (`hint(y: 7)` beside
`fix(x == 3) a`).

**Seeds** go in the trailing clause, whole or by member, keys in any order:
`p := point hint((0, 0))`, `a := arc(center: c, start: s, end: e) hint(r: 5)`. An omitted
coordinate is 0; an omitted radius is computed from the geometry, never 0. A point with no clause
starts off the origin and apart from other unseeded points, and a solve writes the pose it reached
back as its clause. A point its constraints place without a choice (a midpoint, where two lines
cross) needs no seed at all.

**A seed may read other seeds, never solved values**: `hint((k.center.x + k.r, pin.y))`. Naming a
place needs no arithmetic:

- `hint(at: pin)` starts where `pin` starts; `hint(at: k, bearing: 90deg)` on circle `k`'s rim;
- `hint(at: a, toward: b, by: 0.5, turn: 90deg)` steps from `a` the fraction `by` (1 if unsaid) of
  the way to `b`, turned `turn` about `a` (`by: -1` is a reflection); `along: l` in place of
  `toward:` steps by `by` times line `l`'s run;
- `hint(at: P, (3, 4))`, `P` a plane, is in `P`'s own coordinates (1.12).

A place drawn in another view is read in space and projected into the seeded point's view. A point
in space is seeded `hint((x, y, z))`. Seeds settle in statement order, so one reading a seed written
below reads its provisional start. Under a `unit` line a geometry read is a length: write
`pin.x - 10mm`. A value may **not** read geometry, since a seed must never change what a document
says. An unbound numeric formal (1.8) reads as 0 in a hint.

### 1.3 Statement forms

**A name is defined one way: `NAME := VALUE`** — a number, an element, a chain, an instance, a
group, a set or a curve. `(NAME := VALUE)` is itself a value, so a chain link is named where it
stands: `(ab := line(a, b)) -> …` (`:=` binds loosest, hence the parentheses). A `label:` never
defines a name; it fills a slot (an argument, a child, a hint key, a group member). A dimension's
number defines no name.

```
use NAME[.NAME...] [(NAME, ...)]        import a module                              (1.9)
unit NAME                               the unit the document's numbers are in       (1.6)
NAME := EXPR                            a value                                      (1.6)
param NAME[: TYPE] [:= EXPR] [hint(E)]  an input; with no value, an unknown          (1.6)
NAME := {LABEL: VALUE, ...}             a group: values and geometry as one argument (1.8)
NAME := (E, E[, E])                     a vector, read as NAME.x, NAME.y             (1.4)
[private] [construction] [NAME :=] KIND[(CHILD | hint(...), ...)] [hint(MEMBER: E, ...)]
     [knots [...]] [weights [E, ...]] [in REF[, REF]]   an entity; every part optional (1.4)
NAME := point(x: XEXPR, y: YEXPR)       a computed point, drawn only as a curve      (1.10)
[NAME :=] axis [hint(dir: (E, E, E))]   a directed line in space                     (1.12)
[NAME :=] plane[(u: R, v: R)]           a plane over two axes or lines               (1.12)
in REF { statement* }                   every declaration inside is drawn in REF     (1.12)
NAME := { p | statement* }              a set: the points satisfying the body        (1.13)
NAME := CHAIN                           a chain joined by `->`, named as a traversal (1.7)
NAME := face(EDGE, ..., holes: LOOP)    a planar region                              (1.14)
NAME := solid(FACE, SWEEP...)           a face swept: depth:/from:/to:/through:, about:, along:
NAME := solid(SOLID) | solid(SET)       a body over a stock; the points of a region
NAME := solid(SOLID, under: MOTION, at: E | from: E, to: E)   placed by, or swept through, a motion
REF union REF | REF cut REF | REF bound REF   material added to, removed from, kept within a body
NAME := motion | envelope | surface | patch | seam | vertex | edge(...)              (1.15)
WORD[(ARGS)] REF | REF WORD[(ARGS)] REF [hint(SLOT: E, ...)]   a constraint          (1.5)
claim CONSTRAINT                        judged, never solved for                     (1.11)
fix((E, E[, E])) REF | fix(MEMBER == E, ...) REF   hold an entity's own numbers      (1.4)
ccw(a, b, c) | cw(a, b, c)              record a root choice; adds no equation       (1.5)
K minimizes E | K maximizes E           a free curve's energy                        (1.10)
[NAME :=] Component(ARGS) [in REF]      an instance                                  (1.8)
component NAME(FORMALS) { statement* }  a component definition (not a value)
a WORD(PARAMS) b := RELATION | { ... }  a relation word                              (1.9)
repeat N [as i] { ... }                 N unrelated copies                           (1.7)
cycle N [as i] { ... }                  N copies that close; `next`, `prev` in scope
ring N about C { ... }                  N copies, each the first turned about C
repeat e in CHAIN [as i] { ... }        a copy per link of a named chain (`cycle e in` closes)
repeat a in SOLID [if ...] { ... }      a copy per face of a solid                   (1.14)
when COND { ... }                       made once where COND holds, else not at all  (1.7)
LHS <= RHS | LHS >= RHS                 in a set's body: a bound on measurements     (1.14)
NAME := INSTANCE.POINT over FORMAL in (A, B)   a curve                               (1.10)
NAME := Component(ARGS).POINT over FORMAL in (A, B)
preview { statement* }                  added when the file is opened, not used      (1.9)
```

A reference is `name`, `name.field` or `name[expr]` (a copy of a repeated statement; the index may
read any number or binder in scope). They chain: `l.e[2].p1` is `p1` of copy 2 of `e` inside
instance `l`.

### 1.4 Entities

| kind | children | own numbers | ends (for chains) |
|---|---|---|---|
| `point` | — | `x`, `y` in its plane; `x`, `y`, `z` in space | |
| `line` | `p1`, `p2` | | `p1 -> p2` |
| `circle` | `center` | `r` | |
| `arc` | `center`, `start`, `end` | `r` | `start -> end`, counter-clockwise |
| `spline` | control points, all named | | |
| `curve` | its two ends (a free curve) or its instance's arguments | a free curve's length | |
| `axis` | — | `dir`, `origin` | |
| `plane` | `u`, `v` (axes or lines); member `origin` | `origin` | |

**Members and vectors.** An entity's numbers are its members. A vector is `(a, b)` or `(a, b, c)`.
A point *is* its place, so it is seeded and held whole by a vector — `hint((3, 4))`,
`fix((0, 0)) p` — or by one of `x`, `y`, `z`; an axis has the vectors `dir` and `origin`, a plane
`origin`, read by member as `dir.x`, `origin.z`: `fix(dir == (0, 0, 1)) t`,
`fix(origin == (0, 0, 12)) back`. A vector held whole is as long as what it holds, so
`fix((0, 0)) p` cannot leave a point in space's height free unsaid (E105; `fix(x == 0, y == 0) p`
says so). A number is held once: two `fix`es at different values, or `ccw` and `cw` of one
triangle, are E031 at both. `o := (10mm, 20mm)` names a vector, read as `o.x` and `o.y`.

**Children** go by position or by label; a label skips earlier slots (`l := line(p2: c)` leaves
`p1` for a chain to thread). A slot may be left out, which mints an anonymous point, or hold a
`hint(…)`, which mints a seeded one (with no `z`: a line in space names declared points):

```
l := line                                  two points: l.p1, l.p2
c := circle hint(r: 25)                    an unnamed centre, a seeded radius
a := arc                                   a.center, a.start, a.end
l := line(hint((0, 0)), hint((60, 20)))    two points, seeded
alt_a := line(A, hint((15, 5)))            one named end and one not
```

**The dotted path is the name.** `l.p1` is an ordinary point: it constrains, drags and takes
dimensions. Name a point yourself when several statements mention it. The element's own name is
optional too: `line(p1, p2)` and `arc(center: c)` are complete statements, and if something later
must reference one, the app splices a name in.

**Splines** need declared, named control points: `s := spline(k0, k1, k2, k3)` (`s := spline` alone
is an error). `weights [1, w, w, 1]` makes one rational — document data like `knots`, never solved
for, one positive plain number per control point (else E103). With `w = (1 + sqrt(2)) / 3` and the
inner points where a quarter circle's end tangents meet the cubic's thirds, the curve *is* that
quarter circle (`nurbs.sv`); every contact reads it as it is, and a solid swept from it is exact.

A **free curve**, `rope := curve(a, b)`, runs from `a` to `b` with a shape its energy settles
(1.10).

### 1.5 Constraints

**Every constraint is a prefix or infix operator.** The word stands before its one operand or
between its two; everything else — the number, a selector, a third entity — goes in parentheses on
the word:

```
horizontal line1                    point1 level(up) point2
radius(25) circle1                  point1 distance(80) point2
distance(6) line1                   point1 symmetry(line1) point2
fix((0, 0)) p1                      l1 angle(30) l2
fix(r == 25) c                      line1 tangent(side: left) circle1
length(40) arc1                     l1 angle(l3, l4) l2
```

One word may name several constraints, told apart by the operands' kinds, by fixity, or by what
stands in the parentheses. **Operand order carries meaning**: `arc tangent line` is a tangency at
the arc's end, `line tangent circle` the ordinary one.

| word | fixity | operands and options |
|---|---|---|
| `coincident` | infix | two points; a point to a line, circle, arc, spline or curve; in space, a point, line or axis to a plane, a point or line to an axis, two axes; a point to a set (1.13) |
| `distance` | infix | two points (length; with `along:`, an **ordinate**: signed from the first to the second along an axis, a line, or a view's word `x`/`y`/`right`/`left`/`up`/`down`); a point and a line, or two lines (a magnitude, `side:` picks the side); two concentric circles or arcs; a point and a plane (`along: u`/`v`/`n`); two planes (along the first's normal) |
| `distance` | prefix | a line: its length |
| `tangent` | infix | line–circle/arc (`at: p1`/`p2` for tangency at that end; `side:` the centre's side); circle/arc–circle/arc (`external: true/false`); arc–line (`at: start`/`end`); spline or curve–line; a line or set to a set (1.13) |
| `equal` | infix | two lines (length) or two circles/arcs (radius) |
| `curvature` | infix | spline or curve and a circle/arc: the osculating circle there |
| `horizontal`, `vertical` | prefix | a line. Between two points they are `std`'s words (1.9): `use std (horizontal)`, then `a horizontal b` |
| `level(dir)` | infix | two points with the same ordinate along `dir` (an axis, a line, or `up`/`right`); a point and a plane with `u`/`v`. Write it rather than `distance(0, along: …)`, which is refused |
| `angle` | infix | two lines, directed (below); in space, an axis and a line or axis, unsigned. `l1 angle(l3, l4) l2` equates two angles |
| `radius` | prefix | a circle or arc |
| `length` | prefix | an arc (radius × counter-clockwise sweep); a spline or free curve (its whole length) |
| `symmetry(line)` | infix | two points |
| `midpoint` | infix | a point and a line |
| `parallel`, `perpendicular` | infix | two lines; an axis and a line or axis; an axis and a plane; `parallel` two planes |
| `project` | infix | two points in two planes: two images of one point in space (1.12) |
| `fix` | prefix | an entity's own numbers, whole or by member (1.4) |
| `ccw(a, b, c)`, `cw(a, b, c)` | call | `c` left (right) of ray `a → b`: a root choice, no equation |

#### Which way: directions are words

| written | means |
|---|---|
| `p distance(12) ax` | 12 from line `ax`, **either side**; the seed picks |
| `p distance(12, side: left) ax` | left of `ax`'s direction `p1 → p2`; `right` likewise |
| `l1 distance(6, side: left) l2` | `l2`'s `p1` lies left of `l1` |
| `a distance(60, along: x) b` | `b.x − a.x = 60`; `along: y` is the rise; `right`, `up`, … say it as a word |
| `a distance(60, along: t) b` | `(b − a)·t̂ = 60` along axis or line `t` |
| `a level(t) b` | level along `t`; `std`'s `a horizontal b` is `a level(up) b` |
| `l1 angle(30) l2` | 30° **counter-clockwise** from `l1`'s direction to `l2`'s |
| `l1 angle(30, sense: cw) l2` | 30° clockwise: `angle(-30)` said openly |
| `l1 angle(l3, l4) l2` | angle `l1 → l2` equals angle `l3 → l4` |
| `l1 angle(l3, l4, sense: cw) l2` | it equals the **mirror image** of `l3 → l4` |
| `length(40) a` | 40 along arc `a`, counter-clockwise from `start` to `end` |
| `l tangent(side: left) c` | the circle's centre is left of `l` |
| `p distance(5, along: n) P` | `p` is 5 along plane `P`'s normal, in space |

- **A distance from a line is a magnitude**: a negative value is refused and the side is `side:`. A
  component that must work either way takes a `Side` formal (`s: Side`, called `Part(…, s: right)`)
  and writes `side: s`. Where a sign is arithmetic, use a signed ordinate.
- **Runs, rises and angles keep their signs**, because components compute them.
- **`angle` is directed**, counter-clockwise positive over the full turn, so it fixes the side as
  well as the tilt; swapping the lines or reversing one changes the reading. An angle equal to
  another angle is one equation with no unknown: a bisector is `ab angle(ad, ac) ad` (2.16).
- **`length(L) a`** measures an arc counter-clockwise in `(0°, 360°]`, so go the long way round by
  going round counter-clockwise; a spline's by quadrature. It may read an unknown.

**Slots a constraint owns** (a contact's curve parameter, always `t`) are normally omitted. Seed one
with `p coincident s hint(t: 0.4)`, pin one with `p coincident(t == 0.4) s`. Pinned to an unknown,
contacts on one curve share it: with `param s: Angle hint(318)`, `path tangent(t == s) ground` and
`path curvature(t == s) osc` make the circle osculate the path *where* it touches the ground (a
`hint(t: …)` beside the pin, or a contact on another curve pinned to `s`, is E040).

**A point at a fraction of a line or an arc** writes the same `t` (0.79): `m coincident(t == 0.25)
g` is a quarter of the way from `g.p1` to `g.p2`, and on an arc `t` is the fraction of its turn
from `start` to `end` by angle. Without `t` the contact is plain `coincident` and owns nothing. A
line's `t` runs on past its ends (`t == 1.5`, an external division); an arc's is held to `[0, 1]`
and a pin outside it is E040, as is any `t` on a circle. One unknown may be the fraction of
several lines and arcs (`m coincident(t == s) g`, `n coincident(t == s) h`: similar triangles).
A point in another view or in space reads it in space, so a loft's station needs no length or
sweep passed in.

**Tangency trap.** If the contact point is already held on the circle, state the tangency *at* it:
`line tangent(at: p2) circle`, `arc tangent(at: start) line`. `p coincident circle` plus a bare
`line tangent circle` is rank-deficient at every solution (the diagnosis reports a motion "blocked
at second order").

#### Across views, a word means space

When a relation's operands are drawn in different views (1.12), or any stands in space, it relates
their places in space; no selector is needed:

| written across views | means |
|---|---|
| `a coincident b` | the same point in space |
| `a distance(30) b` | the true length |
| `a distance(5) l`, `l1 distance(17.5) l2` | to the infinite line; along the common perpendicular |
| `a coincident l`, `a coincident c` | on the line's extension; on the circle in its own view |
| `l1 angle(90deg) l2` | the **unsigned** angle, strictly between 0° and 180° (say `parallel` for 0°) |
| `parallel`, `perpendicular`, `equal` | directions, and true lengths |
| `a midpoint l`, `a symmetry(l) b` | the midpoint in space; a half turn about the line |
| `a distance(20, along: std.z) b` | how far apart along the axis, in space |

A word with no meaning in space — `horizontal`, a view's own `along: x` (name the axis instead,
`along: std.x`), `tangent` between drawn figures, the angle-equals-angle form — is **E062** across
views; `sense:` and `side:` there are **E040**. Radii mean the same in every view.

### 1.6 Values, unknowns and units

**Expressions**: `+ - * / ^`, parentheses, `pi`, `tau`, and `sqrt abs sin cos tan asin acos atan
atan2 exp ln log floor ceil round min max hypot`. **Trigonometry is in degrees.**

**Values.** `w := 60` is worked out while elaborating. A value may read others written anywhere in
its body or an enclosing one; one defined in terms of itself is E041. A dimension reads names and
defines none: write `w := 60`, then `a distance(w) b`; its callout reads `w`, and typing over it
edits `w`'s line.

**Inputs and unknowns.** `param` marks a number the document takes from outside, the one kind a
host may give another value. With no value it is an **unknown** of the drawing, tying together
everything that reads it:

```
param bore: Length := 50mm        // an input with a value; the type is optional
param beta: Angle hint(30deg)     // no value: an unknown, seeded; a solve writes the hint back
stroke := bore * 1.2              // a value
l angle(beta) m                   // reads the unknown
```

An unknown names its type (`Length`, `Angle` or `Scalar`). It may be read by a dimension, a
contact's place (`t == s`, 1.5) or a value over it. The tie must be affine in one unknown (`a`,
`a / 2`, `2 * a + 5`); `a * a`, `sin(a)` and two unknowns in one dimension are errors. **A name
nothing declares is E101**, so a misspelling is never a degree of freedom. `param` stands at the
top of a document (or its `preview`); a component's unknowns are the formals a call leaves unbound
(1.8). To equate two angles, prefer `l1 angle(l3, l4) l2`.

**Dimensions are checked.** Two base dimensions, length and angle. `*` and `/` derive, `+` and `-`
demand agreement, and the result is checked against its slot: `a distance(45deg) b` is an error. A
bare number is dimensionless and a *context* may accept it — `distance(80)` is a length because
the slot says so, `sin(30)` reads degrees — but a context does not speak for a second operand:
`90 / N + ivp` adds a plain number to an angle and is refused; write `90deg / N + ivp`.

```
unit mm                             // the document's unit
phi := 20deg
ivp := tan(phi) * 1rad - phi        // inv φ = tan φ − φ holds only in radians, and says so

a distance(80mm) b
c distance(1' 6 3/16") d            // one literal: spaces separate the parts, as in 3 1/2
```

- **Without a `unit` line** numbers are in unnamed drawing units and a suffix is refused.
- A name takes its dimension from its value's unit, not its use: `w := 80` used as a length does
  not make `w` a length; `w := 80mm` does, as does a formal declared `Length`.
- `3/16"` is a division; a lone fractional inch is `0.1875"` or `0 3/16"`.
- `pi` is dimensionless; `tau` and `turn` are one full turn; radians are written `* 1rad`.
- **There is no string literal**: `"` is the inch mark, and a word argument is bare (`at: start`).
- A value, input, formal or index named after a built-in (`tau`, `pi`, `min`, …) does not shadow
  it and ends up with two values: W112. Rename it.

**Measurements of the solved drawing** — `length(l)`, `radius(c)`, `distance(a, b)`,
`angle(l1, l2)` (0–180°) — are allowed **only** in a motion's `ratio:`, `phase:` and `advance:`
(1.15), which are read after the solve. Anywhere else they are **E107**: the number is needed
before there is a solve.

### 1.7 Chains and repetition

A chain writes a run of elements and the relations between them on one line.

```
CHAIN  ::= LINK (JOINT LINK)* ['->' INFIX* 'close']
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a one-operand constraint word         horizontal, vertical, radius(..)
JOINT  ::= '->' INFIX* ['->']  |  INFIX+ ['->']  at least one marker or word
INFIX  ::= tangent | equal | any two-operand constraint word
```

**`->` states that two links share a boundary point; nothing else does.** Threading runs left to
right (`p1 -> p2` on a line, `start -> end` counter-clockwise on an arc).

- `->` alone is a plain corner; `-> tangent` is a corner also tangent there (the regular at-form).
- The shared point may be named by either side (or both, agreeing) or by neither, and then the
  chain mints it: `(l1 := line) -> (l2 := line)` is two lines and three points.
- A joint may state several relations, `A -> equal angle(30deg) B`, the marker on either side.
- A word without a marker relates without welding: `(l1 := line(a, b)) perpendicular
  (l2 := line(c, d))` is two separate lines at a right angle.
- At a corner with an element declared elsewhere, the declared side names the shared point:
  `(t := line(p3, k.start)) -> tangent k`.
- `-> close` seals the loop back to the first link. Links may be anonymous:
  `line -> tangent arc -> tangent line` is a whole contour with no names.
- `equal` is polymorphic (length or radius); `a equal b equal c` is two statements.
- `profile := … -> close` names the traversal, which a solid can sweep (1.14); its edges keep
  their own names. Every joint in a named chain carries `->`.

**Repetition.** `repeat N { … }` makes N unrelated copies; `cycle N { … }` makes N copies that
close, with `next` and `prev` in scope. `as i` binds the index; copies are reached as `p[0]`.

**A block may end mid-joint.** A trailing joint before `}` threads onto the next copy's first link:
a `cycle` wraps, closing the loop with no `close`; a `repeat` leaves its last joint unstated, so
`repeat N { line -> angle(a) }` is an open polyline. `cycle 4 { (s := line) -> perpendicular equal
}` is a square but for size and pose (2.7).

**A block made or not.** `when COND { … }` makes its body once where the condition holds and not
at all where it does not, decided over the numbers in scope before the solve: comparisons (`<`,
`<=`, `>`, `>=`, `==`, `!=`) of two numbers of one dimension — `relief > 0mm`, never `relief > 0`
(E103) — joined by `and`, `or` and `not`. There is no copy: what the body declares keeps its own
name (`q`, `t.tip`, never `q[0]`), and is undefined where the block is not made. Refused: an
unknown of the drawing in the condition (E040), an index, a trailing joint.

```
unit mm
use std

relief := 0.2mm

in std.front {
  a := point
  when relief > 0mm {
    q := point hint((8, 3))
    a distance(10mm) q
  }
}
```

**A ring is a cycle whose copies are turns.** `ring N about C { … }` makes N copies, each the first
(the *representative*) turned `k · 360°/N` about `C`: a point (counter-clockwise in its view, the
body drawn in that view) or an axis whose direction is held (the body in space). Only the
representative is solved for — the copies own no unknowns and their relations are not stated
again — so a regular polygon is one vertex, one radius, one edge (`std.Polygon`), a gear one tooth
(`gear.sv`), and no `cos`/`sin` places a copy. The ring's own turn is gauged from outside:

```
use std (horizontal)

in std.front {
  hub := point
  ring 3 about hub {
    tip := point hint((10, 1))
    hub distance(10) tip
    chord := line(tip, next.tip)
  }
  hub horizontal tip[0]
}
```

A copy is an entity like any other: `tip[2]` may be dimensioned, dragged or made an edge, and a
relation on it moves the representative. Refused: reading the index in the body (E015); reaching
from inside to anything the turn would move — outside geometry but the centre, a value, a seed's
place — or to a copy by index rather than `next`/`prev` (E021); a ring in a ring (E022); a plane,
axis or motion declared in it, a point outside its view, or a hold on a copy (E023).

**Repetition over a chain.** `repeat e in CHAIN { … }` makes one copy per link of a named chain, in
walk order, `e` that copy's edge — an operand, a field (`e.p1`), an argument. The chain may be open
or closed and reached through an instance or group (`refs.pinion.profile`); `cycle e in CHAIN`
closes and is refused on an open chain. A hole at every edge's midpoint, sized by index
(`holes.sv`):

```
unit mm
use std
in std.front {
  a := point
  b := point hint((60, 0))
  c := point hint((60, 40))
  d := point hint((0, 40))
  fix((0, 0)) a
  outline := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  horizontal ab
  vertical bc
  horizontal cd
  vertical da
  distance(60) ab
  distance(40) bc

  repeat e in outline as i {
    m := point
    m midpoint e
    hole := circle(center: m) hint(r: 3)
    radius(2mm + i * 1mm) hole
  }
}
```

`solventc --where '#22.2.hole' holes.sv` (the block is statement 22; copy 2 is edge `cd`):

```
holes.sv: solved
  18 params, 18 equations, structural rank 18; DOF 0; 5 components: DOF 0, 0, 0, 0, 0; 2 rigid cluster(s) in the distance graph
  #22.2.hole.center.x = 30
  #22.2.hole.center.y = 40
  #22.2.hole.r = 4
```

A block's copies are one statement however many there are, so seeds inside are not written back
and a gesture on one copy is refused.

### 1.8 Components and groups

```
component Rung(a: point, b: point, len: Length) {
  e := line(a, b)
  horizontal e
  a distance(len) b
  mid := point                // reached from outside as t0.mid
}
t0 := Rung(l0, r0, len: 50)
```

- **Formals** are an entity kind (`point`, `line`, `circle`, `arc`, `plane`, …), a number type
  (`Length`, `Angle`, `Int`, `Scalar`, `Side`), a `group` or a `set` (1.13).
- **Passing an entity is aliasing**: formal and actual are one entity, so a boundary costs nothing.
- **Entities by position, numbers by label**: `Rung(l0, r0, len: 50)`, never `Rung(l0, r0, 50)`,
  and nothing positional after a label. Either mistake is **E004** at the argument.
- **An unbound numeric formal is an unknown** named under the instance (`c.theta`): a mechanism is
  drawn with its crank free so (2.10). Seed one in its place: `Crank(o, datum, theta: hint(30deg))`.
- **An instance's numbers are read by its name**: every numeric formal, bound or not, is
  `inst.formal` — `p distance(ball.r) ball.center` — and travels with an instance passed as a
  `group`. Values a body declares are not read this way.
- **Scope is closed.** A body sees only its formals and its own declarations: root values,
  geometry and standard datums must be passed in. Component definitions and built-in functions
  remain callable. Repetition blocks share the enclosing scope.
- **Everything an instance makes is public by dotted name** (`t0.e`, `five.s[0].p1`) unless
  declared `private`. `construction` marks purpose, independently: shown in the editor, hidden on
  sheets unless styled. A private entity may still be passed explicitly to another component.

- **A component may call itself**, directly or through others, when a `when` over the numbers it
  is given ends it (1.7): each level is an instance named under the last (`d.s.s.m`). A call given
  the numbers its own expansion was given never ends and is **E003**, as is nesting past 32 levels
  (`when` and blocks count). Recursion is for what is defined by itself — a subdivision; a run of
  alike copies is a `repeat`.

```
component Acute(a: point, b: point, c: point, l: Length, n: Int) {   // Penrose's half rhombs
  when n == 0 {
    line(a, b)
    line(a, c)
  }
  when n > 0 {
    phi := (1 + sqrt(5)) / 2
    p := point hint(at: a, toward: b, by: 1 / phi)
    a distance(l / phi) p
    c distance(l / phi) p
    x := Acute(c, p, b, l: l / phi, n: n - 1)
    y := Obtuse(p, c, a, s: l / phi, n: n - 1)                     // which calls Acute again
  }
}
```

A reusable pattern hiding its layout (`use hardware` provides this; see `solid_flange.sv`):

```solvent
component BoltPattern(body: solid, center: point, ref: line,
                      n: Int, pitch_r: Length, hole_r: Length, phase: Angle) {
  private construction layout := std.Polygon(center, ref, n: n, r: pitch_r, phase: phase)
  repeat n as i {
    hole := radius(hole_r) circle(center: layout.v[i])
    private drill := solid(face(hole), through: body)
    drill cut body
  }
}
```

Callers can name `pattern.hole[0]` but not `pattern.layout` or `pattern.drill[0]`. A component may
contribute a `cut` to a body it was handed: the feature owns the void it cuts.

**Groups** bundle values and geometry into one argument, written in braces after `:=`, across lines
if need be, nested by name or in place:

```solvent
unit mm
use std

sizes := {length: 20mm}
layout := {front: std.front, origin: std.origin}
component Bar(layout: group, dims: group) {
  in layout.front {
    tip := point hint((layout.origin.x + dims.length, layout.origin.y))
    ax := line(layout.origin, tip)
    horizontal ax
  }
  distance(dims.length) ax
}
bar := Bar(layout, dims: sizes)

design := {
  bar: {front: std.front, origin: std.origin},
  pin: {length: sizes.length / 2},
}
```

Numeric members keep their units; geometry members alias and add no solver state. An instance may
be passed as a group, exposing its geometry but not its local values. A group formal must be
supplied, and a missing member is an error. A traced component (1.10) needs fixed scalar or entity
formals, so pass members individually there.

**Derive, don't restate**: keep a few design inputs and derive the rest, so changing one carries
its dependents (the V-twin derives its cylinder, pockets and grooves from bore, throw and wall).

### 1.9 Modules, relation words and previews

```
use engine.dims          // engine/dims.sv beside the document or an ancestor, else the library
use std                  // datums (1.12), std.Turned, std.CenteredRectangle, std.Polygon,
                         // std.Hex, std.Ellipse, std.Sphere/Cylinder/Cone, rounds (1.14)
use hardware             // fasteners and fittings, in mm (needs a `unit` line)
use std (horizontal, vertical)        // and import those two words bare
```

A module is a Solvent document read for its components, relation words and top-level values and
groups; its own drawing is not drawn. **Nothing is imported bare unless the `use` names it**: a
module's names are written by full path — `engine.parts.Rod(…)`, `engine.dims.D` — or bare where
listed in brackets. Imports are per file, not transitive, with no prelude; a relation word is only
ever bare, so a file using one imports it, and `std.front` needs the file's own `use std`.
Errors: a missing module E070; an import the module lacks E101; a name imported twice, from two
modules, over the file's own, or over a built-in word E071; two components or words of one name in
a file E071. A module's own error is reported at the `use` that brought it in. `engine.sv` is the
worked case: a dimension module, a parts module and one module per part.

**Relation words.** A file defines a word at its top level: operands, parameters in parentheses,
`:=`, and one relation or several in braces with the geometry they need:

```
a horizontal b := a level(up) b                  // std.sv
a above(d) b := b distance(d, along: up) a       // r above(d: 7) q: r stands 7 above q
l1 flush(d) l2 := {                              // parallel, and d apart
  l1 parallel l2
  l2.p1 distance(d) l1
}
a apart(d) b := {                                // what the body declares, each use makes anew
  private ab := line(a, b)
  distance(d) ab
}
```

Parameters are given by label (`above(d: 7)`; positional is E004, a missing or unknown label E040);
a word parameter stands where a selector or direction would (`side: s`, `level(s)`). The body reads
only its operands, their members and the parameters (E101 otherwise); it may write other words and
use sets. An unknown word is E102; culprits and errors read as the word (`p horizontal q`), and a
dimension passed as a parameter draws and edits at the argument. An infix word relates its operands
to each other; that both stand on a third thing is said of each (`a coincident P`).

`std`'s words, each a pair of dimensions said as one (import by name):

```
b offset(dx: 30, dy: 12) a       // b 30 right of a and 12 above it
b right_of(d: 30) a              // 30 right of a, level with it; left_of, above, below
p coords(du: 20, dv: 5) P        // 20 along P.u and 5 along P.v from P's origin
p on_u(d: 20) P                  // on P's u axis, 20 out; on_v
a skew(theta: 90deg, e: 20mm) b  // two lines crossed: the angle and the common perpendicular
a turned(theta: 180deg) b        // a the length of b, turned from it
hangs(L: 150mm) rope             // a hanging rope's length and energy (1.10)
```

**Previews.** A component file may end with one `preview { … }` holding sample datums, values and
an instance. Opening the file (directly or via a `.svd`) adds those statements; `use` omits the
whole block, units and params included. Imports resolve beside the opened file, then its ancestors,
so a component file opens with the same `use` paths it has in the assembly:

```sv
preview {
  unit mm
  in std.front {
    up := point
    fix((0, 40)) up
    axes := std.Turned(std.origin, up)
    cyl := Cylinder(axes, fw: components.dims.fwA, dims: components.dims.vtwin_dims)
  }
}
```

`vtwin/components/cylinder.sv` is the complete example; `vtwin/cylinder.svd` draws its views.

### 1.10 Curves

A traced curve is **a point of a component, followed as one of its numeric formals runs over an
interval**. There is no curve family: an involute, a cycloid and a leg's stride are three
components.

```
NAME := INSTANCE.POINT over FORMAL in (A, B)          a drawn instance's point
NAME := Component(ARGS).POINT over FORMAL in (A, B)    an instance written in place, never drawn
```

- **Over a drawn instance**, `path := leg.toe over theta in (0, 360)` is where the drawn leg's toe
  goes as `theta` runs, anchored at the drawing's pose. Leave `theta` unbound and the crank is the
  drawing's freedom (2.10, `jansen.sv`, `peaucellier.sv`).
- **Written in place**, `Involute(base, phase: a0).p over u in (…)` is anchored at the value the
  call gives the swept formal, or the interval's start.

The point is placed one of two ways. **Computed**: `p := point(x: XEXPR, y: YEXPR)` over the
formals; such a component can only be traced (2.8). **By constraints**: any point the body places,
as a person states a curve — "the end of a taut string as it unwinds" (2.9). The body must be
square (as many equations as its own coordinates) and drawn in one plane; follow a tilted datum by
passing a reference line and measuring bearings from it. Contacts with a curve (`coincident`,
`tangent`, `curvature`) are exact. A whole turn that comes back is closed: a contact wraps across
its seam.

A locus usually has several solutions. The body selects one by, strongest first: a **signed**
constraint (a point-to-line distance's sign chooses a winding); an orientation predicate `ccw`/`cw`
(read at the anchor and carried by continuity); a seed.

**The ellipse** is `std.Ellipse(c: point, a: Length, b: Length, tilt: Angle, u: Angle)`:
`e := std.Ellipse(c, a: 40, b: 25, tilt: 0deg).p over u in (0deg, 360deg)` is the rim. There is no
`ellipse` element.

#### Free curves and energies

An energy states a curve by the principle that makes it rather than by a formula:

```
rope := curve(a, b)                   // a free curve: its shape is the drawing's to find
length(150mm) rope
rope minimizes integral(p.y over p)   // ...and it hangs: its height along it least
```

`k minimizes E` says the curve `k` makes `E` stationary. `integral(EXPR over p)` integrates `EXPR`
along `k` by arc length, `p` the point running along it; `over (p, t)` also names the unit tangent
`t`. `EXPR` reads `p.x`, `p.y`, `t.x`, `t.y` and numbers in scope (E101 otherwise). An energy is a
sum of such terms times numbers; `maximizes` turns the sign; statements over one curve add up.
Under a `unit` the integrand is dimension-checked (`p.y + 1mm`, not `p.y + 1`).

- **What it is**: the solution of the energy's Euler–Lagrange equation from `a` landing on `b`,
  exact to the integration's accuracy. It adds no unknowns but its length, so its ends keep their
  freedom and can be dragged. Only a free curve takes an energy (a spline is E040).
- **Its length**: `length(150mm) rope` holds it. Otherwise the drawing may determine it (a point
  `m coincident rope` placed by two ordinates hangs the rope through `m`; a held deck `rope
  tangent deck` touches it at its lowest point), or else the energy is stationary in it too (two
  points and an energy with no length is a geodesic). A hanging rope with nothing holding its
  length has none (it only lowers its energy as it lengthens): left free, W114.
- **Contacts**: a relation to free geometry moves that geometry (`rope tangent floor` puts a free
  line on the rope). A held point stated on the curve is a **peg**: the rope drapes over it in a
  corner. A held line or circle pressed against a curve of held length is E040.
- **`rope touches rod`**: a corner held somewhere on a line, sliding to where the pull along the
  line balances (a ring on a rod, `ring.sv`). The rod need not be held. At most two lines a curve.
- **The verdict**: minimum, maximum or saddle (Legendre's and Jacobi's conditions). The CLI prints
  `minimum: rope minimizes integral(p.y over p)` (or `unsolved`), and the app washes the statement.
- **Area** is `integral((p.x * t.y - p.y * t.x) / 2 over (p, t))`: between `k` and its chord,
  positive counter-clockwise (2.18). A speed (`projectile.sv`), a weight `10mm / p.y`
  (`hyperbolic.sv`) or a refractive index (`fisheye.sv`) are energies too.

### 1.11 Claims and bounds

`claim` before a relation asserts it without enforcing it: `claim vertical rail`. A claim joins no
solve, count or conflict set; the drawing is identical with it deleted. The diagnosis judges it:

- **theorem** — holds, and the document implies it;
- **violated** — does not hold (the CLI prints `claim refuted:`);
- **consuming** — holds only where the solve landed (the CLI prints `claim independent:`).

Use claims for what a figure was drawn to show (the altitudes concur; the traced path is straight).
A claim may not own or read an unknown. `claim over rotor_turn in (0deg, 1080deg) { … }` judges at
sampled poses of a named motion, reading solids placed under it, and reports the worst; over an
unknown (`claim over reach in (0.1mm, 0.9mm) { … }`) it solves at each sampled value.

Three claim words apply only to solids: `claim bore inside stock`, `claim disc clear(2mm) cyl`,
`claim head fits(0.15mm) trap`. The report gives the measurement, and one within the faceting of a
round face is **undecided**:

```
  claim bore inside stock — undecided, measured 0 (interval [-0.0006000000000000001, 0.0006000000000000001])
    unresolved: containment is within curved-faceting uncertainty
  claim bore clear(2mm) stock — refuted, measured -9.999665339174008 (interval [-10.00026533917401, -9.999065339174006])
```

**A bound** is a dimension's number that adds no row: it says which of the drawing's solutions is
meant, the job a seed did when it only picked a side.

```sv
a distance(>= 0, along: up) p          // p at or above a
p distance(<= 6) q                     // within 6 of q
a distance(in: (1, 9), along: up) p
foot inside P                          // behind plane P along its normal; outside: in front
```

It takes an ordinate, a point–point distance or a distance from a line (else E040); `>=`, `<=`,
`in:` only. A point seeded on the wrong side is carried across before the solve, everything seeded
from it following; off the solution the solve looks for a root where it holds, else fails with `a
bound does not hold`. The diagnosis lists violated and *active* bounds. Not on a `claim`, and not
the only reader of an unknown. Inside a set's body a bound makes a region (1.13).

**Words, not seeds.** A bound chooses the root of each block of the solve's block-triangular order
it decides, so a drawing whose bounds, senses and signed ordinates say every root needs no seed at
all: the solve constructs it a block at a time, searching a block's roots for the one its bounds
allow. Say a side with a bound wherever a seed would have picked it — which side of a plane
(`foot inside std.top`), which hand (`A distance(>= 0, along: v) std.top`), which way along a line
(`A distance(>= 0, along: generator) M`) — and draw a line whose direction matters so that it runs
by construction (to the foot of a point on it, `M level(ax) foot`), not by a bound on its end,
which the solve cannot carry across by moving one point. A skew distance between two lines is a
magnitude, its side such a choice. `spiral_bevel/hypoid.sv` seeds nothing.

### 1.12 Space, axes and planes

**Standard datums.** `use std` gives the axes `std.x` (right), `std.y` (away from the front's
viewer), `std.z` (up) and `std.back` (−x), four planes at the world origin — `std.front` (u along
x, v along z: where a 2D drawing goes), `std.top` (x, y), `std.side` (y, z, seen from +x) and
`std.up` (u up, v left) — and `std.origin`, a point in `std.front` fixed at its origin. Passing a
plane as an argument does not imply `in`.

**Points in space.** A point no `in` reaches has three coordinates, seeded `hint((x, y, z))` and
held `fix((x, y, z)) p`; a line between two is a line in space. A circle, arc or spline over a
point in space is E060, a face over one E080. In the workspace such points are dragged across the
eye's picture plane; depth stays the constraints'.

- **`p coincident P` is `in P` said as a relation**: the point is drawn in `P`, and relations over
  it read as `P`'s. `l coincident P` puts both ends in. A point its declaration draws in another
  plane is drawn in `P` too, as `in Q, P` would draw it: the way to put a point another component
  declares into a plane only this one knows (`gear.M coincident q`). Said plane first, `q
  coincident gear.M` puts the plane through the point by a row. A point in space put on a second
  plane by a statement is held there by a row, and `project` reads each point in its own plane.
  A point whose seed has a `z`, that a `fix` holds whole, or that is claimed on the plane, stays
  in space.
- **A point may be drawn in two planes**: `O := point in P, G` lies on their common line, and each
  plane's geometry reads it as its own (`arc(center: O)` drawn in `G` is `G`'s). It is what two
  views tied by `project` say, said once. The same plane twice is E040, parallel planes E061.

**An axis** is a directed line in space with no start: a direction, and a place once something
reads it.

```sv
t := axis hint(dir: (0.8, 0, 0.5))   // the seed is a direction, normalised
t perpendicular std.y
t angle(30deg) std.x                  // unsigned, in space; 0° or 180° is E040: say `parallel`
p coincident t                        // a point on it gives it its place
t coincident s                        // one line, either way round (four rows)
l coincident t                        // a drawn line lying on it (four rows)
fix(dir == (1, 0, 0), origin == (0, 0, 0)) x_axis   // held outright
```

Read only as a direction an axis counts two freedoms; a point or line on it adds the two of its
place. Its `origin` is its point nearest the world origin. Wherever an axis is expected a drawn line
is read as one, from `p1` toward `p2`; the reverse does not hold — a motion's `about:` takes a drawn
line, not an `axis` (a revolution's takes either, 1.14).

**A plane is two axes through its origin**: `P := plane(u: r, v: s)`. Right is `u`, out of the
plane `u × v`, up out × u, so `v` need not be square to `u`: it picks the plane and which side is
up. `P.origin` (drawn in `P`, held at `(0, 0)`) lies on both axes, so where the axes stand the plane
stands; two held axes that do not meet are E067. A plane over a line stands on it, and over two
lines sharing an end, at that end. **A slot left out mints a free axis**: `P := plane` is seven
freedoms, `plane(u: std.x)` mints only `P.v`. A minted axis starts as the front's unless seeded,
`plane(v: hint(dir: (0, -1, 0)))` — seed one a relation will turn far. A frame of two square axes:

```sv
u := axis
v := axis
u perpendicular v
p := plane
u coincident p.u
v coincident p.v            // DOF 6: p stands where u and v meet, its normal u × v
```

What else places a plane:

- `P.origin coincident p` puts its origin at `p` (three rows; with an axis already through `p`,
  say less: `P.origin coincident std.front` puts an origin already on `std.y` at the world origin);
- `P coincident p`, the plane through `p`; `l coincident P`, `t coincident P`, a line or axis in
  it; `t parallel P` (one row), `t perpendicular P` (two);
- `p distance(d, along: n) P`, a signed distance along its normal; `P distance(d) Q`, `Q` standing
  `d` along `P`'s normal (one row);
- `P parallel Q`, normals alike either way (two rows; no plane–plane angle yet);
- `fix(origin == (0, 0, 0)) P`, which must lie on its axes (E067).

A plane standing off another takes its own axes (shared ones would pass through the other's
origin). Held outright, as a section or a loft's end needs, or stacked parallel by a distance and
an axis square to both:

```sv
end := plane
fix(origin == (0mm, 40mm, 0mm)) end
fix(dir == (1, 0, 0)) end.u
fix(dir == (0, 0, 1)) end.v

a := plane(u: std.x, v: std.z)       // the front plane, at the world origin
b := plane hint(origin: (0, -10, 0))
b.u parallel std.x
b.v parallel std.z
a distance(12) b
n := axis hint(dir: (0, -1, 0))      // square to both, through both origins: stops b sliding
n perpendicular a
n coincident a.origin
n coincident b.origin                // DOF 0
```

Stacked parts are drawn so; there is no mate word. Two planes lying on one another are warned
(W113). `solventc --where P` lists `P.x`, `P.y`, `P.z` (where it stands) and `P.u`, `P.v`, `P.n`.

**A plane is also a view.** A point drawn `in P` has `P`'s own coordinates, so a dimension within
`P` is measured from `P.origin` along `P.u` and `P.v`. `a project b` says two points drawn in two
planes are images of one corner (one equation: their coordinates along the planes' common line
agree; E061 for points in one plane or parallel planes, E065 if the planes come out parallel).
Views carry no place on paper: the `.svd` places each.

- **`in top { … }`** writes membership once for every declaration inside. An instance joins a view
  whole: `t := Tooth(…) in top`. Inside a component, `in view { … }` over plane formals lets a part
  carry its own views (`engine/conrod.sv`), and `repeat flag { … }` over a 0/1 `Int` formal omits a
  view an instance does not show in.
- A view's attitude may be an unknown: build it on axes the solve turns (2.13, 2.14). Planes and
  axes are settled before the main solve and still move with it.

**Datums turned within a plane.** `std.Turned(o, t)` is the plane through `o` whose u runs toward
`t`, drawn where `o` and `t` are; ordinates against it are signed:

```sv
unit mm
use std
in std.front {
  o := point hint((10, 5))
  t := point hint((40, 20))
  tilted := std.Turned(o, t)                   // tilted.axes is the plane, tilted.u the line o → t
  tilted.u angle(30deg) std.x                  // unsigned, across: the seed picks the side
  p := point hint(at: tilted.axes, (20, -3))   // seeded in the turned axes
  p distance(20mm, along: u) tilted.axes
  p distance(-3mm, along: v) tilted.axes
}
```

`p` is still drawn in `std.front`; only the measurement is against the turned axes. No component
has an implicit frame. **Model relationships, not coordinate tables**: a rectangle is perpendicular
sides, a width, a height and a position against a datum, not an ordinate for every corner; the
library deliberately has no coordinate-placement helper.

### 1.13 Sets and regions

**A set is the points that satisfy a predicate**: `near := { p | p distance(20mm) c }`, its point
named after the `{`, the statements about it after the `|`, on one line or several. It adds nothing
to the drawing; it means what its uses say:

- `q coincident near` states the body with `q` for `p`. A set may be written where used, on the
  right: `q coincident { p | p distance(5mm) c }`.
- `l tangent near`, `l` a line: a contact on `l` lies in the set and `l` runs in its tangent plane
  there, derived from the body. A body may make points of its own (a foot on an axis); their motion
  along the set is solved for too:

  ```
  foot := { p |              // a cylinder of radius 4 about `ax`, by its foot point
    q := point
    q coincident ax
    g := line(p, q)
    g perpendicular ax
    p distance(4mm) q
  }
  l tangent foot
  ```
- `S1 tangent(at: m) S2`: two sets touch at `m`, one tangent plane there; state `m coincident S1`
  and `m coincident S2` beside it. `P tangent(at: m) S`, `P` a plane: `P` is the set's tangent
  plane at `m` (a pitch plane touching a pitch cone). Refused over a body making points.
- A **family** is a component whose body is a set: `component Ball(center: point, r: Length) :=
  { p | p distance(r) center }`; an instance is the set, its formals read by its name. A component
  takes a set as a **`set` formal**, by position.
- Statements in one body are a conjunction (an intersection). A union is a choice of root, not a
  body. A culprit reads as the use (`l tangent shaft`), never the body's rows.
- A set's body is read in space wherever its point is drawn, so `side:` or `sense:` in one is an
  error.
- **A set whose body is a circle's is the circle**: `k := { p | p coincident std.front; p
  distance(25mm) o }`, `o` drawn in `std.front`, is drawn, dragged, dimensioned and swept exactly as
  `circle(center: o)` with `radius(25mm)`.

**The library's sets** draw nothing; each is a centre or a line and one number, which may be left
unbound as an unknown (`r: hint(10mm)`; the solve writes it back):

- `ball := std.Sphere(c, r: 12mm)`; `std.CircleOnSphere(k, ball, view)` puts circle `k` drawn in
  `view` on it.
- `shaft := std.Cylinder(ax, r: 8mm)`; `shaft.about distance(shaft.r) l` states a touch directly
  (the line is `.about`, since `axis` is an element's word).
- `gc := std.Cone(gax, half: 60deg)`: apex at the line's start, opening toward its end; a point on
  it lies on that nappe (either generator within the axis's own view, the seed picking).
- `std.TangentCones(gc, pc, M)` is `gc tangent(at: M) pc`.
- `inner := std.ConeThrough(ax, q, lean: 20deg)`: the cone about `ax` through `q`, its radius
  growing `tan(lean)` along `ax` (negative narrows it); its inside is always nearer `ax`. Its apex
  is the instance's (`inner.apex`), made once: a family's body makes once whatever reads nothing
  of its point.
- `top := std.Level(o, ax, d: 2mm)`: the plane `d` along `ax` from `o`; inside is behind it.

`p coincident ball` puts a point on any of them and `l tangent shaft` makes a line touch one.
`sphere_cone_cylinder.sv` shows one of each; 2.14 lays out pitch cones.

**Regions.** A body that bounds a number is a region: `ball := { p | p distance(<= 5) c }`.
`q inside S` / `q outside S` put a point in or out of any set, reading its one number as at most /
at least: `q inside std.Sphere(c, r: 5)`, `q inside std.Cone(l, half: 30deg)`. Regions meet by
conjunction, and `outside` makes a difference: `{ p | p inside heel; p outside toe }`. Refused:
`outside` a conjunction or interval, a set of several numbers, a tangency to a region. A point in a
region is a bound, so a choice of root.

**Offsets.** `p distance(4mm, side: outside) k` puts `p` 4mm off `k`'s surface (`side:` required);
`p inside(2mm) pitch` is within 2mm outside it; `tip := std.Offset(pitch, d: 2mm)` is the set 2mm
out. `S.surface` is `S` with each bound made an equality: of `{ q | q inside(2mm) pitch }`, the
offset surface — a set like any, so `k coincident S.surface` puts a drawn point on it. A distance
that comes to nothing (`std.Offset(S, d: 0mm)`) is `S` itself. A drawn point measured from a set
gets a foot on it, found by the solve. A
distance from `{ q | q outside S }` is from `S`'s surface, sides turned: so `std.Rounded(beyond,
tip, r: 0.3mm)` rounds the corner where a region is outside `inner` (`beyond := { q | q outside
inner }`).

**A crown tooth as a region**, and its land: two `ConeThrough` flanks, two `Level`s, the tip's
corners rounded — `tooth := { p | p outside inner; p inside outer; p inside back; p outside tip;
p inside std.Rounded(beyond, above, r: …); p inside std.Rounded(outer, above, r: …) }` — is a
revolution, and `solid(tooth)` swept under a generating motion cuts a member exactly. A region
has no tip line for `distance(>= d) tip`: draw the land's ends on the tip, the flanks through
them, and bound the land between them (`eb distance(>= 0.01mm, along: left) ea`), which picks the
root as the drawn section's bound did. Where the flanks go through points the drawing already
places (the hypoid's pitch points, `spiral_bevel/hypoid.sv`), the land has no root to choose and
needs no bound; a round's radius may read an unknown of the drawing (`r: 0.3 * mn`).

### 1.14 Faces and solids

**A solid is a term, never a step.** A solid is a face swept, or a stock **plus everything in
`union` with it, minus everything that `cut`s it, within everything that `bound`s it**. The order
lives inside the term, never between statements: `bore cut body` may be written fifty lines above
`body := solid(…)`. **Nothing about a solid is solved for**: it owns no parameters, numeric
extents are worked out at elaboration, and the geometry swept is the drawing, solved as usual.

#### Faces

A face is a planar region bounded by edges the drawing already has, on the plane they are drawn
in, listed in traversal order, each sharing a point with the next. A circle is a whole loop by
itself (`face(hole)`).

```
unit mm
use std

in std.front {
  a := point
  b := point hint((60, 0))
  c := point hint((60, 40))
  d := point hint((0, 40))

  horizontal (ab := line(a, b)) ->
  vertical   (bc := line(b, c)) ->
  horizontal (cd := line(c, d)) ->
  vertical   (da := line(d, a)) -> close

  a distance(60) b
  b distance(40) c
  fix((0, 0)) a

  sec := face(ab, bc, cd, da)
  block := solid(sec, depth: 30mm)
}
```

```
$ solventc block.sv --where block.volume
block.sv: solved
  6 params, 6 equations, structural rank 6; DOF 0; 2 rigid cluster(s) in the distance graph
  block.volume = 72000
```

The face and solid added nothing to the rectangle's six unknowns and six equations.

- **Inline or named.** `solid(face(ab, bc, cd, da), depth: 30mm)` writes the face where one sweep
  uses it. A named chain needs no face: `block := solid(profile, depth: 30mm)`, edges still `ab`;
  an open chain closes with `face(trail, -> close)`.
- **Holes**: `face(barrel, holes: core)` or `holes: first, second` — circles, closed loops or faces
  strictly inside on the same plane. A hole of loose edges is a face first, `h := face(e, f, g, k)`.
  A hole *through a part* is usually better as a solid that `cut`s the body, which names the bore's
  wall.
- **A face closes itself.** A *point* in the list is a corner walked straight through, and
  `-> close` seals the walk back to its first item: `face(a, bc, cd, -> close)` mints the two
  missing runs, which name faces of the solid (`block.close0`, `block.close1`). A gap between two
  *edges* is E080 (edges out of order), and so is an edge between two gaps.
- **A curve stands in a face as a stretch**, `k from a to b`, between two points held on it by
  contacts; one stretch closed by `-> close` is a loop with its chord. A clamped spline ends at its
  end control points, so it is an ordinary edge.

```
unit mm
use std
component Par(o: point, u: Length) {
  p := point(x: o.x + u, y: o.y + u * u / 1mm)
}
in std.front {
  o := point
  fix((0mm, 0mm)) o
  a := point hint(y: 1mm)
  b := point hint(y: 1mm)
  a coincident k hint(t: -1)
  b coincident k hint(t: 1)
  fix(x == -1mm) a
  fix(x == 1mm) b
  cap := solid(face(k from a to b, -> close), depth: 3mm)
}
k := Par(o).p over u in (-2mm, 2mm)
```

`4 params, 4 equations; DOF 0`, and `cap` is the parabolic cap, 4/3 mm² × 3 mm = 4 mm³
(`examples/solid_tooth.sv` is an involute tooth so).

#### Sweeps

- **Prism**, along the face's normal: `depth: 30mm` is the material *behind* the visible face,
  `from: -30mm, to: 0mm`; write `from:`/`to:` when the face is not at an end. `through: body` spans
  the target's stock and additions both ways and follows the solve.
- **Revolution**, about a line or axis **in the face's own plane**: `about: ax` is a full turn,
  `sweep: 90deg` a partial one (caps `start` and `end`), `sense: cw` reverses it. An axis lies in
  the plane when it is one of the plane's own (`about: std.z` on `std.front`) or the solve puts it
  there (else E081).
- **Guided sweep or loft** along a directed line or circular arc: `solid(section, along: guide)`,
  `solid(inlet.profile, outlet.profile, along: guide)`. Sections stand square to the guide's
  tangents; holes pair in written order; caps are `start` and `end` (`solid_elbow`, `solid_loft`).

```
unit mm
use std (horizontal)

in std.front {
  p0 := point hint((10, 0))
  p1 := point hint((14, 0))
  p2 := point hint((14, 6))
  p3 := point hint((10, 6))

  horizontal (e0 := line(p0, p1)) ->
  vertical   (e1 := line(p1, p2)) ->
  horizontal (e2 := line(p2, p3)) ->
  vertical   (e3 := line(p3, p0)) -> close

  q0 := point
  q1 := point hint((0, 10))
  ax := vertical line(q0, q1)
  fix((0, 0)) q0
  q0 distance(10) q1
  q0 distance(10, along: x) p0
  q0 horizontal p0
  p0 distance(4) p1
  p1 distance(6) p2

  sec := face(e0, e1, e2, e3)
  ring := solid(sec, about: ax)
}
```

```
$ solventc ring.sv --where ring.volume
ring.sv: solved
  10 params, 10 equations, structural rank 10; DOF 0; 2 components: DOF 0, 0; 3 rigid cluster(s) in the distance graph
  ring.volume = 1809.56
```

Pappus gives `2π · 12 · 24 = 1809.557`: a static solid's volume is its exact boundary's. With
`sweep: 90deg` it is `452.389`.

**Writing a part.** Where a part's axis lies relative to its section decides the statement: the
V-twin piston is a turn of its half-profile; the disc, flywheel and throttle have their axis
through the section, so they are prisms with turned features (a radial set screw) only where the
axis lies in it. **A part's section must be where its turned features are**: a turn about a line in
the section centres what it makes on that plane, so the crank disc is sectioned on its mid-plane.

#### Bodies

Add to the 60 × 40 plate, inside its `in std.front` block:

```
o := point hint((30, 20))
a distance(30, along: x) o
a distance(20, along: y) o
hole := circle(center: o) hint(r: 5)
radius(5) hole
hole_f := face(hole)

stock := solid(sec, depth: 30mm)
bore := solid(hole_f, depth: 30mm)
body := solid(stock)
bore cut body
```

`body.volume` is `69643.8` (`72000 − π · 5² · 30`), whichever order the last two lines are in.

- **A swept solid takes features directly**: `bore cut stock` makes `stock` the body, keeping its
  face names. Write `body := solid(stock)` only when the bare sweep and the part both need names.
- **`cut` with `through:`**: `pinhole := solid(face(hole), through: body)` then `pinhole cut body`.
  `through:` only makes the cutter.
- **`tip bound body`** keeps only what lies within `tip`. Union comes first; `cut` and `bound`
  commute.
- **Name intermediates the sets cannot express.** A boss standing in a pocket's floor is not
  `pocket cut body` + `boss union body` — that is stock ∪ boss − pocket, and the pocket eats the
  boss. Name the pocketed shell, then add the boss to it:

  ```
  stock := solid(sec, depth: 30mm)
  pocket := solid(rim_f, depth: 10mm)
  boss := solid(stud_f, from: -10mm, to: -4mm)

  shell := solid(stock)
  pocket cut shell

  body := solid(shell)
  boss union body
  ```

  `shell` is what a history calls "the body as of step 2", named.

**A region is a solid**: `solid(R)`, `R` a set, is the points inside it.

```sv
big := std.Sphere(c, r: 5)
small := std.Sphere(c, r: 3)
skin := { p | p inside big; p outside small }        // a shell
shell := solid(skin)
tube := std.Cylinder(up, r: 2)
can := { p | p inside tube; p outside std.top; p inside lid }   // capped by two planes
tin := solid(can)
```

Where every bound shares one axis (balls centred on it, cylinders and cones about it, planes square
to it) and closes the material, the solid is a revolution of its meridian, built exactly. Where they
share none it is read point by point: it shows and meshes like any solid, but an exact export
(`--step`) refuses it — unless it is one the lowering table builds: the rounds and chamfers of
`std` along straight edges between planes, round circles where a plane meets a cylinder or a
cone, and at corners of three square faces are built exactly (recognised by what their sets say,
so a round written by hand is built too). Its numbers may be the drawing's: leave a radius unbound and say where
the set's surface is (`heel := std.Sphere(apex)`, `M distance(w / 2, side: inside) heel`), and the
solid reads the solved radius — how the hypoid's blank is sized (`spiral_bevel/hypoid.sv`'s
`Blank`).

#### Sketching on a face

A face of a solid names a **set**: the face, bounded by its edges. Its **`.surface`** is the
surface it lies on. A prism's faces are `b0.near` and `b0.far` (the caps at its higher and lower
extent) and `b0.ab` (the side swept from edge `ab`), reached through a body as `body.b0.near`. A
plane lies on a face by touching its surface at its own origin:

```
b0 := solid(f0, depth: 20)
v1 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) hint(origin: (10, 20, 10))
v1.origin coincident b0.far.surface
v1 tangent(at: v1.origin) b0.far.surface
```

Draw on `v1` (`in v1 { … }`) and extrude again; change `depth: 20` and everything on `v1` follows.
`v1` keeps four freedoms (where its origin sits, how its axes turn); hold them as anything else.
Double-clicking a face in the workspace writes these statements, or picks a plane already lying
there. Refused (E040): the bare face (write `.surface`), a curved side, a `through:` solid's caps,
an edge with no name, the faces of a revolution, loft or sweep.

#### Rounds and chamfers

`use std (round, chamfer)`:

```sv
block := solid(outline, depth: 12mm)
block.near round(r: 2mm) block       // every edge where the near face meets another of block's
block.far chamfer(d: 1mm) block      // the far rim, a flat 1 along each face
round(r: 1.5mm) dish.pocket          // every edge among the pocket's faces
pad.boss round(r: 3mm) pad.plate     // where the boss stands on the plate
```

- `A round(r: …) B` rounds every edge where a face of `A` meets one of `B`: a convex edge loses what
  a rolling ball cannot reach, a concave one is filled, three meet in the ball's corner.
  `round(r: …) X` is every edge among `X`'s faces. `chamfer(d: …)` cuts flat, `d` along each face.
- `A` and `B` are **face paths**: `block` (every face), `block.near` (one), `pad.boss` (every face
  the boss gave), `pad.plate.near`, read from the solid through its operands' names, never by
  suffix. Faces that do not meet make nothing; a face a Boolean ate names nothing.
- Each acts on the body after its other terms: a round reads the faces the cuts and unions left.
- Where two faces meet tangent there is no edge: a chain of tangent edges (an obround's foot, a
  rounded pocket's rim) is rounded as one run. A run that ends on a face it does not round is
  carried on past its edge's end and cut off by that face, however the face stands.

They are written in Solvent (`rust/lib/std.sv`), from region bodies you can write yourself
(`rust/examples/rounds/by_hand.sv` rounds one edge in about twenty lines):

| written | means |
|---|---|
| `p inside block.near`, `p outside(d) block.near` | the material's side of the face's surface (grown by `d`) |
| `p distance(<= 1mm) block.near` | within 1 of the face, bounded by its edges |
| `p distance(>= r) block.near.surface` | at least `r` from the whole surface |
| `distance(p, f1) + distance(p, f2) <= 10mm` | a bound on measurements (an ellipsoid by its foci) |
| `crease := { q \| q coincident a; q coincident b }` | the edge where faces `a` and `b` meet |
| `angle(a, b, at: p) <= 180deg` | at a convex edge (the material's angle there) |
| `p inside crease.ends` | stopped by the faces the edge ends on (none at a tangent seam) |
| `meeting := { q \| q coincident a.surface; q coincident b.surface; q inside crease.ends }` | where the surfaces meet, short of where the edge ends |
| `repeat a in A { repeat b in B if b != a { … } }` | a copy per pair of faces |
| `taken := solid(run)`, `taken cut A.body` | a solid of a set, cut from the body `A`'s faces are of |

`.ends`, `.surface` and `.body` are members only past a set or a solid where nothing declared
takes the name. A drawn point may not read a bare face (draw on `.surface`), and a bound on
measurements stands only in a region's body. In a region's own body,
`p inside std.Rounded(wall, top, r: 1.5mm)` and `std.Chamfered(wall, bottom, la: 1mm, lb: 1mm)`
round or chamfer between two of its bounds, built exactly where the region is a revolution.
`rust/examples/rounds/` and its README walk them.

#### Reports and refusals

`--where body` lists volume, area, bounding box and each face's area under its document name,
through the operand it came from:

```
$ build/solventc plate.sv --where body
plate.sv: solved
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.area = 11585.4
  body.bore.hole.area = 942.467
  body.bounds.x0 = 0
  body.bounds.x1 = 60
  body.bounds.y0 = 0
  body.bounds.y1 = 30
  body.bounds.z0 = 0
  body.bounds.z1 = 40
  body.stock.ab.area = 1800
  body.stock.bc.area = 1200
  body.stock.cd.area = 1800
  body.stock.da.area = 1200
  body.stock.far.area = 2321.46
  body.stock.near.area = 2321.46
  body.volume = 69643.8
```

A face a Boolean consumed keeps its name with no area. `--stl PATH` writes binary STL, `--step
PATH` an exact B-rep; `--solid NAME` picks which. Views and dimensions of solids are asked for in a
`.svd` ([Solvent Drawing](solvent-drawing.md)); generated dimensions there are readings, not
statements.

| written | reported |
|---|---|
| `face(ab, cd, bc, da)` | E080 — "`ab` and `cd` share no point: a face is a loop, walked in order" |
| `face(a, bc, d, -> close)` | E080 — "`bc` meets neither of its neighbours" |
| `face(ab, hole)` | E080 — "a circle is a whole loop: it stands in a face by itself" |
| `face(a, b)` | E080 — a loop that does not come back closes with `-> close` |
| `solid(ab, depth: 3mm)`, `solid(sec)` | E080 — a sweep is over a face; a body is made of solids |
| `solid(sec, from: 0mm, to: 0mm)` | E080 — "a prism swept nowhere is no solid" |
| `solid(sec, about: ax, sweep: -90deg)` | E040 — "a sweep is a magnitude: which way it turns is `sense: cw`" |
| `solid(sec, about: std.y)`, face on `std.front` | E081 — "`std.y` does not lie in the face's own plane" |
| `solid(sec, depth: 3mm, about: ax)` | E001 — swept along its normal or turned, not both |
| `x cut y` and `y cut x` | E041 at the second — "`x` would be made of itself" |
| `x coincident y`, two solids | refused: material is added with `union` |

### 1.15 Motions and generated geometry

**Motions** are rigid families over one shared angle; a motion never moves the sketch:

```sv
crown := motion(about: crown_axis)
blank := motion(about: blank_axis, ratio: 2, phase: 10deg)   // twice the angle, plus 10°
generating := motion(crown, relative_to: blank)              // crown seen from the blank
tap := motion(about: hole_axis, advance: 1.5mm)              // a screw: 1.5 mm per turn
plunge := motion(along: spindle, advance: 20mm)              // a translation
```

`motion(about: o)` over a point is a turn in its view. A ratio may **measure the drawing** (1.6),
re-read whenever the motion is: `pinion := motion(about: ax, ratio: -length(wheel_radius) /
length(pinion_radius))` turns at the solved ratio, and editing a radius re-times it (`ratio:` is
`Scalar`, so a bare `length(…)` there is E080).

**A motion moves a solid**: `solid(tool, under: indexing, at: 90deg)` is one copy at a constant
angle (`solid_indexed_pattern.sv`); `solid(tool, under: turn, from: -75deg, to: 75deg)` is the
union of the tool over the whole interval, a continuous sweep meshed from its material field
(`swept_torus.sv`). `at:` and `from:`/`to:` are exclusive, a sweep of a sweep is refused, and for an
exact export a swept solid may only be `cut`. Exact exports cover the generating classes
([generating-sweeps](generating-sweeps.md)): revolved tools cutting gears (`lantern_generation.sv`,
`spiral_bevel/`), a wheel carried along a screw (`twist_drill/`), a pocketed prism under a planar
motion (`wankel/`).

**Envelopes in the plane are curves the solve sees.** Over a tool of the sheet (a point, line,
circle, arc or formula curve) and a planar motion, `flank := envelope(rack_flank, under: cutting,
from: -25deg, to: 25deg)` is the profile the tool cuts in the moving frame. `coincident`, `tangent`
and `curvature` hold against it, and the tool's and motion's geometry are its columns, so a
conjugate is synthesised, not stated (2.11). `side: near|far` picks where a circle cuts twice (a
cam). A point's envelope is its path: `bore := envelope(apex, under: rotor_turn, from: 0deg, to:
1080deg)` is the epitrochoid, typed nowhere, closed after whole turns and a face's loop by itself.

**The rotor is a cut**: the largest rotor turning in a bore is the blank less everything the
housing's wall sweeps through — `swept := solid(housing, under: housing_turn, from: 0deg, to:
1080deg)`, `swept cut rotor` (`bound` by a sweep keeps the union's inside, a different set).

**A prism's side** is named as a revolution's is, `side := surface(rack_tooth, edge: rack_flank)`;
under a motion keeping its view, `envelope(side, under: cutting, …)` is the planar envelope
extruded, and `p coincident flank` from any view is one equation. Draw the prism's face `in` a view
and name the edge in it.

**Spatial geometry read after the solve** adds no unknown and has no 2D glyph; each certifies only
local evaluations ([analytic face boundaries](analytic-face-boundaries.md); the spiral bevel's
checks make the same objects of their parts, from the faces its built members leave —
`faces::face_source`, #212):

- `surface(crown, edge: rack.outer[, from: 180deg, to: 360deg])` — the surface one profile edge
  of an unmodified revolution sweeps, `u` along the edge, `v` round; read the domain, never assume
  [0, 1];
- `envelope(surface, under: generating, from:, to:)` — its zero-normal-velocity locus over the
  roll interval; chooses no branch and closes no solid;
- `patch(flank, inside: tip_body, outside: root_body, …)` — which material keeps it;
- `seam(a, b)` — two envelopes meeting tangentially at a shared profile vertex, or a generated
  face meeting a finite boundary surface;
- `vertex(seam1, seam2)` — where seams meet on one face;
- `edge(seam, from: v1, to: v2, along: axis)` — a seam's finite extent;
- `face(e1, e2, …, on: support)` — an ordered loop of such edges on an exact support.

### 1.16 Checking your work

```
make solventc                                         # once
build/solventc drawing.sv                             # parse, elaborate, solve, diagnose; --json
build/solventc drawing.sv --where hinge               # where a name landed
build/solventc drawing.sv --stl part.stl --solid body # a solid, for a printer
```

Exit codes: 0 solved, 1 did not parse or elaborate, 2 did not solve. A successful solve does not
mean the model is fully constrained: read the diagnosis — parameter and equation counts, the
**DOF**, and culprit lines (`over:`, `conflict:`, `implied:`). The status (`diagnosis.status` under
`--json`):

| state | meaning | what to do |
|---|---|---|
| `well` | DOF 0, consistent | check that it is the solution you meant (the branch, not just the count) |
| `under` | DOF > 0 | add a constraint or a gauge, or leave a mechanism's freedom on purpose |
| `over` | a dimension takes part in a consistent redundancy | remove one `over:` line; editing it would be the next conflict |
| `conflict` | statements that cannot all hold | the `conflict:` lines are the *minimal* disagreeing set |
| `unsolved` | the solver stopped short | check the model, then reseed nearer the intended branch |

A redundancy among pure relations (a fourth `perpendicular` round a rectangle) is a theorem, listed
`implied:`, never an error.

**`--where NAME`** prints the numbers under a name: a point's (`hinge.x`), an instance's
(`--where t0`), a plane's (`--where std.side`), a solid's measurements (`--where body`), or one
number (`--where body.volume`). Under `--json` they are a `positions` table. It is quicker than a
`claim` for testing a position.

Two habits: **fix something** (a figure nothing holds is under by three however determined its
shape), and **seed for the branch** (the solver finds the solution nearest its start, so an arc
seeded on the wrong side comes out mirrored).

---

## 2. Examples

Each was run through `solventc`; the DOF and state quoted are what it reported.

### 2.1 One dimensioned line: DOF 0, well

```
use std

in std.front {
  a := point
  b := point hint((30, 10))

  ab := line(a, b)
  horizontal ab
  a distance(40) b

  fix((0, 0)) a
}
```

`2 params, 2 equations, structural rank 2; DOF 0`: the `fix` takes `a`'s coordinates out of the
solve, and the level and the length settle `b`'s. `b`'s seed is nowhere near the answer and needn't
be: it only says which side of `a` to put `b`.

### 2.2 A rectangle, as a chain: DOF 0, well

```
use std

w := 60
h := 40

in std.front {
  p0 := point
  p1 := point hint((w, 0))
  p2 := point hint((w, h))
  p3 := point hint((0, h))

  horizontal (bottom := line(p0, p1)) ->
  vertical   (right := line(p1, p2)) ->
  horizontal (top := line(p2, p3)) ->
  vertical   (left := line(p3, p0)) -> close

  p0 distance(w) p1
  p1 distance(h) p2
  fix((0, 0)) p0
}
```

`w` and `h` are values, never unknowns. The chain states nothing four separate lines would not; it
reads as the outline.

### 2.3 An input: DOF 0, well

```
// replaces 2.2's two dimensions and its `w :=` and `h :=` lines (the seeds' `h` becomes
// `w / 2`); the input stands outside the `in` block, the dimensions inside it
param w := 60
p0 distance(w) p1                // states it
p1 distance(w / 2) p2            // the height follows the width
```

Edit the 60 — in the source, or by typing over the `w` callout — and the height follows; a host may
give the input another value.

### 2.4 An unknown: DOF 1, under, on purpose

```
use std

param s: Length         // an unknown: declared, no value...

in std.front {
  a := point
  b := point hint((10, 0))
  c := point hint((0, 9))

  ab := line(a, b)
  ac := line(a, c)
  horizontal ab
  vertical ac
  a distance(s) b         // ...so the two lengths are tied,
  a distance(s) c         // and their value is the solver's
  fix((0, 0)) a
}
```

The lengths must agree, but nothing says what they are, so one freedom remains. Give `s` a value
(`param s := 10`) and it closes. Without the `param` line, `s` is E101.

### 2.5 An arc, tangent to what it joins: DOF 0, well

```
use std

in std.front {
  a := point
  b := point hint((30, 0))
  c := point hint((40, 10))
  d := point hint((40, 40))
  o := point hint((30, 10))

  horizontal (run := line(a, b)) -> tangent
  (fillet := arc(center: o) hint(r: 10)) -> tangent
  vertical (rise := line(c, d))

  radius(10) fillet
  a distance(30) b
  c distance(30) d
  fix((0, 0)) a
}
```

`fillet` names only its centre; the chain threads `b` in as its start and `c` as its end, and each
`tangent` is stated *at* that shared point. `rect_fillets.sv` does this round four corners.

### 2.6 A component, instanced: DOF 0, well

```
use std

component Rung(a: point, b: point, len: Length) {
  e := line(a, b)
  horizontal e
  a distance(len) b
}

in std.front {
  l0 := point
  r0 := point hint((50, 0))
  l1 := point hint((0, 20))
  r1 := point hint((50, 20))

  t0 := Rung(l0, r0, len: 50)
  t1 := Rung(l1, r1, len: 50)

  stile := line(l0, l1)
  vertical stile
  l0 distance(20) l1
  fix((0, 0)) l0
}
```

The formals alias the actuals; nothing is added at the boundary.

### 2.7 Repetition: DOF 5, under

```
use std

n := 6
r := 40

in std.front {
  cycle n as i {
    p := point hint((r, i * 60))
    e := line(p, next.p)
    e equal next.e
  }
  fix((r, 0)) p[0]
}
```

Six equal links: round a loop one `equal` is redundant (all six are listed `implied:`), and nothing
sizes the ring, so five freedoms remain. Under-constrained repetition is normal.

A body ending mid-joint writes a closed contour with no corner names. DOF 1, under:

```
use std

in std.front {
  cycle 4 {
    (s := line) -> perpendicular equal
  }
  distance(50) s[0]
  fix((0, 0)) s[0].p1
}
```

One dimension sizes it and one `fix` places it, leaving it free to swing about that corner. Round a
closed loop one `perpendicular` and one `equal` are implied; dimension every corner instead
(`-> angle(90)`) and it reads `over`. `square.sv` is this figure; `ngon.sv` the parametric version,
seeded round a circle because only a seed can choose the winding.

### 2.8 A curve from a computed point: DOF 1, under

```
use std

component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)), y: c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)))
}

in std.front {
  o := point
  base := circle(center: o) hint(r: 20)

  t := point hint((25, 8))
  t coincident f
  fix((0, 0)) o
  fix(r == 20) base
}

f := Involute(base, phase: 0).p over u in (0, 90)
```

The remaining freedom is how far along `f` the point `t` sits, which is why a contact slides rather
than breaks when the geometry beneath it moves. Seed it with `t coincident f hint(t: 30)`, or pin it
with `t coincident(t == 30) f` (DOF 0).

### 2.9 A curve stated as a locus: DOF 1, under

```
use std

component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point
  p := point
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c                                 // the string leaves the circle...
  datum angle(u + phase) rad                     // ...at bearing u from the datum,
  rad perpendicular s                            // square to the radius there,
  p distance(-(c.r * u / 1rad)) rad              // and taut: as long as the arc
}

in std.front {
  o := point
  x := point hint((20, 0))
  base := circle(center: o) hint(r: 20)
  datum := line(o, x)

  g := point hint((25, 8))
  g coincident f

  fix((0, 0)) o
  fix(r == 20) base
  horizontal datum
  o distance(20) x
}

f := Unwind(base, datum, phase: 0).p over u in (0, 90)
```

The same curve as 2.8 with no formula: the body is the textbook definition. The point-to-line
distance is **signed** here, so the minus sign unwinds the string one way for a positive roll and
the other for a negative one — one component serves both flanks of a tooth — and the directed
`angle` puts `t` at bearing `u + phase` with no `ccw`. A traced body may read the numbers of the
geometry it is written over (`c.r`); a dimension anywhere else reading geometry is **E103** — state
the relation instead (`l equal m`).

### 2.10 A curve of a drawn instance: DOF 1, under

```
use std

component Crank(o: point, datum: line, theta: Angle) {
  p := point hint((20, 10))
  arm := line(o, p)
  o distance(30) p
  datum angle(theta) arm
}

in std.front {
  o := point
  x := point
  datum := line(o, x)
  fix((0, 0)) o
  fix((10, 0)) x

  c := Crank(o, datum)                                  // theta unbound: the crank turns
}

rim := c.p over theta in (0, 360)
```

`c.theta` is the one freedom, and `rim` is where `p` goes over a full turn, anchored at the pose on
the sheet. `rim` comes back where it started, so it is **closed**: a contact wraps across its seam.
Another numeric formal left unbound (`len: Length`) is an unknown `c.len` and a column of the curve,
so `rim tangent ground` can solve the length. `jansen.sv` is this at full size.

### 2.11 A profile generated by a motion: DOF 0, well

```
use std

in std.front {
  o := point
  fix((0, 0)) o
  rk0 := point
  rk1 := point hint((30, 10))
  fix((30, 0)) rk0
  fix((30, 10)) rk1
  slide := line(rk0, rk1)
  blank := motion(about: o, ratio: 1)                     // the gear blank turns...
  rack := motion(along: slide, advance: 2 * pi * 30)      // ...as the rack rolls on its pitch circle
  cutting := motion(rack, relative_to: blank)
  f0 := point hint((30, 5))
  f1 := point hint((39.4, 8.4))
  flank := line(f0, f1)                                   // the rack's flank, at roll 0
  fix((30, 5)) f0
  f0 distance(10) f1
  cut := envelope(flank, under: cutting, from: -25deg, to: 25deg)
  g := point
  fix((31.5, -3)) g
  g coincident cut hint(t: 5)                             // the cut must pass through g
}
```

The flank's slope is left free, and `g coincident cut` decides it: the solve turns the rack's flank
until the involute it cuts passes through `g`. Nothing states an involute. A roller (`circle`) cuts
a cam, a formula curve its conjugate tooth.

### 2.12 Three views: DOF 0, well

```
// a 60-wide, 40-tall, 30-deep block, three views, one corner tied across them
use std

in std.front {
  Af := point
  fix((0, 0)) Af
  Bf := point hint((60, 40))
  Af distance(60, along: x) Bf
  Af distance(40, along: y) Bf
}
in std.top {
  At := point
  fix((0, 0)) At
  Bt := point
  At distance(30, along: y) Bt
}
in std.side {
  Ar := point
  fix((0, 0)) Ar
  Br := point
}

Bf project Bt          // width agrees front <-> top
Bf project Br          // height agrees front <-> side
Bt project Br          // depth agrees top <-> side
```

The standard planes all stand at the world origin, so the three `A`s are images of one point and
need no projection. `B` in the top and side views is placed by projection plus the one depth
dimension (`--where Br`: `(30, 40)`). `bracket.sv` is the full case, with an auxiliary plane on the
incline's own direction.

### 2.13 Two skew axes at a stated shaft angle and offset: DOF 0, well

```
unit mm
use std (horizontal)
shaft_angle := 90deg
offset := 17.5mm

in std.front {
  gax := line
  fix((0, 0)) gax.p1
  fix((0, 50)) gax.p2
}

t := axis hint(dir: (0.87, 0, 0.5))
t perpendicular std.y
side := plane(u: t, v: std.y)
side.origin coincident std.front     // on std.y already, as its axes pass through it
in side {
  pax := line(hint((0, 10)), hint((60, 12)))
  pax.p1 level(u) side
  pax.p2 distance(60, along: u) side
  pax.p1 horizontal pax.p2
}

gax angle(shaft_angle) pax      // the shaft angle, in space
gax distance(offset) pax        // the offset: their common perpendicular, in space
```

`28 params, 28 equations, structural rank 28; DOF 0`, and `--where pax` gives `pax.p1.y = 17.5`,
`pax.p2.y = 17.5`. The axes are in different planes, so `angle` and `distance` are spatial. The side
plane stands on `std.y` and an axis `t` held square to it, so it can only turn about y; the two
statements settle where `t` points (at 90°, along x) and the pinion axis's height. Without the
offset it is DOF 1 (the pinion axis slides along the common perpendicular); at a 60° shaft angle the
plane tilts 30°. `skew_axes.sv` adds a shaft about each axis.

### 2.14 A hypoid's pitch cones: DOF 0, well

```
unit mm
use std
Ng := 48          // gear teeth
Np := 24          // pinion teeth
module := 4mm
Rg := Ng * module / 2
Rp := Np * module / 2
E := 20mm

// the pitch plane P is the front plane, and M is on it
M := point in std.front
fix((0, 0)) M

// the gear's axial plane, square to P about its vertical axis; the pinion's, solved, through M
G := plane(u: std.z, v: std.y)
fix(origin == (0, 0, 0)) G
qu := axis hint(dir: (0.1618, -0.4935, -0.8546))
qv := axis hint(dir: (0.0918, 0.8698, -0.4847))
qu perpendicular qv
Q := plane(u: qu, v: qv)
M coincident Q.origin

// each axis from its apex, drawn at a length that says nothing about the cone; the pinion's
// drawn level in its plane, which is where the plane's own turn is held
gax := line(hint((110.85, 0)), hint((50.85, 103.9))) in G
pax := line(hint((-84.62, -48)), hint((-4.62, -48))) in Q
gax.p1 coincident std.front
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2
horizontal pax

// the pitch cones: the gear's pitch angle, M on both, and the two touching there
gc := std.Cone(gax, half: 60deg)
pc := std.Cone(pax, half: hint(30deg))
M coincident gc
M coincident pc
std.TangentCones(gc, pc, M)

// the two pitch radii at M, and the shafts: square, and E apart
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
```

`44 params, 44 equations, structural rank 44; DOF 0`. The gear's apex on P makes P the gear cone's
tangent plane at M, and `std.TangentCones` makes it the pinion's, so the pinion's apex lands on P
unasked. The pinion's plane stands on two axes the solve turns, held square, with its origin at M;
`horizontal pax` fixes its remaining turn. Γ = 60°, γ = 29.564957707°, ε = 10.722102067°.
`hypoid_pitch_cones.sv` is this document in the app; `gcs-core/tests/fixtures/hypoid_pitch_cones.sv`
builds the same pair without naming the cones, and the tests find the two agree to 2e-11.

### 2.15 An arc placed by its length: DOF 0, well

```
use std (horizontal)

in std.front {
  o := point
  s := point hint((10, 0))
  e := point hint((3, 9))

  a := arc(o, s, e)
  radius(10) a
  length(5 * pi) a        // a quarter of the circumference: the sweep is 90°
  o horizontal s
  fix((0, 0)) o
}
```

`5 params, 5 equations, structural rank 5; DOF 0`, with `e` straight above the centre at `(0, 10)`
from a seed that was not; `length(15 * pi) a` puts it straight below. `belt_wrap.sv` is the useful
version: an open belt as one tangent chain over two pulleys, with `length(wrap) big` in place of a
centre distance.

### 2.16 A bisector, stated as two equal angles: DOF 0, well

```
use std

in std.front {
  a := point
  b := point hint((40, 0))
  c := point hint((10, 30))
  d := point hint((25, 10))

  ab := line(a, b)
  ac := line(a, c)
  ad := line(a, d)
  horizontal ab
  ab angle(60deg) ac
  a distance(40) b
  a distance(30) c
  a distance(20) d

  ab angle(ad, ac) ad     // the angle from ab to ad is the angle from ad to ac
  fix((0, 0)) a
}
```

`6 params, 6 equations, structural rank 6; DOF 0`, with `d = (17.3205, 10)`: `ad` at 30°. A shared
unknown instead (`param beta: Angle`, `ab angle(beta) ad`, `ad angle(beta) ac`) is `7 params, 7
equations` — an extra unknown only to be equated away. `ab angle(ab, ac, sense: cw) ad` puts `ad`
at −60°. `reflection.sv` states the law of reflection as `incoming angle(m, outgoing) m`, with the
classical proof as a `claim` judged a theorem.

### 2.17 A hanging rope: DOF 2, under, on purpose

```
unit mm
use std

in std.front {
  a := point
  b := point hint((100, 0))
  fix((0mm, 0mm)) a
  rope := curve(a, b)
  length(150mm) rope
}
rope minimizes integral(p.y over p)
```

`3 params, 1 equations, structural rank 1; DOF 2` and `minimum: rope minimizes integral(p.y over
p)`: the two freedoms are `b`'s (the third parameter is the length, which `length` holds), and
dragging `b` re-hangs the rope. With `b` held at `(100mm, 0mm)` the rope is
`a cosh((x − 50)/a) + c` with `2a sinh(50/a) = 150` to 1e-9 (`catenary.sv`). `maximizes` stands it
up as an arch, reported a maximum.

### 2.18 Dido's problem: DOF 0, well

```
unit mm
use std

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b
  shore := line(a, b)
  strip := curve(a, b)
  length(130mm) strip
}
strip maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t))
```

`1 params, 1 equations, structural rank 1; DOF 0` and `maximum: …`: the strip enclosing the most
land against the shore is an arc of a circle, `R sin α = 50`, `2Rα = 130`, to 1e-9 (`dido.sv`).

---

## 3. Working checklist

1. Declare points, seeded roughly where you mean them — near enough to pick the right branch.
2. Declare lines, arcs and circles from them; use a chain for a contour.
3. State relations (levels, tangencies, equalities), then dimensions. Model relationships, not
   coordinate tables.
4. `fix` one point (`fix((0, 0)) a`), and any number that is given rather than solved.
5. Run `solventc`. Aim for `well` and DOF 0 unless freedom is intended.
6. On `conflict`, read the minimal set. On `over`, find the dimension the others imply. On
   `under`, ask what can still move.
7. Prefer a word to an unknown: equal angles are `l1 angle(l3, l4) l2`, an arc's extent is
   `length(L) a`, a motion's ratio may measure the drawing. An unknown you do need is declared,
   `param a: Length hint(…)`.
8. In space, draw each thing in the plane that shows it true, relate across planes with ordinary
   words, build a plane the solve turns on axes, and check with `--where PLANE`.
9. Add `claim`s for the consequences the figure was drawn to show.
10. Build solids from the solved profiles, then write their presentation in a `.svd`.

The worked corpus is `rust/examples/`, each file headed with its purpose. Read `rect_fillets.sv`
first; `gear_trace.sv` is the deepest; `engine.sv` and its `engine/` modules are the largest.

| task | example |
|---|---|
| tangent contours | [rounded rectangle](../rust/examples/rect_fillets.sv) |
| arc-length and equal-angle constraints | [belt wrap](../rust/examples/belt_wrap.sv), [reflection](../rust/examples/reflection.sv) |
| traced mechanisms and curves | [Jansen](../rust/examples/jansen.sv), [Peaucellier](../rust/examples/peaucellier.sv), [gear trace](../rust/examples/gear_trace.sv) |
| repeated features and components | [flange](../rust/examples/solid_flange.sv) |
| guided sweeps and lofts | [elbow](../rust/examples/solid_elbow.sv), [loft](../rust/examples/solid_loft.sv) |
| a reusable solid with a preview | [cylinder component](../rust/examples/vtwin/components/cylinder.sv), [its drawing](../rust/examples/vtwin/cylinder.svd) |
| views and projection | [bracket](../rust/examples/bracket.sv) |
| layouts in space | [skew axes](../rust/examples/skew_axes.sv), [spatial surfaces](../rust/examples/sphere_cone_cylinder.sv), [hypoid pitch cones](../rust/examples/hypoid_pitch_cones.sv), [sundials](../rust/examples/sundial/horizontal.sv) |
| rounds and chamfers | [rounds](../rust/examples/rounds/) |
| motions and generated solids | [indexed pattern](../rust/examples/solid_indexed_pattern.sv), [lantern generation](../rust/examples/lantern_generation.sv) |
| a point's envelope, an inner envelope and a claim over a motion | [Wankel](../rust/examples/wankel/wankel.sv) |
| a modular assembly | [engine](../rust/examples/engine.sv) |
