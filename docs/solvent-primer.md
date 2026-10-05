# Solvent: a primer for agents

Solvent is the language a drawing in this project is written in. You do not place geometry. You
declare entities, state what must be true of them, and the solver finds coordinates that satisfy
every statement at once. A sketch is a set of facts, not a sequence of commands, so reordering its
statements never changes its meaning.

**The one rule: a number inside a `hint(…)` clause is a seed, and every other number is not.** A
seed is where the solve starts; the solver may move it. Every other number is a constraint the
solver must satisfy and never rewrites. Your job is to state enough true things that exactly one
drawing satisfies them, then check that the diagnosis agrees (1.16).

| form | meaning |
|---|---|
| `p := point hint(x: 30, y: 10)` | start the point here; the solver may move it |
| `a distance(40) b` | require the distance to be 40 |
| `w := 40` | a value: arithmetic done while elaborating, never an unknown |
| `param a: Length` | an unknown the solve answers for, declared (1.6) |
| `claim vertical rail` | check a consequence without enforcing it (1.10) |

Deleting every seed leaves the set of solutions unchanged, without exception: a held number is
stated by `fix`, pinned with `==` like any pin (`fix(x == 0, y == 0) a`), never by a seed.

Scope:

- Models are `.sv` files. Presentation — `class`, `style`, view and section requests, callout
  placement — belongs in a separate [Solvent Drawing (`.svd`)](solvent-drawing.md). A model file
  holds geometry, constraints, hints and claims only. A dimension is still a constraint in the
  model; its callout is the drawing's.
- Everything here is what the implementation accepts, and every quoted report came from
  `solventc`. [`solvent-spec.md`](../solvent-spec.md) is normative, but specifies constructs that
  do not parse yet (`hint` as a statement, `path`). Examples in section 1 are fragments unless
  they show a report; section 2's are complete.

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

A newline ends a statement, except inside brackets, after a trailing chain joint (1.7), and at a
`}` closing a block.

### 1.2 Three kinds of number

| written | kind | meaning |
|---|---|---|
| `hint(x: 0, y: 0)`, `hint(r: 25)`, `hint(t: 0.4)` | seed | where the solve begins; the solver may move it |
| `distance(80)`, `angle(30deg)`, `t == 0.4` | constraint | must hold; never rewritten |
| `w := 100`, `param w := 100` | value | arithmetic done while elaborating; never an unknown |

Use seeds to select a useful starting pose and constraints to express the design.

**Brackets say what a thing is made of; `hint(…)` says where its solve begins.** Write
`c := circle(center: o) hint(r: 25)`, never `circle(center: o, r: 25)`: the centre is structure,
the radius a guess.

### 1.3 Statement forms

**A name is defined one way: `NAME := VALUE`.** The value may be a number, an element, a chain, an
instance, a group or a curve. `(NAME := VALUE)` is itself a value, so a name can be introduced
where the value stands: a chain link `(ab := line(a, b)) -> …`. A dimension's number defines no
name. `:=` binds loosest, which is why a link is named inside parentheses. A `label:` never
defines a name; it fills a slot (an argument, a child, a hint key, a group member). `param`
before a definition marks one of the document's inputs, and `param NAME: TYPE` with no value
declares an unknown (1.6).

```
use NAME[.NAME...]                      import a module                             (1.12)
unit NAME                               the unit the document's numbers are in       (1.6)
NAME := EXPR                            a value                                      (1.6)
param NAME[: TYPE] [:= EXPR] [hint(E)]  an input; with no value, an unknown          (1.6)
NAME := {LABEL: VALUE, ...}             values and geometry passed as one argument   (1.8)
[private] [construction] [NAME :=] KIND[(CHILD | hint(x: E, y: E), ...)] [hint(SCALAR: E, ...)]
     [knots [...]] [in REF]             an entity declaration; every part optional   (1.4)
NAME := point(x: XEXPR, y: YEXPR)       a computed point, drawn only as a curve       (1.9)
[NAME :=] plane(origin: R, toward: R[, from: R, fold: E | , from: R, offset: E
                                      | , u: (E,E,E), v: (E,E,E)[, o: (E,E,E)]])
                                        a datum (1.11), or a view with an attitude   (1.13)
NAME := plane(origin: R, toward: R, from: R, fold: NAME | fold: along LINE | ..., through: R
              | attitude: free[, offset: free]) [hint(u: .., v: .., offset: E)]
                                        a view the solve places                      (1.13)
NAME := sphere(center: R) | cone(axis: LINE) | cylinder(axis: LINE)  [hint(...)]
                                        a surface in space, on no sheet              (1.13)
in REF { statement* }                   every declaration inside is drawn in that plane
NAME := CHAIN                           a chain joined by `->`, named as a traversal (1.14)
NAME := face(EDGE, ..., holes: LOOP, ...)  a planar region                           (1.14)
NAME := solid(FACE, SWEEP...)           a face swept: depth:/from:/to:/through:, about:, along:
NAME := solid(SOLID)                    a body over a stock
NAME := solid(SOLID, under: MOTION, at: E | from: E, to: E)   placed by, or swept through, a motion
REF union REF | REF cut REF | REF bound REF   material added to, removed from, kept within a body
REF.FACE against REF.FACE               a mate between two parts' caps
NAME := surface | motion | envelope | patch | seam | vertex | edge(...)
                                        spatial geometry read after the solve        (1.15)
WORD[(ARGS)] REF  |  REF WORD[(ARGS)] REF  [hint(SLOT: E, ...)]
                                        a constraint, prefix or infix                (1.5)
claim CONSTRAINT                        judged, never solved for                     (1.10)
fix(FIELD == EXPR, …) REF               hold an entity's own numbers at the values stated:
                                        fix(x == 0, y == 0) p, fix(x == 5) p, fix(r == 25) c
ccw(a, b, c) | cw(a, b, c)              record a root choice; adds no equation
[NAME :=] Component(ARGS) [in REF]      an instance                                  (1.8)
component NAME(FORMALS) { statement* }  a component definition (not a value)
repeat N [as i] { ... }                 N unrelated copies                           (1.7)
cycle N [as i] { ... }                  N copies that close; `next`, `prev` in scope
repeat e in CHAIN [as i] { ... }        a copy per link of a named chain
cycle e in CHAIN [as i] { ... }         the same over a closed chain, closing
NAME := INSTANCE.POINT over FORMAL in (A, B)          a curve                        (1.9)
NAME := Component(ARGS).POINT over FORMAL in (A, B)
preview { statement* }                  drawn when the file is opened, not when used (1.12)
```

A reference is `name`, `name.field` or `name[expr]` (a copy of a repeated statement; the index may
read any number or binder in scope). Indices and fields chain: `l.e[2].p1` is `p1` of copy 2 of
`e` inside instance `l`; `cyl[0].small` reaches into copy 0's instance `cyl`.

### 1.4 Entities

| kind | children | own scalars | ends (for chains) |
|---|---|---|---|
| `point` | — | `x`, `y` | |
| `line` | `p1`, `p2` | | `p1 -> p2` |
| `circle` | `center` | `r` | |
| `arc` | `center`, `start`, `end` | `r` | `start -> end`, counter-clockwise |
| `spline` | control points, all named | | |
| `plane` | `origin`, `toward` | rotor `c`, `s` (never seeded by hand), plus a constant basis in space | |
| `curve` | its arguments | | |
| `sphere` | `center` | `r` | |
| `cone` | `axis` (a line; its start is the apex) | `half`, the half-angle | |
| `cylinder` | `axis` (a line) | `r` | |

**Seeds.** Every scalar is seeded by name in the trailing clause, keys in any order:
`p := point hint(x: 0, y: 0)`, `a := arc(center: c, start: s, end: e) hint(r: 5)`. An omitted
coordinate is 0; an omitted radius is computed from the geometry, never 0. A point with no clause
starts somewhere off the origin and apart from other unseeded points, and a solve writes the pose
it reached back into the source as its clause.

**A seed may read geometry**, but only other seeds, never solved values:
`hint(x: k.center.x + k.r, y: pin.y)`. Prefer naming a place, which needs no arithmetic:
`hint(at: pin)` starts a point where another starts; `hint(at: k, bearing: 90deg)` starts it on
circle `k`'s rim at that bearing; and `hint(at: a, toward: b, by: 0.5, turn: 90deg)` steps from
`a` the fraction `by` (1 if unsaid) of the way to `b`, turned `turn` about `a` — `by: 0.5` a
midpoint, `by: -1` a reflection. `along: l` in place of `toward:` steps by `by` times line `l`'s
run. A place drawn in another view is read in space and projected into the seeded point's view,
so `lp := point hint(at: inner) in n` starts at `inner`'s image in `n`. A clause with `at` has no
`x` or `y`. A seed whose constraints fix it without a choice (a midpoint, where two lines cross)
needs none at all. Seeds settle in statement order, so a seed reading
one written below reads its provisional start. With a `unit` line, a geometry read is a length:
write `pin.x - 10mm`, not `pin.x - 10`. A value may **not** read geometry: it feeds constraints,
and a seed must never change what a document says. Where a numeric formal is unbound (1.8), hints
read it as a provisional 0; constraints still treat it as unknown.

**Children** go by position or by label; a label skips earlier slots (`l := line(p2: c)` leaves
`p1` for a chain to thread). Any slot may be left out, or hold a `hint(…)` instead of a name,
which mints an anonymous seeded point:

```
l := line                                          two points: l.p1, l.p2
c := circle hint(r: 25)                            an unnamed centre, a seeded radius
a := arc                                           a.center, a.start, a.end
l := line(hint(x: 0, y: 0), hint(x: 60, y: 20))    two points, seeded
alt_a := line(A, hint(x: 15, y: 5))                one named end and one not
```

**The dotted path is the name.** `l.p1` is an ordinary point: it constrains, drags and takes
dimensions. Name a point yourself when several statements mention it. The exception is a spline:
its control points must be declared, named points (`s := spline(k0, k1, k2, k3)`); `s := spline`
alone is an error.

**The element's own name is optional.** `line(p1, p2)`, `point hint(x: 3, y: 4)` and
`arc(center: c)` are complete statements. If something later needs to reference an anonymous
element (a constraint applied from the app), a name is spliced in: `l0 := line(…)`, or
`(l0 := line(…))` inside a chain. A curve is always a definition's value, so always named.

### 1.5 Constraints

**Every constraint is a prefix or infix operator.** The word stands before its one operand or
between its two; everything else — the number, a selector, a third entity — goes in parentheses
on the word:

```
horizontal line1                    point1 horizontal point2
radius(25) circle1                  point1 distance(80) point2
distance(6) line1                   point1 symmetry(line1) point2
fix(x == 0, y == 0) p1             l1 angle(30) l2
fix(r == 25) c                      line1 tangent(side: left) circle1
length(40) arc1                     l1 angle(l3, l4) l2
```

One word may name several constraints, told apart by the operands' kinds, by fixity, or by what
stands in the parentheses:

| word | fixity | operands and options |
|---|---|---|
| `coincident` | infix | two points; a point to a line, circle, arc, spline or curve. In space (1.13): a point or line to a **plane**; a point or circle to a **sphere**; a point to a **cone** or **cylinder**. Not between two **solids**: material is added with `union` (1.14) |
| `distance` | infix | two points (length; `along: x`/`y` or `right`/`left`/`up`/`down` for a signed run or rise); a point and a line, or two lines (a magnitude; `side:` picks the side); two concentric circles or arcs (radial gap); a point and a datum (`along: u`/`v` signed ordinates, `along: n` signed distance along the normal, in space) |
| `distance` | prefix | a line: its length |
| `tangent` | infix | line–circle/arc (`at: p1`/`p2` for tangency at that end; `side:` for the centre's side); circle/arc–circle/arc (`external: true/false`); arc–line (`at: start`/`end`); spline or curve–line; in space, sphere–line, sphere–sphere, cylinder–line (round thing first); two cones at a point, `k1 tangent(M) k2` |
| `equal` | infix | two lines (length) or two circles/arcs (radius) |
| `curvature` | infix | spline or curve and a circle/arc: the circle becomes the osculating circle there. On a traced curve, exact (the body's Taylor orders); refused only for a body using a relation with no Taylor form, named in the error |
| `horizontal`, `vertical` | prefix / infix | a line; or two points with no line between them |
| `angle` | infix | two lines, directed (below). With a second line pair instead of a number, `l1 angle(l3, l4) l2` equates two angles |
| `angle` | prefix | a cone: its half-angle |
| `radius` | prefix | a circle, arc, sphere or cylinder |
| `length` | prefix | an arc: radius × sweep, counter-clockwise from `start` to `end` (a magnitude) |
| `symmetry(line)` | infix | two points |
| `midpoint` | infix | a point and a line |
| `parallel`, `perpendicular` | infix | two lines |
| `project` | infix | two points, each `in` a plane: two images of one point in space (1.13) |
| `fix` | prefix | an entity's own numbers, each named and pinned: `x`/`y` of a point (either or both), `r` of a circle, arc, sphere or cylinder, `half` of a cone (degrees) |
| `ccw(a, b, c)`, `cw(a, b, c)` | call | all three points in the parentheses; `ccw` means `c` is left of ray `a → b` |

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end;
`line tangent circle` is the ordinary one. `a distance(80, along: x) b` is signed from `a` to `b`.

#### Which way: directions are words

| written | means |
|---|---|
| `p distance(12) ax` | 12 from line `ax`, **either side**; the seed picks |
| `p distance(12, side: left) ax` | left of `ax`'s direction `p1 → p2`; `right` likewise |
| `l1 distance(6, side: left) l2` | `l2`'s `p1` lies left of `l1` |
| `a distance(60, along: x) b` | `b.x − a.x = 60`; `along: y` is the rise |
| `a distance(60, along: right) b` | the same, said as a word; also `left`, `up`, `down` |
| `l1 angle(30) l2` | 30° **counter-clockwise** from `l1`'s direction to `l2`'s |
| `l1 angle(30, sense: cw) l2` | 30° clockwise, i.e. `angle(-30)` said openly |
| `l1 angle(l3, l4) l2` | angle `l1 → l2` equals angle `l3 → l4`, both counter-clockwise |
| `l1 angle(l3, l4, sense: cw) l2` | it equals the **mirror image** of `l3 → l4` |
| `length(40) a` | 40 along arc `a`, counter-clockwise from `start` to `end` |
| `l tangent(side: left) c` | the circle's centre is left of `l` |
| `p distance(5, along: n) P` | `p` is 5 along plane `P`'s normal (toward its viewer), in space |

- **A distance from a line is a magnitude.** A negative value is refused; the side is `side:`. A
  component that must work either way takes a `Side` formal (`s: Side`, called `Part(…, s: right)`)
  and writes `side: s`. Where a sign is arithmetic rather than convention, use a signed datum
  ordinate (`p distance(v, along: v) f`).
- **Runs, rises and angles keep their signs**, because components compute them; the words are how
  a drawing should say it.
- **`angle` is directed**: the full-turn angle from `l1`'s direction to `l2`'s, counter-clockwise
  positive. It fixes the side as well as the tilt, so a bearing needs no orientation predicate.
  Swapping the lines or reversing one changes the reading.
- **An angle may equal another angle**, with no number and no unknown: a bisector is
  `ab angle(ad, ac) ad`, a reflection `incoming angle(mirror, outgoing) mirror`. That is one
  equation; the same tie through a shared unknown is two equations and an unknown (2.15).
  Two numbers in the parentheses are refused.
- **`length(L) a`** is only for arcs (a line's length is `distance`; a circle has no ends). The
  sweep is read counter-clockwise in `(0°, 360°]`, so measure the long way round by going round
  counter-clockwise. It may read an unknown and is drawn with a `⌒` mark.

**Slots a constraint owns** (a contact's curve parameter, always called `t`) are normally omitted.
Seed one with `p coincident s hint(t: 0.4)`; pin one with `p coincident(t == 0.4) s`.
Pin one to an unknown and contacts on the same curve share it: with `param s: Angle hint(318)`,
`path tangent(t == s) ground` and `path curvature(t == s) osc` state the circle osculating the
path *where* it touches the ground — one place, one parameter, seeded where it is declared (a
`hint(t: …)` beside the pin is E040). A contact on another curve pinned to `s` is refused (E040).

**Tangency trap.** If the contact point is already held on the circle, state the tangency *at*
it: `line tangent(at: p2) circle`, `arc tangent(at: start) line`. `p coincident circle` plus a bare
`line tangent circle` is rank-deficient at every solution; the diagnosis reports a motion "blocked
at second order" rather than a DOF, but the at-form is the one to write.

#### Across views, a word means space

When a relation's operands are drawn in different views (1.13), it relates their places in space;
no selector is needed:

| written across views | means |
|---|---|
| `a coincident b` | the same point in space |
| `a distance(30) b` | the true length |
| `a distance(5) l`, `l1 distance(17.5) l2` | to the infinite line; along the common perpendicular (magnitudes) |
| `a coincident l`, `a coincident c` | on the line's extension; on the circle in its own view |
| `l1 angle(90deg) l2` | the **unsigned** angle between directions, 0–180° |
| `parallel`, `perpendicular`, `equal` | directions, and true lengths |
| `a midpoint l`, `a symmetry(l) b` | the midpoint in space; a half turn about the line |

A word with no meaning in space (`horizontal`, `along: x`, `tangent` between drawn figures, the
angle-equals-angle form) is **E062** across views, as is a datum point read beside another view's
points; `sense:` and `side:` there are **E040**. Radii, and `along: u`/`v` ordinates on a datum,
mean the same in every view.

### 1.6 Numbers, names and units

**Expressions.** `+ - * / ^`, parentheses, `pi`, and `sqrt abs sin cos tan asin acos atan atan2
exp ln log floor ceil round min max hypot`. **Trigonometry is in degrees.**

**Values.** `w := 60` as a statement is worked out while elaborating; never an unknown or a seed.
A value may read others written anywhere in its body or an enclosing one; one defined in terms of
itself is E041. A dimension's number reads names and defines none: write `w := 60`, then
`a distance(w) b`. Its callout reads `w`, and typing a number over it changes the line `w` is
defined on.

**Inputs.** `param` before a definition marks a number the document takes from outside — the
one kind a host may give another value (the case library's arguments, `examples::with_params`):

```
param bore: Length := 50mm        // an input with a value; the type is optional
param beta: Angle hint(30deg)     // an input nothing binds: an unknown, seeded
stroke := bore * 1.2              // a value
```

**Unknowns are declared.** An input with no value is one unknown of the drawing, tying together
everything that reads it: a dimension (`a distance(beta) b`), a fold (`fold: beta`, 1.13), a
contact's place (`t == s`, 1.5), a value over it (`q := 2 * beta`). It names its type — `Length`,
`Angle` or `Scalar` — since nothing else says what it is, and its seed is its `hint(…)`, which a
solve writes back. Inside a component the unknowns are the formals a call leaves unbound (1.8).
**A name nothing declares is E101**, wherever it is read: a misspelling is never a degree of
freedom. The tie must be affine in one unknown (`a`, `a / 2`, `2 * a + 5`); `a * a`, `sin(a)` and
two unknowns in one dimension are errors, as is reading a declared `Length` where an angle goes.
`param` stands at the top of a document (or its `preview`): a component's inputs are its formals,
a block's copies share the document's, and a module's numbers have values. To equate two angles,
prefer `l1 angle(l3, l4) l2` (1.5).

**Dimensions are checked.** Two base dimensions, length and angle. `*` and `/` derive, `+` and `-`
demand agreement, and the result is checked against its slot: `a distance(45deg) b` is an error,
as is a length plus an angle. A bare number is dimensionless and a *context* may accept it —
`distance(80)` is a length because the slot says so, `sin(30)` reads degrees — but a context does
not speak for a second operand: `90 / N + ivp` adds a plain number to an angle and is refused;
write `90deg / N + ivp`.

```
unit mm                             // the document's unit
phi := 20deg
ivp := tan(phi) * 1rad - phi        // inv φ = tan φ − φ holds only in radians, and says so

a distance(80mm) b
c distance(1' 6 3/16") d            // one literal: spaces separate the parts, as in 3 1/2
l angle(45deg) m
```

- **Without a `unit` line** the document is in unnamed drawing units, and a suffix like `mm` or
  `"` is refused.
- A name takes its dimension from its value's unit, not from where it is used: `w := 80` used as a
  length does not make `w` a length; `w := 80mm` does, as does a formal declared `Length`.
- `3/16"` is a division (a `Length^-1`); a lone fractional inch is `0.1875"` or `0 3/16"`.
- `pi` is dimensionless; `tau` and `turn` are one full turn. `floor`/`ceil`/`round` take plain
  numbers.
- **There is no string literal**: `"` is the inch mark, and a word argument is bare (`at: start`).
- **Built-in names cannot be redeclared.** A value, input, formal or block index named `tau`,
  `pi`, `min`, … does not shadow the built-in, and ends up with two values (`tau := 35deg` passed
  to a `tau: Angle` formal arrives as a full turn). It is W112 at the declaration; rename it.

**Measurements of the solved drawing.** `length(l)` (line or arc, in space), `radius(c)` (circle,
arc, sphere, cylinder), `distance(a, b)` (two points, or a point and a line produced) and
`angle(l1, l2)` (0–180°) take geometry names and read as `Length`/`Angle` under a `unit` line, so
`length(a) / length(b)` is a plain ratio. They are allowed **only** in a motion's `ratio:`,
`phase:` and `advance:` (1.15), which are read after the solve. Anywhere else — a value, a seed, a
constraint's number, a solid's extent or placement angle — they are **E107**, because the number
is needed before there is a solve.

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

- `->` alone is a plain corner; `-> tangent` is a corner that is also tangent there, written as
  the regular at-the-point tangency.
- The shared point may be named by either side (or both, agreeing) or by neither, in which case
  the chain mints it: `(l1 := line) -> (l2 := line)` is two lines and three points.
- A joint may state several relations, `A -> equal angle(30deg) B`, with the marker on either
  side of the words or both.
- A word without a marker relates without welding: `(l1 := line(a, b)) perpendicular
  (l2 := line(c, d))` is two separate lines at a right angle.
- Declarations and names may mix. At a corner with an element declared elsewhere, the declared
  side names the shared point, usually by the other's child: `(t := line(p3, k.start)) -> tangent k`.
- `-> close` seals the loop back to the first link. Links may be anonymous:
  `line -> tangent arc -> tangent line` is a whole contour with no names.
- `equal` is polymorphic (length between lines, radius between round things);
  `a equal b equal c` is two statements.

**Repetition.** `repeat N { … }` makes N unrelated copies. `cycle N { … }` makes N copies that
close, with `next` and `prev` in scope. `as i` binds the index. Copies are reached as `p[0]`.
(`ring` is refused; use `cycle`.)

**A block may end mid-joint.** A trailing joint before `}` threads onto the next copy's first
link. A `cycle` wraps, closing the loop with no `close`; a `repeat` leaves its last joint unstated,
so `repeat N { line -> angle(a) }` is an open polyline of N sides. Both boundary links must be the
body's own declarations, and at most one boundary slot may name its point. Since a statement ends
at `}`, `cycle 4 { (s := line) -> perpendicular equal }` is a square but for size and pose.

**Repetition over a chain.** `repeat e in CHAIN { … }` makes one copy per link of a named chain
(1.14), in walk order, with `e` that copy's edge — usable as an operand, a field (`e.p1`), an
argument or a `surface` edge. The chain may be open or closed, declared anywhere, or reached
through an instance or group (`refs.pinion.profile`). Otherwise the copies behave as a `repeat`'s:
named `#<id>.<k>.m`, reached as `m[2]`, one statement however many copies (so seeds inside are not
written back, and a gesture on one copy is refused). `cycle e in CHAIN` closes like `cycle N` and
is refused on an open chain. A hole at every edge's midpoint, sized by index (`holes.sv`):

```
unit mm
a := point
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 40)
d := point hint(x: 0, y: 40)
fix(x == 0, y == 0) a
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
```

`solventc --where '#22.2.hole' holes.sv` (the block is statement 22; copy 2 is edge `cd`):

```
holes.sv: solved
  18 params, 18 equations, structural rank 18; DOF 0; 5 components: DOF 0, 0, 0, 0, 0; 2 rigid cluster(s) in the distance graph
  #22.2.hole.center.x = 30
  #22.2.hole.center.y = 40
  #22.2.hole.r = 4
```

Errors: iterating over something that is not a named chain is E103 at the reference; an undeclared
or private chain is E101; a body declaration with the same name as the edge binder is E001.
`edge_tabs.sv` puts a tab on every edge of a plate.

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

- **Formals** are a `group`, an entity kind (`point`, `line`, `circle`, `arc`, `plane`, …) or a
  number type (`Length`, `Angle`, `Int`, `Scalar`, `Side`).
- **Passing an entity is aliasing**, not a constraint: formal and actual are one entity, so a
  component boundary costs nothing.
- **Entities by position, numbers by label**: `Rung(l0, r0, len: 50)`, never `Rung(l0, r0, 50)`,
  and nothing positional after a label. Either mistake is **E004** at the argument — an argument
  one place off would otherwise bind silently to the wrong formal.
- **An unbound numeric formal is an unknown** named under the instance (`c.theta`): that is how a
  mechanism is drawn with its crank free (2.9.1). A call seeds one by leaving it unbound with a
  `hint(…)` in its place, `Crank(o, datum, theta: hint(30deg))`; the body reads it as the instance's
  own, and the sheet as `c.theta`.
- **Scope is closed.** A body sees only its formals and its own declarations; root and module
  values, geometry and standard datums must be passed in. Component definitions (the file's own
  bare, a used module's by path) and built-in functions remain callable. Repetition blocks share
  the enclosing component's scope.
- **Everything an instance makes is public by dotted name** (`t0.e`, `five.s[0].p1`, a named
  dimension as `t0.w`) unless declared `private`, which keeps the name inside the component.
  `construction` and `centerline` mark purpose independently of privacy; the editor shows
  construction geometry, and drawing sheets hide it unless styled. A private entity may still be
  passed explicitly to another component, but passing a whole instance grants no access to its
  private members.

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

Callers can name `pattern.hole[0]` but not `pattern.layout` or `pattern.drill[0]`.

`use std` also provides `std.CenteredRectangle(center, w:, h:)`, an axis-aligned rectangle with
public corners `a`–`d` and boundary `loop` (its centring diagonal is private construction):

```solvent
outer := std.CenteredRectangle(center, w: 18mm, h: 18mm)
inner := std.CenteredRectangle(center, w: 14mm, h: 14mm)
section := face(outer.loop, holes: inner.loop)
```

**Groups** bundle related values and geometry into one argument. The members are written in
braces straight after `:=`, and may run across lines:

```solvent
unit mm
use std

sizes := {length: 20mm}
layout := {frame: std.front, origin: std.origin}
component Bar(layout: group, dims: group) {
  tip := point hint(x: layout.origin.x + dims.length * layout.frame.c,
                 y: layout.origin.y + dims.length * layout.frame.s)
  reference := line(layout.frame.origin, layout.frame.toward)
  axis := line(layout.origin, tip)
  axis parallel reference
  distance(dims.length) axis
}
bar := Bar(layout, dims: sizes)
```

Groups nest, by name (`design := {bar: bar}`) or written in place, to any depth:

```solvent
design := {
  bar: {at: o, size: {length: 2cm, half: 1cm}},
  pin: {length: design.bar.size.half, at: o},
}
part := Bar(design.bar)          // a nested group is handed on like any group
```

Numeric members keep their units; geometry members alias and add no solver state. A
component instance can be passed as a group, exposing its geometry (`layout.pivot`,
`layout.bank[0].axis`) but not its local values — bundle those in an explicit group. A group
formal must be supplied, and a missing member is an error. A traced component (1.9) needs fixed
scalar or entity formals, so pass members individually there.

**Derive, don't restate.** The V-twin keeps a few shared design inputs (bore, throw, rod length,
piston height, wall thickness, hardware choices) and derives everything else — the cylinder's
mouth and top, mating pockets with explicit clearances, groove sizes from the chosen ring — so
changing a source dimension carries its dependents with it.

### 1.9 Curves

A curve is **a point of a component, traced as one of its numeric formals runs over an
interval**. There is no curve family: an involute, a cycloid and a leg's stride are three
components.

```
NAME := INSTANCE.POINT over FORMAL in (A, B)          a drawn instance's point
NAME := Component(ARGS).POINT over FORMAL in (A, B)    an instance written in place, never drawn
```

- **Over a drawn instance**: `path := leg.toe over theta in (0, 360)` is where the drawn leg's toe
  goes as `theta` runs. The trace is anchored at the drawing's pose, so it needs no extra seeds.
  Leave `theta` unbound and the crank is the drawing's freedom (`jansen.sv`, `peaucellier.sv`).
- **Written in place**: `Involute(base, phase: a0).p over u in (…)`. The anchor is the value the
  call gives the swept formal, or the interval's start.

A component's point is placed one of two ways:

- **Computed**: `p := point(x: XEXPR, y: YEXPR)` over the formals and values. Such a component can
  only be traced, never drawn (2.8).
- **By constraints**: any point the body declares, held where the body's statements put it. This
  is how a person states a curve — "the end of a taut string as it unwinds" (2.9). The body must
  be square: as many equations as its own coordinates.

The swept formal is an `Angle` or a `Length`; the traced point must be one the component places,
not one it was given. A `plane` formal is the cheapest frame to pass (origin, second point and
bearing in one), and its `f.angle` (degrees) lets seeds such as `hint(at: c, bearing: u + f.angle)`
follow a tilted datum.

A locus usually has several solutions. A body selects one by, strongest first: a **signed**
constraint (a point-to-line distance's sign chooses a winding); an **orientation predicate**
`ccw`/`cw` (no equation; read at the anchor and carried by continuity); a **seed**.

**The ellipse** is `std.Ellipse(f: plane, a: Length, b: Length, u: Angle)`:
`e := std.Ellipse(f, a: 40, b: 25).p over u in (0, 360)` is the rim, and `p coincident e`,
`e tangent l` and `e curvature k` are exact. There is no `ellipse` element.

### 1.10 Claims

`claim` before a relation asserts it without enforcing it: `claim vertical rail`. A claim joins no
solve, count or conflict set; the drawing is identical with it deleted. The diagnosis judges it:

- **theorem** — holds, and the document implies it;
- **violated** — does not hold (the CLI prints `claim refuted:`);
- **consuming** — holds only where the solve happened to land; enforcing it would cost a freedom.

Use claims for what a figure was drawn to show (the altitudes concur; the traced path is
straight). A claim may not own an unknown, so claiming a curve contact or reading an unknown
is an error.

A claim may also run over a named motion's roll: `claim over rotor_turn in (0deg, 1080deg) {
arc_rotor_at clear(0.25mm) housing }` reads the solids placed under the motion (`solid(arc_rotor,
under: rotor_turn, at: 0deg)`) at each sampled pose, solving nothing again, and reports the worst.

Three claim words apply only to solids (1.14): `claim bore inside stock`,
`claim disc clear(2mm) cyl`, `claim head fits(0.15mm) trap`. The report gives the measurement; a
claim decided within the faceting of a round face is **undecided**:

```
  claim bore inside stock — undecided, measured 0 (interval [-0.0006000000000000001, 0.0006000000000000001])
    unresolved: containment is within curved-faceting uncertainty
  claim bore clear(2mm) stock — refuted, measured -9.999665339174008 (interval [-10.00026533917401, -9.999065339174006])
```

### 1.11 Datums

A `plane` with no attitude written is a **datum** on the page: an origin, a point it faces toward,
and a unit rotor slaved to the chord between them. It adds no freedom. Hints may read `f.angle`
(degrees), or `f.c` and `f.s` (cosine and sine of its starting direction). (`frame` is refused.)

A datum gives **signed local coordinates**: u points from `origin` to `toward`, v to its left.

```sv
p := point                          // f is a datum
p distance(20mm, along: u) f
p distance(-3mm, along: v) f
```

Each ordinate is an ordinary constraint: zero, negative, or an unknown, and the datum may itself
move under constraints on `p`. Membership is separate: `p := point in front` puts `p` in a view,
while a relation to `f` measures against the datum. No component has an implicit frame.

**Model relationships, not coordinate tables.** A rectangle states perpendicular sides, a width, a
height and a position relative to a datum; a bolt pattern states a pitch circle and spacing. Use an
ordinate where the design calls for a datum measurement, not two for every corner. The standard
library deliberately has no coordinate-placement helper. Hints may compute good starting
positions, but the constraints must express the geometry on their own.

### 1.12 Modules and previews

```
use engine.dims          // engine/dims.sv beside the document or an ancestor, else the library
use std                  // std.front, std.up, std.origin; std.ThreeViews (2.10),
                         // std.CenteredRectangle (1.8), std.Ellipse (1.9), std.Polygon, std.Hex
use hardware             // fasteners and fittings: hardware.hexbolt14_af, …
```

A module is a Solvent document read for its components and its top-level values and groups; its
own drawing is not drawn. **Nothing is imported bare**: a module's names are always written with
its full path — `engine.parts.Crank(…)`, `engine.dims.bore`, `components.dims.vtwin_dims` (values
and groups in the root body, components anywhere). A module names its own definitions bare. Only
modules the file itself `use`s may be named; a transitive import needs its own `use` (the error
says so), and that includes `std` for `std.front`. Two modules may define one name; two
definitions in one file are E071; a missing module is E070, and a module's own error is reported at
the `use` that brought it in. A callout shows a param's bare name (`D`, not `engine.dims.D`).
`engine.sv` is the worked case: a four-cylinder engine as a dimension module, a parts module and
one module per part.

**Standard datums.** `std.front` (u right, v up) and `std.up` (u up, v left) are fixed at the page
origin `std.origin`, created only if referenced. Passing one as an argument does not imply `in`:
unassigned geometry stays on the page unless you write `in std.front`.

**Previews.** A component file may end with one file-level `preview { … }` holding sample datums,
values and an instance. Opening the file (directly or via a `.svd`) solves those statements;
`use` omits the whole block, units and values included. Imports resolve beside the opened file and
then its ancestors, so a component file opens with the same `use` paths it has in the assembly.

```sv
preview {
  unit mm
  cyl := Cylinder(std.up, fw: 12mm, dims: components.dims.vtwin_dims)
}
```

The cylinder's bore follows the datum's u axis, so `std.up` stands it upright. The call may be
unnamed, but keep `cyl :=` when a drawing or another statement references its members.
`vtwin/components/cylinder.sv` is the complete example; `vtwin/cylinder.svd` draws its views.

### 1.13 Planes and views

A `plane` is also a **view**: it carries a constant attitude in space (the page's when none is
written). A point says which view it is drawn `in`, and `a project b` says two points are two
images of one corner — their coordinates along the shared fold line agree (one equation).

- **`fold:`** is the fold line's bearing in the parent view. From the page, `0deg` folds up a top
  view and `-90deg` a right view; the new view's second axis points away from the parent's viewer,
  so distance from the fold line is depth (third-angle projection). Any attitude is two folds
  away, or give it outright as `u: (…), v: (…)`.
- **`from: P, offset: 12mm`** with no `fold:` is a plane *moved*, 12 along its parent's normal.
  Parallel views share no fold line, so this is a stack, not a projection — and where a section is
  cut (1.14).
- **`o:`** beside `u:`/`v:` says where in space that basis stands (written for a stand-off plane).
- **`in top { … }`** writes membership once for every declaration inside, `cycle` copies
  included. An instance joins a view whole: `t := Tooth(…) in top`. Inside a component, `in view
  { … }` over plane formals lets a part carry its own views (`engine/conrod.sv`), and `repeat flag
  { … }` over a 0/1 `Int` formal omits a view for an instance that does not show in it.
- The standard library lays out three views at once (2.10).

**The workplane rule.** A point drawn `in` a view is that view's lift into space; a point with no
membership is on the page and has no place in space. Within one view a relation is the 2D one;
across views it is the relation in space (1.5), and naming a page point there is E062. `project`
ties two drawn images of an undrawn point. `p coincident P`, `l coincident P` and
`p distance(d, along: n) P` put a point or line on a plane in space whatever view it is drawn in
(a view's own points are on it already: E061).

**The role rule.** A plane's own `origin` and `toward` place the view on the sheet. Beside only
other datum points they are sheet layout (`o distance(120) o2`); beside points of their own view
they are that view's datum (`o2 distance(15) b` is an ordinate); otherwise their membership decides.

**A view may be solved for.** A plane is fixed unless its brackets name an unknown; a plane's
`hint(…)` seeds only the attitude and offset it leaves free (2.11, 2.13):

```
param beta: Angle hint(30deg)
side := plane(origin: o2, toward: t2, from: front, fold: beta)
aux := plane(origin: o3, toward: t3, from: front, fold: along l)
sec4 := plane(origin: o4, toward: t4, from: front, fold: 0deg, through: m)
q := plane(origin: o5, toward: t5, attitude: free, offset: free) hint(u: (0, 1, 0), v: (0, 0, 1))
```

- `fold: beta` over a declared unknown (`param beta: Angle`, or a formal no call binds) is a fold
  the solve answers, seeded by the unknown's own `hint(…)`; `hint(fold: …)` on the plane is E040.
- `fold: along l` folds square to the parent about line `l` drawn in it, and follows the line.
- `attitude: free` is three freedoms; `offset: free` is one; `through: m` stands the plane where
  a point of another view is.
- A view folded from a solved view follows it; `project` over a solved view is the projector rule
  in space.
- A solved view's *place on the sheet* (its origin and toward) is presentation, held silently and
  not counted unless a statement names it. A free view with nothing else in it reports DOF 4 with
  no `fix` written.
- Errors: a seed for a stated quantity is E040; a position stated twice, or a fold along another
  view's line, E064; views that come out parallel under a `project`, E065.
- `against` (1.14) works with solved views when the two planes turn together (one derived `from:`
  the other, or both from one, with no fold); a solved datum offset then makes the gap a row of the
  solve. Views that turn apart are E066.
- `solventc --where side` reports `side.u.x` … `side.o.z`, and a report lists what is still free:
  `free views: side.attitude, side.offset`.

**Spheres, cones and cylinders** live in space, on no sheet (the glass box draws them).

- `s := sphere(center: p) hint(r: 12)` (or `sphere(p)`): the centre is drawn in some view.
  `radius(12) s`, `a coincident s`, `s tangent l` and `s tangent s2` are spatial. `k coincident s`
  puts a whole circle `k` on the sphere (a gear blank's toe circle on its end sphere); `s tangent k`
  is refused, since a circle and sphere may touch at a point or all round.
- `gc := cone(axis: gax) hint(half: 60deg)`: the apex is the axis's start, opening toward its end;
  the half-angle is written in degrees. `angle(60deg) gc` states it; `p coincident gc` puts a point
  on the nappe the axis points into.
- `bore := cylinder(axis: ax) hint(r: 8)`: `radius(8) bore`, `p coincident bore`, and
  `bore tangent l` (the side read from the seed).
- `gc tangent(M) pc`: two cones share one tangent plane at M. State `M coincident gc` and
  `M coincident pc` beside it, as an on-circle stands beside a tangency at a named end.
- A line lying on a cone or cylinder is not yet a relation; state it of the line's points.

`sphere_cone_cylinder.sv` shows one of each; `hypoid_pitch_cones.sv` is 2.13.

### 1.14 Faces and solids

**A solid is a term, never a step.** A feature tree is a history: step *n* acts on the anonymous
body left by step *n − 1* and names faces by creation order. In Solvent a solid is a face swept,
or a stock **plus everything in `union` with it, minus everything that `cut`s it, within
everything that `bound`s it**. The order lives inside a term, never between statements: `bore cut body` may be
written fifty lines above `body := solid(…)` and means the same.

**Nothing about a solid is solved for.** A solid owns no parameters; numeric extents are worked
out at elaboration, and a `through:` extent follows its target after the solve. The geometry swept
is the drawing, solved in 2D as usual.

#### Faces

A face is a planar region bounded by edges the drawing already has, on the plane its edges are
drawn `in` (the page by default). Edges are listed in traversal order, each sharing a point with
the next. A **circle is a whole loop by itself** (`face(hole)`), and cannot stand among lines.

```
unit mm
a := point
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 40)
d := point hint(x: 0, y: 40)

horizontal (ab := line(a, b)) ->
vertical   (bc := line(b, c)) ->
horizontal (cd := line(c, d)) ->
vertical   (da := line(d, a)) -> close

a distance(60) b
b distance(40) c
fix(x == 0, y == 0) a

sec := face(ab, bc, cd, da)
block := solid(sec, depth: 30mm)
```

```
$ solventc block.sv --where block.volume
block.sv: solved
  6 params, 6 equations, structural rank 6; DOF 0; 2 rigid cluster(s) in the distance graph
  block.volume = 72000
```

The face and solid added nothing to the rectangle's six unknowns and six equations.

- **Holes**: `face(barrel, holes: core)` or `holes: first, second` — circles or named closed
  loops, strictly inside the outer boundary on the same plane, not touching each other. Works for
  extrusions and revolutions. A hole *through a part* is usually better as a solid that `cut`s the
  body, which keeps the section simple and names the bore's wall.
- **Named chains**: `profile := (ab := line(a, b)) -> … -> close` binds the traversal, so
  `block := solid(profile, depth: 30mm)` needs no face statement (same volume, no new unknowns).
  In a component the chain is an ordinary member: `solid(boss.profile, depth: 8mm)`, with edges
  still `boss.ab`. Every joint in a named chain must carry `->`. An **open** named chain
  (`trail := … -> (cd := line(c, d))`) cannot be swept directly; close it with
  `face(trail, -> close)`, or finish the chain with `-> (da := line(d, a)) -> close`.
- **A face closes itself.** A *point* in the list is a corner the walk goes straight through, and
  `-> close` seals the walk back to its first item. `brief := face(a, bc, cd, -> close)` mints the
  two missing straight runs. A minted run has class `.closure` (hidden by default) and still names
  a face of the swept solid: `block.close0`, `block.close1` beside `block.bc`, skipping names
  already in use. Three refusals keep mistakes visible: the last-to-first gap is minted only under
  `-> close`; a gap between two *edges* is E080 (edges out of order); and an edge between two gaps
  is E080 (`face(a, bc, d, -> close)` could walk `bc` either way).
- **A curve stands in a face as a stretch**, `k from a to b`, between two points held on it by
  contacts; it follows the solve. (A clamped spline already ends at its end control points, so it
  is an ordinary edge.) One stretch closed by `-> close` is a loop with its chord:

```
unit mm
component Par(o: point, u: Length) {
  p := point(x: o.x + u, y: o.y + u * u / 1mm)
}
o := point
fix(x == 0mm, y == 0mm) o
k := Par(o).p over u in (-2mm, 2mm)
a := point hint(y: 1mm)
b := point hint(y: 1mm)
a coincident k hint(t: -1)
b coincident k hint(t: 1)
fix(x == -1mm) a
fix(x == 1mm) b
cap := solid(face(k from a to b, -> close), depth: 3mm)
```

`4 params, 4 equations; DOF 0`, and `cap` is the parabolic cap, 4/3 mm² × 3 mm = 4 mm³. A curve
without `from … to …`, a point not held `coincident` the curve, and a stretch from a point to itself
are refused. `examples/solid_tooth.sv` is an involute tooth written this way.

- **A face may be written inline** where one sweep uses it:
  `block := solid(face(ab, bc, cd, da), depth: 30mm)` or
  `solid(face(a, bc, cd, -> close), from: -30mm, to: 0mm)` (both 72000). Name it when several
  sweeps reuse it (`barrel_f`, `core_f` in `vtwin/components/throttle.sv`).

#### Sweeps

- **Prism**, along the face's normal. `depth: 30mm` is the material *behind* the visible face,
  i.e. `from: -30mm, to: 0mm`; write `from:`/`to:` when the face is not at an end. `through: body`
  spans the target's stock and additions both ways along the normal, ignoring other cutters, and
  follows the solve.
- **Revolution**, about a line **in the face's own plane**: `about: ax` is a full turn;
  `sweep: 90deg` a partial one (caps `start` and `end`); `sense: cw` reverses it.
- **Guided sweep or loft** along a directed line or circular arc:
  `duct := solid(section, along: guide)`, `reducer := solid(inlet.profile, outlet.profile,
  along: guide)`. The start section is perpendicular to the start tangent, an explicit end
  section to the end tangent; omitting it repeats the start. Along an arc the section turns with
  the tangent. Holes pair in written order, and paired loops need equal source-edge counts. Side
  names come from the inlet; caps are `start` and `end`. See `solid_elbow`, `solid_loft`.

```
unit mm
p0 := point hint(x: 10, y: 0)
p1 := point hint(x: 14, y: 0)
p2 := point hint(x: 14, y: 6)
p3 := point hint(x: 10, y: 6)

horizontal (e0 := line(p0, p1)) ->
vertical   (e1 := line(p1, p2)) ->
horizontal (e2 := line(p2, p3)) ->
vertical   (e3 := line(p3, p0)) -> close

q0 := point
q1 := point hint(x: 0, y: 10)
ax := vertical line(q0, q1)
fix(x == 0, y == 0) q0
q0 distance(10) q1
q0 distance(10, along: x) p0
q0 distance(0, along: y) p0
p0 distance(4) p1
p1 distance(6) p2

sec := face(e0, e1, e2, e3)
ring := solid(sec, about: ax)
```

```
$ solventc ring.sv --where ring.volume
ring.sv: solved
  10 params, 10 equations, structural rank 10; DOF 0; 2 components: DOF 0, 0; 3 rigid cluster(s) in the distance graph
  ring.volume = 1809.56
```

Pappus gives `2π · 12 · 24 = 1809.557`: a static solid's volume is its exact boundary's, not its
facets'. With `sweep: 90deg` the report says `452.389`, and `ring.start.area` and
`ring.end.area` are both 24.

#### Bodies

Add to the 60 × 40 plate:

```
o := point hint(x: 30, y: 20)
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

```
$ solventc plate.sv --where body.volume
plate.sv: solved
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.volume = 69643.8
```

`72000 − π · 5² · 30 = 69643.8`. Writing `bore cut body`
before `body := solid(stock)` gives the same number: both sides of the body rule are sets.

- **A swept solid takes features directly**: `bore cut stock` makes `stock` the body (volume
  `69643.8`), keeping its face names (`stock.near`, `stock.ab`) and reaching the bore's through it
  (`stock.bore.hole`). Write `body := solid(stock)` only when the bare sweep and the finished part
  both need names.
- **`cut` with `through:`** for a cutter spanning the part:
  `pinhole := solid(face(hole), through: body)` then `pinhole cut body`. `through:` only makes
  the cutter; the `cut` statement applies it. For a blind pocket use `depth:` or `from:`/`to:`.
- **`bound` keeps what lies within.** `tip bound body` keeps only the part of the body inside
  `tip` — one statement instead of `heel − (heel − tip)`:

  ```solvent
  body := solid(heel)
  tip bound body
  toe cut body
  ```

  Union comes first; `cut` and `bound` commute.
- **Name intermediates the sets cannot express.** A boss standing in a pocket's floor is not
  `pocket cut body` + `boss union body` — that is stock ∪ boss − pocket, and the pocket eats the boss:

  ```
  // the plate again, with `o` at its middle
  rim := circle(center: o) hint(r: 15)
  stud := circle(center: o) hint(r: 5)
  radius(15) rim
  radius(5) stud
  rim_f := face(rim)
  stud_f := face(stud)

  stock := solid(sec, depth: 30mm)
  pocket := solid(rim_f, depth: 10mm)
  boss := solid(stud_f, from: -10mm, to: -4mm)

  shell := solid(stock)
  pocket cut shell

  body := solid(shell)
  boss union body
  ```

  `shell.volume` is 64931.4 (`72000 − π · 15² · 10`) and `body.volume` 65402.7, that plus
  `π · 5² · 6`. Flat on one body it comes out 64931.4, boss gone. `shell` is what a
  history calls "the body as of step 2", named.
- **Mates.** `k.far against m.near` says two caps touch. Part `k` is drawn in a *placed* plane —
  `from:` another with neither `fold:` nor `offset:` (`back := plane(origin: o, toward: q, from:
  front)`) — and the mate computes its offset, so a stack keeps its numbers in step. Only caps a
  sweep makes can mate (a side face is E082); a placed plane needs exactly one mate (E083, "`back`
  is a plane nothing places: write `offset:` or state one `against`"). Solved views: 1.13.

#### Writing a part

The statement follows from where the part's axis lies relative to its section. The V-twin piston
is a **turn** of its half-profile about the rod line. The disc, flywheel and throttle have their
axis *through* the section, so they are prisms, with only features whose axis lies *in* the section
(a radial set screw, a cross-hole) as turns. The plate is both. The rule to remember: **a part's
section must be where its turned features are.** A turn about a line in the section centres what it
makes on that plane, so the crank disc is sectioned on its mid-plane (the set screw runs through
it), as is the plate (plenum, boss, vents and coupling hole are centred there). Sectioned on a face,
half of each would be in the air.

#### Reports, exports and drawings

`--where body` lists the volume, surface area, bounding box and each face's area under its
document name, through the operand it came from: `near`/`far` for prism caps, `start`/`end` for a
partial revolution, otherwise the drawn edge it was swept from.

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

`body.bore.hole.area` (`2π · 5 · 30`) is the bore's wall, named by its circle under its
cutter. A face a Boolean consumed keeps its name with no area. `--stl PATH` writes a solid as binary
STL; `--solid NAME` picks which (optional when there is one Boolean root).

Views and dimensions of solids are asked for in a `.svd` ([Solvent Drawing](solvent-drawing.md)),
which projects solved solids with its own directions, positions and scales;
`vtwin/cylinder.svd` loads the preview of `vtwin/components/cylinder.sv`, and every V-twin part
(`piston.sv`, `disc.sv`, `flywheel.sv`, `throttle.sv`, the plate in `frame.sv`) works the same way.
`dimensions in front` generates the overall extents and the diameters of round features seen square
on. Generated dimensions are readings of the solved drawing, not statements: no equation, no
unknown, not draggable. Which datum a stack measures from and which fit is critical remain the
sheet's to state.

#### Refusals

| written | reported |
|---|---|
| `bad := face(ab, cd, bc, da)` | E080 — "`ab` and `cd` share no point: a face is a loop, walked in order" |
| `bad := face(a, bc, d, -> close)` | E080 — "`bc` meets neither of its neighbours: a face is a loop, walked in order" |
| `bad := face(ab, hole)` | E080 — "a circle is a whole loop: it stands in a face by itself" |
| `bad := face(a, b)` | E080 — "`b` and `a` share no point: a face is a loop, and one that does not come back to where it started closes with `-> close`" |
| `bad := solid(ab, depth: 3mm)` | E080 — "a swept solid is written over a face, and this is a line" |
| `bad := solid(sec)`, `sec` a face | E080 — "a body is made of solids, and this is a face" |
| `bad := solid(sec, from: 0mm, to: 0mm)` | E080 — "a prism swept nowhere is no solid" |
| `bad := solid(sec, about: ax, sweep: -90deg)` | E040 — "a sweep is a magnitude: which way it turns is `sense: cw`" |
| `bad := solid(sec, about: a)` | E081 — "a face turns about a line, and `a` is a point" |
| `bad := solid(sec, depth: 3mm, about: ax)` | E001 — "a solid is a face swept along its normal (`from:`/`to:`, `depth:`) or turned about a line (`about:`), not both" |
| `x cut y` and `y cut x` | E041 — "`x` is made of itself" |
| `h cut h` | E080 — "`h` is cut itself" |

### 1.15 Spatial geometry read after the solve

These declarations name exact geometry derived from solved entities. None adds an unknown or has a
2D glyph; each may be `private` or passed through a formal of its kind, like any member.

**Surfaces.** `flank := surface(crown, edge: rack.outer)` names the surface swept by one line, arc
or circle of an unmodified revolution's profile. It follows solved dimensions with exact positions
and tangents, parameterised from 0 to 1 along the edge (`u`) and the revolution (`v`). Tangent
orientation follows the declared traversal and does not say which side is material.
`surface(crown, edge: rack.outer, from: 180deg, to: 360deg)` keeps part of the revolution: angles
follow its sweep and must lie within it, and they restrict `v` (here to 0.5–1) without
renumbering. Read the domain rather than assuming [0, 1]. One surface per profile edge is
`repeat e in rack.profile { s := surface(crown, edge: e) }`.

**Motions** are rigid families over one shared angle:

```sv
crown := motion(about: crown_axis)
blank := motion(about: blank_axis, ratio: 2, phase: 10deg)
generating := motion(crown, relative_to: blank)
tap := motion(about: hole_axis, advance: 1.5mm)     // a screw: slides 1.5 mm per turn
plunge := motion(along: spindle, advance: 20mm)     // a translation, no turn
```

`blank` turns through twice the angle plus 10°; `generating` is the crown seen from the rotating
blank. Defaults are ratio 1, phase 0deg. Axes are read from solved geometry; evaluation is in
radians and returns exact pose and velocity per radian. A motion never moves the sketch.

**A motion may measure the drawing** (1.6), so a ratio is read off geometry rather than typed:

```sv
unit mm
o := point
z := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
axis := line(o, z)
c := point
d := point hint(x: 25, y: -2)
fix(x == 0, y == -2) c
wheel_radius := horizontal line(c, d)
c distance(30mm) d
e := point
f := point hint(x: 12, y: -4)
fix(x == 0, y == -4) e
pinion_radius := horizontal line(e, f)
e distance(10mm) f
wheel := motion(about: axis)
pinion := motion(about: axis, ratio: -length(wheel_radius) / length(pinion_radius))
```

```
$ solventc measured.sv
measured.sv: solved
  4 params, 4 equations, structural rank 4; DOF 0; 2 components: DOF 0, 0; 2 rigid cluster(s) in the distance graph
```

The ledger is the drawing's alone. `pinion` turns at −3 (30 over 10 as solved, not 25 over 12 as
seeded), recomputed each time the motion is read, so editing `30mm` re-times it and rebuilds any
mesh cached on it. Measured names resolve like references (`one.wheel_radius`), and deleting a
measured line deletes the motion. Slot dimensions are checked, and measuring outside a motion is
refused:

```
$ solventc wrong.sv            # bad := motion(about: axis, ratio: length(wheel_radius))
wrong.sv:19:1: error[E080]: `ratio` is Scalar, and this is Length
$ solventc refused.sv          # k := length(wheel_radius)
refused.sv:19:11: error[E107]: `k`: `length(wheel_radius)` measures the solved drawing, and only a motion's `ratio:`, `phase:` and `advance:` are read after the solve; a param, a seed, a constraint's number and a solid's extent are needed before it
```

`twist_drill/` grinds flutes with a wheel carried along a screw (`motion(about: axis, advance:
lead)`): the constant-twist class, whose sheet is the wheel's characteristic carried along the
helix (docs/generating-sweeps.md); `solventc twist_drill/drill.sv --step drill.step --tolerance 0.01mm`
writes it. `lantern_generation.sv` is the worked case: a pinion rolls against a wheel blank at that ratio and
its pin cuts a tooth space (`solventc lantern_generation.sv --stl out.stl --stl-backend mesh`, a
couple of seconds); edit a pitch radius and the cut follows.

**A motion moves a solid.**

- `indexed := solid(tool, under: indexing, at: 90deg)` is one copy at a constant angle
  (`solid_indexed_pattern.sv`).
- `removal := solid(tool, under: turn, from: -75deg, to: 75deg)` is the union of the tool's
  material over the whole interval — a continuous sweep, not a row of copies (`swept_torus.sv`).
  It is meshed from its material field ([field-meshing](field-meshing.md)), refined in the
  background in the app.
- `at:` and `from:`/`to:` are exclusive; a sweep of a sweep is refused; a swept solid may only be
  `cut` from a body.

**Envelopes.** `flank := envelope(crown_flank, under: generating, from: -35deg, to: 35deg)` is the
source surface's zero-normal-velocity locus over the roll interval. An evaluator intersects it with
two section equations and checks the envelope equation too; an arbitrary transformed source point
is not on it. Declaring one chooses no branch, designates no material and closes no solid.

**Envelopes in the plane are curves** (§6.15.1). Over a tool of the sheet — a point, line, circle,
arc or formula curve — and a planar motion (`motion(about: o, …)` is a *turn* about a point), the
same word is a curve the solve sees: the profile the tool cuts in the moving frame.
`flank := envelope(rack_flank, under: motion(rack, relative_to: blank), from: -25deg, to: 25deg)`
cuts an involute; `profile := envelope(roller, under: rel, from: 0deg, to: 360deg, side: near)` a
cam (`side: near|far` of the instant centre, where a circle cuts twice). `coincident`, `tangent` and
`curvature` hold against it, and the tool's and the motion's geometry are its columns, so they
solve to suit — a conjugate is synthesised, not stated. See 2.9.2.

A point's envelope is its path: `bore := envelope(apex, under: rotor_turn, from: 0deg, to:
1080deg)` is where the motion carries the apex, the epitrochoid typed nowhere. One that rolls whole
turns of the motion back to where it started is closed and stands alone in a face, as a circle
does: `chamber := solid(face(bore), …)`. A stationary root (the point a turn is about) is never a
contact. `wankel/` is the case.

**The rotor is a cut.** The largest rotor that turns in a bore is an *intersection* over the
motion, of the bore at every pose. By De Morgan that is the blank less everything the housing's
wall passes through, which the language already says: `swept := solid(housing, under: housing_turn,
from: 0deg, to: 1080deg)` and `swept cut rotor`. (`bound` by a sweep keeps what lies within the
*union* over the motion — an outer envelope, a different set.) Such a sweep — a pocketed prism under
a planar motion over a whole period — is the **planar generating class**, rows P1–P5
([generating-sweeps](generating-sweeps.md)), and is exported exactly: the blank within the pocket's
inner envelope (corners where the pieces meet: the apexes), carried through the prisms. A fold, a
crossing, a gap and a roll short of a period are refused by name.

**A prism's side generating in its view is a surface the solve sees** (§6.15.2). `side :=
surface(rack_tooth, edge: rack_flank)` names a prism's side (as it names a revolution's), and under
a motion keeping the prism's view `flank := envelope(side, under: cutting, from: -30deg, to: 30deg)`
is `rack_flank`'s planar envelope extruded square to the view, built with the drawing. A point drawn
in any view is `coincident` it by its place in that view (`p coincident flank`: one equation, the
roll its own).
Draw the prism's face `in` a view (not the page, which has no place in space); the face must name
the edge (`face(t0, t1, rack_flank, t3, -> close)`). `tests/extruded_envelope.rs`.

**Patches** state which material keeps a surface or envelope; every condition must hold, and each
includes its solid's boundary:

```sv
bounded := patch(flank, inside: tip_body, inside: heel_body,
              outside: root_body, outside: toe_body)
```

The evaluator currently accepts full revolutions with line or circular profile edges (holes
allowed). A patch selects no branch and orients no loop, so it does not make a closed solid.

**Seams** name where two faces meet:

- `flank_join := seam(flank_region, transition_region)` — two envelopes (or patches of them)
  meeting tangentially at a shared vertex of their source profile. They must share the source
  revolution, generating motion and actual profile vertex; tangency is checked after the solve.
- `tip_edge := seam(flank_region, tip.wall)` — a generated face (first) meeting a finite boundary
  surface (second). The envelope equation, the boundary incidence and the face's material
  conditions are all checked, so a tip cone or toe sphere belongs in the model.

**Vertices** meet seams: `tip_toe := vertex(tip_edge, toe_edge)` meets two boundaries on one
generated face; `join_toe := vertex(flank_join, toe_edge)` meets a junction with a boundary on one
of its faces. Every defining face is checked. A vertex has no planar coordinates; repeated uses
share identity, separately declared coincident vertices stay distinct.

**Edges** give a seam finite extent: `toe_span := edge(toe_edge, from: join_toe, to: tip_toe,
along: shaft_axis)`. A fraction 0–1 selects a slicing plane perpendicular to `along` between the
endpoints' projections, and the seam point in it is solved; endpoints reuse the named corners.

**Spatial faces** bind an ordered loop of edges to an exact support:

```sv
working := face(toe_span, tip_span, heel_span, join_span, on: flank_region)
transition := face(round_toe_span, join_span, round_heel_span, root_span, on: fillet_region)
```

The two share `join_span`, traversed oppositely. Each edge must lie on the named support, and the
loop closes through shared corners. An `on:` face is spatial and cannot be extruded.

All of these are local evaluations: they certify no global branch uniqueness, no closed boundary
as a valid disk, and assemble no solid
([analytic face boundaries](analytic-face-boundaries.md)). `spiral_bevel/verification.sv` uses
them throughout.

### 1.16 Checking your work

```
make solventc                                         # once
build/solventc drawing.sv                             # parse, elaborate, solve, diagnose; --json
build/solventc drawing.sv --where hinge               # where a name landed
build/solventc drawing.sv --stl part.stl --solid body # a solid, for a printer
```

Exit codes: 0 solved, 1 did not parse or elaborate, 2 did not solve. A successful solve does not
mean the model is fully constrained: read the diagnosis. The report gives parameter and equation
counts, the **DOF**, and culprit lines (`over:`, `conflict:`, `implied:`). The status
(`diagnosis.status` under `--json`) is one of:

| state | meaning | what to do |
|---|---|---|
| `well` | DOF 0, consistent | check that it is the solution you meant (the branch, not just the count) |
| `under` | DOF > 0 | something can move; add a constraint or a gauge, or leave a mechanism's freedom on purpose |
| `over` | a dimension takes part in a consistent redundancy | remove one `over:` line; editing it would be the next conflict |
| `conflict` | statements that cannot all hold | the `conflict:` lines are the *minimal* disagreeing set |
| `unsolved` | the solver stopped short | check the model, then reseed nearer the intended branch |

A redundancy among pure relations (a fourth `perpendicular` round a rectangle) is a theorem, listed
as `implied:`, never an error.

**`--where NAME`** prints the numbers under a name: a point's (`hinge.x`, `hinge.y`), a whole
instance's (`--where views`), a view's basis and origin (`--where side`), a solid's measurements
(`--where body`, 1.14), or one number (`--where hinge.x`, `--where body.volume`). Under `--json`
every name answers in a `positions` table, narrowed by `--where`. It is quicker than writing a
`claim` to test a position.

Two habits: **fix something** (a figure nothing holds is under by three however determined its
shape), and **seed for the branch** (the solver finds the solution nearest its start, so an arc
seeded on the wrong side comes out mirrored).

---

## 2. Examples

Each was run through `solventc`; the DOF and state quoted are what it reported.

### 2.1 One dimensioned line: DOF 0, well

```
a := point
b := point hint(x: 30, y: 10)

ab := line(a, b)
horizontal ab
a distance(40) b

fix(x == 0, y == 0) a
```

`2 params, 2 equations, structural rank 2; DOF 0`: the `fix` takes `a`'s two coordinates out of
the solve, at the numbers it states, and the level and the length settle `b`'s. `b`'s seed is nowhere near the
answer and needn't be: it only says which side of `a` to put `b`.

### 2.2 A rectangle, as a chain: DOF 0, well

```
w := 60
h := 40

p0 := point
p1 := point hint(x: w, y: 0)
p2 := point hint(x: w, y: h)
p3 := point hint(x: 0, y: h)

horizontal (bottom := line(p0, p1)) ->
vertical   (right := line(p1, p2)) ->
horizontal (top := line(p2, p3)) ->
vertical   (left := line(p3, p0)) -> close

p0 distance(w) p1
p1 distance(h) p2
fix(x == 0, y == 0) p0
```

`w` and `h` are values: 60 and 40 wherever they appear, never unknowns. The chain states nothing
four separate `horizontal`/`vertical` lines would not; it reads as the outline.

### 2.3 An input: DOF 0, well

```
// replaces 2.2's two dimensions and its `w :=` and `h :=` lines
param w := 60
p0 distance(w) p1                // states it
p1 distance(w / 2) p2            // the height follows the width
```

Edit the 60 — in the source, or by typing over the `w` callout — and the height follows.
`hint(x: w)` may read `w` too, and a host may give the input another value.

### 2.4 An unknown: DOF 1, under, on purpose

```
param s: Length         // an input nothing binds...
a := point
b := point hint(x: 10, y: 0)
c := point hint(x: 0, y: 9)

ab := line(a, b)
ac := line(a, c)
horizontal ab
vertical ac
a distance(s) b         // ...so the two lengths are tied,
a distance(s) c         // and their value is the solver's
fix(x == 0, y == 0) a
```

The lengths must agree, but nothing says what they are, so one freedom remains. Give `s` a value
(`param s := 10`) or add a constraint and it closes. Without the `param` line, `s` is E101.

### 2.5 An arc, tangent to what it joins: DOF 0, well

```
a := point
b := point hint(x: 30, y: 0)
c := point hint(x: 40, y: 10)
d := point hint(x: 40, y: 40)
o := point hint(x: 30, y: 10)

horizontal (run := line(a, b)) -> tangent
(fillet := arc(center: o) hint(r: 10)) -> tangent
vertical (rise := line(c, d))

radius(10) fillet
a distance(30) b
c distance(30) d
fix(x == 0, y == 0) a
```

`fillet` names only its centre; the chain threads `b` in as its start and `c` as its end, and each
`tangent` is stated *at* that shared point. State how things meet and let positions follow.
`rect_fillets.sv` does this round four corners.

### 2.6 A component, instanced: DOF 0, well

```
component Rung(a: point, b: point, len: Length) {
  e := line(a, b)
  horizontal e
  a distance(len) b
}

l0 := point
r0 := point hint(x: 50, y: 0)
l1 := point hint(x: 0, y: 20)
r1 := point hint(x: 50, y: 20)

t0 := Rung(l0, r0, len: 50)
t1 := Rung(l1, r1, len: 50)

stile := line(l0, l1)
vertical stile
l0 distance(20) l1
fix(x == 0, y == 0) l0
```

The formals alias the actuals; nothing is added at the boundary.

### 2.7 Repetition: DOF 5, under

```
n := 6
r := 40

cycle n as i {
  p := point hint(x: r, y: i * 60)
  e := line(p, next.p)
  e equal next.e
}
fix(x == r, y == 0) p[0]
```

Six equal links: round a loop one `equal` is implied (listed `implied:`), and nothing sizes the
ring, so five freedoms remain. Under-constrained repetition is normal.

A body ending mid-joint (1.7) writes a closed contour with no corner names. DOF 1, under:

```
cycle 4 {
  (s := line) -> perpendicular equal
}
distance(50) s[0]
fix(x == 0, y == 0) s[0].p1
```

Each side welds to the next at a square, equal corner; the wrap closes the loop. One dimension sizes
it and one `fix` places it, leaving it free to swing about that corner. Round a closed loop one
`perpendicular` and one `equal` are implied; dimension every corner instead (`-> angle(90)`) and
it reads `over`, since editing one number would then conflict. `square.sv` is this figure;
`ngon.sv` is the parametric version, seeded round a circle because equal chords fix each central
angle's size but not its sign, and only a seed can choose the winding.

### 2.8 A curve from a computed point: DOF 1, under

```
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)), y: c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)))
}

o := point
base := circle(center: o) hint(r: 20)
f := Involute(base, phase: 0).p over u in (0, 90)

t := point hint(x: 25, y: 8)
t coincident f
fix(x == 0, y == 0) o
fix(r == 20) base
```

The remaining freedom is how far along `f` the point `t` sits — which is why a contact slides
rather than breaks when the geometry beneath it moves. Seed it with `t coincident f hint(t: 30)`, or
pin it with `t coincident(t == 30) f` (DOF 0).

### 2.9 A curve stated as a locus: DOF 1, under

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point
  p := point
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c                                                 // the string leaves the circle...
  datum angle(u + phase) rad                             // ...at bearing u from the datum,
  rad perpendicular s                                    // square to the radius there,
  p distance(-(c.r * u / 1rad)) rad                      // and taut: as long as the arc
}

o := point
x := point hint(x: 20, y: 0)
base := circle(center: o) hint(r: 20)
datum := line(o, x)

f := Unwind(base, datum, phase: 0).p over u in (0, 90)

g := point hint(x: 25, y: 8)
g coincident f

fix(x == 0, y == 0) o
fix(r == 20) base
horizontal datum
o distance(20) x
```

The same curve as 2.8 with no formula: the body is the textbook definition, and the solver derives
the rest. The point-to-line distance is **signed**, so the minus sign unwinds the string one way
for a positive roll and the other way for a negative one — one component serves both flanks of a
tooth. `angle` is **directed**, so `t` sits at bearing `u + phase` rather than opposite, with no
`ccw` needed. For a genuinely discrete choice (which of two intersections), `ccw(a, b, x)` states
it.

A traced body's dimension may read the numbers of the geometry it is written over (`c.r`): they
are columns of the curve. Nowhere else may a dimension: `distance(k.r) l` on the sheet, or in a
component that is drawn, is **E103** (an unknown would be minted for `k.r`, and a drawn
instance would disagree with its own trace). State the relation instead (`l equal m`).

### 2.9.1 A curve of a drawn instance: DOF 1, under

```
component Crank(o: point, datum: line, theta: Angle) {
  p := point hint(x: 20, y: 10)
  arm := line(o, p)
  o distance(30) p
  datum angle(theta) arm
}

o := point
x := point
datum := line(o, x)
fix(x == 0, y == 0) o
fix(x == 10, y == 0) x

c := Crank(o, datum)                                  // theta unbound: the crank turns
rim := c.p over theta in (0, 360)
```

`c.theta` is the one freedom (a declared formal left unbound), and `rim` is where
`p` goes over a full turn, anchored at the pose on the sheet: drag `c.p` and the anchor follows.
`jansen.sv` is this at full size.

`rim` comes back where it started after its whole turn, so it is **closed**: a contact on it wraps
across the seam at 0/360 instead of stopping there. Any other numeric formal the drawn instance
leaves unbound (`component Crank(…, theta: Angle, len: Length)`, `c := Crank(o, datum)`) is an
unknown `c.len` and a **column** of the curve: `ground tangent rim` can then solve the length.
(Passed as an expression in another unknown — `len: 2 * k` — it is E103.)

### 2.9.2 A profile generated by a motion: DOF 0, well

```
o := point
fix(x == 0, y == 0) o
rk0 := point
rk1 := point hint(x: 30, y: 10)
fix(x == 30, y == 0) rk0
fix(x == 30, y == 10) rk1
slide := line(rk0, rk1)
blank := motion(about: o, ratio: 1)                     // the gear blank turns...
rack := motion(along: slide, advance: 2 * pi * 30)      // ...as the rack rolls on its pitch circle
cutting := motion(rack, relative_to: blank)
f0 := point hint(x: 30, y: 5)
f1 := point hint(x: 39.4, y: 8.4)
flank := line(f0, f1)                                   // the rack's flank, at roll 0
fix(x == 30, y == 5) f0
f0 distance(10) f1
cut := envelope(flank, under: cutting, from: -25deg, to: 25deg)
g := point
fix(x == 31.5, y == -3) g
g coincident cut hint(t: 5)                                     // the cut must pass through g
```

The flank's slope is left free, and `g coincident cut` decides it: the solve turns the rack's flank
until the involute it cuts passes through `g`. Nothing states an involute; the cut point at each
roll is where the flank's normal passes through the instant centre (the pitch point), and that is
enough. `cut curvature k` and `cut tangent l` work the same way, exactly (`C''` from the roll's
Taylor orders). A roller (`circle`) cuts a cam, a formula curve cuts its conjugate tooth.

### 2.10 Three views: DOF 0, well

```
// a 60-wide, 40-tall, 30-deep block, three views, one corner tied across them
Af := point in front
qf := point
front := plane(origin: Af, toward: qf)                             // the page itself
At := point in top
qt := point
top := plane(origin: At, toward: qt, from: front, fold: 0deg)      // folded up from the x-axis
Ar := point in right
qr := point
right := plane(origin: Ar, toward: qr, from: front, fold: -90deg)  // folded from z, turned so z is up
fix(x == 0, y == 0) Af
fix(x == 40, y == 0) qf
fix(x == 0, y == 90) At
fix(x == 40, y == 90) qt
fix(x == 150, y == 0) Ar
fix(x == 150, y == -40) qr

Bf := point hint(x: 60, y: 40) in front
Af distance(60, along: x) Bf
Af distance(40, along: y) Bf
Bt := point in top
Br := point in right
Bf project Bt          // width agrees front <-> top
Bf project Br          // height agrees front <-> right
Bt project Br          // depth agrees top <-> right
At distance(30, along: y) Bt
```

Each view's origin is corner `A` as that view sees it, so origins need no projections. `B` in the
top and right views is placed by projection plus the one depth dimension.

`use std` and `views := std.ThreeViews(O, right: 150, up: 90)` write this layout once: `views.front`
is the page, `views.right` and `views.top` fold from it, and `views.right_origin`/`views.top_origin`
are the corner in those views. Ground `O` and draw `in views.top`. `bracket.sv` is the full case,
with an auxiliary view folded at an inclined face's bearing.

### 2.11 Two skew axes at a stated shaft angle and offset: DOF 0, well

```
// two shafts at a stated angle and offset: the gear's axis drawn in the front view, the
// pinion's in a view folded from it by a fold the drawing solves for
unit mm
o := point hint(x: 0, y: 0)
t := point hint(x: 40, y: 0)
front := plane(origin: o, toward: t)
gax := line in front
fix(x == 0, y == 0) gax.p1
fix(x == 0, y == 50) gax.p2

o2 := point hint(x: 120, y: 0)
t2 := point hint(x: 160, y: 0)
param beta: Angle hint(30deg)
side := plane(origin: o2, toward: t2, from: front, fold: beta)
pax := line(hint(x: 120, y: 10), hint(x: 180, y: 12)) in side
pax.p1 distance(0, along: u) side
pax.p2 distance(60, along: u) side
pax.p1 horizontal pax.p2

gax angle(90deg) pax      // the shaft angle, in space
gax distance(17.5) pax    // the offset: the common perpendicular, in space
```

`solventc` reports `27 params, 27 equations, structural rank 27; DOF 0`, and `--where pax` gives
`pax.p1.y = 17.5`, `pax.p2.y = 17.5`. The axes are in different views, so `angle` and `distance` are
spatial: the unsigned angle between directions and the common-perpendicular distance. They settle
the fold `beta` (0°: the side view is the top view, the pinion axis 17.5 behind the gear's, crossing
it square) and the pinion axis's height. No datum point is grounded; the views' sheet placement is
held silently (1.13). Without the offset: `27 params, 26 equations, structural rank 26; DOF 1` (the
pinion axis slides along the common perpendicular). At a 60° shaft angle the fold is 30°.
`skew_axes.sv` adds a shaft about each axis.

### 2.12 A hypoid's pitch cones through the mean point: DOF 0, well

```
unit mm
module := 4mm
Rg := 48 * module / 2
Rp := 24 * module / 2
E := 20mm

// the pitch plane, and M on it
o := point hint(x: 0, y: 0)
t := point hint(x: 40, y: 0)
P := plane(origin: o, toward: t)
M := point in P
fix(x == 0, y == 0) M
O := point hint(x: 110, y: 0) in P
A := point hint(x: 95, y: 18) in P
gen_g := line(O, M) in P
gen_p := line(A, M) in P
horizontal gen_g

// the axial views, folded square to P along the generators
og := point hint(x: 0, y: 200)
tg := point hint(x: 40, y: 200)
G := plane(origin: og, toward: tg, from: P, fold: along gen_g)
oq := point hint(x: 0, y: -200)
tq := point hint(x: 40, y: -200)
Q := plane(origin: oq, toward: tq, from: P, fold: along gen_p)

// each axis in its axial view, from its apex: the apex's image is on the fold line (on P) and
// projects to the apex drawn in P; how long an axis is drawn says nothing about the cone
gax := line(hint(x: -110, y: 200), hint(x: -50, y: 304)) in G
pax := line(hint(x: -97, y: -200), hint(x: -28, y: -239)) in Q
gax.p1 coincident P
O project gax.p1
pax.p1 coincident P
A project pax.p1
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2

// the gear's pitch angle, the two pitch radii at M, and the shafts: square, and E apart
gen_g angle(60deg) gax
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
```

`56 params, 56 equations, structural rank 56; DOF 0`, with `--where A` at `(95.5837, 18.0989)`.
Each axial view is folded `along` its pitch generator, so it is square to P and holds the
generator; drawing each axis from its apex's image makes P each cone's tangent plane along its
generator. Pitch radii, shaft angle and offset are four conditions; the gear's 60° pitch angle is
the fifth, and the pinion's apex and pitch angle follow (ε = 10.72°, γ = 29.56°).

### 2.13 The same pitch cones, named: DOF 0, well

```
unit mm
module := 4mm
Rg := 48 * module / 2
Rp := 24 * module / 2
E := 20mm

o := point
t := point
P := plane(origin: o, toward: t)
fix(x == 0, y == 0) o
fix(x == 40, y == 0) t
M := point in P
fix(x == 0, y == 0) M

// the gear's axial plane, square to P about its vertical axis; the pinion's, solved, through M
og := point
tg := point
G := plane(origin: og, toward: tg, from: P, fold: 90deg)
fix(x == 0, y == 200) og
fix(x == 40, y == 200) tg
oq := point hint(x: 0, y: -200)
tq := point hint(x: 40, y: -200)
Q := plane(origin: oq, toward: tq, attitude: free, through: M) hint(u: (0.1618, -0.4935, -0.8546), v: (0.0918, 0.8698, -0.4847))

gax := line(hint(x: 110.85, y: 200), hint(x: 50.85, y: 303.9)) in G
pax := line(hint(x: -84.62, y: -248), hint(x: -4.62, y: -248)) in Q
gax.p1 coincident P
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2
horizontal pax

gc := cone(axis: gax) hint(half: 60deg)
pc := cone(axis: pax) hint(half: 30deg)
angle(60deg) gc
M coincident gc
M coincident pc
gc tangent(M) pc

M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
```

`39 params, 39 equations, structural rank 39; DOF 0`. Here the contact is stated rather than
constructed: the gear's apex on P makes P the gear cone's tangent plane at M, and
`gc tangent(M) pc` makes it the pinion's, so the pinion's apex lands on P unasked.
`horizontal pax` fixes the free view's remaining turn. 2.12 and 2.13 give the same hypoid
(Γ = 60°, γ = 29.564957707°, ε = 10.722102067°, |MA| = 97.282181449, within 2e-11;
`tests/spatial_surfaces.rs`). `hypoid_pitch_cones.sv` shows the cones touching on P in the app.

### 2.14 An arc placed by its length: DOF 0, well

```
o := point
s := point hint(x: 10, y: 0)
e := point hint(x: 3, y: 9)

a := arc(o, s, e)
radius(10) a
length(5 * pi) a        // a quarter of the circumference: the sweep is 90°
o horizontal s
fix(x == 0, y == 0) o
```

`5 params, 5 equations, structural rank 5; DOF 0`, with `e.x ≈ 2e-15`, `e.y = 10`: straight
above the centre, from a seed that was not. `length(15 * pi) a` puts it straight below.
`belt_wrap.sv` is the useful version: an open belt as one tangent chain over two pulleys, with
`length(wrap) big` in place of a centre distance (`12 params, 12 equations, structural rank 12;
DOF 0`; `c2.x = 66.0205` for a wrap of 90 on radius 25, since π + 2·asin(15 / 66.0205) = 3.6 rad).

### 2.15 A bisector, stated as two equal angles: DOF 0, well

```
a := point
b := point hint(x: 40, y: 0)
c := point hint(x: 10, y: 30)
d := point hint(x: 25, y: 10)

ab := line(a, b)
ac := line(a, c)
ad := line(a, d)
horizontal ab
ab angle(60deg) ac
a distance(40) b
a distance(30) c
a distance(20) d

ab angle(ad, ac) ad     // the angle from ab to ad is the angle from ad to ac
fix(x == 0, y == 0) a
```

`6 params, 6 equations, structural rank 6; DOF 0`, with `d.x = 17.3205`, `d.y = 10`: `ad` at 30°.
With a shared unknown instead (`param beta: Angle`, `ab angle(beta) ad`, `ad angle(beta) ac`) it
is `7 params, 7 equations` — an extra unknown only to be equated away.
`ab angle(ab, ac, sense: cw) ad` puts `ad` at −60°, `ac`'s mirror in `ab`. `reflection.sv` states
the law of reflection as `incoming angle(m, outgoing) m`, with the classical proof as a `claim`
judged a theorem.

---

## 3. Working checklist

1. Declare points, seeded roughly where you mean them — near enough to pick the right branch.
2. Declare lines, arcs and circles from them; use a chain for a contour.
3. State relations (levels, tangencies, equalities), then dimensions. Model relationships, not
   coordinate tables.
4. `fix` one point (`fix(x == 0, y == 0) a`), and a number that is given rather than solved.
5. Run `solventc`. Aim for `well` and DOF 0 unless freedom is intended.
6. On `conflict`, read the minimal set. On `over`, find the dimension the others imply. On
   `under`, ask what can still move.
7. Prefer a word to an unknown: equal angles are `l1 angle(l3, l4) l2`, an arc's extent is
   `length(L) a`, a motion's ratio may measure the drawing. An unknown you do need is declared,
   `param a: Length hint(…)`.
8. In space, draw each thing in the view that shows it true, relate across views with ordinary
   words, let the solve place a view rather than computing its attitude, and check with
   `--where VIEW`.
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
| layouts in space | [skew axes](../rust/examples/skew_axes.sv), [spatial surfaces](../rust/examples/sphere_cone_cylinder.sv), [hypoid pitch cones](../rust/examples/hypoid_pitch_cones.sv) |
| motions and generated solids | [indexed pattern](../rust/examples/solid_indexed_pattern.sv), [lantern generation](../rust/examples/lantern_generation.sv) |
| a point's envelope, an inner envelope and a claim over a motion | [Wankel](../rust/examples/wankel/wankel.sv) |
| a modular assembly | [engine](../rust/examples/engine.sv) |
