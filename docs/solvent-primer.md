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
  It describes the language as it is; what changed, and when, is the
  [changelog](solvent-changelog.md)'s.
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
                and for a design table's values: 745.7W  48V  255rpm  0.85T  27.9Nm
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
  `toward:` steps by `by` times line `l`'s run, and along an arc, a spline or a traced curve by
  `by` times its rate along its own run where `a` meets it — ahead along it;
- `hint(at: P, (3, 4))`, `P` a plane, is in `P`'s own coordinates (1.12).

A place drawn in another view is read in space and projected into the seeded point's view. A point
in space is seeded `hint((x, y, z))`. Seeds settle in statement order, so one reading a seed written
below reads its provisional start. A geometry read is a length: write `pin.x - 10mm`, not
`pin.x - 10`. A value may **not** read geometry, since a seed must never change what a document
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
param NAME: TYPE over (E, E)            a parameter that runs over an interval       (1.6)
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
NAME := solid(FACE, SWEEP...)           a face swept: depth:/from:/to:/through:, about:
NAME := sweep(Comp(ARGS).FACE over F in (E, E))   a face family: the face as the formal F runs
NAME := solid(SOLID) | solid(SET)       a body over a stock; the points of a region
NAME := solid()                         an empty body: what is in `union` with it (1.14)
NAME := REF at[(under: M, seen_from: G)] E   a frame, a solid or a point read at an instant (1.15)
NAME := REF in F                        the same, drawn in the jointed frame F at its value (1.15)
NAME := sweep(SOLID over t in (E, E) | SOLID, through: BODY[, under: M, seen_from: G])
                                        a solid a frame's motion sweeps                (1.15)
REF union REF | REF cut REF | REF common REF  material added to, removed from, shared with a body
NAME := envelope | meshing | surface(...)                                           (1.15)
WORD[(ARGS)] REF | REF WORD[(ARGS)] REF [hint(SLOT: E, ...)]   a constraint          (1.5)
claim CONSTRAINT                        judged, never solved for                     (1.11)
claim { DECL* CONSTRAINT* }             a claim with witnesses of its own              (1.11)
fix((E, E[, E])) REF | fix(MEMBER == E, ...) REF   hold an entity's own numbers      (1.4)
ccw(a, b, c) | cw(a, b, c)              choose a root; adds no equation              (1.5)
K minimizes E | K maximizes E           a free curve's energy                        (1.10)
[NAME :=] Component(ARGS) [in REF]      an instance                                  (1.8)
component NAME(FORMALS) { statement* }  a component definition (not a value)
part NAME(FORMALS) { statement* }       a component that is a function of its inputs  (1.8)
a WORD(PARAMS) b := RELATION | { ... }  a relation word                              (1.9)
repeat N [as i] { ... }                 N unrelated copies                           (1.7)
cycle N [as i] { ... }                  N copies that close; `next`, `prev` in scope
ring N about C { ... }                  N copies, each the first turned about C
repeat e in CHAIN [as i] { ... }        a copy per link of a named chain (`cycle e in` closes)
repeat e in FACE [as i] { ... }         a copy per edge of a face's walk          (1.7)
cycle (a, b) in (W1, W2) { ... }        walks in step: link k of each in copy k   (1.7)
repeat a in SOLID { ... }               a copy per face of a solid                   (1.14)
when COND { ... }                       made once where COND holds, else not at all  (1.7)
LHS <= RHS | LHS >= RHS                 in a set's body: a bound on measurements     (1.14)
NAME := trace(INSTANCE.POINT over FORMAL in (A, B))   a curve                         (1.10)
NAME := trace(Component(ARGS).POINT over FORMAL in (A, B))
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

**A spline through points** names them as a chain: `rail := spline(through: rims, start:
l, end: m)` is the cubic through the corners of the named open chain of lines `rims`, one span a
link, so its `t` is `k` at corner `k`; at the first corner it runs along `start:` and at the last
along `end:` (a line or an axis), and an end given neither runs straight out. Its control points
are minted (`rail.k1` …); the chain may be written after it.

A **free curve**, `rope := curve(a, b)`, runs from `a` to `b` with a shape its energy settles
(1.10).

### 1.5 Constraints

**Every constraint is a prefix or infix operator.** The word stands before its one operand or
between its two; everything else — the number, a selector, a third entity — goes in parentheses on
the word:

```
horizontal line1                    point1 level(up) point2
radius(25) circle1                  point1 distance(80) point2
length(6) line1                     point1 symmetry(line1) point2
fix((0, 0)) p1                      l1 angle(30) l2
fix(r == 25) c                      line1 tangent(side: left) circle1
length(40) arc1                     l1 angle(l3, l4) l2
```

One word may name several constraints, told apart by the operands' kinds, by fixity, or by what
stands in the parentheses. **Operand order carries meaning**: `arc tangent line` is a tangency at
the arc's end, `line tangent circle` the ordinary one. A line and a spline or a curve touch
whichever is written first (`l tangent g` is `g tangent l`).

| word | fixity | operands and options |
|---|---|---|
| `coincident` | infix | two points; a point to a line, circle, arc, spline or curve; in space, a point, line or axis to a plane, a point or line to an axis, two axes; a point to a set (1.13) |
| `distance` | infix | two points (length; with `along:`, an **ordinate**: signed from the first to the second along an axis, a line, or the view's own `u`/`v`, `right`/`left`/`up`/`down` with the sign said); a point and a line, or two lines (a magnitude, `side:` picks the side); two concentric circles or arcs; a point and a plane (`along: u`/`v`/`n`); two planes (along the first's normal) |
| `tangent` | infix | line–circle/arc (`at: p1`/`p2` for tangency at that end; `side:` the centre's side); circle/arc–circle/arc (`external: true/false`); arc–line (`at: start`/`end`); spline or curve–line; any path–line `at:` a point a relation holds on the path (below); a line or set to a set (1.13) |
| `equal` | infix | two lines (length) or two circles/arcs (radius) |
| `curvature` | infix | spline or curve and a circle/arc: the osculating circle there |
| `horizontal`, `vertical` | prefix | a line. Between two points they are `std`'s words (1.9): `use std (horizontal)`, then `a horizontal b` |
| `level(dir)` | infix | two points with the same ordinate along `dir` (an axis, a line, or `up`/`right`); a point and a plane with `u`/`v`. Write it rather than `distance(0, along: …)`, which is refused |
| `angle` | infix | two lines, directed (below); in space, an axis and a line or axis, unsigned. `l1 angle(l3, l4) l2` equates two angles |
| `radius` | prefix | a circle or arc |
| `length` | prefix | any path: a line (the distance between its ends); an arc (radius × counter-clockwise sweep); a spline or free curve (its whole length). `distance` relates two things and `length` measures one, so `distance(d) l` on a line is refused, naming `length(d) l` |
| `symmetry(line)` | infix | two points |
| `midpoint` | infix | a point and a line |
| `parallel`, `perpendicular` | infix | two lines; an axis and a line or axis; an axis and a plane; `perpendicular` a line and a plane (the line along its normal, in space); `parallel` two planes. `l1 perpendicular(sense: ccw) l2`: `l2` a quarter turn anticlockwise of `l1` (`cw` the other way), two lines in a plane only |
| `project` | infix | two points in two planes: two images of one point in space (1.12) |
| `mapped(from -> a, …)` | infix | `q mapped(from -> a, to -> b, third -> c) p`: `q` where the affine map taking the three sources to the three targets carries `p`; two pairs the similarity, `mirror: true` reversing its turn. Sources in one plane, `q` and targets in one (may differ). Two circles under two pairs: centre carried, radius scaled |
| `fix` | prefix | an entity's own numbers, whole or by member (1.4) |
| `ccw(a, b, c)`, `cw(a, b, c)` | call | `c` left (right) of ray `a → b`: a root choice, no equation, held as a bound is (1.11) |

#### Which way: directions are words

| written | means |
|---|---|
| `p distance(12) ax` | 12 from line `ax`, **either side**; the seed picks |
| `p distance(12, side: left) ax` | left of `ax`'s direction `p1 → p2`; `right` likewise |
| `l1 distance(6, side: left) l2` | `l2`'s `p1` lies left of `l1` |
| `a distance(60, along: u) b` | `b.x − a.x = 60`, the run along the view's `u`; `along: v` is the rise, along its `v` (world z in `std.front`); `right`, `up`, … say it as a word |
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

**A point at a fraction of a line or an arc** writes the same `t`: `m coincident(t == 0.25)
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

**The tangent at a point of any path**: `g tangent(at: m) l`, `m` a point a relation already holds
on `g`, says `l` runs along `g` there — one equation. Of a spline or a curve it reads the place of
the contact holding `m` (`m coincident g`, pinned or not; with none it is E040, naming it), so
`m coincident(t == s) g` beside it determines the line where `g tangent(t == s) l` would restate the
point; of a circle or an arc the radius to `m` is square to `l`; of a line it is `parallel`. Of a
spline or a curve it is the **rate** of that contact along its place, `rate(m over t) parallel l`:
`t` names the contact's own slot, and the rate is its derivative there with every other
unknown held — never along a `param` (an unknown several contacts may read is refused).

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

A word with no meaning in space — `horizontal`, a view's own `along: u` between two points (name
the axis instead, `along: std.x`), `tangent` between drawn figures, the angle-equals-angle form — is **E062** across
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

**Parameters that run.** `param t: Angle over (0deg, 360deg)` declares a number that runs
over an interval: neither a value nor an unknown, so it has no `:=` and no `hint(…)`, and it says
what it is (`Length`, `Angle` or `Scalar`). Its ends are numbers of that type and increase (E103
otherwise). It is the one notion a face family's running number (`over s`, a declared `s`, 1.14), a
traced body's formals (1.10) and the roll a reading runs over (`over t in (a, b)`, 1.15) are uses
of. **A reading over the roll reads it**: `envelope(tool over t)`, `sweep(tool over t)`
and `meshing(f1, f2 over t)` take a declared `Angle`'s interval for their own, as `over t in (a,
b)` states one in place (a parameter of another type, or a name nothing declares, is E103; a
component, which sees no parameter, states its interval).

**A relation over a parameter holds for every value of it.** A dimension of the document's
own relations may read `t`, and the drawing is then a function of `t`: drawn at the interval's
start, and **followed** from there by continuation, one branch (a linkage's assembly mode) carried
to the end or to a **singular instant**, where the march stops and says whether the motion turns
back there or two branches cross. A four-bar driven round:

```
unit mm
use std
param t: Angle over (0deg, 360deg)
in std.front {
  a := point
  d := point
  b := point hint((10, 0))
  c := point hint((40, 25))              // above the ground: the assembly mode carried
  fix((0, 0)) a
  fix((40, 0)) d
  a distance(10) b
  b distance(35) c
  c distance(25) d
  crank := line(a, b)
  line(a, d) angle(t) crank              // the driver: held for every t
  bc := line(b, c)
  m := point
  m midpoint bc
}
coupler := trace(m over t)               // the coupler curve (1.10)
```

`solventc` says what each parameter drives and how far the drawing runs over it:

```
  6 params, 6 equations, structural rank 6; DOF 0; 1 DOF over t; 3 rigid cluster(s) in the distance graph
  driven by t: L1 angle(t = 0°) crank
  t: followed from 0deg to 360deg
```

A crank that cannot turn round reports `t: followed from 0deg to 129.838deg, singular at t =
129.838deg: no drawing stands past it, the motion turns back`. The drawing must be held at the
start: a freedom left beside a parameter is a free *function* of it (`each DOF left a free function
of t`), and nothing says how it moves, so it is not followed. A value reading a parameter is E103
(a value is worked out once), and a component sees none.

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

- **Without a `unit` line** the document is in millimetres, exactly as under `unit mm`: a suffix
  converts (`2in` is 50.8) and a geometry read is a length. The line is only written back where
  the document had one.
- A name takes its dimension from its value's unit, not its use: `w := 80` used as a length does
  not make `w` a length; `w := 80mm` does, as does a formal declared `Length`.
- `3/16"` is a division; a lone fractional inch is `0.1875"` or `0 3/16"`.
- `pi` is dimensionless; `tau` and `turn` are one full turn; radians are written `* 1rad`.
- **A design table's figures carry units too**: mass, time and current are bases beside length
  and angle, and derive the rest — `kg g s ms A mA Hz kHz rpm N kN Nm J W kW V mV kV ohm Pa kPa
  MPa GPa Wb mWb T mT`. They are for values: no slot takes one, so a figure reaches a constraint
  divided by its unit (`torque / 1Nm`). There is no `h` or `min`. `rpm` is an angle over a time:

  ```
  power := 745.7W
  omega := 255rpm
  torque := power / (omega / 1rad)    // a torque: torque / 1Nm is 27.9
  amps := power / (48V * 0.86)        // amps + torque is refused
  freq := 46 / 2 * omega / turn       // 97.8Hz
  ```
- **There is no string literal**: `"` is the inch mark, and a word argument is bare (`at: start`).
- A value, input, formal or index named after a built-in (`tau`, `pi`, `min`, …) does not shadow
  it and ends up with two values: W112. Rename it.

**Measurements of the solved drawing** — `length(l)`, `radius(c)`, `distance(a, b)`,
`angle(l1, l2)` (0–180°) — are **E107** in a number: the number is needed before there is a solve.
A frame's ratio is stated from the document's own numbers (`ratio: -wheel_pitch / pinion_pitch`,
1.15), not measured off the drawing.

### 1.7 Chains and repetition

A chain writes a run of elements and the relations between them on one line.

```
CHAIN  ::= LINK (JOINT LINK)* ['->' INFIX* 'close']
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a one-operand constraint word         horizontal, vertical, radius(..)
         | step(u: E, dv: E, from: P)            where a line's end stands (below)
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

**A step says where a line's end stands, one number per run.** `step(…)` before a line, or before
a chain's link, gives the end's ordinates by label: `du`, `dv` from the line's start, `u`, `v` from
the frame's origin. **What it does not name is kept**, so `step(u: s)` is a run along `u` to the
station `s`, level with where it began, and `step(dv: h)` a rise of `h`. One step may mix them,
`step(u: s, dv: b)`. The frame is the view the line is drawn in (its `in` block), or `from: P`, a
plane. Each number is a dimension, drawn and edited where it is written; a literal zero (`v: 0mm`,
back onto the axis) is a level and states no number. The edge that closes a loop takes no step:
it is whatever the others leave. A turned part's section, every number a radius from the axis or
a height from the base:

```
unit mm
use std
in std.front {
  profile := step(u: 32mm) (ab := line) -> step(v: 6mm) (bc := line) ->
             step(u: 15mm) (cd := line) -> step(dv: 10mm) (de := line) ->
             step(u: 8mm) (ef := line) -> step(v: 0mm) (fa := line) -> close
}
```

That is twelve rows for six corners, DOF 0, with no point declared and no `hint` written: **a
corner a step places starts where the walk puts it**, so its seed follows its numbers. A step
`from:` another frame, or over an unknown, leaves its corner where it was minted. A prefix word
goes outside a named link, `step(u: s) (ab := line)`. Refused: no ordinate (E040), `u` with `du`
(E040), an unlabelled number (E004), no frame (E040). `rust/examples/solid_flange.sv` and the hub
motor's cover are written so.

**Repetition.** `repeat N { … }` makes N unrelated copies; `cycle N { … }` makes N copies that
close, with `next` and `prev` in scope. `as i` binds the index; copies are reached as `p[0]`.

**A block may end mid-joint.** A trailing joint before `}` threads onto the next copy's first link:
a `cycle` wraps, closing the loop with no `close`; a `repeat` leaves its last joint unstated, so
`repeat N { line -> angle(a) }` is an open polyline. `cycle 4 { (s := line) -> perpendicular equal
}` is a square but for size and pose (2.7).

**A block made or not.** `when COND { … }` makes its body once where the condition holds and not
at all where it does not, decided over the numbers in scope before the solve: comparisons (`<`,
`<=`, `>`, `>=`, `==`, `!=`) of two numbers of one dimension — `relief > 0mm`, never `relief > 0`
(E103) — joined by `and`, `or` and `not`. Two drawn things compare by **identity**: `a != b`
holds where the names stand for different things (a formal is what its call gave), and `c in B`
where `c` is one of the faces `B` names (1.14). There is no copy: what the body declares keeps
its own name (`q`, `t.tip`, never `q[0]`), and is undefined where the block is not made. Refused:
an unknown of the drawing in the condition (E040), a number against something drawn (E103), an
index, a trailing joint. `when` is the one conditional: write a switch as `when flag == 1 { … }`,
never `repeat flag { … }`, and two as `when a == 1 and b == 1`.

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
  length(60) ab
  length(40) bc

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

**A face's walk, and walks in step.** `repeat e in f` runs over a face's edges in walk
order — a chain it is made of, the edges it names, or for a face of corners (`face(a, b, c, ->
close)`) the straight run from each corner to the next, `e.p1` the corner it leaves. `cycle (ea,
eb) in (a, b)` runs two walks (chains or faces) together, link `k` of each in copy `k`; the walks
must be as long. And a face may be made of a block's copies, `face(side)` for `side[0]`,
`side[1]`, …, its count the block's, never written:

```
unit mm
use std (coords)
in std.front {
  cycle 6 as i {
    p := point
    p coords(du: 10mm * cos(60deg * i), dv: 10mm * sin(60deg * i)) std.front
    side := line(p, next.p)
  }
  hex := face(side)
}
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
  (`Length`, `Angle`, `Int`, `Scalar`, `Side`), a `group` or an interface (below), a `set`
  (1.13), or a `path`: a line,
  an arc, a circle, a spline or a curve, whose words are settled by what the call gives it
  (`m coincident g`, `g tangent(at: m) l`); given anything else, E103 at the argument.  A face
  family's station may also be given a named open chain for a `path`, which it runs over link by
  link (the face family, below).
- **Passing an entity is aliasing**: formal and actual are one entity, so a boundary costs nothing.
- **Entities by position, numbers by label**: `Rung(l0, r0, len: 50)`, never `Rung(l0, r0, 50)`,
  and nothing positional after a label. Either mistake is **E004** at the argument.
- **An unbound numeric formal is an unknown** named under the instance (`c.theta`): a mechanism is
  drawn with its crank free so (2.10). Seed one in its place: `Crank(o, datum, theta: hint(30deg))`.
- **An instance's numbers are read by its name**: every numeric formal, bound or not, is
  `inst.formal` — `p distance(ball.r) ball.center` — and travels with an instance passed as a
  `group`. Values a body declares are not read this way.
- **Scope is closed.** A body sees only its formals and its own declarations: geometry and the
  document's unknowns must be passed in. Component definitions and built-in functions remain
  callable. Repetition blocks share the enclosing scope.
- **A body's standard datums are its frame's**: `std.front`, `std.top`, `std.side`, `std.up`,
  `std.x`, `std.y`, `std.z`, `std.back` and `std.origin` in a body name the datums of the frame the
  instance stands in — so a body says `in std.side { … }` itself and is handed no view (1.12).
- **A body reads its file's constants**: the values its file states at the top and those of the
  modules the file `use`s (`engine.dims.L`, or `L` with `use engine.dims (L)`), with nothing passed
  at the call. A number a parent should be able to vary or solve for — a crank's angle, a ratio — is
  a formal; a number the part's design fixes is a constant. A `param` with no value is an unknown,
  never a constant (E103 in a body). The same holds in a relation word's body.
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
  length(dims.length) ax
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

**Interfaces** name a group's shape. Written at the top of a file, as a component is, each
member as a formal is, separated by commas or line breaks; a member may be typed by another
interface:

```solvent
interface Size { length: Length }
interface Layout {
  origin: point
  size: Size
}
component Bar(layout: Layout) {
  tip := point hint((layout.origin.x + layout.size.length, layout.origin.y))
  layout.origin distance(layout.size.length) tip
}
```

A formal typed by one (`layout: Layout`) is a group formal the call must give **every member, of
its type** — a number of the member's dimension, an entity of its kind, a set, a group of the
nested shape — or **E103** at the argument; whatever else the group or instance carries is
allowed, and nothing says it is a `Layout` but having the members. The formal is a group of its
own **holding only what the interface declares**: `layout.width` is **E101** where it is read, as
is handing it on (`Pin(layout.width)`), and `layout` handed on whole holds no more, whatever the
formal it is handed to. A module exports
its interfaces as it does its components (`spiral.Gear`, `use spiral (Gear)`). The spiral bevel's
steps read one another so (`gear: Gear, trace: Trace` in `spiral_bevel/hypoid.sv`).

**Derive, don't restate**: keep a few design inputs and derive the rest, so changing one carries
its dependents (the V-twin derives its cylinder, pockets and grooves from bore, throw and wall).

**A part is a component that promises to be a function of what it is given**: `part Name(…) {
… }`, written where `component` would be. The elaborator holds it to two rules, over each
instance:
- its own rows place everything it makes, or **E108** names what they leave free;
- it states nothing about its operands alone, or **E109** names the row.

A freedom it leaves is a formal the call leaves unbound (`Crank(o, d, theta: hint(30deg))`), which
the parent places. A parent may relate a part's outputs (`r distance(10mm) a.tip`), but such a
statement is the parent's and never completes the part. Choose a root with `ccw`/`cw` or a bound
(1.11, held by the solve), not with a seed: `solventc --lint` names a part whose rows have another
pose its stated choices admit (W207, advice). A part is solved like any component. Use one for
anything a parent should be able to trust as a unit: a pulley, a rod on its pin, a belt run.

```
part Crank(o: point, datum: line, theta: Angle) {
  in o.plane {
    tip := point hint((20, 20))
    arm := line(o, tip)
    o distance(30mm) tip
    datum angle(theta) arm
  }
}
```

### 1.9 Modules, relation words and previews

```
use engine.dims          // engine/dims.sv beside the document or an ancestor, else the library
use std                  // datums (1.12), std.Turned, std.CenteredRectangle, std.Polygon,
                         // std.Hex, std.Ellipse, std.Sphere/Cylinder/Cone, rounds (1.14)
use hardware             // fasteners and fittings, in mm
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
a above(d) b := b distance(d, along: up) a       // r above(7) q: r stands 7 above q
l1 flush(d) l2 := {                              // parallel, and d apart
  l1 parallel l2
  l2.p1 distance(d) l1
}
a apart(d) b := {                                // what the body declares, each use makes anew
  private ab := line(a, b)
  length(d) ab
}
```

A word of one parameter takes it bare or labelled (`above(7)`, `above(d: 7)`); one of two or more
takes them by label (`offset(dx: 30, dy: 12)`; positional is E004, a missing or unknown label
E040, anything given a word of none E040);
a word parameter stands where a selector or direction would (`side: s`, `level(s)`), and one read
where an entity stands is given a reference (`n square(up: std.z) g`), an alias in the body as an
operand is (E101 at the argument where it names nothing). The body reads
only its operands, their members and the parameters (E101 otherwise) — among a point's members the
plane it is drawn in, `o.plane`, so a word about a point in a view needs no plane given (`f.axis
parallel o.plane.normal`; a point in space has none, E040 at the use); it may write other words and
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
n square(up: std.z) g            // plane n square to line g, its v along up (up square to g)
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
    cyl := Cylinder(axes, fw: components.dims.fwA)
  }
}
```

`vtwin/components/cylinder.sv` is the complete example; `vtwin/cylinder.svd` draws its views.

### 1.10 Curves

A traced curve is **a point of a component, followed as one of its numeric formals runs over an
interval**. There is no curve family: an involute, a cycloid and a leg's stride are three
components.

```
NAME := trace(INSTANCE.POINT over FORMAL in (A, B))          a drawn instance's point
NAME := trace(Component(ARGS).POINT over FORMAL in (A, B))  an instance written in place, never drawn
```

- **Over a drawn instance**, `path := trace(leg.toe over theta in (0, 360))` is where the drawn
  leg's toe goes as `theta` runs, anchored at the drawing's pose. Leave `theta` unbound and the
  crank is the drawing's freedom (2.10).
- **Written in place**, `Involute(base, phase: a0).p over u in (…)` is anchored at the value the
  call gives the swept formal, or the interval's start.
- **Over a declared parameter**, `coupler := trace(m over t)` is where a point of the
  drawing itself goes as the drawing runs over `t` (1.6), over the parameter's interval or a
  stretch of it (`over t in (30deg, 90deg)`), ending where the drawing stops at a singular
  instant. It is a curve of the point's view; a point in space is traced in space, drawn in the
  workspace and in no view, and held to by nothing yet. A relation holds to a view's trace
  as to any curve, and a number that does not run with `t` solves through it: the
  drawing gives the leg `theta: t`, leaves its rod `h` unbound, and `path tangent ground` solves
  the rod (`stride.sv`, `jansen.sv`, `peaucellier.sv`, `dwell.sv`). What moves with `t` must
  place every number it moves: a coordinate it leaves free is E061, so declare the number to
  solve instead (`param l: Length`).

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

**In space.** A body whose relations do not read in one plane — ordinates along lines in space, a
point a motion carries — is a curve **in space** (`p coincident k`, `k tangent(at: p) l`), and a
`path` (`p coincident(t == 0.3) k` at its formal, a station's guide); over two formals, `S :=
trace(C(…).p over (phi in (a, b), s in (c, d)))`, a **surface** (`p coincident S`, and at a point
held there `tangent`, `perpendicular` for its normal, `curvature`). Write it in place, seed its
points with numbers over the formals, and give it points, lines, motions and numbers:

```
component Helix(o: point, x: line, y: line, z: line, r: Length, lead: Length, u: Angle) {
  p := point hint((r * cos(u), r * sin(u), lead * u / 360deg))
  o distance(r * cos(u), along: x) p
  o distance(r * sin(u), along: y) p
  o distance(lead * u / 360deg, along: z) p
}
k := trace(Helix(o, xl, yl, zl, r: 5mm, lead: 3mm).p over u in (0deg, 720deg))
```

**Rates.** A body may say how a point *moves* as the formal runs: `rate(p over s)` is
`p`'s velocity, and it stands where a direction is read — `rate(p over s) parallel tl`,
`perpendicular`, a directed `angle`, and alone before `distance(k, along: l)` for its component
along `l` (`rate(p over s) distance(1, along: tl)` is unit speed, so `s` is arc length). Never
`coincident` and never `length(…) rate(…)`. Such a body is **integrated from its home**: drawn, from
the instance as it stands on the sheet (its rate rows are no rows there, so the sheet holds what
they leave free); written in place, from the call's value of the formal (or the interval's start),
where `initially { … }` in the body says what holds there alone. The relations without a rate plus
the rate rows must fix every rate — in the plane as many rows as the body places numbers; one short
is refused naming the points it leaves free. A component handed the running formal as itself
(`Drag(o, a: a, u: u)`) rates over the caller's; the formal must be a `Scalar` or a `Length`.
A point a motion carries (`X coincident(under: m, t == τ) x`, below) is a relation like any here,
in the body or in `initially`: a starting circle may be a drawn point turned about an
axis, `initially { p coincident(under: spin, t == u) s }` (`rates/geodesic_circles.sv`).

```
component Clothoid(A: Length, s: Length) {
  p := point
  q := point
  k := point
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)                 // unit speed: s is arc length
  rate(q over s) distance(10mm * s / A^2, along: nl)    // the tangent turns at κ = s/A²
}
in std.front {
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((10mm, 0mm)) x
  datum := line(o, x)
  cl := Clothoid(A: 50mm, s: 0mm)
  cl.p coincident o                                    // the home: where it starts...
  datum angle(0deg) cl.tl                              // ...and which way
}
spiral := trace(cl.p over s in (-100mm, 100mm))
```

Contacts on such a curve are exact (its tangent is the rate itself), and an unknown of the drawing
the body reads is solved through the flow: the hanging chain's pulls `c` and `v`, left unbound,
are found by its far end landing on `b` (`b coincident(t == 150mm) chain`). A march stops where
its rates cannot be had — `solventc` says how far each curve is integrated, or where it is refused.
The [rates](../rust/examples/rates/) series shows each idiom in a document of its own.

**A speed that reads where the point is.** In a body traced in space a dimension may read
a coordinate of a point the body declares (`p.x`, `p.y`, `p.z`) as well as the formals, the numbers
and the geometry it is written over: `rate(p over s) distance((1 + ((p.x - c.x)^2 + (p.z -
c.z)^2) / R^2) / 2, along: nl)` moves a front at `1/n`, the index `n` read at `p` — the eikonal in a
graded medium ([`fisheye_fronts.sv`](../rust/examples/rates/fisheye_fronts.sv)). The text is a
number the body places, solved and integrated with its points. Over two formals it stands in a rate
row or a relation placing a point the rates do not move, and never in `initially`. A trace in a
plane still reads only its formals and its geometry.

**Over two formals** a body in space is a surface **marched** along the first and sampled
along the second (the method of lines): `front := trace(Front(c, z1, k).p over (d in (0mm, 3mm), u
in (0deg, 180deg)))`. `initially { … }` is the Cauchy data where `d` starts, for every `u`; it may
read a curve the drawing has — `k: curve` given a spline, a curve of a held plane (a
formula, a trace, an envelope) or a curve traced in space:

```
component Bore(c: point, z1: point, k: curve, d: Length, u: Angle) {
  za := line(c, z1)
  p := point hint((0, 0, 0))
  m := point hint((0, 0, 0))
  nl := line(p, m)
  length(10mm) nl
  c distance(0mm, along: za) m               // the normal, placed at every node...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge
  rate(p over d) distance(-1, along: nl)     // the law: unit speed, away from `nl`
  initially {
    p coincident(t == 3 * u) k               // starts on `k` where its parameter is 3u
  }
}
```

Read only there, and only as `p coincident(t == …) k`. A fold — a node's area `|x_d × x_u|` gone
to a hundredth of its home's — is refused where it happens, which is a roller's undercut when the
front is a cam's pitch curve marched from its profile (`rates/pitch.sv`, `rates/bore.sv`).

**The ellipse** is `std.Ellipse(c: point, a: Length, b: Length, tilt: Angle, u: Angle)`: `e :=
trace(std.Ellipse(c, a: 40, b: 25, tilt: 0deg).p over u in (0deg, 360deg))` is the rim. There is no
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
The integrand is dimension-checked (`p.y + 1mm`, not `p.y + 1`).

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
sampled poses of a frame's motion, reading solids placed under it, and reports the worst; over an
unknown (`claim over reach in (0.1mm, 0.9mm) { … }`) it solves at each sampled value.

**A claim with witnesses of its own.** Some claims need geometry to be stated at all: "the two
runs of a belt meet on the line of centres" needs a point on both runs. Declared as ordinary
geometry, that point is part of the drawing — and at equal pulleys, where the runs are parallel,
it cannot exist, so a valid drive no longer solves. Declare it in the claim's block instead:

```sv
in std.front {
  claim {
    x := point
    x coincident upper.span
    x coincident lower.span
    x coincident centres
  }
}
```

What the block declares is a **witness**: the drawing holds it while it solves, so the drawing
solves, counts and diagnoses exactly as it would with the block deleted. After the solve the
witness is solved by the block's relations, the drawing held where it landed, and the block is
judged as one claim — *theorem* where a witness exists and keeps existing as the drawing moves
(here, at any pair of unequal radii), *refuted* where none exists (equal radii: the drive still
solves), *independent* where one exists only at this pose. Every statement of the block carries
the one verdict. A block declares geometry and states relations of it (a `when` inside is fine);
nothing outside it may name a witness (E040), and its relations are claims, so none may own an
unknown. A precondition is a `when` over the drawing's numbers, `when rb != ra { claim { … } }`:
where it is false the claim is not made at all.

Three claim words apply only to solids: `claim bore inside stock`, `claim disc clear(2mm) cyl`,
`claim head fits(0.15mm) trap`. The report gives the measurement, and one within the faceting of a
round face is **undecided**:

```
  claim bore inside stock — undecided, measured 0 (interval [-0.0006000000000000001, 0.0006000000000000001])
    unresolved: containment is within curved-faceting uncertainty
  claim bore clear(2mm) stock — refuted, measured -9.999665339174008 (interval [-10.00026533917401, -9.999065339174006])
```

**A bound** is a dimension's number that adds no row: it says which of the drawing's solutions is
meant, which a seed could only pick by where it started.

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

**An orientation is held the same way.** `ccw(a, b, x)` adds no row and chooses the root with `x`
left of `a → b` (`cw`, right): a point of the triangle seeded on the other side is carried across
the line through the other two before the solve, a solution on the other side is steered to the
root that turns the stated way, and where none is found the solve fails with `a stated orientation
does not hold` (`orientation broken: ccw(a, b, x)`). Three points the drawing puts on one line
turn neither way, so they break it. A point 5 from `a` at `(0, 0)` and from `b` at `(6, 0)` comes
to `(3, 4)` under `ccw(a, b, x)` whatever its seed. Inside a traced body it is read at the anchor
instead, and continuity carries it (1.10).

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
held `fix((x, y, z)) p`; a line between two is a line in space. An arc over points in space is
drawn in the plane through its centre and its ends, the short way round, so a chain of lines and
arcs may bend in any plane with none written; a circle about a point in space takes the axis it
stands square to, `circle(center: o, normal: ax)` (with none it is E060). In a plane an arc runs
counter-clockwise from its start; `arc(center: c, sense: cw)` runs clockwise, so a chain in a
plane may turn right at it. A chain may also enter an arc at its end and walk it to its start —
threaded with both ends written, `-> tangent (k := arc(center: c, start: f, end: e)) ->` entered
at `e`, or the arc named there — and `k.start` is still `f`. A face over a point in space is E080;
a spline over points in space is a curve in space: its contacts a traced curve's (`m coincident(t == 0.5) k`, `k tangent(at: m) l`), a `path`, and
`spline(through: rims)` over corners in space the cubic through them (it bounds no face; the
workspace draws it in space, as an axis). In the workspace such points are dragged across the
eye's picture plane; depth stays the constraints'.

- **`p coincident P` is `in P` said as a relation**: the point is drawn in `P`, and relations over
  it read as `P`'s. `l coincident P` puts both ends in. A point its declaration draws in another
  plane is drawn in `P` too, as `in Q, P` would draw it: the way to put a point another component
  declares into a plane only this one knows (`gear.M coincident q`). Said plane first, `q
  coincident gear.M` puts the plane through the point by a row. A point in space put on a second
  plane by a statement is held there by a row, and `project` reads each point in its own plane.
  A point whose seed has a `z`, that a `fix` holds whole, or that is claimed on the plane, stays
  in space.
- **A point on a curve drawn in a plane is in that plane**: `m coincident g`, `g` a spline or a
  traced curve drawn in `P` and `m` standing in space, draws `m` in `P` as `m coincident P` would,
  and `g tangent l` draws a line's free ends there — so a word handed `g` need not be handed its
  plane. Neither curve has a contact in space, so a point kept there (a `z` seed, a `fix`) is
  E062, saying what keeps it. A circle or an arc keeps its contact in space.
- **A point may be drawn in two planes**: `O := point in P, G` lies on their common line, and each
  plane's geometry reads it as its own (`arc(center: O)` drawn in `G` is `G`'s). It is what two
  views tied by `project` say, said once. The same plane twice is E040, parallel planes E061 —
  judged where the planes stand once placed, or where the solve leaves a plane only seeded.

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
stands; `P.normal` is the axis along `u × v` through `P.origin`, read wherever an axis is (`l
parallel P.normal`, `plane(u: P.normal, v: P.u)`) and adding no freedom — declare `P` before
anything naming it; two held axes that do not meet are E067. A point drawn in a plane names it as
`p.plane` (`t perpendicular p.plane`); one standing in space has none (E040). A plane over a
line stands on it, and over two lines sharing an end, at that end. **A slot left out mints a free
axis**: `P := plane` is seven freedoms, `plane(u: std.x)` mints only `P.v`. A minted axis starts as the front's unless seeded,
`plane(v: hint(dir: (0, -1, 0)))` — seed one a relation will turn far. A seed may read the
direction off geometry: an axis (`hint(dir: up)`), a line from its `p1` to its `p2` in space
(`hint(dir: g)`), a plane's normal (`hint(dir: P.normal)`, or `P.n`), turned about another (`hint(dir: up, turn:
-90deg, about: g)`), so a word standing a plane square to a line it is handed seeds it from that
line: `plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))`. A frame of two square axes:

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

Stacked parts are drawn so. Two planes lying on one another are warned
(W113). `solventc --where P` lists `P.x`, `P.y`, `P.z` (where it stands) and `P.u`, `P.v`, `P.n`
(and `P.normal`, where the document reads it).

**A plane is also a view.** A point drawn `in P` has `P`'s own coordinates, so a dimension within
`P` is measured from `P.origin` along `P.u` and `P.v`. `a project b` says two points drawn in two
planes are images of one corner (one equation: their coordinates along the planes' common line
agree; E061 for points in one plane or parallel planes, E065 if the planes come out parallel).
Views carry no place on paper: the `.svd` places each.

- **`in top { … }`** writes membership once for every declaration inside. An instance joins a view
  whole: `t := Tooth(…) in top`. Inside a component, `in std.side { … }` draws in its frame's
  own datums (`engine/block.sv`); `in view { … }` over plane formals lets a part carry views it is
  handed (`engine/conrod.sv`), and `when flag == 1 { … }` over a 0/1 `Int` formal omits a view an
  instance does not show in. A body may instead draw in the plane an operand is
  drawn in, `in pin.plane { … }` (or `in pin.end.plane`, through a member): no view is handed to
  it, and it is drawn wherever what it is mated to is. A point standing in space has no plane
  (E040 at the reference).
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

`p` is still drawn in `std.front`; only the measurement is against the turned axes.

**A frame declared bare**: `f := frame` is a spatial frame that owns nothing until its
axes are named. Name `f.u`, `f.v` or `f.origin` and it has them, seven numbers a plane's are, placed
by relations; a point `in f` then has three numbers along them — `x` along `f.u`, `y` up, `z` out
— and without them it is a point in space. `in (f := frame) { … }` declares it in the header. A
frame is no set: `p coincident f` or `f distance(d) g` over one is E040, and an ordinate from its
origin (`p distance(d, along: u) f`) is not.

```sv
f := frame
fix(dir == (0, 1, 0)) f.u
fix(dir == (0, 0, 1)) f.v                       // up is z, out is x
fix(origin == (10, 20, 30)) f
in f {
  p := point
  fix((1, 2, 3)) p                              // stands at (13, 21, 32)
}
```

**An instance is a frame.** An instance stands in a frame, and its body's `std.*` name that
frame's datums. From outside they are `b.std.x`, `b.std.y`, `b.std.z`, `b.std.front`, …,
`b.std.origin`, the body's spellings, and once anything outside reads one the instance stands in a
frame of its own: square (six freedoms, not a bare frame's seven), starting where the frame around
its call stands and riding it, placed by relations on those datums — a mate:

```sv
cyl := components.cylinder.Cylinder(fw: z - D / 2)
cyl.std.origin coincident piv
cyl.std.y parallel pin.plane.normal
cyl.std.x perpendicular rod          // its z runs up the rod: DOF 0
```

Left unplaced, the diagnosis names `cyl.attitude` and `cyl.origin`. An instance whose datums
nothing outside reads stands in the frame its call stands in: the innermost frame of the `in`
blocks around the call or of an `in f` trailer on it, else its parent instance's, so at the root
the world. A plane is no frame: `t := Tooth(…) in top` changes nothing `std` names inside it. In a
frame `f`:

| datum | in frame `f` | the world's |
|---|---|---|
| `std.x`, `std.y`, `std.z` | `f.u`, `f.v`, `f.normal` (`f.u × f.v`) | x, y, z |
| `std.back` | `f.u` reversed | −x |
| `std.top`, `std.front`, `std.side` | `plane(u: f.u, v: f.v)`, `(f.u, f.normal)`, `(f.v, f.normal)` | (x, y), (x, z), (y, z) |
| `std.up` | `plane(u: f.normal, v: back)` | (z, −x) |
| `std.origin` | a point in that front at `(0, 0)`, which is `f.origin` | |

A point `in f` at `(x, y, z)` stands at `std.origin + x·std.x + y·std.y + z·std.z`, so a body in a
frame moved by a rigid motion is the root's drawing moved by it; the datums add no freedom, and
what the body declares rides `f` (a joint carries it whole). A body that reads no datum is placed
by what it is given (`ConRod(crank.pin1, end.bore)`): reading its `b.std` is E111. An axis starts
as the frame's around it, and a relation leaving it either way round takes the nearer: write the
mate so. The body's file says `use std`, a module's by its own; a relation word reads only its
operands, `std` included (E101).

**Model relationships, not coordinate tables**: a rectangle is perpendicular
sides, a width, a height and a position against a datum, not an ordinate for every corner; the
library deliberately has no coordinate-placement helper. A stepped outline is a chain of runs
(1.7), one number each, and not a table of corners.

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
  takes a set as a **`set` formal**, by position. A family the same set in any order of some
  formals says so before `:=`: `component Lens(a: point, b: point, r: Length) symmetric(a, b) :=
  { … }`, or `symmetric((a, la), (b, lb))` for formals that change places together; then
  `Lens(q, p, r: 5mm)` is `Lens(p, q, r: 5mm)`. Say it only where the body bears it out.
- A point formal may name the plane formal it is drawn in, `center: point in on`, and a call may
  then leave it out: `k := Circle(std.front, r: 25)` owns its centre, `k.center`, drawn in the
  front (`fix((5, 5)) k.center`); `center: hint((40, 0))` seeds it. Any other entity left out of
  a call is E103.
- Statements in one body are a conjunction (an intersection): `{ p | p coincident ball; p
  coincident std.front }` is the circle the front cuts from `ball`, and a point drawn in the front
  meets the plane's statement by being there. A union is a choice of root, not a body. A culprit
  reads as the use (`l tangent shaft`), never the body's rows.
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
- Two cones touching at `M` with one tangent plane there: `gc tangent(at: M) pc` (§6.21), `M`
  on each beside it. The library has no component for it.
- `inner := std.ConeThrough(ax, q, lean: 20deg)`: the cone about `ax` through `q`, its radius
  growing `tan(lean)` along `ax` (negative narrows it); its inside is always nearer `ax`. Its apex
  is the instance's (`inner.apex`), made once: a family's body makes once whatever reads nothing
  of its point.
- `top := std.HalfSpace(o, ax, d: 2mm)`: the half-space behind the plane `d` along `ax` from `o`
  (its surface that plane); inside is behind it, so `p inside top` is at most `d` along `ax`.

`p coincident ball` puts a point on any of them and `l tangent shaft` makes a line touch one.
`sphere_cone_cylinder.sv` shows one of each; 2.14 lays out pitch cones.

**Regions.** A body that bounds a number is a region: `ball := { p | p distance(<= 5) c }`.
`q inside S` / `q outside S` put a point in or out of any set, reading its one number as at most /
at least: `q inside std.Sphere(c, r: 5)`, `q inside std.Cone(l, half: 30deg)`. Regions meet by
conjunction, and `outside` makes a difference: `{ p | p inside heel; p outside toe }`. Refused:
`outside` a conjunction or interval, a set of several numbers, a tangency to a region. A point in a
region is a bound, so a choice of root.

**Offsets.** `p distance(4mm, side: outside) k` puts `p` 4mm off `k`'s surface (`side:` required);
a bound on it, `p distance(<= 2mm, side: outside) pitch`, is within 2mm outside it (`side: inside`,
at most 2mm in); `tip := std.Offset(pitch, d: 2mm)` is the set 2mm out, for where a set is passed
or used (`p inside tip`, `std.Rounded(tip, …)`). `inside`/`outside` take no number: `p inside(2mm)
pitch` is refused, naming these two. `S.surface` is `S` with each bound made an equality: of
`{ q | q distance(<= 2mm, side: outside) pitch }`, the offset surface — a set like any, so
`k coincident S.surface` puts a drawn point on it. A distance that comes to nothing
(`std.Offset(S, d: 0mm)`) is `S` itself. A drawn point measured from a set gets a foot on it, found
by the solve. A distance from `{ q | q outside S }` is from `S`'s surface, sides turned: so `std.Rounded(beyond,
tip, r: 0.3mm)` rounds the corner where a region is outside `inner` (`beyond := { q | q outside
inner }`).

**A crown tooth as a region**, and its land: two `ConeThrough` flanks, two `Level`s, the tip's
corners rounded — `tooth := { p | p outside inner; p inside outer; p inside back; p outside tip;
p inside std.Rounded(beyond, above, r: …); p inside std.Rounded(outer, above, r: …) }` — is a
revolution, and `solid(tooth)` swept under a generating motion cuts a member exactly. A region
has no tip line for `length(>= d) tip`: draw the land's ends on the tip, the flanks through
them, and bound the land between them (`eb distance(>= 0.01mm, along: left) ea`), which picks the
root as the drawn section's bound did. Where the flanks go through points the drawing already
places (the hypoid's pitch points, `spiral_bevel/hypoid.sv`), the land has no root to choose and
needs no bound; a round's radius may read an unknown of the drawing (`r: 0.3 * mn`).

### 1.14 Faces and solids

**A solid is a term, never a step.** A solid is a face swept, or a stock **plus everything in
`union` with it, minus everything that `cut`s it, within everything it is in `common` with**. The
order lives inside the term, never between statements: `bore cut body` may be written fifty lines
above `body := solid(…)`. **Nothing about a solid is solved for**: it owns no parameters, numeric
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
  end control points, so it is an ordinary edge, and so is a free curve (`rope := curve(a, b)`) at
  the two points it is written with: `face(rope, deck)` or `face(rope, -> close)` (1.10).

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
k := trace(Par(o).p over u in (-2mm, 2mm))
```

`4 params, 4 equations; DOF 0`, and `cap` is the parabolic cap, 4/3 mm² × 3 mm = 4 mm³
(`examples/solid_tooth.sv` is an involute tooth so).

#### Sweeps

- **Prism**, along the face's normal: `depth: 30mm` is the material *behind* the visible face,
  `from: -30mm, to: 0mm`; write `from:`/`to:` when the face is not at an end. `through: body` spans
  the target's stock and additions both ways and follows the solve.
- **Draft**: `draft: 3deg` beside `from:`/`to:` or `depth:` tapers the sides from the face's own
  plane (the parting plane, ordinate 0): the section at `h` is the face moved `|h| tan 3°` into its
  material, so the sides lean away from that plane on both sides of it, a hole widening as the
  outside narrows. Lines' sides stay planes and circles' become cones; faces keep their names, a
  side crossing the plane two pieces of one name. An arc meeting a side at an angle (not tangent)
  is refused by the exact kernel. `conrod/rod.sv` drafts its eyes and shank.
- **Revolution**, about a line or axis **in the face's own plane**: `about: ax` is a full turn,
  `sweep: 90deg` a partial one (caps `start` and `end`), `sense: cw` reverses it. An axis lies in
  the plane when it is one of the plane's own (`about: std.z` on `std.front`) or the solve puts it
  there (else E081).
- **Face family**: a loft, or a sweep along a path, is a face of an instance as a declared
  parameter runs (below).

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
  q0 distance(10, along: u) p0
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

#### Face families: lofts and sweeps along a path

**A face family is a face of an instance as a formal runs.** With `param s: Scalar over (0, 1)`
declared, `body := sweep(Reducer(guide, std.z, inlet: 24mm, outlet: 12mm, wall:
2mm).sec.profile over s)` is the face `sec.profile` of the instance as its formal `s` runs over the
parameter's interval (`solid_loft`, `solid_elbow`, `solid_bend`, `solid_twist`). The formal that
runs is the one named as the parameter, or the one the call gives it (`Tube(hose, std.x, r: r, s:
s2).f over s2`, `paths/cable.sv`). A family stands in the document (a component sees no
parameter). The instance is written in place, leaves `s` unbound and is never drawn; it is solved
at every station, `s` a number there, so its body may use any expression in `s` and read a number
of the geometry it was given (`rim.r`).

- **Every station must solve** to DOF 0 with the section's shape intact; an end may shrink to a
  point (a pyramid's apex). Caps are `start` and `end`, walls are named by the section's edges
  (`body.sec.ab`).
- **Seed the section where it belongs at the middle of the interval**: the branch the drawn station
  takes is the one every station keeps (`docs/lofts/l0/`).
- A body reads its frame's `std.z` itself (1.12), so a station whose `up` is always the world's
  needs no formal for it; an `axis` formal is for an `up` the call chooses.

**Where the section stands, and what it is**, are constraints in the component: a station word of
`std` for the plane square to the guide at `s` — `std.OnLine(g, up: up, s: s)` on a line,
`std.OnArc` an arc, `std.OnSpline` a spline, `std.On` any of a line, an arc, a spline or a traced
curve — then the section drawn `in st.n`, its sizes and turns reading `s`. `On`'s `v` leans toward
`up` where the family is drawn (the middle of its interval) and from there its frame turns only as
the guide makes it: with `up` the normal of the guide's plane its `v` stays along `up`, and along a
guide in space its plane stays square to the guide.

**A law is any expression in `s`.** Draw the section once and carry it whole: `tw :=
std.Twisted(st.n, twist: 90deg, reach: 10mm, s: s)` is the point `tw.t` turned `twist · s`, then
`sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n` carries every corner and edge of `prof`
into `st.n` by the similarity taking `o` and `x` (the section's frame, `reach` apart) to the
origin and `tw.t`. A `reach` that runs with `s` scales it; the edges keep their names (`sec.ab`), an
arc or a circle its radius scaled (two pairs only), and `mirror: true` gives a handed pair's other
hand, wound as its source (`lofts/tapered.sv`, `hub_motor/motor/cover.sv`). For a section written
as coordinates, `p scaled(du: 10, dv: 5, scale: 0.5, s: s) st.n` places a corner shrinking to half
and `scaled_radius(r: 8mm, scale: 2, s: s) k` a radius (`use std (scaled, scaled_radius)`); each
runs its total linearly, and the word given another fraction runs another law (`s: 3 * s^2 - 2 *
s^3` eases it). A `when` cannot read `s` (it is made once); write a law in pieces with `min`, `max`
or `abs`.

**A screw.** `std.Helix(std.top, pitch: 4mm, turns: 3, s: s)` is `std.top` carried by the screw
about its normal to `360deg · turns · s` — turned that much and lifted `pitch · turns · s`
(`std.top` drawn in a screw whose value is that roll, 1.15). Draw a thread's section in its
`axial` plane (through the axis, `u` out from it, `v` up it) and an auger's flight in its `n` (the
base carried); `std.OnHelix(std.top, r: 10mm, pitch: 5mm, turns: 4, s: s)`'s `n` stands square to
the helix of radius `r`, for a coil's wire. A negative pitch winds left-handed
(`rust/gcs-core/tests/helix.rs`). A family whose station a motion carries this way is exported as
that motion's sweep, exactly (`lofts/spring.sv` and `swept_spring.sv` are one tube).

**A named open chain of lines and arcs is one path.** `sweep(Tube(route, std.z, r: 4mm).f over s)`
over `param s: Scalar over (0, 5)` runs the station over `route`'s five links one after the next,
link `k` while `s` is in `(k, k + 1)`; a law in `s` reads the chain's whole `s`, and the walls keep
their names across the links (`docs/lofts/l0/pipe.sv`).

**Several sections** are joined by **rails**, one a corresponding corner — its places in order, one
a section — the station's corner where its rail meets the plane: `rim := point in st.n, front` and
`rim coincident rail` (or `point in st.n` for a rail of lines in space). Every chain a station is
given runs **in step**, link by link (of as many links), so a guide cut at the sections (`axis :=
line(base, shoulder) -> line(shoulder, top)`) and a chain of lines through each corner (`rims :=
line(r0, r1) -> line(r1, r2)`) is a **ruled** loft, an edge at each section; `spline(through:
rims, start: axis_line, end: axis_line)` is the **smooth** one's rail, square to its end sections
(or tangent to a neighbouring face's edge). With no guide, `std.Across(ra, rb, rc, s: s)` is the
plane through three rails at `s` (`docs/lofts/l0/bottle.sv`, `rust/gcs-core/tests/sections.rs`).
A profile drawn once is **held to its rails** by `sec := std.Similar(prof, from, to, st.n, l, r)`
— `from` and `to` carried to the station's points `l` and `r` where two rails cross it, the
profile turned and scaled with them — or by `std.Affine(prof, from, to, third, st.n, a, b, c)` on
three (`docs/lofts/l0/two_faces.sv`, `rust/gcs-core/tests/correspondence.rs`).

**Two faces drawn apart** are lofted by `sweep(std.Ruled(inlet, outlet).f over s)`: corner `k` of
each walk joined to corner `k` of the other by a straight rail, the ends the faces themselves. The
pairing is the written order — start a walk at another corner to connect it differently, and name a
point on an edge (`m coincident(t == 0.5) half`, `face(t0, t1, t2, m, -> close)`) to give a
triangle a fourth corner for a square; keep the first three corners out of line.
`std.Apex(base, tip).f` lofts a face to a point.

**A frame that does not twist.** Along a path that leaves its plane (a helix), `st :=
std.OnHelix(…)` turns with the path's Frenet frame; `rm := std.Minimal(st.n, up: base.normal, s:
s)` is the plane through the same place square to the path whose axes turn only as the path makes
them, so draw the section `in rm.f`. It states its turn as a **rate**, `rate(f.u over s)
perpendicular f.v`, and what holds where the family is drawn (the middle of the interval) in
`initially { f.u perpendicular up }`; the family integrates it from there both ways
(`rust/gcs-core/tests/rates.rs`). A rate is of an axis or a point, against an axis, a line or a
plane, by `perpendicular` or `parallel`, and names the formal it runs over; give a word that states
one the family's running number as that formal.

**A guide drawn in space** is a curve traced in space handed to a `path` formal: `m coincident(t ==
s) g` stands the station at the body's own formal `s` and `hint(at: m, along: g)` steps along it;
`std.On(g, up: std.z, s: s)` stands on it as on any path, its frame the rotation-minimising one
(`rust/gcs-core/tests/space_paths.rs`). A surface traced over two formals is no path. **Frenet's
frame**: `std.Frenet(g, s: s)` turns with the curve, `u` toward its centre of curvature (`g
curvature(at: m) c`, a point) and `v` the binormal; it is refused where the curve runs straight.

**Writing a part.** Where a part's axis lies relative to its section decides the statement: the
V-twin piston is a turn of its half-profile; the disc, flywheel and throttle have their axis
through the section, so they are prisms with turned features (a radial set screw) only where the
axis lies in it. **A part's section must be where its turned features are**: a turn about a line in
the section centres what it makes on that plane, so the crank disc is sectioned on its mid-plane.

#### Bodies

Add to the 60 × 40 plate, inside its `in std.front` block:

```
o := point hint((30, 20))
a distance(30, along: u) o
a distance(20, along: v) o
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
- **`tip common body`** keeps only what the body has in common with `tip`. Union comes first;
  `cut` and `common` commute.
- **A pattern of solids.** A solid declared inside a `ring` is one solid a copy, each
  reached by its index (`peg[3]`), as a point of the ring is. To join every copy alike, start from
  an empty body — its first `union` is its stock — or cut each from the body:

  ```
  ring 6 about std.origin {
    …                                     // c on a circle, hole := circle(center: c) …
    peg := solid(face(hole), from: 0mm, to: 10mm)
  }
  pegs := solid()
  repeat 6 as i {
    peg[i] union pegs
  }
  ```

  `peg[i] cut plate` in the same `repeat` cuts each from a body instead. `solid()` with nothing in
  `union` with it is E080. Faces are reached through the copies, `pegs.peg[3].near` one copy's and
  `pegs.peg.near` every copy's.
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

#### Materials

A material is declared once and named by the solids made of it:

```
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.top {
  rim := radius(20mm) circle(center: std.top.origin)
  disc := solid(face(rim), depth: 5mm) of steel
}
```

- **The properties** are `E` (Young's modulus) and `yield` (yield strength), both pressures
  (`Pa`, `kPa`, `MPa`, `GPa`); `nu` (Poisson's ratio), a plain number between -1 and 0.5; and
  `rho` (density), a mass over a volume. Each one is optional. **Write the unit** on each one with
  a dimension: `E: 200000` is E103, because a bare pressure is in the document's units, which no
  one means. There is no single-word unit of density, so write `7850kg / 1m^3` or
  `2.7g / 1cm^3`.
- **`of`** follows a `solid(…)` declaration, as `in` and `hint` do. It names a material in scope.
  Naming anything else is E103. A material can be passed to a component as a `group` formal
  (`component Plate(m: group, …)`, then `of m`).
- `steel.E` reads as a number with its dimension, the way a group's members do. `steel.E / 1GPa`
  is 200.
- Nothing is solved for a material. A frame's analysis reads one (below); nothing reads a solid's
  yet (#510). `solventc` prints each material and what is made of it: `material steel: E 200GPa,
  nu 0.3, …, of disc`.

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
- **One copy of a pattern** (0.157): an operand written in a `repeat` or a `ring` is named with
  its copy's index, as `--where` reports it — the hole the second drill cut is
  `flange.drill[1].hole`. `flange.cd chamfer(d: 1mm) flange.drill[1]` chamfers that one hole
  (`drill[i]` in a `repeat`); `flange.drill` is every copy's.
- Each acts on the body after its other terms: a round reads the faces the cuts and unions left,
  and not another round's — unless it names that round (below).
- Where two faces meet tangent there is no edge: a chain of tangent edges (an obround's foot, a
  rounded pocket's rim) is rounded as one run. A run that ends on a face it does not round is
  carried on past its edge's end and cut off by that face, however the face stands.

- **Name a round to compose over it** (0.149): `bevel := plate.rim chamfer(d: 4mm) plate.far` puts
  its faces under the name (`plate.bevel.taken.flat`, `plate.bevel` every face it made), and a
  round that names them reads the body as that one leaves it: `plate.bevel round(r: 1mm) plate`
  rounds the two edges the chamfer made, whichever line comes first. Two named rounds' faces stay
  apart; written bare, every round's are `taken.round`. Rounds that name each other are E041.
- **The ball and the flat follow a face across a seam**: where the far face goes on tangent into
  a rim's round, a chamfer at a boss standing near the rim lands its leg on the round
  (`F.smooth`, a face's surface carried across its tangent seams). Where the leg or the ball
  reaches such a seam beside its edge the round is read point by point, not built exactly.

They are written in Solvent (`rust/lib/std.sv`), from region bodies you can write yourself
(`rust/examples/rounds/by_hand.sv` rounds one edge in about twenty lines):

| written | means |
|---|---|
| `p inside block.near`, `p distance(<= d, side: outside) block.near` | the material's side of the face's surface (grown by `d`) |
| `p inside block.near.smooth`, `distance(p, block.near.smooth)` | the same of its surface carried on across every tangent seam, the nearest piece's |
| `p distance(<= 1mm) block.near` | within 1 of the face, bounded by its edges |
| `p distance(>= r) block.near.surface` | at least `r` from the whole surface |
| `distance(p, f1) + distance(p, f2) <= 10mm` | a bound on measurements (an ellipsoid by its foci) |
| `crease := { q \| q coincident a; q coincident b }` | the edge where faces `a` and `b` meet |
| `angle(a, b, at: p) <= 180deg` | at a convex edge (the material's angle there) |
| `p inside crease.ends` | stopped by the faces the edge ends on (none at a tangent seam) |
| `meeting := { q \| q coincident a.surface; q coincident b.surface; q inside crease.ends }` | where the surfaces meet, short of where the edge ends |
| `repeat a in A { repeat b in B { when b != a { … } } }` | a copy per pair of faces, each made where the two differ |
| `when c in B and c != a { … }` | made where `c` is one of `B`'s faces and not `a` |
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

#### Load scenarios

A `scenario` states a load case over the solved drawing: supports, loads and requirements, one
per line. It stands at the top of the document (or in its `preview`), reads the document's names
and numbers, and adds no geometry and no row. A frame's requirements are answered and judged
(below), and so are a solid's read in its plane; a solid read in three dimensions is not analysed
yet (§9.11).

```
scenario lift {
  fixed plate.west                       // a face path; `pinned X` holds without turning
  load tip (0, -500N, 0)                 // a force, world axes; or `load tip 500N along: down`
  deflection(tip, along: down) <= 0.5mm  // a bound
  safety(plate) == 2                     // a driving row: one scenario's may drive
}
scenario overload: lift {                // lift's supports and loads, then these
  pressure plate.near 2MPa
  claim mode(plate, 1) >= 120Hz          // judged, never solved for
}
```

| written | reported |
|---|---|
| `patch := point` inside | syntax — a scenario adds no geometry: draw it in the body |
| `load tip (0, -5N)` | syntax — a load's vector is `(x, y, z)`; in a plane use `along:` |
| `load tip (0, -5mm, 0)` | E103 — a force is a `Length·Mass·Time^-2` (`N`) |
| `pressure tip 1MPa` | E040 — a pressure is on a face |
| `fixed std.front` | E040 — applied to a point, an edge, a face or a solid |
| `claim deflection(tip) <= u` | E040 — `u` is an unknown; a scenario's numbers are stated |
| `== v` in two scenarios | E106 — state the second a claim |
| a scenario in a component | syntax — at the top of a document |

#### Frames

A frame is drawn on the lines its members run along, and analysed under a scenario (§9.11):

```
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
b := 25mm
h := 50mm
in std.front {
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  fix((1000mm, 0mm)) end
  span := line(foot, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * h * b^3,
            Sy: h * b^2 / 6, Sz: b * h^2 / 6) of steel
scenario tip_load {
  fixed foot
  load end 1kN along: down
  claim deflection(end, along: down) <= 7mm   // solventc: … is 6.4mm: claimed <= 7mm, kept
  claim safety(arm) >= 2
}
```

- **The section** is stated in each member's own axes: x along it, z the normal of the plane its
  line is drawn in. `Iz` is its second moment bending in that plane, `Iy` across it, `A` its area
  and `J` its torsion constant — all four required; `Sy` and `Sz` (section moduli) only where a
  `safety` is asked. Write each from the section's sizes, or as a number of its unit: a unit is
  a suffix, so `65104mm^4` is (65104 mm)⁴, and a figure is `65104 * 1mm^4`. A beam may name
  several lines: `beam(left, top, right, …)`.
- **Joints** are where members' ends meet, or an end lies on another member; a point the scenario
  names on a member (a load's) cuts it there. Every joint is rigid.
- **What is answered**: `deflection(p[, along: d])`, `mass(arm)`, `mode(arm, n)`, `buckling(arm)`,
  `compliance(arm)`, `safety(arm)` (yield over `|N|/A + |My|/Sy + |Mz|/Sz`). `load arm 2kN along:
  down` spreads a force along the beam; `accelerate arm (0m / 1s^2, 0m / 1s^2, -9.81m / 1s^2)` is
  its weight. A claim is judged, never solved for: `solventc` prints its value and `kept` or
  `refuted`.
- **A driving deflection sizes the frame**: `deflection(end, along: down) == 4mm` at a joint is a
  row of the drawing. Declare what it sizes, `param h: Length hint(40mm)`, and read the section
  over it (`Iz: b * h^3 / 12`, any formula); or let it place a joint (`foot distance(len) end`,
  `len` an unknown). `solventc` prints `driven == 4mm, held` and the unknown's value. `along:` is
  an axis or a line the drawing holds. Any other driving requirement (`mode(arm, 1) == 30Hz`) is
  only judged yet. The library shows each: `sized_cantilever.sv` (a depth),
  `reach_cantilever.sv` (a length), `shelf_bracket.sv` (two members, a load spread along the arm)
  and `portal_frame.sv` (a tube's diameter, a point on a member, the frame's weight).
- `pinned` holds a joint's moves, `fixed` its turns too, so a beam pinned at both ends is free to
  twist about its own line — and `solventc` says so (`leaves `arm` free to turn about x …`).

| written | reported |
|---|---|
| `beam(p, A: …)`, `p` a point | E040 — drawn on lines |
| a line standing in space | E040 — a beam is drawn in a plane |
| no `J` | E103 — states `A`, `Iy`, `Iz` and `J` |
| `A: 50mm` | E103 — `A` is Length^2 |
| `fixed arm` | E040 — hold its ends |
| `safety(arm)` with no `Sz` | no answer — reads `Sy` and `Sz` |
| `Iz: b * h^3 / 12`, `h` an unknown nothing drives | E040 — nothing determines it |
| `deflection(end, along: d) == 4mm` on a frame left free | E040 — the frame is free to turn |

#### Solids read in their plane

A prism — a face swept square to its plane, `solid(F, from: a, to: b)` — is read over its face
when a scenario says so: `plane stress plate` for a thin plate (free across its thickness),
`plane strain dam` for a long one (held along its length). Its depth is the thickness.

```
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  …
  hole := arc(center: o, start: m, end: s) hint(r: 10mm)
  quarter := face(cut_x, pulled, side, cut_y, hole)
}
plate := solid(quarter, from: -1mm, to: 1mm) of steel
scenario pull {
  plane stress plate
  pinned plate.cut_x along: std.z            // a wall of the prism is a side of the face
  pinned plate.cut_y along: std.x
  pressure plate.pulled -50MPa               // a tension: a pressure pushes in
  claim safety(plate) >= 1.6                 // yield over the greatest von Mises stress
}
```

- **Where a line falls**: a face path naming a wall of the prism (`plate.cut_x`), or the line it
  was swept from, is that side of the face; the solid, its face or a cap is all of it; a point in
  it is a node of the mesh. A hold across the thickness (`pinned m along: std.y` in the front)
  holds nothing the plane moves, and `fixed` holds what `pinned` does.
- **What is answered**: `deflection(p[, along: d])` (or a side's, the solid's worst),
  `compliance`, `mass`, `mode(plate, n)` and `safety(plate)`; the material states `E` and `nu`.
  `buckling` has no answer (a plate buckles out of its plane), nor does a load across the plane,
  nor a `safety` where a sharp inside corner leaves the stress unbounded: round it.
- **A driving deflection sizes the region**: `deflection(corner, along: down) == 0.5mm` at a
  point of the region is a row. Its mesh is made once, where the drawing stands when it is
  elaborated, and its nodes move with the drawing — so seed the region near where it will be
  solved (a circle's `hint(r: …)` too). Its sides are lines, arcs and circles. The library's
  `sized_plate.sv` finds a plate's depth so, and `drilled_plate.sv` a hole's radius.
- **The library shows the rest**: `tuning_fork.sv` (modes in the plane), `rounded_bracket.sv` (a
  round that gives a corner's stress an answer), `pressure_pipe.sv` (plane strain, Lamé's closed
  form) and `gravity_wall.sv` (plane strain under its own weight).

| written | reported |
|---|---|
| `plane stress p`, `p` a point | E040 — a solid is read in its plane |
| `plane stress body`, `body` turned or cut | E040 — a prism, its face swept square to its plane |
| `plane stress plate` twice in a scenario | E001 — read in the plane already |
| `plane stretch plate` | syntax — `plane stress` or `plane strain` |
| `claim buckling(plate) >= 1` | no answer — a plate buckles out of its plane |
| `deflection(p, along: d) == v` over a spline's region | E040 — a spline or a curve is not moved yet |

### 1.15 Frames that move, and generated geometry

**A motion is a frame's joint.** A frame declared bare (`f := frame`, 1.12) is a rigid frame in
space; its joint is an axis `f.axis`, a ratio `f.ratio` (radians a radian of the one shared roll)
and an advance `f.advance` (along the axis a turn of the roll), held by relations like anything
else. The standard library states them as relation words, the frame the subject and what it
moves about or along the object (`use std (turns, screws, spins, slides)`):

```sv
crown := frame
crown turns(1) crown_axis            // turns about a line, a radian a radian of the roll
blank := frame
blank turns(2) blank_axis            // twice the angle
tap := frame
tap screws(1.5mm) hole_axis          // a screw: a turn a turn, 1.5 mm along it
plunge := frame
plunge slides(20mm) spindle          // a translation, 20 mm a turn
wheel := frame
wheel spins(1) o                     // about a point, square to the view it is drawn in
```

`spins` reads the view from its point (`o.plane`); a point standing in space has none (E040).
`screws` turns a radian a radian of the roll; a screw at another ratio is its relations written
out, `l coincident f.axis` and `fix(ratio == r, advance == a) f`. The components `std.Turn`,
`std.Spin` and `std.Slide` said the same through 0.153 and are kept for one release.
Roll 0 is as drawn: a member that must stand turned there is drawn turned.
A motion never moves the sketch, and a frame with no joint stands still: the world. The ratio is
`Scalar`; a measurement of the drawing there is E107.

**A ratio may be the solve's**: given a declared unknown, `param k: Scalar` then `pinion
turns(k) shaft`, it is found by a relation reading the motion. `std.Rolls` says the usual one —
two pitch circles rolling at their pitch point without slipping along their common tangent:

```sv
wheel_turn turns(1) spindle
param pinion_ratio: Scalar                                          // not stated: found
pinion_turn turns(pinion_ratio) pinion_axis
std.Rolls(pitch, pinion_turn, wheel_turn, along: std.front.normal)  // −R/r, from the radii
```

State the ratio instead and the same relation places the pitch point (`lantern_generation.sv`). A
profile generated in the plane needs its numbers stated; a joint nothing holds is reported free.

**Seen from another frame.** Every reading of a frame's motion — `under:` on a solid, an
envelope, a meshing, a carried point — may say whose view it is in: `under: crown, seen_from:
blank` is the crown as the blank sees it, which is what generates a gear. Left out, the observer is
the world. A component that reads a motion takes the frame (`m: frame`) and, where it needs one,
the observer as a second (`seen_from: frame`); a frame formal a call leaves out is the instance's
own frame at rest, the world.

**Riding.** Draw a part in the frame that carries it, and readings at a roll need no
`under:`. `in` blocks nest; what is declared in nested blocks rides on every frame of the path and
is drawn in the innermost one with axes (a view, usually), and a frame declared in a block rides
on that block's frame:

```sv
in (arm := frame) {
  in std.top {
    c := point
    fix((30, 0)) c
  }
  in (planet := frame) {
    in std.top {
      x := point
      fix((40, 0)) x
    }
  }
}
arm spins(1) o
planet spins(-3) c                                  // its axis where it is drawn, at rest
X coincident(t == 60deg) x                          // (10, 17.3205): the arm's turn, then the planet's
```

At rest everything stands as drawn, so a relation naming no roll never sees riding. A reading at
a roll that leaves `under:` out reads what its tool rides on: `envelope(flank over t in (a, b),
seen_from: blank)` with `flank` drawn in the rack, `meshing(f1, f2 over t in (a, b))`, `tool at
30deg`, `sweep(tool over t in (-20deg, 20deg))` and `sweep(tool, through: body)`. `under:` stays
for a tool that rides on nothing; naming it for one that rides is an error, as is a tool whose
ends ride on different frames. What a reading makes rides on its observer (the profile a rack cuts
rides on the blank). Two views nested draw in the inner one, with a warning (W115).

**Two operations on what moves**: every reading of a frame's motion is one of
them. **At an instant**, `x at τ` is `x` where the roll `τ` puts it — one verb for a frame, a solid
and a point — its frames on the `at`, `x at(under: m, seen_from: g) τ`, where `x` rides on none:

```sv
n := std.front at(under: screw) 140deg          // a frame: the plane carried, origin and axes
indexed := tool at(under: indexing) 90deg       // a solid: one copy at that roll
X := x at(under: screw) 40deg                   // a point: one of its own, held there
P coincident x at(under: screw) 40deg           // ... or a point drawn elsewhere, held there
```

**Over an interval**, `over t in (a, b)` names the interval of the roll a reading runs over, after
the operand that runs: `sweep(tool over t in (-75deg, 75deg), under: turn)` is the solid the tool
sweeps, `envelope(tool over t in (a, b), …)` what its boundary leaves, `meshing(f1, f2 over t in
(a, b), …)` the contact as the first flank's phase runs. `over t` alone names a declared `param t:
Angle over (a, b)` (1.6) and reads its interval. The same word is a face family's and a traced
curve's: `sweep(Station(g).f over s)` over a declared `s` (1.14), `trace(leg.toe over theta in (0, 360))`
(1.10).

**A frame's motion moves a solid**: `tool at(under: indexing) 90deg` is one copy at a constant
angle (`solid_indexed_pattern.sv`); `sweep(tool over t in (-75deg, 75deg), under: turn)` is the
union of the tool over the whole interval, a continuous sweep meshed from its material field
(`swept_torus.sv`). `sweep(tool, through: body, under: crown, seen_from: blank)` rolls the tool
over the whole roll about 0 during which it meets `body`'s stock and additions (never its cuts, so
it may cut `body`), found after the solve and clear of the body at both ends — a generating cut
states no roll limit (`spiral_bevel/hypoid.sv`'s `Generated`). A sweep of a sweep is refused, and
for an exact export a swept solid may only be `cut`. Exact exports cover the generating classes
([generating-sweeps](generating-sweeps.md)): revolved tools cutting gears (`lantern_generation.sv`,
`spiral_bevel/`), a wheel carried along a screw (`twist_drill/`), a pocketed prism under a planar
motion (`wankel/`).

**Envelopes in the plane are curves the solve sees.** Over a tool of the sheet (a point, line,
circle, arc or formula curve) and a planar motion, `flank := envelope(rack_flank over t in (-25deg,
25deg), under: rack, seen_from: blank)` is the profile the tool cuts in the moving frame.
`coincident`, `tangent` and `curvature` hold against it, and the tool's and motion's geometry are
its columns, so a conjugate is synthesised, not stated (2.11). `side: near|far` picks where a
circle cuts twice (a cam). A point's envelope is its path: `bore := envelope(apex over t in (0deg,
1080deg), under: rotor_turn)` is the epitrochoid, typed nowhere, closed after whole turns and a
face's loop by itself.

**The rotor is a cut**: the largest rotor turning in a bore is the blank less everything the
housing's wall sweeps through — `swept := sweep(housing over t in (0deg, 1080deg), under:
housing_turn)`, `swept cut rotor` (`common` with a sweep keeps the union's inside, a different set).

**A solid's side is a `surface`, and the word has two meanings, one per kind of solid.** Over a
revolution, `surface(crown, edge: rack.outer[, from: 180deg, to: 360deg])` is the side one profile
edge sweeps round, `u` along the edge, `v` round (read the domain, never assume [0, 1]): what a
spatial envelope over it reads as its source, held to by the solve (below). Over a prism drawn in
a view it is the side its edge extrudes, a planar envelope's source:

**A prism's side** is named as a revolution's is, `side := surface(rack_tooth, edge: rack_flank)`;
under a motion keeping its view, `envelope(side over t in (a, b), under: rack, seen_from: blank)` is
the planar envelope extruded, and `p coincident flank` from any view is one equation. Draw the
prism's face `in` a view and name the edge in it.

**An envelope in space is held to by the solve** where its source is a cone, a cylinder or a
plane set (`std.Cone`, `std.ConeThrough` with its lean stated, `std.Cylinder`, `std.HalfSpace`) or
a revolution's side turned
from a line, an arc or a circle, under turns about lines (or about a point of a view, the line
through it square to the view), screws and slides — driven by a law or not — seen from the world
or another frame
(`spiral_bevel/mean_point.sv`):

```
flank := std.Cone(spine, half: hint(20deg))           // the half an unknown
pinion_flank := envelope(flank over t in (-30deg, 30deg), under: gear_turn, seen_from: pinion_turn)
M coincident pinion_flank                             // owns the place [u, v, roll]
pinion_flank perpendicular(at: M) normal              // along its normal at M
pinion_flank tangent(at: M) along                     // a line (or a plane) touching it there
pinion_flank curvature(at: M) profile                 // its section by profile's plane
```

The source's geometry, its number and the motion's axes are columns, so the cone's half solves from
what is stated at `M`. A plane sliding along its own normal relative to a turn sweeps an involute
helicoid: `spiral_bevel/phillips.sv`'s rack, `std.HalfSpace(P, L, d: 0mm)` under
`slide, seen_from: pinion_turn`. The `at:` relations read the place of the `coincident` holding `M`, which must
be stated: each is that contact's frame at `M` — its normal, a plane or a line square to it, a
section's second order — read by the one kernel a rate of a contact is. A set's envelope is only held to: it has no chart
after the solve.

**A point a frame carries**: `X coincident(under: m, t == τ) x` is `x` as the moving frame sees it,
carried by `m`'s pose at the roll `τ` (degrees; leave it unpinned and the solve finds it) — a
rolling circle's point seen by the fixed one is an epicycloid. **At a stated instant**,
`X coincident x at(under: m) τ`, the roll is the relation's number, an angle — the
same reading as `t == τ`, never both: written over a face family's running number (`at 360deg *
turns * s`) it is read again at every station, and over a `param` it is that unknown. Drawn in a
plane, `X` is held where that plane sees the carried point (two rows). Leave `under:` out where `x`
rides on a frame (`in f { … }`), as for any reading. **A frame carried**: `n := base at τ` —
`at(under: m)` where `base` rides on nothing — is the plane `base` there, origin, axes and normal;
every frame that moves is a frame read at a roll. A frame turns about another's axis as about a
line: `screw screws(pitch) base.normal`.

**A joint's value**: a jointed frame stands at an instant of the roll,
`fix(value == τ) f` beside its ratio and advance — its joint turned `ratio · τ` and risen `advance
· τ / 360°` from where it is drawn; with no value it stands at rest. `n := x in f` is `x` drawn in
`f`, where its value puts it: the reading `x at(under: f) τ` with the frame's instant, a frame, a
solid or a point as `x` is. The value is an angle worked out as a dimension is, so it may read a
face family's running number (read again at every station) or name an unknown the solve finds:

```sv
screw := frame
screw screws(pitch) base.normal
fix(value == 360deg * turns * s) screw
n := base in screw                       // `std.Helix`'s station: base where the screw stands
```

`std.Helix`, `std.OnHelix` and `std.Twisted` are written so. A frame with a value rides on none,
and no frame rides on it. A value over a declared `param t: … over (a, b)` holds for every `t`
(1.6): what is drawn in the frame is a trajectory.

**Drawn in a frame with a value**: what its `in` block draws stands where the
value puts it. Each plane the block draws in is read as that plane drawn in the frame (`P in f`),
and so is the plane a relation in the block names, so the block is the drawing at rest carried
whole; a face of it is swept as a declared parameter runs with `sweep(sec over s)` — a face family
whose station is the drawing itself (`lofts/thread.sv`'s thread):

```sv
param s: Scalar over (0, 1)
screw := frame
screw screws(pitch) std.top.normal
fix(value == 360deg * turns * s) screw
in screw {
  in std.front {
    c := point
    c on_u(d: r) std.front                // the front plane as the screw carries it
  }
  sq := std.CenteredRectangle(c, w: size, h: size) in std.front
  sec := face(sq.loop)
}
thread := sweep(sec over s)
```

Drawn in the valued frame and no plane of it is E080 (a value carries a plane); a frame declared
in the block still rides on the frame with the value (E080).

**A frame along a path**: a joint held along a path rather than about a line, `g
coincident f.axis` with `fix(ratio == 0, advance == 0mm) f`, stands where the path's parameter puts
it — square to the tangent, turned as the path makes it from its home (where it is drawn), the
rotation-minimising frame for ratio 0 and the path's own (toward its centre of curvature) for
ratio 1. `std.Along(n, g, up: std.z, home: 0)` and `std.AlongFrenet(n, g)` say it and draw `n` at
its home; a face drawn in `n` and swept over a declared `s` stands at every station where the frame
does, `s` the path's own parameter. Its pose is read off the path after the solve, so a relation the
solve holds reads none (E080).

**A value over a parameter is a drive**: any expression of it, a cam's law, and the frame
stands at the law's value for every `t`. A reading of its motion over the roll reads the roll as
`t`, so a follower's groove in its cam is the follower's roller swept as the cam sees it:

```sv
param t: Angle over (0deg, 360deg)
follower := frame
follower slides(360mm) stem                        // its value in degrees is its rise in mm
fix(value == 5deg * (1 - cos(t))) follower
roller := r0 in follower                           // where the law puts it at the held `t`
groove := sweep(ball over t, under: follower, seen_from: cam)
```

A value reads one parameter. A point carried by a driven frame to a roll is held where the law
puts the joint at that roll, and its rate reads the law's rate; the law reads stated numbers
alone (one the solve finds is E080). The roll may be left to the solve, to find an instant:

```sv
Z := point
Z coincident(under: follower, seen_from: cam) r0   // the roller as the cam sees it, at a roll
Z distance(23mm) std.origin                        // found where it has risen 3 mm
```

An envelope in space generated under a drive is held to as any envelope in space is, the joint
standing at the law's value of the roll where the source touches it:

```sv
tube := envelope(skin over t, under: follower, seen_from: cam)
P coincident tube                                  // the ball's radius solves through P
```

A meshing on a drive reads no drive (E061): draw in the frame instead.

**A frame the drawing moves**: a frame with axes and no joint, placed on geometry that moves as a
declared parameter runs, is read over that parameter as a motion — where the drawing followed along
it puts the frame, carried from where the frame now stands. So a tool on a linkage's coupler is
swept, placed and cuts a profile as one on a joint does:

```sv
param t: Angle over (0deg, 360deg)
// … a four-bar in std.front, its coupler b–c, a ball and a circle `roller` about a point of it
coupler := std.Turned(b, c) in std.front          // a frame on the coupler
groove := sweep(ball over t, under: coupler.axes)  // the tube about the coupler curve
cut := envelope(roller over t, under: coupler.axes) // what the roller clears, in the plane
held := ball at(under: coupler.axes) 90deg         // at an instant: the document's one parameter
```

The reading must run over the declared parameter (`over t`, an angle); over an interval of the roll
that names none the frame stands still. Nothing the solve holds reads it: a point carried by it is
E080, and the envelope is drawn and swept but held to by nothing.

**A motion's twist is read by rates.** The roll `t` is the slot of the `coincident` holding
`X`, and `rate(X over t)` is how fast `X` runs a radian of it — the twist there. A rate of a contact
is held along a line or square to one, or measured along one:

```
X coincident(under: gear_turn, seen_from: pinion_turn, t == 0deg) x
rate(X over t) parallel l                             // X moves along l
rate(X over t) perpendicular l                        // … or square to it
rate(X over t) distance(v, along: l)                  // its speed along l, a length a radian
```

The standard library says the two things a gear designer reads of a twist this way (`use std
(reciprocal)`): `L reciprocal(seen_from: pinion_turn) gear_turn` — a force along `L` does no work
on the motion, for a mesh the meshing law of a line of action (the members move as fast along `L`)
— and `screw := std.Screw(isa, gear_turn, seen_from: pinion_turn)`, the instantaneous screw axis `isa` and `screw.pitch`, its advance along
`isa` a radian of the roll (give `pitch:` to pin it). Any motion the solve reads, seen from any frame (`spiral_bevel/phillips.sv`); seen from the
world, `seen_from:` names a frame at rest. A `claim L reciprocal(seen_from: g) m` carries its point all the same
and judges the rate.

**Two flanks meshing are a locus** (`spiral_bevel/contact.sv`): tooth contact analysis. The first
flank turns with the pinion and the second with the gear — the reading's observer and source — each an
envelope in space as above or a cone, a cylinder or a revolution's side standing still (a formed
gear's). At each phase of the pinion they touch with the gear turned to the ratio plus a **lag**,
the transmission error:

```
bearing := meshing(pinion_flank, flank over t in (-15deg, 15deg), under: gear_turn,
                   seen_from: pinion_turn)
Q coincident(t == 0deg) bearing                       // where they touch at phase 0
R coincident(lag == 0.07) bearing                     // the phase the gear lags 0.07° at
```

A point on it owns the phase `t` and the lag (both in degrees) and a place on each flank; pin
either. A flank may also be a surface traced in space (1.10) — the contact element as its member
sees it, `X coincident(under: turn, t == -phi) x` in its body (`spiral_bevel/phillips_pure.sv`). A conjugate pair touches along a line with no lag (the point is then free along it); a
mismatched one at a point. After the solve the contact is traced over its interval from the first
point held to it, and `solventc` reports the transmission error and the path of contact.

At a point held to it, `bearing perpendicular(at: Q) l` holds a line along the two flanks'
common normal. The **path of contact** — where the contact moves in space as the pinion turns, not
the bearing on either flank — is Q's rate along its phase, its lag and places following:
`rate(Q over t) parallel l`, or as the tangent it is, `bearing tangent(at: Q) l`. Both read Q's
phase, lag and places, so they hold at any phase Q is at, and an angle stated of either solves a
number:

```
pq := point hint((10, 48, 0))
path_q := line(Q, pq)
rate(Q over t) parallel path_q
Q distance(10mm) pq
path_q angle(2deg) std.x                              // R's phase, a cutter's number, …
```

At a line contact the path has no direction and a line along it does not hold.

A conjugate pair touches along a line; real pairs are mismatched on purpose. **Ease-off**
relieves the first flank along its normal by a parabola about a point — `lengthwise:` its curvature
along a line, `profile:` across it, each per length — so the pair touches at a point and the gear
lags by the relief:

```
lengthwise := line(M, L)                              // the face mill's circle at M
param kl: Scalar hint(0.001)
bearing := meshing(pinion_flank, flank over t in (-15deg, 15deg), under: gear_turn,
                   seen_from: pinion_turn, ease: M, along: lengthwise, profile: 0.002 / 1mm,
                   lengthwise: kl / 1mm)
Q coincident(t == 0deg) bearing                       // at M: no relief there, no lag
R coincident(t == 15deg, lag == 0.047) bearing        // the transmission error solves kl
```

The relief runs along the flank's normal as the solve reads its source (for a cone, toward its
axis); a negative curvature relieves the other way. A traced flank is not eased.

**No undercut.** A generated flank folds where the tool's contact runs back on itself.
`regular` reads the flank's least **area factor**, how much it scales the tool's area: 1 rolling
without sliding, 0 at a fold. It is read over the part the tool cuts within the blank. Name the tool
whose face the source is (`by:`), then the blank after `within`. Both are region solids:

```
pinion_flank := envelope(mill.inner_flank over t in (-60deg, 60deg), under: gear_turn,
                         seen_from: pinion_turn)
pinion_flank regular(by: gear_tooth.crown) within pinion_blank.material   // no fold: >= 0
pinion_flank regular(>= 0.3, by: gear_tooth.crown) within pinion_blank.material
pinion_flank regular(0.3, by: gear_tooth.crown) within pinion_blank.material  // solves a setting
```

With no number, or a `>=`, it is a check the solve reports broken (`bound broken`) and does not
steer by. A number stated outright is an equation, so a cutter's pressure shift left unknown
solves to where the margin binds. The source must be the tool's face itself, not grown by a
backlash, and the blank is read as balls, cylinders, cones and half-spaces.

**The B-rep read after the solve is not Solvent.** Trimmed patches, the seams where faces meet,
their corners, edges and face boundaries add no unknown, no solve row reads them and nothing
exports from them, so the language has no word for them: they are Rust API, made of the surfaces
and envelopes a drawing names (`TrimmedPatch::of_envelope`, `EnvelopeSeam::new`,
`SpatialFaceBoundary::new`, …), as the spiral bevel's checks make them from the faces its built
members leave (`faces::face_source`; [analytic face boundaries](analytic-face-boundaries.md)).

### 1.16 Checking your work

```
make solventc                                         # once
build/solventc drawing.sv                             # parse, elaborate, solve, diagnose; --json
build/solventc drawing.sv --where hinge               # where a name landed
build/solventc drawing.sv --stl part.stl --solid body # a solid, for a printer
build/solventc part.sv --png part.png                 # look at it: the workspace's objects
```

**Look at a solid before you trust it.**  `--png PATH` draws what the workspace shows, with no
browser or GPU: each object shaded in its instance's colour, depth-buffered, its creases and
silhouettes inked.  A part file draws its `preview`, about the world's origin.  `--view` is `home`
(the workspace's opening view), a view a sheet names (`front`, `back`, `right`, `left`, `top`,
`bottom`, `isometric`) or `az,el` in degrees; `--size` the longer side in pixels (1200); `--solid NAME` one solid, or every object of one instance; `--hide
a,b` leaves those out, each a name and everything under it, for a cutaway
(`--hide block.casting,head`).  An open box, a belt off its pulleys, a piston not at its stroke —
a picture shows what no number in the report does.

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
| `unsolved` | the solver stopped short, or no root keeps a bound or a `ccw` | check the model and the `bound broken:`/`orientation broken:` lines, then reseed nearer the intended branch |

A redundancy among pure relations (a fourth `perpendicular` round a rectangle) is a theorem, listed
`implied:`, never an error.

**`--where NAME`** prints the numbers under a name: a point's (`hinge.x`), an instance's
(`--where t0`), a plane's (`--where std.side`), a solid's measurements (`--where body`), or one
number (`--where body.volume`). Under `--json` they are a `positions` table. It is quicker than a
`claim` for testing a position.

Two habits: **fix something** (a figure nothing holds is under by three however determined its
shape), and **say the branch** — with a bound or a `ccw` where a side matters, since the solver
otherwise finds the solution nearest its start, so an arc seeded on the wrong side comes out
mirrored.

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
  length(50) s[0]
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

f := trace(Involute(base, phase: 0).p over u in (0, 90))
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

f := trace(Unwind(base, datum, phase: 0).p over u in (0, 90))
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

rim := trace(c.p over theta in (0, 360))
```

`c.theta` is the one freedom, and `rim` is where `p` goes over a full turn, anchored at the pose on
the sheet. `rim` comes back where it started, so it is **closed**: a contact wraps across its seam.
Another numeric formal left unbound (`len: Length`) is an unknown `c.len` and a column of the curve,
so `rim tangent ground` can solve the length. `jansen.sv` is the leg at full size, given the crank
angle `t` of a declared parameter and traced over it (1.10).

### 2.11 A profile generated by a motion: DOF 0, well

```
use std (spins, slides)

in std.front {
  o := point
  fix((0, 0)) o
  rk0 := point
  rk1 := point hint((30, 10))
  fix((30, 0)) rk0
  fix((30, 10)) rk1
  slide := line(rk0, rk1)
  g := point
  fix((31.5, -3)) g
}
in (rack := frame) {
  in std.front {
    f0 := point hint((30, 5))
    f1 := point hint((39.4, 8.4))
    flank := line(f0, f1)                                 // the rack's flank, riding the rack
    fix((30, 5)) f0
    f0 distance(10) f1
  }
}
blank := frame
blank spins(1) o                                          // the gear blank turns...
rack slides(2 * pi * 30) slide                            // ...as the rack rolls on its pitch circle
cut := envelope(flank over t in (-25deg, 25deg), seen_from: blank)   // rides on the blank
g coincident cut hint(t: 5)                               // the cut must pass through g
```

The flank is drawn in the rack, so the envelope reads the rack's motion seen from the blank with
no `under:` (`under: rack` over a flank drawn at rest says the same). The flank's slope is left
free, and `g coincident cut` decides it: the solve turns the rack's flank
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
  Af distance(60, along: u) Bf
  Af distance(40, along: v) Bf
}
in std.top {
  At := point
  fix((0, 0)) At
  Bt := point
  At distance(30, along: v) Bt
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
gc tangent(at: M) pc

// the two pitch radii at M, and the shafts: square, and E apart
M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
```

`44 params, 44 equations, structural rank 44; DOF 0`. The gear's apex on P makes P the gear cone's
tangent plane at M, and `gc tangent(at: M) pc` makes it the pinion's, so the pinion's apex lands
on P unasked. The pinion's plane stands on two axes the solve turns, held square, with its origin
at M; `horizontal pax` fixes its remaining turn. Γ = 60°, γ = 29.564957707°, ε = 10.722102067°.
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
   `length(L) a`, a ratio left out of a turn is found by what reads the motion. An unknown you do need is declared,
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
| curves stated by their rates | the [rates](../rust/examples/rates/) series: a tractrix drawn and written in place, a clothoid transition, a chain shot to its far end, a pursuit refused at the capture, a geodesic on a cone |
| repeated features and components | [flange](../rust/examples/solid_flange.sv) |
| lofts and sweeps along a path | [elbow](../rust/examples/solid_elbow.sv), [reducer](../rust/examples/solid_loft.sv), [bend](../rust/examples/solid_bend.sv), [square to diamond](../rust/examples/solid_twist.sv), and the [lofts](../rust/examples/lofts/) series: a chain as a path, twist and scale laws, threads and springs, several sections on rails, two faces drawn apart, a section held to its rails |
| a reusable solid with a preview | [cylinder component](../rust/examples/vtwin/components/cylinder.sv), [its drawing](../rust/examples/vtwin/cylinder.svd) |
| views and projection | [bracket](../rust/examples/bracket.sv) |
| layouts in space | [skew axes](../rust/examples/skew_axes.sv), [spatial surfaces](../rust/examples/sphere_cone_cylinder.sv), [hypoid pitch cones](../rust/examples/hypoid_pitch_cones.sv), [sundials](../rust/examples/sundial/horizontal.sv) |
| rounds and chamfers | [rounds](../rust/examples/rounds/) |
| motions and generated solids | [indexed pattern](../rust/examples/solid_indexed_pattern.sv), [lantern generation](../rust/examples/lantern_generation.sv) |
| a point's envelope, an inner envelope and a claim over a motion | [Wankel](../rust/examples/wankel/wankel.sv) |
| a modular assembly | [engine](../rust/examples/engine.sv) |
