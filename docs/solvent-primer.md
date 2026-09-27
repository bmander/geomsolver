# Solvent: a primer for agents

Models are `.sv` files; authored presentation is a separate [Solvent Drawing (`.svd`)](solvent-drawing.md).
`class`, `style`, output requests, and callout placements are no longer model syntax.

Solvent is the language a drawing in this project *is*. You do not place geometry; you declare
entities and state what must be true of them, and the solver finds coordinates satisfying every
statement at once. A sketch is a set of claims, not a sequence of drawing commands, so reordering
its statements cannot change what it means.

One rule carries the discipline: **a number inside a `hint(…)` clause is a seed, and every other
number is not.** A seed is only where the solve starts, and the solver may move it; delete every
seed and the set of valid solutions is unchanged. Every other number is a claim the solver must
satisfy and may never rewrite. Your job on a sketch task is not to compute positions but to say
enough true things that exactly one drawing satisfies them, and then to check that the diagnosis
agrees.

Everything here is what the implementation accepts; every example was run through `solventc` and
the figures quoted are what it reported. `solvent-spec.md` is the normative specification and
describes constructs the parser does **not** yet accept (`hint` as a statement, `path`).

---

## 1. The language

### 1.1 Lexical

```
program     ::= statement*
statement   ::= one of the forms in 1.3, ended by a newline or ';'
comment     ::= '//' ... end of line  |  '/*' ... '*/'
identifier  ::= [A-Za-z_][A-Za-z0-9_]*
number      ::= decimal, exponent form allowed; `3 1/2` is a mixed fraction;
                a unit may follow: 80mm  45deg  0.5rad  6"  1' 6 3/16"   (see 1.6)
```

Whitespace only separates. A newline ends a statement, except inside brackets and except when the
line ends with a chain joint (1.7).

### 1.2 Seeds and constraints

| written | class | meaning |
|---|---|---|
| `hint(x: 0, y: 0)`, `hint(r: 25)`, `hint(t: 0.4)` | seed | where the solve begins; the solver may move it |
| `distance(80)`, `angle(30deg)`, `t == 0.4` | constraint | must hold; never rewritten |
| `param w = 100` | neither | arithmetic done while elaborating; never an unknown |

**The brackets after a name are what the thing is made of; the `hint(…)` after them is where the
solve begins.** So `circle c(center: o) hint(r: 25)`, never `circle c(center: o, r: 25)`: the
centre is structure and the radius is a guess.

Callout selection and placement belong in `.svd`, independently of model constraints.

### 1.3 Statement forms

```
use NAME[.NAME...]                      bring in a module's components and params   (1.12)
unit NAME                               what the document's numbers are in          (1.6)
param NAME = EXPR                       a number worked out while elaborating
KIND [NAME][(CHILD | hint(x: E, y: E), ...)] [hint(SCALAR: E, ...)] [knots [...]]
     [in REF]           an entity declaration; every part is optional (1.4)
point NAME = (XEXPR, YEXPR)             a computed point, drawn only as a curve      (1.9)
plane [NAME](origin: R, toward: R[, from: R, fold: E | , from: R, offset: E
                                      | , u: (E,E,E), v: (E,E,E)])
                                        the datum, and a view with an attitude    (1.13)
in REF { statement* }                   every declaration inside is drawn in that plane
face NAME(EDGE, ..., holes: LOOP, ...)  a planar region with optional holes        (1.14)
solid NAME(FACE, SWEEP...)              that face swept: depth:/from:/to:, about:   (1.14)
solid NAME(SOLID)                       a body, made of a stock
surface NAME(SOLID, edge: EDGE)         an exact analytic patch of a revolution
REF on REF  |  REF through REF          material added to, or taken from, a body    (1.14)
section(SOLID, at: REF) in REF          the same, cut at a plane
WORD[(ARGS)] REF  |  REF WORD[(ARGS)] REF
     [hint(SLOT: E, ...)]
                                        a constraint, prefix or infix               (1.5)
claim CONSTRAINT                        judged, never solved for                    (1.10)
ground REF                              pin both coordinates of a point
fix REF.FIELD                           pin one scalar: fix c.r
ccw(a, b, c) | cw(a, b, c)              record a root choice; adds no equation
[NAME:] Component(ARGS) [in REF]   an instance                      (1.8)
component NAME(FORMALS) { statement* }  a definition
repeat N [as i] { ... }                 N copies, unrelated
cycle N [as i] { ... }                  N copies that close; `next` and `prev` are in scope
repeat e in CHAIN [as i] { ... }        a copy per edge of a named chain, `e` that edge (1.7)
curve NAME = INSTANCE.POINT over FORMAL in (A, B)          a curve                  (1.9)
curve NAME = Component(ARGS).POINT over FORMAL in (A, B)
```

A reference is `name`, `name.field`, or `name[expr]`, the copy of a repeated statement (the
expression may read any `param` or binder in scope). An index may stand on a dotted name and take
a field after it: `l.e[2].p1` is the `p1` of copy 2 of the `e` inside instance `l`; `cyl[0].small`
reaches into copy 0's instance `cyl`.

### 1.4 Entities

| kind | children | own scalars | ends (for chains) |
|---|---|---|---|
| `point` | none | `x`, `y` | |
| `line` | `p1`, `p2` | | `p1 -> p2` |
| `circle` | `center` | `r` | |
| `arc` | `center`, `start`, `end` | `r` | `start -> end`, counter-clockwise |
| `spline` | control points, all named | | |
| `plane` | `origin`, `toward` | `c`, `s` (unit rotor, never seeded by hand), plus a constant basis in space | |
| `curve` | its arguments | | |

**Seeds.** Every scalar is seeded by name in the trailing clause: `point p hint(x: 0, y: 0)`,
`circle c(center: o) hint(r: 25)`, `arc a(center: c, start: s, end: e) hint(r: 5)`. Keys come in
any order. An omitted coordinate is 0. An omitted radius is computed from the geometry, never 0.
A point with no clause at all starts where the implementation puts it, off the origin and apart
from every other unseeded point, and a solve writes the pose it reached back in as the clause.

**Children.** Give them positionally or by label; a label lets you skip an earlier one
(`line l(p2: c)` leaves `p1` for a chain to thread). Any slot may be left implicit, and a slot may
hold a `hint(…)` instead of a name, which mints an anonymous seeded point:

```
line   l                                          two points: l.p1, l.p2
circle c hint(r: 25)                              an unnamed centre, a seeded radius
arc    a                                          a.center, a.start, a.end
line   l(hint(x: 0, y: 0), hint(x: 60, y: 20))    two points, seeded
line   alt_a(A, hint(x: 15, y: 5))                one named end and one not
```

**The dotted path is the name.** `l.p1` is an ordinary point: it constrains, drags, and takes a
dimension. Name a point yourself when several statements mention it. A spline is the exception to
all of this: its control points must be declared points and every one must be named
(`spline s(k0, k1, k2, k3)`), so `spline s` alone is an error.

**The element's own name is optional too.** `line`, `line(p1, p2)`, `point hint(x: 3, y: 4)`,
`arc(center: c)` and `line` are all statements. The token after the kind
keyword decides, so a word that may follow a declaration (an element keyword, a constraint word,
`hint`, `knots`, `class`, `at`, `close`, `in`) cannot be a declaration's name. When the source
must later reference an anonymous element (a constraint applied from the app, say), a name is
spliced into its declaration. `curve` always requires a name.

**A seed may read geometry.** `hint(x: k.center.x + k.r, y: pin.y)` reads another scalar's *seed*,
never a solved value, so the clause is still only a starting point. Two keys name a place
outright: `hint(at: pin)` starts a point where another starts, and `hint(at: k, bearing: 90deg)`
puts it on the circle's rim at that bearing; a clause with `at` carries no `x` or `y`. Inside a
component the names are the formals'. Seeds
settle in statement order, so a seed reading one written below it reads that one's provisional
start. Where the document names a `unit`, a geometry read is a length: write `pin.x - 10mm`, not
`pin.x - 10`. A `param` may **not** read geometry; it feeds constraints, and a seed must never
change what a document says.

A **plane** is the datum: an origin, a point it is turned toward, and a unit rotor slaved to the
chord between them, drawn as a small datum glyph and adding no freedom. One with no attitude
written is a view of the page (1.13). Its use on the sheet is `f.angle`, the bearing in degrees,
which a traced component or a hint may read (1.9). There is no separate `frame`: the word is refused.

Use a datum to state **signed local coordinates**. Its u axis points from `origin` to `toward`,
and v points to the left of u:

```sv
point p
p distance(20mm, along: u) f
p distance(-3mm, along: v) f
```

Each ordinate is an ordinary constraint. Either can be zero, negative, or an unknown solved
by other constraints. The datum can move in response to constraints on `p`, including constraints
outside the component that declares it. Plane membership is independent: `point p in front`
assigns membership, while a relation to `f` measures against that datum. `in` on a component
instance passes membership through its nested components. No component owns an implicit frame.

**Model relationships, not a table of point coordinates.** A rectangle states perpendicular
sides, width, height, and its position relative to a datum. A bolt pattern states a pitch circle
and angular spacing. Use an ordinate when the design calls for a datum measurement, rather
than assigning two ordinates to every corner. Coordinate-placement helpers are not part of the
standard library. Hints may calculate useful starting coordinates; the constraints must still
express the part's geometry independently of those hints.

A component file can end with `preview { … }`: put its sample datums, parameter values and
instance there. Opening the file previews those statements in the ordinary solve. Importing
its components with `use` omits the preview, including its parameters and units. There is at
most one preview per file, at file scope. `vtwin/components/cylinder.sv` is a complete example;
`vtwin/cylinder.svd` takes its three views from that preview. Project imports are looked up
beside the opened model and then in its ancestor directories, so the same `use` paths work
when opening the component directly.

`use std` supplies shared fixed datums without a setup block: `std.front` has u right and
v up at `(0, 0)`; `std.up` has u up and v left at the same origin. They are ordinary library
geometry, created once only when referenced. A short upright preview is:

```sv
preview {
  unit mm
  cyl: Cylinder(std.up, fw: 12mm, dims: vtwin_dims)
}
```

`Cylinder(std.front, fw: 12mm, dims: vtwin_dims)` also works as an unnamed call. Its bore follows the datum's
u axis, so use `std.up` for this cylinder's upright pose. Keep `cyl:` when a drawing or another
statement references its members. The datum argument does not imply `in`: unassigned geometry
stays on the page, or write `in std.front` to assign membership explicitly.



Hints can read `f.c` and `f.s`, the dimensionless cosine and sine of its starting direction,
or `f.angle`. When a numeric formal is left unbound, hints use a provisional zero for it while
constraints retain the unknown. This does not make a hint a constraint or fix the resulting point.

### 1.5 Constraints

**Every constraint is a prefix or an infix operator.** The word stands before its one operand or
between its two, and everything else, the number, a selector, a third entity, goes in parentheses
on the word:

```
horizontal line1                    point1 horizontal point2
radius(25) circle1                  point1 distance(80) point2
distance(6) line1                   point1 symmetry(line1) point2
ground p1                           l1 angle(30) l2
fix c.r                             line1 tangent(side: -1) circle1
length(40) arc1                     l1 angle(l3, l4) l2
```

| word | fixity | operands |
|---|---|---|
| `on` | infix | a point to a line, circle, arc, spline or curve; a point or a line to a **plane**, a point or a circle to a **sphere**, and a point to a **cone** or a **cylinder**, in space (1.13); between two **solids** it is not a constraint at all but the body rule (1.14) |
| `distance` | infix | two points (`along: x` / `along: y` for the run and the rise, signed first to second, or `along: right \| left \| up \| down` to say the direction in a word); a point and a line, or two lines (a magnitude — `side: left \| right` pins which side, and without one the seed picks); two concentric circles or arcs (the radial gap); a point and a datum (`along: u` / `along: v` for signed local ordinates, `along: n` for the signed distance along the plane's normal, in space) |
| `distance` | prefix | a line: the distance between its own ends |
| `tangent` | infix | a line and a circle or arc (`at: p1` / `p2` for a tangency at that end; `side: left \| right` says which side of the line the centre is); two circles or arcs (`external: true/false`); an arc and a line (`at: start` / `end`); a spline or a curve and a line; a sphere and a line or a sphere, a cylinder and a line, in space (the sphere or the cylinder first); two cones at a point, `k1 tangent(M) k2` |
| `equal` | infix | two lines (length), or two circles or arcs (radius) |
| `curvature` | infix | a spline or a curve and a circle or arc: the circle becomes the osculating circle there. Refused on a traced curve |
| `horizontal`, `vertical` | prefix / infix | a line, or a pair of points with no line drawn between them |
| `angle` | infix | two lines; a bare number is degrees, and `sense: cw` turns it the other way. With a second pair of lines in the parentheses instead of a number, `l1 angle(l3, l4) l2`, the angle is **equal to** the angle from `l3` to `l4` (directed; `sense: cw` makes it that angle's mirror image) |
| `angle` | prefix | a cone: its half-angle |
| `radius` | prefix | a circle, an arc, a sphere or a cylinder |
| `length` | prefix | an arc: its length along itself, radius times sweep (a magnitude) |
| `coincident`, `symmetry(line)` | infix | two points |
| `midpoint` | infix | a point and a line |
| `parallel`, `perpendicular` | infix | two lines |
| `project` | infix | two points, each `in` a plane: two images of one point in space (1.13) |
| `ground`, `fix` | prefix | pin both coordinates of a point, or one scalar |
| `ccw(a, b, c)`, `cw(a, b, c)` | a call | all three in the parentheses: the predicate is about the triangle |

One word covers several constraints, told apart by the **kinds of its operands** (`on` is five,
`distance` six, `tangent` six), by **fixity** (`horizontal` on a line versus between two points),
or by **what stands in its parentheses** (`angle` with a number, or with a second pair of lines).

**Across views the same word means space** (1.13). When a relation's operands are drawn in
different views it is the relation between their places in space — no selector says so:

| written across views | means |
|---|---|
| `a coincident b` | the same point in space |
| `a distance(30) b` | the true length between them |
| `a distance(5) l` / `l1 distance(17.5) l2` | to the infinite line / along the common perpendicular — magnitudes |
| `a on l`, `a on c` | on the line's infinite extension, on the circle in its own view |
| `l1 angle(90deg) l2` | the angle between the two directions, **unsigned**, 0–180° |
| `l1 parallel l2`, `perpendicular`, `equal` | directions, and true lengths |
| `a midpoint l`, `a symmetry(l) b` | the midpoint in space; the half turn about the line (the mirror, on its own plane) |

A word with no meaning in space (`horizontal`, `along: x`, `tangent` between drawn figures) is
**E062** across views, as is a datum point read beside another view's points (relate a point
drawn in its view at it instead); `sense:` and `side:` name a turn and a side on a
page and are **E040** there. A radius means the same in every view, and `along: u` / `v` is an
ordinate on a datum as it stands on the sheet, wherever the point is drawn.

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end;
`line tangent circle` is the ordinary one. `a distance(80, along: x) b` is signed from `a` to `b`.

### Which way, in words

Every direction in the language is a **word**, and the sign behind it is stated here once.

| written | means |
|---|---|
| `p distance(12) ax` | `p` is 12 from the line — **either side**; the seed says which |
| `p distance(12, side: left) ax` | to the **left of `ax`'s own direction**, `p1 → p2` |
| `p distance(12, side: right) ax` | to its right |
| `l1 distance(6, side: left) l2` | `l2`'s **`p1`** lies left of `l1` |
| `a distance(60, along: x) b` | `b.x − a.x = 60`; `along: y` is the rise, first point to second |
| `a distance(60, along: right) b` | the same, with the direction said: `left`, `up`, `down` too |
| `l1 angle(30) l2` | 30° **counter-clockwise** from `l1`'s direction to `l2`'s |
| `l1 angle(30, sense: cw) l2` | 30° clockwise — the same as `angle(-30)`, said in the open |
| `l1 angle(l3, l4) l2` | the angle from `l1` to `l2` **equals** the angle from `l3` to `l4`, both counter-clockwise |
| `l1 angle(l3, l4, sense: cw) l2` | it equals that angle turned the other way: the **mirror image** |
| `length(40) a` | 40 along arc `a`, counter-clockwise from its `start` to its `end` |
| `l tangent(side: left) c` | the circle's centre lies left of `l` |
| `ccw(a, b, c)` | `c` is left of the ray `a → b` |
| `p distance(5, along: n) P` | `p` stands 5 along plane `P`'s **normal** (towards its viewer) from it, in space |

**A distance measured from a line is a magnitude**: a negative one is refused, and which side is
`side:`. A component that must work either way up takes a `Side` formal (`s: Side`, called as
`Part(…, s: right)`) and writes `side: s`. Where a side is *arithmetic* rather than a
convention, a signed datum ordinate (`p distance(v, along: v) f`) can state the measurement.

The run, the rise and the angle keep their signs, because there the sign is arithmetic a
component computes (`dy` is a coordinate; `alphaL` is a bank leaning the other way). The words
are how a *drawing* should say it.

**`angle` is directed**: the full-turn angle from `l1`'s direction (`p1` to `p2`) to `l2`'s,
counter-clockwise positive. It pins which side, not just the tilt, so a bearing needs no
orientation predicate. Swapping the lines or reversing one's endpoints changes the reading.

**An angle may be stated as another angle.** `l1 angle(l3, l4) l2` says the angle from `l1` to
`l2` is the angle from `l3` to `l4`, with no number: a bisector is `ab angle(ad, ac) ad`, a
reflection `incoming angle(mirror, outgoing) mirror`. Both angles are read as `angle` reads one —
directed, counter-clockwise from the first line's direction, on the full turn — so the equality
is directed too; `sense: cw` equates the first with the *negative* of the second, which is what
a mirror image is. The spelling is the operator grammar's own: the word between two operands,
everything else in the parentheses, and a kind states at most one number, so two items there
can only be the second pair (two numbers there are refused). It replaces the shared free
variable the same statement used to need (`ab angle(beta) ad` beside `ad angle(beta) ac`, two
W111s and an unknown nobody wanted to name) with one row and nothing to solve for. Across views
it is **E062**: it relates two turns on a page, and `angle`'s reading in space is unsigned.

**An arc's length is `length(L) a`**: the radius times the arc's sweep, counter-clockwise from
its `start` to its `end` (in `(0°, 360°]`), so a chain that should be measured the long way
round goes round counter-clockwise. It is a magnitude (a negative one is refused), may be
written in terms of a free variable like any dimension, and is drawn as an arc concentric with
the one it measures, its number marked `⌒`. Only an arc takes it: a line's length is `distance`,
and a whole circle has no ends to measure between.

**A slot the constraint owns** (a contact's curve parameter) is normally omitted. Seed it with a
trailing `p on s hint(t: 0.4)`; **pin** it with `p on(t == 0.4) s`, a stated number in the
parentheses beside every other stated number. A contact's parameter is `t` on a spline and on a curve alike,
whatever its swept formal is called.

**Tangency trap.** If the contact point is already held on the circle, state the tangency *at* that
point: `line tangent(at: p2) circle`, `arc tangent(at: start) line`. Pairing `p on circle` with a
bare `line tangent circle` is rank-deficient at every solution. The diagnosis reports it as a
motion "blocked at second order" rather than a DOF, but the regular form is the one to write.

### 1.6 Numbers, names and units

A dimension may be an expression: `+ - * / ^`, parentheses, `pi`, and `sqrt`, `abs`, `sin`,
`cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `exp`, `ln`, `log`, `floor`, `ceil`, `round`,
`min`, `max`, `hypot`. **Trigonometry is in degrees.**

Four more calls **measure the solved drawing**: `length(l)` (a line's, or an arc's, in space),
`radius(c)` (a circle, arc, sphere or cylinder), `distance(a, b)` (two points, or a point and a
line produced) and `angle(l1, l2)` (between two lines' directions, 0° to 180°). Their arguments are
names of geometry, never numbers. A length reads as a `Length` in a document with a `unit` line and
an angle as an `Angle`, so `length(a) / length(b)` is a plain ratio. They stand only where a number
is read *after* the solve, which today is a motion's `ratio:`, `phase:` and `advance:` (1.14);
everywhere else the number is needed before there is a solve to measure, and a measurement there
is **E107**: a `param` (it feeds constraints), a seed (it is where the solve starts), a
constraint's own number (it is what the solve solves for) and a solid's extent or placement angle
(settled at elaboration).

A dimension may be **named** and read elsewhere:

```
// a fragment: two statements of a larger document
a distance(w = 60) b            // states 60 and names it w
c distance(w / 2) d             // reads it
```

A named dimension **declares its name in the body it is written in**, exactly as `param w = 60`
would: a `param`, a seed or a count may read its number (`param h = w / 2`), and a second `w` in
the same body — a param or a dimension, either way — is declared twice (E001). The two differ
only in where the number is edited: a `param` in the source, a named dimension on the drawing.

A name nothing defines is a **free variable**: one unknown of the sketch, tying together every
dimension that reads it (the CLI reports it as W111). Where all it ties is two angles, say so
instead: `l1 angle(l3, l4) l2` (1.5) states the equality with no unknown at all. The tie must be affine in one free name
(`a`, `a / 2`, `2 * a + 5`); `a * a`, `sin(a)` and two free names in one dimension are errors.
Inside a component the unknown is the **instance's own** — `t1.w`, `t2.w` — the same rule as a
formal left unbound (1.8), so a component cannot reach into the document that draws it by
writing a name the document happens to define.

**Every number has a dimension, and it is checked.** There are two, length and angle. `*` and `/`
derive them, `+` and `-` demand agreement, and what an expression comes to is checked against its
slot: `a distance(45deg) b` is an error, and so is a length added to an angle. A bare number is
dimensionless and a *context* may take one: `distance(80)` is a length because the slot says so,
and `sin(30)` reads degrees because the function does. A context does not speak for a second
operand: `90 / N + ivp` is a plain number added to an angle and is refused; write `90deg / N + ivp`.

A number may say what it is:

```
unit mm                             // what this document's numbers are in
param phi = 20deg
param ivp = tan(phi) * 1rad - phi   // inv(phi) = tan(phi) - phi holds only in radians, and says so

a distance(80mm) b
c distance(1' 6 3/16") d            // one literal: the space tells the readings apart, as in 3 1/2
l angle(45deg) m
```

**Without a `unit` line the document is in drawing units**, a length with no name; a suffix like
`mm` or `"` is then refused, since there is nothing to convert to. A name is worth a number, and
where it is *used* decides what it is: `w = 80` in a length slot does not make `w` a length, but
`w = 80mm` does, as does a component formal declared `Length`.

A bare fraction with a unit is a division, not a fraction: `3/16"` is 3 divided by 16 inches (a
`Length^-1`), so a lone fractional inch is written `0.1875"` or as a mixed literal (`0 3/16"`).

`pi` is dimensionless; `tau` and `turn` are one full turn. `floor`/`ceil`/`round` take a plain
number. **There is no string literal**: `"` is the inch mark, and a word argument is written bare
(`at: start`).

**A built-in name cannot be declared over.** Every expression knows the constants and the
functions before it knows the document, so a `param`, a component formal or a block's index named
`tau`, `pi`, `min`, … does *not* shadow the built-in: substituting a text reads the declaration
and working a number out reads the built-in, which is one name with two values (`param tau =
35deg` handed to a `tau: Angle` formal arrives as a full turn). Naming a *dimension* that way is
refused outright; the other three are **W112** at the declaration, and the fix is the rename.

### 1.7 Chains and repetition

A chain writes a run of elements and the relations between them on one line.

```
CHAIN  ::= LINK (JOINT LINK)* ['->' INFIX* 'close']
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a one-operand constraint word         horizontal, vertical, radius(..)
JOINT  ::= '->' INFIX* ['->']  |  INFIX+ ['->']  at least one marker or word
INFIX  ::= tangent | equal | any two-operand constraint word
```

**Threading is stated at the joint, never inferred.** `->` says the two links beside it share a
boundary point, threaded left to right (`p1 -> p2` on a line, `start -> end` on an arc, CCW). Its
absence says they do not. So:

- `->` alone is a plain corner; `-> tangent` is a corner that is also tangent there, which
  desugars to the regular at-the-point form.
- The shared point may be named by one side (or both, agreeing), or by neither, in which case the
  chain mints it: `line l1 -> line l2` is two lines and three points, one shared.
- A joint may state several relations: `A -> equal angle(30deg) B`. The marker may stand on
  either side of the words or both.
- A word with no marker states only the relation: `a_br equal a_tr` welds nothing, and
  `line l1(a, b) perpendicular line l2(c, d)` is two separate lines at a right angle.
- Declarations and names may mix. At a corner with an element declared elsewhere, the declared
  side names the shared point, usually by the other element's own child:
  `line t(p3, k.start) -> tangent k`.
- `-> close` seals a loop back to the first link. Links may be anonymous:
  `line -> tangent arc -> tangent line` is a full contour with no names at all.
- `equal` is polymorphic (a length between lines, a radius between round things).
  `a equal b equal c` is two statements.

**Repetition.** `repeat N { … }` makes N unrelated copies; `cycle N { … }` makes N copies that
close, with `next` and `prev` in scope. `as i` binds the copy index for expressions. The spec's
`ring` (§12.3) is not yet a construct of this implementation: the word is refused with a note
saying `cycle`, whose copies are congruent by the numbers each is given.

**A body may end mid-joint.** A trailing joint at the body's `}` threads the chain onto the next
copy's first link. `cycle` wraps, so the loop closes with no `close`; a `repeat`'s last
copy leaves it unstated, so `repeat N { line -> angle(a) }` is an open polyline of N sides. Both
boundary links must be the body's own declarations, and at most one of the two boundary slots may
name its point. A statement inside braces ends at the `}`, so a block fits one line:
`cycle 4 { line s -> perpendicular equal }` is a square but for a size and a pose.

**Repetition over a chain's edges.** `repeat e in CHAIN { … }` makes one copy of the body per
link of a named chain (1.14), in the order the chain walks them, with `e` naming that copy's
edge; `as i` binds the index as before. `e` is an ordinary reference: a constraint operand, a
field (`e.p1`), an argument to an instance, the edge of a `surface`. The copies are a `repeat`'s
in every other respect — named `#<id>.<k>.m`, reached from outside as `m[2]`, and one statement
however many copies it makes, so a seed inside is not written back and a gesture on one copy is
refused. The chain may be open or closed, declared anywhere in the body (P2), or reached through
an instance or a group formal (`refs.pinion.profile`). A hole at every edge's midpoint, sized by
its index (`holes.sv`):

```
unit mm
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
point d hint(x: 0, y: 40)
ground a
outline = line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
horizontal ab
vertical bc
horizontal cd
vertical da
distance(60) ab
distance(40) bc

repeat e in outline as i {
  point m
  m midpoint e
  circle hole(center: m) hint(r: 3)
  radius(2mm + i * 1mm) hole
}
```

`solventc --where '#22.2.hole' holes.sv` (the block is statement 22; copy 2 is the third edge,
`cd`, so its hole has radius 4 at the middle of the top edge):

```
holes.sv: solved
  18 params, 18 equations, structural rank 18; DOF 0; 5 components: DOF 0, 0, 0, 0, 0; 2 rigid cluster(s) in the distance graph
  #22.2.hole.center.x = 30
  #22.2.hole.center.y = 40
  #22.2.hole.r = 4
```

`cycle e in CHAIN { … }` closes as `cycle N` does — `next.m` is the following edge's copy and
the last copy's `next` is the first's — and is refused on an open chain (E103, at the reference).
Iterating over something that is not a named chain is E103 at the reference (`` `ab` is not a
named chain``), a name nothing declares E101, a private chain reached from outside E101, and a
body declaration called what the copies call their edge E001. `edge_tabs.sv` puts a tab on every
edge of a plate; `spiral_bevel/verification.sv` makes each rack section's generated flanks —
surface, envelope and material region — one block per section.

### 1.8 Components

```
component Rung(a: point, b: point, len: Length) {
  line e(a, b)
  horizontal e
  a distance(len) b
  point mid                // reached from outside as t0.mid, like everything the body makes
}
t0: Rung(l0, r0, len: 50)
```

A formal is a `group`, an entity kind (`point`, `line`, `circle`, `arc`, `plane`, …), or a number
type (`Length`, `Angle`, `Int`, `Scalar`). Passing an entity is **aliasing**, not a constraint: the
formal and the actual are one entity, so a component boundary costs nothing. **The entities are
given by position, in order, and every number is given by label** — `Rung(l0, r0, len: 50)`, never
`Rung(l0, r0, 50)`, and nothing positional after the first label. Position is a count, and a count
is the one thing a reader of a long formal list cannot check: an argument written one place off
binds to the formal beside the one it was meant for, and what comes back is a complaint about
something else. Either mistake is **E004**, at the argument. A numeric formal left unbound is a
free unknown of the drawing, named under the instance (`c.theta`), which is how a mechanism is
drawn with its crank free (2.9.1).

A component reads its arguments and its own declarations. Root and module parameters,
geometry, and standard datums must be passed in. Visible component definitions and built-in
functions remain available. Repetition blocks share the enclosing component's local scope.

`use std` provides an axis-aligned rectangle centered on an existing point:

```solvent
outer: CenteredRectangle(center, w: 18mm, h: 18mm)
inner: CenteredRectangle(center, w: 14mm, h: 14mm)
face section(outer.loop, holes: inner.loop)
```

Its public `loop` is the rectangular boundary, and `a`, `b`, `c`, `d` are its corners.
The diagonal that constrains its center is private construction geometry. Width and height
are constrained dimensions; the coordinate hints only select the initial arrangement.

Members are public by default. Prefix a helper declaration or instance with `private` to keep
its name within the enclosing component. `construction` and `centerline` describe geometry's
purpose independently of privacy. A supporting axis can carry both roles. The editor shows
construction geometry; authored drawing sheets hide it unless a style reveals it.

A reusable pattern can hide its polygon layout while exposing its holes:

```solvent
component BoltPattern(body: solid, center: point, ref: line,
                      n: Int, pitch_r: Length, hole_r: Length, phase: Angle) {
  private construction layout: Polygon(center, ref, n: n, r: pitch_r, phase: phase)
  repeat n as i {
    radius(hole_r) circle hole(center: layout.v[i])
    private solid drill(face(hole), through: body)
    drill cut body
  }
}
```

`use hardware` provides this component (`n >= 3`); see `rust/examples/solid_flange.sv`.
Callers can reference `pattern.hole[0]`, but cannot name `pattern.layout` or `pattern.drill[0]`.
The polygon's geometry is construction geometry; the holes and the caller's center and reference
line retain their own roles. A component can pass a private entity explicitly to another component.
Passing an entire layout does not grant access to private members inside it.

Use a **group** to pass related dimensions or layout geometry together:

```solvent
unit mm
use std

group sizes(length: 20mm)
group layout(frame: std.front, origin: std.origin)
component Bar(layout: group, dims: group) {
  point tip hint(x: layout.origin.x + dims.length * layout.frame.c,
                 y: layout.origin.y + dims.length * layout.frame.s)
  line reference(layout.frame.origin, layout.frame.toward)
  line axis(layout.origin, tip)
  axis parallel reference
  distance(dims.length) axis
}
bar: Bar(layout, dims: sizes)
```

Groups may contain numbers, geometry references, and other groups. Numeric members keep
their units; geometry members alias existing geometry and add no constraints or solver state.
A `group` argument may also be a component instance: passing a layout sketch lets the body
refer to `layout.pivot` or `layout.bank[0].axis` in the same shared solve. Its local parameters
are not exported; bundle dimensions in an explicit group. Groups must be supplied, and a
missing member is an error. A traced component needs fixed scalar or entity formals; pass
individual members when constructing a curve.

Everything an instance makes is
reachable by dotted name (`c.p`, `t0.e`, `five.s[0].p1`, and a dimension named inside it as
`t0.w`) — there is no export list, and passing
one instance's entity to another as an argument makes the two one entity. (`port` is retired;
a document that writes one is told what to write instead.)

The V-twin example distinguishes shared design inputs from private derived dimensions.
Its bore, throw, rod length, piston height, and wall thickness determine the cylinder mouth,
width, and top. The head position stays shared because it also locates the ports. Hardware
choices determine mating pockets and holes with explicit clearances; the bearing boss follows
the bearing stack. Piston and throttle derive their grooves locally from the selected ring and
`dims.seal`. Changing a source dimension therefore carries its dependent features with it.

### 1.9 Curves

A curve is **a point of a component, as one of the component's numeric formals runs over an
interval**. There is no curve family: an involute, a cycloid and a walking leg's stride are three
components.

```
curve NAME = INSTANCE.POINT over FORMAL in (A, B)          a drawn instance's point
curve NAME = Component(ARGS).POINT over FORMAL in (A, B)    an instance written in place
```

**Over a drawn instance.** `leg: Leg(axle, pivot)` is drawn once, and
`curve path = leg.toe over theta in (0, 360)` is where its toe goes as `theta` runs. The trace is
anchored at the drawing's own pose, so the component needs no seeds for the curve's sake. Leave
`theta` unbound and the crank is the drawing's freedom; the curve follows wherever it stands
(`jansen.sv`, `peaucellier.sv`).

**Over an instance written in place.** `Involute(base, phase: a0).p` binds an instance that is
never drawn; the curve is the only thing made of it. Its anchor is the value it gives the swept
formal, or the interval's start when it gives none.

**The ellipse is one of these.** `use std` brings in `Ellipse(f: plane, a: Length, b: Length,
u: Angle)`, a computed point at eccentric angle `u` on the datum `f`, and
`curve e = Ellipse(f, a: 40, b: 25).p over u in (0, 360)` is the rim: `p on e`, `e tangent l`
and `e curvature k` are the curve's contacts, exact to third order. There is no `ellipse`
element; the word is refused with this spelling.

A component's point is placed one of two ways:

- **Computed**: `point p = (XEXPR, YEXPR)`, coordinates as expressions over the formals and
  params. A component with a computed point can only be traced, never drawn (2.8).
- **Placed by constraints**: any point the body declares, held where the body's statements put it
  as the formal runs. This is how a person actually states a curve: "the end of a taut string as
  it unwinds" is the body (2.9). A traced body must be square, as many equations as it has
  coordinates of its own.

The swept formal may be an `Angle` or a `Length`; the point must be one the component places, not
geometry it is written over. A formal `f: plane` offers `f.angle`, the datum's bearing in degrees:
seeds written `hint(at: c, bearing: u + f.angle)` follow a tilted datum where page-fixed ones go
stale. A plane
is also usually the shortest formal list, since it carries an origin, a second point and a bearing;
fewer entity formals mean cheaper curve evaluations.

A locus generally has several solutions. A body picks one by three means, strongest first: a
**signed** constraint (point-to-line distance is signed, so its sign chooses a winding); an
**orientation predicate** `ccw`/`cw`, which adds no equation, is read at the anchor and carried
along the curve by continuity; and a **seed**, for what neither can say.

### 1.10 Claims

`claim` before a relation states it as expected to add no rank: an assertion about the drawing,
not part of what determines it.

```
claim vertical rail          // checked, never enforced
```

A claim joins no solve, no count and no conflict set; the drawing is exactly what it would be with
the claim deleted. The diagnosis judges it **theorem** (holds, and the document implies it),
**violated** (does not hold; the CLI prints `claim refuted:`), or **consuming** (holds only by
where the solve landed; enforcing it would have cost a freedom). Use it for the fact a figure was
drawn to illustrate: the altitudes concur, the traced path is straight. A claim may not own an
unknown, so claiming a curve contact or binding a free variable in a claim is an error.

### 1.11 Presentation

Use a [Solvent Drawing](solvent-drawing.md) for styles, views, paper layout, and annotations.
A model file contains no presentation clauses. A geometric dimension remains a constraint;
its callout belongs to the drawing.

### 1.12 Modules

```
use engine.dims          // engine/dims.sv beside the document, else the library compiled in
use std                  // the standard library: ThreeViews (1.13), Ellipse (1.9), Polygon and Hex
use hardware             // fasteners and fittings by name: hexbolt14_af, brg608_od, oring014_cs, …
```

A module is a Solvent document read for its `component`s and its top-level `param`s and groups; its own
drawing is not drawn, so `gear.sv` is a module as it stands. A module's own `use`s are followed,
once each. No such module is E070, a component defined twice is E071, and a module's own error is
reported at the `use` that brought it in. `rust/examples/engine.sv` is the worked case: a
four-cylinder engine in three views, written as a dimension module, a parts module and one module
per part.

### 1.13 Planes and views

A `plane` is the datum, and a view: it carries a constant attitude in space, the page's where none
is written. A point says
which view it is drawn `in`, and `a project b` says two points are two images of one corner: their
coordinates along the fold line the two views share agree, which is one equation. A view's
attitude is stated unless its brackets say otherwise (below).

`fold` is the bearing of the fold line in the parent view. From the page, `0deg` folds up the top
view and `-90deg` the right view; the new view's second axis points away from the parent's viewer,
so distance from the fold line is depth (third-angle projection). Any plane can be reached in two
folds or given outright as `u: (…), v: (…)`. `from: P, offset: 12mm` with no `fold:` is a plane
*moved* rather than turned, twelve along its parent's own normal: two parallel views share no fold
line, so that is a stack rather than a projection, and it is where a section is cut (1.14).

`in top { … }` writes the membership once for every declaration in the block, a `cycle`'s copies
included; the statements are otherwise ordinary. An instance joins a view whole: `t: Tooth(…) in
top`. Inside a component body, `in view { … }` blocks over plane formals let a part carry its own
views, so the whole design of a connecting rod is one module (`engine/conrod.sv`); `repeat flag
{ … }` over a 0-or-1 `Int` formal leaves a view undrawn for an instance that does not show in it.

The standard library lays out the three views once (2.10).

**A view may be solved for** (spec §6.7 [0.23]). A plane is fixed unless its brackets name an
unknown, and `hint(…)` seeds only those:

```
plane side(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 30deg)
plane aux(origin: o3, toward: t3, from: front, fold: along l)
plane cut(origin: o4, toward: t4, from: front, fold: 0deg, through: m)
plane q(origin: o5, toward: t5, attitude: free, offset: free) hint(u: (0, 1, 0), v: (0, 0, 1))
```

`fold: beta` over a name nothing defines is a free variable (W111) the solve answers; `fold:
along l` folds square to the parent about a line drawn in it and follows the line; `attitude:
free` is three freedoms and `offset: free` one; `through: m` stands the plane where a point of
another view is. A view folded from a solved one follows it. A point drawn in a view is that
view's lift of it, and `project` over a solved view is the projector rule in space. A seed for a
stated quantity is E040; a position stated twice or a fold along a line of another view is E064;
views that come out parallel under a `project` are E065. `against` works with solved views where
the two faces' planes turn together — one derived `from:` the other, or both from one, with no
fold: the placed plane turns with its datum and stands off it by the faces' gap, and where the
datum's offset is itself solved (`offset: free`, `through:`) the gap is a row of the solve. Two
views that turn apart are E066.
`solventc --where side` reports a solved view's `side.u.x` … `side.o.z`, and a report names what
is left free as `side.attitude` and `side.offset`.

**The workplane rule, in one place.** A point drawn `in` a view is that view's lift of it into
space; a point with no membership is on the page, which has no place in space. A relation reads
its operands' views: within one view it is the 2D relation, which the rigid lift makes the same
statement; across views it is the relation in space (1.5), and naming a page point there is E062.
`project` is the exception — it ties two drawn images of an undrawn point. `p on P`, `l on P` and
`p distance(d, along: n) P` put a point or a line on a plane in space whatever view it is drawn
in (a view's own points are on it already, E061).

**The role rule.** A plane's own origin and toward place the view on the sheet, and their
membership says nothing about what they mean. So a datum point is read by what it is related to:
beside only other datum points it is sheet **layout**, on the page (`o distance(120) o2`); beside
points of the view it is the datum of, it is that view's (`o2 distance(15) b` with `b` in that
view: an ordinate); otherwise, where its membership puts it. Every cross-membership relation the
corpus had before reads as it did (`tests/cross_view_audit.rs`).

**A solved view's place on the sheet is held silently.** Where a solved view's picture sits on the
page (its datum's origin and toward) is presentation: those points are held where they were drawn
and the ledger does not count them, unless a statement names them. A free view with nothing else
in it reports DOF 4 — its attitude and its offset — with no `ground` written.

**A sphere** is `sphere s(center: p) hint(r: 12)`: a centre drawn in some view (`in side`) and a
radius, on no sheet (the glass box draws it as three great circles). `radius(12) s`, `a on s`,
`s tangent l` and `s tangent s2` are all in space. `k on s` puts a whole circle drawn in a view
on the sphere — its centre on the circle's axis, and the radii and the gap a right triangle —
which is what a gear blank's toe or heel circle is to its end sphere. `s tangent k` is refused: a
circle and a sphere may touch at a point or all the way round.

**A cone and a cylinder** are built about a line already drawn in a view, and each owns one
number, as a sphere owns its radius:

```
cone gc(axis: gax) hint(half: 60deg)
cylinder bore(axis: ax) hint(r: 8)
```

A cone's apex is its axis's start and it opens toward the end; its half-angle is written in
degrees and held in radians. `angle(60deg) gc` and `radius(8) bore` state the numbers; `p on gc`
puts a point on the nappe the axis points into (its distance from the generator in its meridian
half-plane, `ρ cos α − h sin α`) and `p on bore` a point its radius off the axis; `bore tangent l`
makes a line touch the cylinder (the common perpendicular with the axis is the radius, the side
read off the seed). `gc tangent(M) pc` says two cones touch at M with one tangent plane there —
the second's normal square to both of the first's tangent directions; `M on gc` and `M on pc` say
M is on them, as an on-circle stands beside a tangency at a named end. A line on either (a
generator) is not a relation yet and says so: state it of the line's points. Neither is on a
sheet; the glass box draws each as two circles square to the axis and four rulings.

**`o:` says where a basis given outright stands.** `u: (…), v: (…), o: (0, -12, 0)` is the basis
at that origin in space. A lifted program writes it for a stated plane that stands off the shared
origin — a stand-off, a mate, a view folded from one — which `u:` and `v:` alone would lose.

### 1.14 Faces, solids and derived views

A feature tree is imperative because it is a *history*: step *n* acts on the anonymous body as of
step *n − 1*, and names faces by the order they were cut in. Solvent names everything, so **a solid
is a term, never a step** — a face swept, or a stock plus everything `on` it minus everything
that `cut`s it within everything that `bound`s it — and the implementation finds the order the way
it finds the order of `h = w / 2`.
**The order lives inside a term and never between statements**, so `bore cut body` may be
written above the `solid body(…)` it belongs to or fifty lines below it and says the same thing.

**Nothing about a solid is solved for.** A solid owns no parameter. Numeric extents are
expressions the elaborator works out; a
`through:` extent follows the target after solving. The geometry swept is the drawing, solved
in 2D as it always was.

**A face** is a planar region bounded by edges the drawing already has — on the plane its edges
are drawn `in`, the page where nothing says otherwise.

A section may also have holes. Write its outer boundary first, then `holes:` with
circles or named closed loops:

```solvent
face groove_section(barrel, holes: core)
solid groove(groove_section, from: z, to: z + width)
groove cut body
```

This sweeps the ring between `barrel` and `core`; `cut` then removes that ring from
the body. Several holes use `holes: first, second`. They must lie strictly inside
the outer boundary on the same plane, without touching or overlapping each other.
The same section works with an extrusion or a revolution.

```
unit mm
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
point d hint(x: 0, y: 40)

horizontal line ab(a, b) ->
vertical   line bc(b, c) ->
horizontal line cd(c, d) ->
vertical   line da(d, a) -> close

a distance(60) b
b distance(40) c
ground a

face sec(ab, bc, cd, da)
solid block(sec, depth: 30mm)
```

```
  6 params, 6 equations, structural rank 6; DOF 0; 2 rigid cluster(s) in the distance graph
  block.volume = 72000
```

Six unknowns and six equations: the count is the rectangle's own, and the face and the solid added
nothing to it. The edges are given in traversal order and each must share a point with the next; a
**circle is a whole loop by itself**, so `face hole_f(hole)` is a face and a circle among lines is
refused. A face has no inner loops, because a hole is not a hole in a face — it is a solid
that `cut`s the body.

**Name the chain where you draw it.** A contour can bind its whole traversal, so no second
statement needs to list its edges again:

```solvent
profile = line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
solid block(profile, depth: 30mm)
```

With the rectangle's points and dimensions above, this gives the same volume, 72000, and adds
no unknowns or equations. In a component the chain is a normal member:
`solid iboss(boss.profile, depth: 8mm)`. The constituent edge is still `boss.ab`. The two `Box`
helpers in the engine and V-twin libraries expose `profile` this way.

`trail = line ab(a, b) -> line bc(b, c) -> line cd(c, d)` names an **open chain**. A sweep
requires a closed loop, so use `face(trail, -> close)` to supply the missing closing edge, or
finish the original chain with `-> line da(d, a) -> close`. Every joint in a named chain must
carry `->`; its prefix and joint constraints work just as in an unnamed chain. Anonymous links
also work: `profile = line -> line -> line -> close` names a triangle, whose dimensions and pose
remain to be constrained. Named chains can stand in plane blocks and repeated components.

**A face closes itself.** The brackets hold a *walk*, and a **point** in the list is a corner the
walk goes straight to and straight on from; `-> close` — the chain's own word — seals the run back
to the first item. So the rectangle above needs no `ab` and no `da`:

```
face brief(a, bc, cd, -> close)
```

which mints exactly the two straight runs the source would otherwise have declared, in the two
places the loop had a gap. A minted run is not on the sheet: it carries the class `.closure`, whose
shipped rule is `display: none`, so a drawing may override `.closure` to see one. It still
names a face of whatever is swept from the loop — `brief` swept gives `block.close0` and
`block.close1` beside `block.bc`. The numbering skips names already used by the loop’s existing
edges.

Three things it will not do. The gap between the **last** item and the first is minted only where
`-> close` says so, so "the loop closes" stays something the source states. And an interior gap
between two *edges* is still refused: a point in a list can mean nothing else, but two edges that do not meet
are edges listed out of order. For the same reason an edge standing between two gaps is refused —
`face bad(a, bc, d, -> close)` could be walked `b`-first or `c`-first, and nothing there says
which — so an edge takes its direction from a neighbour it actually meets.

**A solid** is that face swept along its normal, along a guide, or about an axis.

**A face may be written where it is used.** A section needed by one sweep can go directly in its
brackets, without a separate name:

```
solid block(face(ab, bc, cd, da), depth: 30mm)
// Or close the same section through its corners:
solid brief(face(a, bc, cd, -> close), from: -30mm, to: 0mm)
```

Both have volume 72000, like the named section above. `face(…)` also works with `about:` for a
revolution, and a circle stands alone in its loop: `solid bore(face(hole), depth: 30mm)`.
Boundary names resolve in the surrounding component, and the section gets its plane from those
boundaries. Keep a named face when several sweeps reuse it, as the throttle's `barrel_f` and
`core_f` do in `vtwin/components/throttle.sv`.

A **prism** runs along the face's own normal. `depth: 30mm` is the draughtsman's reading — the
material *behind* the face the view shows, which is `from: -30mm, to: 0mm` — and `from:`/`to:` are
written out when the face is somewhere other than an end, as a boss standing off a floor is below.

A **guided sweep or loft** follows solved geometry:

```
solid duct(section, along: guide)
solid reducer(inlet.profile, outlet.profile, along: guide)
```

The guide is a directed line or circular arc. The start section lies perpendicular to the
start tangent; an explicit end section lies perpendicular to the end tangent at the other end.
Omitting the end section repeats the start section. Along a line the section translates; along
an arc it turns with the tangent. A loft interpolates between corresponding section boundaries
in that moving frame. Holes pair in written order, and each pair of loops needs the same number
of source edges. Side names come from the inlet; the caps are `start` and `end`.

The `solid_elbow` example uses a constrained arc and one hollow `CenteredRectangle` section.
`solid_loft` joins two hollow square components with a dimensioned line. Their lengths and
shapes follow the guide and section relationships after solving.

A **revolution** turns the face about a line **in the face's own plane**: `about: ax` is a full
turn, `sweep: 90deg` is a quarter of one, and `sense: cw` turns it the other way.

An **analytic surface** names one of that revolution's profile boundaries:

```sv
surface flank(crown, edge: rack.outer)
surface root_transition(crown, edge: rack.outer_round)
```

The edge must be a line, circular arc or circle belonging to the unmodified revolution's
profile. The surface follows solved dimensions and retains exact positions and tangents;
it adds no unknowns and has no 2D drawing glyph. It can be public or private and passed through
a `surface` component formal. `solid` and `edge` are its dependency fields. Evaluation uses
normalized edge and revolution parameters, both from 0 to 1. Tangent orientation follows
the declared traversal; it does not decide which side of the surface contains material.

Write `from:` and `to:` angles to retain part of the generating revolution:

```sv
surface flank(crown, edge: rack.outer, from: 180deg, to: 360deg)
```

Angles follow the source's sweep direction. Both bounds must lie within that sweep. They
restrict the original parameter range, so this half of a full revolution has `v` from 0.5
to 1. Its parameters and tangent directions stay unchanged. Envelope evaluators inherit
the declared span; a caller reads the domain instead of supplying its own branch limits.

A **motion** describes how generating geometry moves relative to another rotating member:

```sv
motion crown(about: crown_axis)
motion blank(about: blank_axis, ratio: 2, phase: 10deg)
motion generating(crown, relative_to: blank)
```

All three use one shared angle. The first turns about the directed `crown_axis` line; the
second turns about `blank_axis` through twice that angle plus 10 degrees. `generating` is
the crown viewed in the rotating blank frame. Axes come from solved world geometry, and
these declarations add no unknown coordinates. Defaults are ratio 1 and phase 0deg.

Two more kinds share the same angle. A rotation with `advance:` is a **screw**: it also
slides along its axis by that length every full turn, which is a tap or a helical mill.
A **translation** slides `along:` a directed line by `advance:` per turn without turning,
which is a plunge or a feed; a plunge of 20mm is then one turn of the parameter.

```sv
motion tap(about: hole_axis, advance: 1.5mm)
motion plunge(along: spindle, advance: 20mm)
```

A motion can be private or passed through a `motion` component formal. Core and browser
evaluation use radians and return exact position and velocity per radian. A motion family
does not move the sketch itself.

**A motion's numbers may measure the solved drawing** (1.6). A motion is read after the solve
(by a placement, a sweep, an envelope, a mesh), so its `ratio:`, `phase:` and `advance:` may be
written over what the solve decided, and a generating ratio is then read off the pitch geometry
instead of typed beside it:

```sv
unit mm
point o hint(x: 0, y: 0)
point z hint(x: 0, y: 1)
ground o
ground z
line axis(o, z)
point c hint(x: 0, y: -2)
point d hint(x: 25, y: -2)
ground c
horizontal line wheel_radius(c, d)
c distance(30mm) d
point e hint(x: 0, y: -4)
point f hint(x: 12, y: -4)
ground e
horizontal line pinion_radius(e, f)
e distance(10mm) f
motion wheel(about: axis)
motion pinion(about: axis, ratio: -length(wheel_radius) / length(pinion_radius))
```

```
$ solventc measured.sv
measured.sv: solved
  4 params, 4 equations, structural rank 4; DOF 0; 2 components: DOF 0, 0; 2 rigid cluster(s) in the distance graph
```

The measurement adds no unknown and no equation: the ledger is the drawing's alone. `pinion`
turns at −3 (30 as solved over 10, not 25 over 12 as seeded), and it is worked out afresh each
time the motion is read, so editing `30mm` re-times it with nothing else touched; a mesh cached on
the motion (`solid::reads`) reads the number and is rebuilt. Inside a component the measured names
resolve like any reference (`one.wheel_radius`), and a motion goes with a line it measures when
that line is deleted. The slot's dimension is checked when the motion is built, and reading the
same measurement where the solve has not happened is refused:

```
$ solventc wrong.sv            # motion bad(about: axis, ratio: length(wheel_radius))
wrong.sv:19:1: error[E080]: `ratio` is Scalar, and this is Length
$ solventc refused.sv          # param k = length(wheel_radius)
refused.sv:19:11: error[E107]: `k`: `length(wheel_radius)` measures the solved drawing, and only a motion's `ratio:`, `phase:` and `advance:` are read after the solve; a param, a seed, a constraint's number and a solid's extent are needed before it
```

`rust/examples/lantern_generation.sv` is the worked case: a pinion rolls against a wheel blank at
`-length(wheel_radius) / length(pinion_radius)`, and the one pin it carries cuts a tooth space
(`solventc lantern_generation.sv --stl out.stl --stl-backend mesh` meshes it in a couple of
seconds). Edit either pitch radius and the roll, and so the cut, follows.

Name its generated envelope with:

```sv
envelope flank(crown_flank, under: generating, from: -35deg, to: 35deg)
```

This describes the source surface's zero-normal-velocity locus over the stated roll interval.
It adds no sketch unknowns. An evaluator can intersect that locus with two section equations;
the core checks the envelope equation as well as those equations. An arbitrary transformed
source point is not necessarily on the envelope. Private construction envelopes and public
`envelope` formals follow the same component rules as surfaces and motions. Declaring the
envelope does not choose a branch, designate material, or close a solid. The core can intersect
an envelope with two named analytic boundary patches and checks that the result belongs to
their finite domains. The spiral-bevel example uses ordinary components for its spherical
toe/heel and conical tip/root/back boundaries.

A spatial patch can state which material region retains a source surface or envelope:

```sv
patch bounded(flank, inside: tip_body, inside: heel_body,
              outside: root_body, outside: toe_body)
```

Each condition names a solid and includes its boundary. All conditions must hold. The
current analytic evaluator accepts full revolutions with line or circular profile edges,
including holes. Its point queries use the curves themselves and explicit numerical
tolerances. A patch still needs a selected branch and oriented boundary loops before it can
participate in checked closed-solid assembly; those capabilities are the next step.

Where two generated faces meet tangentially at a shared vertex of their source profile,
name their common characteristic once:

```sv
seam flank_join(flank_region, transition_region)
```

The operands can be envelopes or trimmed patches of envelopes. They must share the source
revolution, generating motion and one actual profile vertex. The evaluator checks tangency
after solving and checks both faces when finding a seam point. This lets a flank and its
root fillet refer to the same boundary geometry. Oriented uses of these curves in closed
face loops are still needed for solid assembly.

An envelope also meets a finite boundary surface along a named seam:

```sv
seam tip_edge(flank_region, tip.wall)
seam toe_edge(flank_region, toe.wall)
```

Here the generated face comes first and a surface reference comes second. The evaluator
checks both the envelope equation and incidence with the finite boundary, plus the face's
material conditions. A tip cone or toe sphere therefore belongs in the model; the verifier
can read these named edges instead of rebuilding their intersection equations.

Name shared corners through their incident seams:

```sv
vertex tip_toe(tip_edge, toe_edge)
vertex join_toe(flank_join, toe_edge)
```

The first form meets two finite boundaries on the same named generated face. The second
meets a generating junction with a boundary on one of its faces. The core checks every
defining face and its material conditions; a loose seam-coincidence tolerance cannot
substitute for a tighter boundary-incidence check. A vertex owns no planar coordinates.
Repeated uses share its declared identity, while separately declared coincident vertices
remain distinct. Local evaluation and root finding do not yet choose a globally unique
branch or assemble the surrounding faces into an analytic solid.

Give a seam finite extent between those corners:

```sv
edge toe_span(toe_edge, from: join_toe, to: tip_toe, along: shaft_axis)
```

`along` supplies a geometric slicing direction. A fraction from zero to one selects a
plane perpendicular to that line between the endpoint projections; the evaluator solves
for the seam point in that plane. It checks actual curved geometry and reuses the named
corners at the ends. The evaluator currently checks local slices, so global branch
uniqueness, turning points and curve approximation bounds still need validation before
analytic solid assembly.

An ordered finite-edge loop can now name its analytic support:

```sv
face working(toe_span, tip_span, heel_span, join_span, on: flank_region)
face transition(round_toe_span, join_span, round_heel_span, root_span, on: fillet_region)
```

Both faces share `join_span` and traverse it in opposite directions. Each edge must
belong to the exact named support, and the loop closes through shared corner identities.
The face reader maps boundary parameters into that support's chart and checks incidence.
An `on:` face is spatial and cannot be extruded as a planar profile. These boundary
declarations still need interior and closed-solid validation; see
[Analytic face boundaries](analytic-face-boundaries.md).

```
unit mm
point p0 hint(x: 10, y: 0)
point p1 hint(x: 14, y: 0)
point p2 hint(x: 14, y: 6)
point p3 hint(x: 10, y: 6)

horizontal line e0(p0, p1) ->
vertical   line e1(p1, p2) ->
horizontal line e2(p2, p3) ->
vertical   line e3(p3, p0) -> close

point q0 hint(x: 0, y: 0)
point q1 hint(x: 0, y: 10)
vertical line ax(q0, q1)
ground q0
q0 distance(10) q1
q0 distance(10, along: x) p0
q0 distance(0, along: y) p0
p0 distance(4) p1
p1 distance(6) p2

face sec(e0, e1, e2, e3)
solid ring(sec, about: ax)
```

```
  10 params, 10 equations, structural rank 10; DOF 0; 2 components: DOF 0, 0; 3 rigid cluster(s) in the distance graph
  ring.volume = 1809.55
```

Pappus, from the source: a 4 × 6 section whose centroid stands 12 from the axis is
`2π · 12 · 24 = 1809.557`. Write `solid ring(sec, about: ax, sweep: 90deg)` and the report says
`452.387`, a quarter of it, with `ring.start.area` and `ring.end.area` both 24 — the face itself,
at each end of the turn. Round faces are read as fine polygons, so every volume here is exact to
that faceting and not beyond it.

**The body rule** is one sentence: a body is its **stock, plus everything `on` it, minus everything
that `cut`s it**. Add to the plate above:

```
point o hint(x: 30, y: 20)
a distance(30, along: x) o
a distance(20, along: y) o
circle hole(center: o) hint(r: 5)
radius(5) hole
face hole_f(hole)

solid stock(sec, depth: 30mm)
solid bore(hole_f, depth: 30mm)
solid body(stock)
bore cut body
```

```
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.volume = 69643.8
```

`72000 − π · 5² · 30 = 69643.81`. Swap the last two lines — `bore cut body` before the `solid
body(stock)` it belongs to — and the number is the same, because both sides of the rule are *sets*
and a set has no order.

**Body-relative cutter extents.**

Use `through:` to let a cutter span a part, and `cut` to subtract it:

```solvent
solid pinhole(face(hole), through: body)
pinhole cut body
```

The extent includes the target's stock and additions in both directions along the section's
normal, ignoring other cutters. It follows the solved geometry and placement. The constructor
alone creates the cutter; the infix statement removes its material from the body. For a blind
pocket, keep an explicit `depth:` or `from:`/`to:` extent and apply it with the same `cut` word.
Old Boolean `X through B` statements are now written `X cut B`.

**`bound` keeps what lies within.** The third side of the body rule is the intersection:
`tip bound body` keeps of the body only what is inside `tip`. A rim is a sphere within a cone,
which is one statement rather than `heel − (heel − tip)` through a named intermediate:

```solvent
solid body(heel)
tip bound body
toe cut body
```

Difference and intersection commute, so `cut` and `bound` statements need no order between them;
union comes first, as before.

Being sets is also the one thing you have to write out. A boss standing in the floor of a pocket is
**not** `pocket cut body` and `boss on body`: that is stock ∪ boss − pocket, and the pocket eats
the boss. Name the intermediate the feature tree would have left anonymous:

```
// the plate again, with `o` at its middle
circle rim(center: o) hint(r: 15)
circle stud(center: o) hint(r: 5)
radius(15) rim
radius(5) stud
face rim_f(rim)
face stud_f(stud)

solid stock(sec, depth: 30mm)
solid pocket(rim_f, depth: 10mm)
solid boss(stud_f, from: -10mm, to: -4mm)

solid shell(stock)
pocket cut shell

solid body(shell)
boss on body
```

`shell.volume` is `72000 − π · 15² · 10 ≈ 64931.5` and `body.volume` is that plus
`π · 5² · 6 ≈ 65402.7`; written flat on one body it comes out 64931.5, the boss gone. The extra
name is honest rather than a limitation: `shell` is exactly what a history calls "the body as of
step 2", and this is the language letting you say it.

**What the report says.** `--where body` is the reader's only picture of an object no view of the
sheet shows whole: the volume, the surface area, the box it stands in, and each face's area under
the name the document wrote it by — `near` and `far` for a prism's caps, `start`/`end` for a
partial revolution, and otherwise the drawn edge that side was swept from.

```
$ build/solventc plate.sv --where body
plate.sv: solved
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.ab.area = 1800
  body.area = 11585.4
  body.bc.area = 1200
  body.bounds.x0 = 0
  body.bounds.x1 = 60
  body.bounds.y0 = 0
  body.bounds.y1 = 30
  body.bounds.z0 = 0
  body.bounds.z1 = 40
  body.cd.area = 1800
  body.da.area = 1200
  body.far.area = 2321.46
  body.hole.area = 942.474
  body.near.area = 2321.46
  body.volume = 69643.8
```

`body.hole.area` is `2π · 5 · 30`: the bore's wall, named by the circle it was swept from. A face
a boolean ate leaves a name the document still writes and no area under it. `--stl PATH` writes one
solid as binary STL for a printer; `--solid NAME` says which, and without it the only Boolean root is selected automatically. Multiple roots require a selection.

**Views, derived.** A `.svd` file selects solved model solids and projects them with its own
viewing directions, positions, and scales. See [Solvent Drawing](solvent-drawing.md) and
`rust/examples/vtwin/cylinder.svd`. The `preview` block in `vtwin/components/cylinder.sv` supplies the geometric datums
and cylinder instance. The same file defines the component used by the assembly. The piston,
disc, flywheel, throttle, and frame also have previews in their component files; their `.svd`
sheets load those files directly. For example, the flywheel preview is simply:

```sv
preview {
  unit mm
  fw: Flywheel(std.up, dims: vtwin_dims)
}
```

The disc and piston additionally place their pin at `R` or `L` from `std.origin`: those
endpoints describe physical dimensions as well as a datum's direction.

**Every part of that engine is now written this way** — `vtwin/components/piston.sv`, `disc.sv`,
`flywheel.sv`, `throttle.sv` and the plate in `frame.sv` beside the cylinder — and what each one
turned out to *be* is worth reading, because the shape of the statement follows from where the
part's own axis lies relative to its section. The piston is a **turn**: its left-hand profile
about the rod's line, which is why the view from the crown comes out a disc with nothing anywhere
saying so. The disc, the flywheel and the throttle have their axis running *through* the section,
so they are prisms, and only the features whose axis lies *in* it — a radial set screw, a
cross-hole — are turns. The plate is both.

One rule keeps falling out of that, and it is the thing to know before writing a part:
**where a part's turned features are is where its section has to be.** A turn about a line lying
in the section puts what it makes *on* that plane whatever else is written, so the crank disc is
sectioned on its mid-plane because the set screw runs through it, and the plate on its mid-plane
because the plenum, the boss, the vents and the coupling's hole are all centred there. Sectioned
on a face instead, half of each of those would have been in fresh air.

**And the dimensions, asked for too.** A sheet may ask the machine for the callouts that follow
from the object:

```
```

It gives the part's **overall extents** in that view — one along each of the view's own axes,
measured between the faces that bound them and stood clear of the outline — and the **diameter**
of every round feature that view sees square on. Nothing is placed by hand: they go through the
same layout engine every stated dimension goes through, so a generated dimension stands off a
stated one because neither knows the other is different. Adding the line to the sheet above puts
`60`, `40` and `⌀16` on the front view and `40` and `30` on the right, and the document says
nothing about where any of them go.

A generated dimension is a **reading of the drawing and not a statement in it**: it adds no
equation, no unknown and no freedom, it cannot be dragged or edited, and it reads the *solved*
pose — so it says what the part came to and follows an edit without being one.

What it will not do is guess. Which datum a stack is measured from, which fit is critical, what
is a reference and what controls the drawing are the design, and those a sheet still states the
way it always did. This is only the part that was never a decision.

**What is refused, and why.**

| written | reported |
|---|---|
| `face bad(ab, zz, cd, da)` | E080 — "`zz` and `cd` share no point: a face is a loop, walked in order" |
| `face bad(ab, hole)` | E080 — "a circle is a whole loop: it stands in a face by itself" |
| `face bad(a, b)` | E080 — "a face is bounded by lines, arcs and circles, and `a` is a point" |
| `solid bad(ab, depth: 3mm)` | E080 — "a swept solid is written over a face, and this is a line" |
| `solid bad(sec)`, `sec` a face | E080 — "a body is made of solids, and this is a face" |
| `solid bad(sec, from: 0mm, to: 0mm)` | E080 — "a prism swept nowhere is no solid" |
| `solid bad(sec, about: ax, sweep: -90deg)` | E040 — "a sweep is a magnitude: which way it turns is `sense: cw`" |
| `solid bad(sec, about: a)` | E081 — "a face turns about a line, and `a` is a point" |
| `solid bad(sec, depth: 3mm, about: ax)` | E001 — "a solid is a face swept along its normal (`from:`/`to:`, `depth:`) or turned about a line (`about:`), not both" |
| `x cut y` and `y cut x` | E041 — "`x` is made of itself" |
| `h cut s`, `s` a face swept | E080 — "`s` is a face swept, and only a body takes features: give it a stock (`solid s(s_stock)`) and write them there" |

The `h cut s` one carries the most: only a *body* takes features, so a face swept is a
primitive and a body is the term over primitives, and the two are never the same name. The negative
sweep is 1.5's rule again — which way is a word, not a sign.

### 1.15 Checking your work

```
make solventc                     # once
build/solventc drawing.sv         # parse, elaborate, solve, diagnose; --json for structure
build/solventc drawing.sv --where hinge     # where a name landed
build/solventc drawing.sv --stl part.stl --solid body    # a solid, for a printer     (1.14)
```

Exit codes: 0 solved, 1 did not elaborate, 2 elaborated but did not solve. The text report gives
the parameter and equation counts, the **DOF**, and the culprit lines (`over:`, `conflict:`,
`implied:`); the diagnosis's status, `diagnosis.status` under `--json`, is one of five:

| state | meaning | what to do |
|---|---|---|
| `well` | DOF 0, everything consistent | done |
| `under` | DOF > 0 | something can still move; add a constraint or a gauge |
| `over` | a dimension takes part in a consistent redundancy | remove one of the `over:` lines; editing one is the next conflict |
| `conflict` | statements that cannot all hold | the `conflict:` lines are the *minimal* disagreeing set |
| `unsolved` | the solver stopped short of a solution | usually a bad seed; reseed nearer the intended branch |

**Where something landed** is `--where NAME`, which answers with the numbers under that name —
its own if it is a point (`--where hinge` gives `hinge.x`, `hinge.y`), a whole assembly's if it is
an instance (`--where views` gives every view's origin and bearing), one number if you name it
(`--where hinge.x`). Under `--json` every name in the document answers, in a `positions` table,
and `--where` narrows it. It is the question a reader without a picture asks most, and it beats
writing a `claim` to see whether it is refuted.

A redundancy among pure relations (a fourth `perpendicular` round a rectangle) is a theorem: listed
as `implied:`, never an error. Two habits: **ground something**, since a figure with no `ground` is
under by three however determined its shape; and **seed for the branch**, since the solver finds
*a* solution near where it started, so an arc seeded on the wrong side comes out mirrored.

---

## 2. Examples

Each was run through `solventc`; the DOF and state quoted are what it reported.

### 2.1 One dimensioned line: DOF 0, well

```
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 10)

line ab(a, b)
horizontal ab
a distance(40) b

ground a
```

Four unknowns, four equations (level, length, the two the ground pins). `b`'s seed is nowhere near
the answer and need not be: it says which side of `a` to put `b` on, and nothing more.

### 2.2 A rectangle, as a chain: DOF 0, well

```
param w = 60
param h = 40

point p0 hint(x: 0, y: 0)
point p1 hint(x: w, y: 0)
point p2 hint(x: w, y: h)
point p3 hint(x: 0, y: h)

horizontal line bottom(p0, p1) ->
vertical   line right(p1, p2) ->
horizontal line top(p2, p3) ->
vertical   line left(p3, p0) -> close

p0 distance(w) p1
p1 distance(h) p2
ground p0
```

`param` is arithmetic done while reading: `w` is 60 wherever it appears and never an unknown. A
`param` may read another written anywhere in its body or an enclosing one, in any order; one
defined in terms of itself is E041. The chain states nothing four separate `horizontal`/`vertical`
lines would not; it reads as the outline it is.

### 2.3 Naming a dimension: DOF 0, well

```
// substitute for the two dimensions in 2.2, and drop its param lines
p0 distance(w = 60) p1          // states it and names it
p1 distance(w / 2) p2           // reads it: the height follows the width
```

Edit the 60 and the height follows. A number stated once and read everywhere is the difference
between a drawing and a picture of one. The name is declared in the body like a `param`'s, so
`param h = w / 2` may read it too, and `hint(x: w)` may seed from it.

### 2.4 A free variable: DOF 1, under, on purpose

```
point a hint(x: 0, y: 0)
point b hint(x: 10, y: 0)
point c hint(x: 0, y: 9)

line ab(a, b)
line ac(a, c)
horizontal ab
vertical ac
a distance(s) b         // s is defined nowhere...
a distance(s) c         // ...so the two lengths are tied, and their value is the solver's
ground a
```

`s` names an unknown. The two lengths must agree; nothing says what they are, so one freedom is
left. Give `s` a value anywhere, or add a third constraint, and it closes. Written inside a
component, `s` is that instance's unknown (`t1.s`): two instances have two, as they would with a
formal left unbound.

### 2.5 An arc, tangent to what it joins: DOF 0, well

```
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 0)
point c hint(x: 40, y: 10)
point d hint(x: 40, y: 40)
point o hint(x: 30, y: 10)

horizontal line run(a, b) -> tangent
arc fillet(center: o) hint(r: 10) -> tangent
vertical line rise(c, d)

radius(10) fillet
a distance(30) b
c distance(30) d
ground a
```

`fillet` names only its centre; the chain threads `b` in as its start and `c` as its end, and each
`tangent` becomes a tangency stated *at* that shared point. This is the shape of most real work:
state how things meet and let the positions follow. `rect_fillets.sv` is the same idea round four
corners.

### 2.6 A component, instanced: DOF 0, well

```
component Rung(a: point, b: point, len: Length) {
  line e(a, b)
  horizontal e
  a distance(len) b
}

point l0 hint(x: 0, y: 0)
point r0 hint(x: 50, y: 0)
point l1 hint(x: 0, y: 20)
point r1 hint(x: 50, y: 20)

t0: Rung(l0, r0, len: 50)
t1: Rung(l1, r1, len: 50)

line stile(l0, l1)
vertical stile
l0 distance(20) l1
ground l0
```

The formals alias the actuals; nothing is added at the boundary.

### 2.7 Repetition: DOF 5, under

```
param n = 6
param r = 40

cycle n as i {
  point p hint(x: r, y: i * 60)
  line e(p, next.p)
  e equal next.e
}
ground p[0]
```

Six links, all told to be the same length, which round a loop is one statement more than is
independent (listed as `implied:`), and nothing sizes the ring, so five freedoms remain.
Under-constrained repetition is normal. `p[0]` indexes a copy.

The body may end mid-joint (1.7), which is how a closed contour is written with no names for its
corners at all. DOF 1, under:

```
cycle 4 {
  line s -> perpendicular equal
}
s[0].p1 distance(50) s[0].p2
ground s[0].p1
```

Each copy's side is welded onto the next's at a corner held square and equal; the wrap seals the
loop. One dimension sizes it and a grounded corner places it, leaving the square free to swing
about that corner. Round a closed loop one `perpendicular` and one `equal` are theorems, noted as
implied and never painted; dimension every corner instead (`-> angle(90)`) and the same closure
reads `over`, since editing one of those numbers is the next conflict. `square.sv` is this figure;
`ngon.sv` is the parametric case, a component taking `n` with its corners seeded once round a
circle on purpose, since equal chords fix each central angle's size and not its sign, and the
winding is a branch only a seed can choose.

### 2.8 A curve from a computed point: DOF 1, under

```
component Involute(c: circle, phase: Angle, u: Angle) {
  point p = ( c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
             c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)) )
}

point o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve f = Involute(base, phase: 0).p over u in (0, 90)

point t hint(x: 25, y: 8)
t on f
ground o
fix base.r
```

An involute is a component with one computed point, and the curve is that point as `u` runs. The
remaining freedom is the contact's own parameter, *how far along* `t` sits, which nothing here
states; it is why a contact slides along a curve instead of breaking when the geometry beneath it
moves. Seed it with `t on f hint(t: 30)` or pin it with `t on(t == 30) f` (which makes this DOF 0).

### 2.9 A curve stated as a locus: DOF 1, under

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  point t
  point p
  line rad(c.center, t)
  line s(t, p)
  t on c                                                 // the string leaves the circle...
  datum angle(u + phase) rad                             // ...at bearing u from the datum,
  rad perpendicular s                                    // square to the radius there,
  p distance(-(c.r * u / 1rad)) rad                      // and taut: as long as the arc
}

point o hint(x: 0, y: 0)
point x hint(x: 20, y: 0)
circle base(center: o) hint(r: 20)
line datum(o, x)

curve f = Unwind(base, datum, phase: 0).p over u in (0, 90)

point g hint(x: 25, y: 8)
g on f

ground o
fix base.r
horizontal datum
o distance(20) x
```

The same curve as 2.8 with no formula: every line of the body is the textbook definition said
once, and the solver derives what 2.8 derived by hand. Two details do real work. Point-to-line
distance is **signed**, so the minus sign is what unwinds the string one way for a positive roll
and the other for a negative one; that is why one component serves both flanks of a gear tooth.
And `angle` is **directed**, so `t` sits at bearing `u + phase` and not opposite it, with no `ccw`
needed. Where a body has a genuinely discrete choice (which of two intersections), `ccw(a, b, x)`
states it, read at the anchor and carried by continuity.

### 2.9.1 A curve of a drawn instance: DOF 1, under

```
component Crank(o: point, datum: line, theta: Angle) {
  point p hint(x: 20, y: 10)
  line arm(o, p)
  o distance(30) p
  datum angle(theta) arm
}

point o hint(x: 0, y: 0)
point x hint(x: 10, y: 0)
line datum(o, x)
ground o
ground x

c: Crank(o, datum)                                  // theta unbound: the crank turns
curve rim = c.p over theta in (0, 360)
```

`c: Crank(o, datum)` leaves `theta` unbound, so `c.theta` is the one freedom (reported as a free
variable), and `rim` is where the drawn `p` goes as it runs a full turn. The trace is anchored at
the pose on the sheet: drag `c.p` and the anchor moves with it. `jansen.sv` is this at full size.

### 2.10 Three views: DOF 0, well

```
// a 60-wide, 40-tall, 30-deep block, three views, one corner tied across them
point Af hint(x: 0, y: 0) in front
point qf hint(x: 40, y: 0)
plane front(origin: Af, toward: qf)                             // the page itself
point At hint(x: 0, y: 90) in top
point qt hint(x: 40, y: 90)
plane top(origin: At, toward: qt, from: front, fold: 0deg)      // folded up from the x-axis
point Ar hint(x: 150, y: 0) in right
point qr hint(x: 150, y: -40)
plane right(origin: Ar, toward: qr, from: front, fold: -90deg)  // folded from z, turned so z is up
ground Af
ground qf
ground At
ground qt
ground Ar
ground qr

point Bf hint(x: 60, y: 40) in front
Af distance(60, along: x) Bf
Af distance(40, along: y) Bf
point Bt in top
point Br in right
Bf project Bt          // width agrees front <-> top
Bf project Br          // height agrees front <-> right
Bt project Br          // depth agrees top <-> right
At distance(30, along: y) Bt
```

Each view's origin is the same corner `A` as that view sees it, so no projection between origins
is needed. `B` in the top and right views is placed by projection and the one depth dimension.

The standard library writes this layout once. `use std` and `views: ThreeViews(O, right: 150,
up: 90)` declares the page as `views.front` and folds `views.right` and `views.top` from it, with
`views.right_origin` and `views.top_origin` the corner as those views see it; a drawing grounds
`O` and writes its geometry `in views.top`. `bracket.sv` is the full case, with an auxiliary view
folded at the bearing of an inclined face.

### 2.11 Two skew axes at a stated shaft angle and offset: DOF 0, well

```
// two shafts at a stated angle and offset: the gear's axis drawn in the front view, the
// pinion's in a view folded from it by a fold the drawing solves for
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
line gax(hint(x: 0, y: 0), hint(x: 0, y: 50)) in front
ground gax.p1
ground gax.p2

point o2 hint(x: 120, y: 0)
point t2 hint(x: 160, y: 0)
plane side(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 30deg)
line pax(hint(x: 120, y: 10), hint(x: 180, y: 12)) in side
pax.p1 distance(0, along: u) side
pax.p2 distance(60, along: u) side
pax.p1 horizontal pax.p2

gax angle(90deg) pax      // the shaft angle, in space
gax distance(17.5) pax    // the offset: the common perpendicular, in space
```

`solventc` reports `warning[W111]: \`beta\` is a free variable: the solver answers for it`, then
`solved`, `27 params, 27 equations, structural rank 27; DOF 0`, and `--where pax` gives
`pax.p1.y = 17.5`, `pax.p2.y = 17.5`. The two axes are drawn in different views, so `angle` and
`distance` between them are relations in space (1.5): the unsigned angle between their
directions and the common-perpendicular distance. The fold `beta` and the pinion axis's height in
its view are the two unknowns they settle (the fold comes out at 0°: the side view is the top
view, and the pinion's axis lies in it 17.5 behind the gear's, crossing it square). No datum point is grounded: both views' place on
the sheet is held silently (1.13), and a `ground o2` would change nothing but make the hold a
statement. Without the offset the same document reports `27 params, 26 equations, structural rank
26; DOF 1` — the pinion axis may slide along the common perpendicular. `tests/spatial_lang.rs`
holds this document against an independent reading of the solved axes.

### 2.12 A hypoid's pitch cones through the mean point: DOF 0, well

```
unit mm
param module = 4mm
param Rg = 48 * module / 2
param Rp = 24 * module / 2
param E = 20mm

// the pitch plane, and M on it
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane P(origin: o, toward: t)
point M hint(x: 0, y: 0) in P
ground M
point O hint(x: 110, y: 0) in P
point A hint(x: 95, y: 18) in P
line gen_g(O, M) in P
line gen_p(A, M) in P
horizontal gen_g

// the axial views, folded square to P along the generators
point og hint(x: 0, y: 200)
point tg hint(x: 40, y: 200)
plane G(origin: og, toward: tg, from: P, fold: along gen_g)
point oq hint(x: 0, y: -200)
point tq hint(x: 40, y: -200)
plane Q(origin: oq, toward: tq, from: P, fold: along gen_p)

// each axis in its axial view, from its apex: the apex's image is on the fold line (on P) and
// projects to the apex drawn in P; how long an axis is drawn says nothing about the cone
line gax(hint(x: -110, y: 200), hint(x: -50, y: 304)) in G
line pax(hint(x: -97, y: -200), hint(x: -28, y: -239)) in Q
gax.p1 on P
O project gax.p1
pax.p1 on P
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

`solventc` reports `solved`, `56 params, 56 equations, structural rank 56; DOF 0`, with
`--where A` at `(95.5837, 18.0989)`. Each axial view is folded `along` its member's pitch
generator, so it is square to P and holds the generator; the axis is drawn in it from the apex's
image (on P, and projecting to the apex), so P is each pitch cone's tangent plane along its
generator — the common pitch plane is by construction. The pitch radii, the shaft angle and the
offset are four conditions; the gear's pitch angle (60°) is the fifth the cones need, and the
pinion's apex and pitch angle follow (ε = 10.72°, γ = 29.56°). `tests/spatial_lang.rs` checks
every one of those against the lifted geometry.

### 2.13 The same pitch cones, named: DOF 0, well

```
unit mm
param module = 4mm
param Rg = 48 * module / 2
param Rp = 24 * module / 2
param E = 20mm

point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane P(origin: o, toward: t)
ground o
ground t
point M hint(x: 0, y: 0) in P
ground M

// the gear's axial plane, square to P about its vertical axis; the pinion's, solved, through M
point og hint(x: 0, y: 200)
point tg hint(x: 40, y: 200)
plane G(origin: og, toward: tg, from: P, fold: 90deg)
ground og
ground tg
point oq hint(x: 0, y: -200)
point tq hint(x: 40, y: -200)
plane Q(origin: oq, toward: tq, attitude: free, through: M) hint(u: (0.1618, -0.4935, -0.8546), v: (0.0918, 0.8698, -0.4847))

line gax(hint(x: 110.85, y: 200), hint(x: 50.85, y: 303.9)) in G
line pax(hint(x: -84.62, y: -248), hint(x: -4.62, y: -248)) in Q
gax.p1 on P
gax.p1 distance(120) gax.p2
pax.p1 distance(80) pax.p2
horizontal pax

cone gc(axis: gax) hint(half: 60deg)
cone pc(axis: pax) hint(half: 30deg)
angle(60deg) gc
M on gc
M on pc
gc tangent(M) pc

M distance(Rg) gax
M distance(Rp) pax
gax angle(90deg) pax
gax distance(E) pax
```

It solves to `39 params, 39 equations, structural rank 39; DOF 0`. The pitch cones are named
and the contact is stated rather than built: the gear's apex on P makes P the gear cone's tangent
plane at M, and `gc tangent(M) pc` makes it the pinion's, so the pinion's apex comes out on P with
nothing saying so. `horizontal pax` holds the pinion view's own turn, which is the one freedom a
free view drawn about a line and a point has left. `tests/spatial_surfaces.rs` solves this and
2.12 and finds the same hypoid: Γ = 60°, γ = 29.564957707°, ε = 10.722102067°, |MA| = 97.282181449,
every one within 2e-11.

### 2.14 An arc placed by its length: DOF 0, well

```
point o hint(x: 0, y: 0)
point s hint(x: 10, y: 0)
point e hint(x: 3, y: 9)

arc a(o, s, e)
radius(10) a
length(5 * pi) a        // a quarter of the circumference: the sweep is 90°
o horizontal s
ground o
```

It solves to `5 params, 5 equations, structural rank 5; DOF 0`, with `e.x = 5.8506e-16` and
`e.y = 10`: the end straight above the centre, from a seed that was not. `length(15 * pi) a` puts
it straight below, since the sweep is read counter-clockwise from `start` to `end`. The case
library's `belt_wrap.sv` does the useful version: an open belt closed as one tangent chain over
two pulleys, with `length(wrap) big` in place of a centre distance. It solves to `12 params, 12
equations, structural rank 12; DOF 0`, the second pulley at `c2.x = 66.0205` for a wrap of 90 on
a radius of 25 (π + 2·asin(15 / 66.0205) = 3.6 rad).

### 2.15 A bisector, stated as two equal angles: DOF 0, well

```
point a hint(x: 0, y: 0)
point b hint(x: 40, y: 0)
point c hint(x: 10, y: 30)
point d hint(x: 25, y: 10)

line ab(a, b)
line ac(a, c)
line ad(a, d)
horizontal ab
ab angle(60deg) ac
a distance(40) b
a distance(30) c
a distance(20) d

ab angle(ad, ac) ad     // the angle from ab to ad is the angle from ad to ac
ground a
```

It solves to `6 params, 6 equations, structural rank 6; DOF 0`, with `d.x = 17.3205` and
`d.y = 10`: `ad` at 30°. Written the old way, `ab angle(beta) ad` and `ad angle(beta) ac`, the
same drawing is `7 params, 7 equations` with two `warning[W111]: \`beta\` is a free variable: the
solver answers for it` — one more unknown, stated only to be equated away. `ab angle(ab, ac,
sense: cw) ad` puts `ad` at −60° instead, `ac`'s mirror image in `ab`. The case library's
`reflection.sv` is the law of reflection in one statement, `incoming angle(m, outgoing) m`, with
the classical proof (the source's image, the strike and the target collinear) a `claim` the
diagnosis judges a theorem.

---

## 3. Working checklist

1. Declare points first, seeded roughly where you mean them, near enough to pick the right branch.
2. Declare the lines, arcs and circles built from them; prefer a chain for a contour.
3. State relations (levels, tangencies, equalities), then dimensions.
4. `ground` one point, and `fix` a scalar if a size is given rather than solved.
5. Run `solventc`. Aim for `well` and DOF 0 unless the task wants freedom left.
6. On `conflict`, read the minimal set: it names the statements that disagree, not the whole
   drawing. On `over`, find the dimension already implied by the others. On `under`, ask what can
   still move.

The documents in `rust/examples/` are the worked corpus, each with a header saying what it is for.
`rect_fillets.sv` is the best first read, `gear_trace.sv` the deepest,
`vtwin/components/cylinder.sv` the one to read for solids (1.14), and `engine.sv` with its `engine/` modules
the largest.
