# Solvent: a primer for agents

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

A callout placement, `a distance(80) b at (12, -4)`, keeps a bare `at`. It is inert too, but it
records where a person dragged a dimension, and no solve touches it.

### 1.3 Statement forms

```
use NAME[.NAME...]                      bring in a module's components and params   (1.12)
unit NAME                               what the document's numbers are in          (1.6)
param NAME = EXPR                       a number worked out while elaborating
KIND [NAME][(CHILD | hint(x: E, y: E), ...)] [hint(SCALAR: E, ...)] [knots [...]]
     [class NAME...] [in REF]           an entity declaration; every part is optional (1.4)
point NAME = (XEXPR, YEXPR)             a computed point, drawn only as a curve      (1.9)
plane [NAME](origin: R, toward: R[, from: R, fold: E | , from: R, offset: E
                                      | , u: (E,E,E), v: (E,E,E)])
                                        the datum, and a view with an attitude    (1.13)
in REF { statement* }                   every declaration inside is drawn in that plane
face NAME(EDGE, ...)                    a closed loop of edges, on one plane        (1.14)
solid NAME(FACE, SWEEP...)              that face swept: depth:/from:/to:, about:   (1.14)
solid NAME(SOLID)                       a body, made of a stock
REF on REF  |  REF through REF          material added to, or taken from, a body    (1.14)
view(SOLID) in REF                      a picture asked of a solid                  (1.14)
section(SOLID, at: REF) in REF          the same, cut at a plane
dimensions(SOLID) in REF                the callouts that follow from the object
style .NAME { PROP: VALUE; ... }        what a class looks like                     (1.11)
WORD[(ARGS)] REF  |  REF WORD[(ARGS)] REF
     [hint(SLOT: E, ...)] [class NAME...] [at (t, r)]
                                        a constraint, prefix or infix               (1.5)
claim CONSTRAINT                        judged, never solved for                    (1.10)
ground REF                              pin both coordinates of a point
fix REF.FIELD                           pin one scalar: fix c.r
ccw(a, b, c) | cw(a, b, c)              record a root choice; adds no equation
NAME: Component(ARGS) [in REF] [class NAME...]     an instance                      (1.8)
component NAME(FORMALS) { statement* }  a definition
repeat N [as i] { ... }                 N copies, unrelated
cycle N [as i] { ... }                  N copies that close; `next` and `prev` are in scope
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
`arc(center: c)` and `line class construction` are all statements. The token after the kind
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
which a traced component may read (1.9). There is no separate `frame`: the word is refused.

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
```

| word | fixity | operands |
|---|---|---|
| `on` | infix | a point to a line, circle, arc, spline or curve; between two **solids** it is not a constraint at all but the body rule (1.14) |
| `distance` | infix | two points (`along: x` / `along: y` for the run and the rise, signed first to second, or `along: right \| left \| up \| down` to say the direction in a word); a point and a line, or two lines (a magnitude — `side: left \| right` pins which side, and without one the seed picks); two concentric circles or arcs (the radial gap) |
| `distance` | prefix | a line: the distance between its own ends |
| `tangent` | infix | a line and a circle or arc (`at: p1` / `p2` for a tangency at that end; `side: left \| right` says which side of the line the centre is); two circles or arcs (`external: true/false`); an arc and a line (`at: start` / `end`); a spline or a curve and a line |
| `equal` | infix | two lines (length), or two circles or arcs (radius) |
| `curvature` | infix | a spline or a curve and a circle or arc: the circle becomes the osculating circle there. Refused on a traced curve |
| `horizontal`, `vertical` | prefix / infix | a line, or a pair of points with no line drawn between them |
| `angle` | infix | two lines; a bare number is degrees, and `sense: cw` turns it the other way |
| `radius` | prefix | a circle or an arc |
| `coincident`, `symmetry(line)` | infix | two points |
| `midpoint` | infix | a point and a line |
| `parallel`, `perpendicular` | infix | two lines |
| `project` | infix | two points, each `in` a plane: two images of one point in space (1.13) |
| `ground`, `fix` | prefix | pin both coordinates of a point, or one scalar |
| `ccw(a, b, c)`, `cw(a, b, c)` | a call | all three in the parentheses: the predicate is about the triangle |

One word covers several constraints, told apart by the **kinds of its operands** (`on` is five,
`distance` six, `tangent` six) or by **fixity** (`horizontal` on a line versus between two points).

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
| `l tangent(side: left) c` | the circle's centre lies left of `l` |
| `ccw(a, b, c)` | `c` is left of the ray `a → b` |
| `… at (t, r)` | a callout: `t` along the dimension from its middle, `r` across it — **positive to the left** of the direction it is measured along, and for a radius or an angle, `t` is an angle from the start and `r` a distance out from the centre |

**A distance measured from a line is a magnitude**: a negative one is refused, and which side is
`side:`. A component that must work either way up takes a `Side` formal (`s: Side`, called as
`Part(…, s: right)`) and writes `side: s`. Where a side is *arithmetic* rather than a
convention — `Loc(v: -hw)` passing a coordinate — write `abs(v)` and let the seed, which is the
point worked out, say which side it falls on.

The run, the rise and the angle keep their signs, because there the sign is arithmetic a
component computes (`dy` is a coordinate; `alphaL` is a bank leaning the other way). The words
are how a *drawing* should say it.

**`angle` is directed**: the full-turn angle from `l1`'s direction (`p1` to `p2`) to `l2`'s,
counter-clockwise positive. It pins which side, not just the tilt, so a bearing needs no
orientation predicate. Swapping the lines or reversing one's endpoints changes the reading.

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
dimension that reads it (the CLI reports it as W111). The tie must be affine in one free name
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

A formal is an entity kind (`point`, `line`, `circle`, `arc`, `plane`, …) or a number
type (`Length`, `Angle`, `Int`, `Scalar`). Passing an entity is **aliasing**, not a constraint: the
formal and the actual are one entity, so a component boundary costs nothing. **The entities are
given by position, in order, and every number is given by label** — `Rung(l0, r0, len: 50)`, never
`Rung(l0, r0, 50)`, and nothing positional after the first label. Position is a count, and a count
is the one thing a reader of a long formal list cannot check: an argument written one place off
binds to the formal beside the one it was meant for, and what comes back is a complaint about
something else. Either mistake is **E004**, at the argument. A numeric formal left unbound is a
free unknown of the drawing, named under the instance (`c.theta`), which is how a mechanism is
drawn with its crank free (2.9.1).

A file's top-level `param`s and named dimensions are in scope inside the components it defines,
and so are the params of the modules it uses; a formal of the same name shadows either, and a
body's own definition shadows both. Everything an instance makes is
reachable by dotted name (`c.p`, `t0.e`, `five.s[0].p1`, and a dimension named inside it as
`t0.w`) — there is no export list, and passing
one instance's entity to another as an argument makes the two one entity. (`port` is retired;
a document that writes one is told what to write instead.)

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

### 1.11 Style

What a drawing *is* and how it *looks* are separate statements. A declaration or relation carries
a **class**; a `style` block says what a class looks like. Nothing in the solver reads one.

```
style .construction { dash: 7 4 }                      // shipped by default; a document may override
style .centerline   { dash: 12 3 2 3; width: 0.5; color: #888888 }
style .heavy        { width: 2.5 }
style .dimension    { display: none }                  // no callouts...
style .shown        { display: inline }                // ...except the ones that ask
style .phantom      { dash: 6 3; display: geometry }   // drawn, never dimensioned

line datum(o, q) class construction
line ab(a, b) class centerline heavy                   // several classes cascade, later over earlier
a distance(80) b class shown
g: Throw(o, bore, theta: 220deg) class phantom         // reaches everything the instance makes
```

Properties: `dash`, `width`, `color`, `display: none | inline | geometry`. Lengths are screen
pixels, so a dash pattern does not change when you zoom. An unmatched class is not an error. A
class on an instance overrides the classes of the statements inside it. Every point is drawn under
the implicit class `.point`, so `style .point { display: none }` hides the handles.

### 1.12 Modules

```
use engine.dims          // engine/dims.sv beside the document, else the library compiled in
use std                  // the standard library: ThreeViews (1.13), Ellipse (1.9), Polygon and Hex
use hardware             // fasteners and fittings by name: hexbolt14_af, brg608_od, oring014_cs, …
```

A module is a Solvent document read for its `component`s and its top-level `param`s; its own
drawing is not drawn, so `gear.sv` is a module as it stands. A module's own `use`s are followed,
once each. No such module is E070, a component defined twice is E071, and a module's own error is
reported at the `use` that brought it in. `rust/examples/engine.sv` is the worked case: a
four-cylinder engine in three views, written as a dimension module, a parts module and one module
per part.

### 1.13 Planes and views

A `plane` is the datum, and a view: it carries a constant attitude in space, the page's where none
is written. A point says
which view it is drawn `in`, and `a project b` says two points are two images of one corner: their
coordinates along the fold line the two views share agree, which is one equation. **Nothing
three-dimensional is solved for.**

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

### 1.14 Faces, solids and derived views

A feature tree is imperative because it is a *history*: step *n* acts on the anonymous body as of
step *n − 1*, and names faces by the order they were cut in. Solvent names everything, so **a solid
is a term, never a step** — a face swept, or a stock plus everything `on` it minus everything
`through` it — and the implementation finds the order the way it finds the order of `h = w / 2`.
**The order lives inside a term and never between statements**, so `bore through body` may be
written above the `solid body(…)` it belongs to or fifty lines below it and says the same thing.

**Nothing three-dimensional is solved for.** A solid owns no parameter: every extent is an
expression the elaborator works out, and the geometry it is swept from is the drawing, solved in 2D
as it always was.

**A face** is a closed loop of edges the drawing already has, on one plane — the plane its edges
are drawn `in`, the page where nothing says otherwise.

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
`through` the body.

**A face closes itself.** The brackets hold a *walk*, and a **point** in the list is a corner the
walk goes straight to and straight on from; `-> close` — the chain's own word — seals the run back
to the first item. So the rectangle above needs no `ab` and no `da`:

```
face brief(a, bc, cd, -> close)
```

which mints exactly the two straight runs the source would otherwise have declared, in the two
places the loop had a gap. A minted run is not on the sheet: it carries the class `.closure`, whose
shipped rule is `display: none`, so say `style .closure { display: inline }` to see one. It still
names a face of whatever is swept from the loop — `brief` swept gives `block.close0` and
`block.close1` beside `block.bc`. The numbering skips names already used by the loop’s existing
edges.

Three things it will not do. The gap between the **last** item and the first is minted only where
`-> close` says so, so "the loop closes" stays something the source states. And an interior gap
between two *edges* is still refused: a point in a list can mean nothing else, but two edges that do not meet
are edges listed out of order. For the same reason an edge standing between two gaps is refused —
`face bad(a, bc, d, -> close)` could be walked `b`-first or `c`-first, and nothing there says
which — so an edge takes its direction from a neighbour it actually meets.

**A solid** is that face swept, one of two ways.

A **prism** runs along the face's own normal. `depth: 30mm` is the draughtsman's reading — the
material *behind* the face the view shows, which is `from: -30mm, to: 0mm` — and `from:`/`to:` are
written out when the face is somewhere other than an end, as a boss standing off a floor is below.

A **revolution** turns the face about a line **in the face's own plane**: `about: ax` is a full
turn, `sweep: 90deg` is a quarter of one, and `sense: cw` turns it the other way.

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
`through` it**. Add to the plate above:

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
bore through body
```

```
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.volume = 69643.8
```

`72000 − π · 5² · 30 = 69643.81`. Swap the last two lines — `bore through body` before the `solid
body(stock)` it belongs to — and the number is the same, because both sides of the rule are *sets*
and a set has no order.

Being sets is also the one thing you have to write out. A boss standing in the floor of a pocket is
**not** `pocket through body` and `boss on body`: that is stock ∪ boss − pocket, and the pocket eats
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
pocket through shell

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
solid as binary STL for a printer; `--solid NAME` says which, and without it a document with more
than one is told `say which solid with --solid: stock, bore, body`.

**Views, derived.** *A part carries no views.* It is a solid, and a sheet that wants a picture asks
for one:

```
view(SOLID) in PLANE
section(SOLID, at: PLANE) in PLANE
```

```
unit mm
use std

point a hint(x: 0, y: 0) in views.front
ground a
views: ThreeViews(a, right: 120, up: 80)

in views.front {
  point b hint(x: 60, y: 0)
  point c hint(x: 60, y: 40)
  point d hint(x: 0, y: 40)
  point o hint(x: 30, y: 20)

  horizontal line ab(a, b) ->
  vertical   line bc(b, c) ->
  horizontal line cd(c, d) ->
  vertical   line da(d, a) -> close

  circle hole(center: o) hint(r: 8)
}

a distance(60) b
b distance(40) c
a distance(30, along: x) o
a distance(20, along: y) o
radius(8) hole

face sec(ab, bc, cd, da)
face hole_f(hole)
solid stock(sec, depth: 30mm)
solid bore(hole_f, depth: 30mm)
solid body(stock)
bore through body

plane mid(origin: a, toward: b, from: views.front, offset: -15mm)
view(body) in views.right
section(body, at: mid) in views.front
```

```
  31 params, 31 equations, structural rank 31; DOF 0; 6 components: DOF 0, 0, 0, 0, 0, 0; 2 rigid cluster(s) in the distance graph
  body.volume = 65968.2
```

The right view is not drawn and is not tied back by `project`: it is asked for, so the bore reads
its own diameter there and cannot disagree with the front view about the depth. A **section is
drawn in a view parallel to the plane it is cut at** (E084), or the true shape it shows is not the
shape it is a section of. The strokes come back under the implicit classes `.visible`, `.hidden`
and `.section`, styled the way everything else is (1.11) — `.hidden` ships dashed. A round surface
is drawn by its silhouette and never by the facets it is classified against, so a cylinder seen
from the side is two lines at every zoom.

`vtwin/cylinder.sv` is the worked case: one section, and the solid it is a section of. The same
part used to be written three times — the section, then the body redrawn in two more views as
page-aligned rectangles re-tied by `project`, with every depth ordinate related to the section by
no statement at all. Written once it is **119 lines instead of 144** and its component takes
**6 formals instead of 12**, and the sheet that draws it (`vtwin_cylinder.sv`) went from
`147 params, 147 equations` to `69 params, 69 equations`, DOF 0 both ways: the two extra views cost
the drawing nothing, being questions and not geometry.

**Every part of that engine is now written this way** — `vtwin/piston.sv`, `disc.sv`,
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
dimensions(SOLID) in PLANE
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
| `x through y` and `y through x` | E041 — "`x` is made of itself" |
| `h through s`, `s` a face swept | E080 — "`s` is a face swept, and only a body takes features: give it a stock (`solid s(s_stock)`) and write them there" |
| `section(block, at: front) in side` | E084 — "a section is drawn in a view parallel to the plane it is cut at" |

The `h through s` one carries the most: only a *body* takes features, so a face swept is a
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
`rect_fillets.sv` is the best first read, `gear_trace.sv` the deepest, `vtwin_cylinder.sv` over
`vtwin/cylinder.sv` the one to read for solids (1.14), and `engine.sv` with its `engine/` modules
the largest.
