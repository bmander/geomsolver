# Solvent: A Declarative Language for Constrained Geometry

**Specification, Draft 0.57 — October 2026**

**[0.57] A point drawn in two planes.** `O := point in P, G` draws `O` in both planes, so on the
line where they meet: what a layout draws in two views and ties by `project` is said once, and
each plane's own geometry reads `O` there (§6.7).

**[0.56] A plane tangent to a set.** `P tangent(at: m) S` says `P` is the set's tangent plane at
`m`, for any set — what a hypoid's pitch plane is to each pitch cone, said without drawing the
cone's axis in a view square to the plane (§6.21). It knows no shape: where the drawing already
says what the set's body says along the plane, the redundancy is the system's rank's to find.

**[0.55] A free curve is the solution of its energy.** `rope := curve(a, b)` is a curve from `a`
to `b` whose shape its energy states (§6.1, §9.10): the solution of that energy's Euler–Lagrange
equation, exact, however sharply it bends — not a spline bent towards it, so it has no control
points to count, space or gauge. It owns one number, its length; with none stated, the length too
is where the energy is stationary. A held point the curve passes is a **peg**, and the curve
drapes over it in a corner. The verdict is the classical one: Legendre's condition, and Jacobi's
(no point conjugate to the start before the end). `spline(a, b)` is no longer a free curve, and an
energy over a spline whose control points are written is refused.

**[0.54] Membership is one rule.** A plane is a set of points, and `in` is membership of it:
`p coincident P`, said of a point standing in space, draws the point in `P` as `p := point in P`
does — two spellings of one statement, the same rows (§6.7). And a set whose body is a circle's,
`{ p | p coincident P; p distance(r) o }` with `o` drawn in `P`, is the circle: drawn, dimensioned
`R`, dragged and swept as `circle(center: o)` with `radius(r)` is (§6.21). The elaborator
recognises the cheap representation; the meaning stays the set's. Sets compose by conjunction —
intersection is two statements in one body — and §6.21 says what union and difference are.

**[0.53] Variational curves.** A curve may be stated by the principle that makes it rather than
by its formula: `rope minimizes integral(p.y over p)` says the curve has the shape that makes
its height, integrated along its length, least — with `length(150mm) rope` beside it, a hanging
rope, the catenary, though nothing says so (§9.10). An energy is a sum of integrals of the point
`p` running along the curve and its unit tangent `t`, weighted by arc length; `maximizes` turns it
over. Like every statement, it says what the curve is, not what to do. **[0.55]** `curve(a, b)` is
a **free curve**: its ends written, its shape the drawing's to find (§6.1). `length(L) s` holds a
spline's whole length, and a free curve's. The diagnosis says what each stationary curve is — a minimum, a maximum, a
saddle — and `std.hangs(L) k` is a hanging rope in one word.

**[0.52] Two gauges that disagree are refused.** A `fix` adds no equation, so two holding one
number at different values, or two orientations choosing opposite roots of one triangle, are a
contradiction no conflict set can show — and applied as read, whichever came last would win,
an outcome hanging on the order of statements. They are **E031** at each statement (§13); holds
that agree are one hold.

**[0.51] Tangencies, differentiated.** `l tangent S` states each row of the set's body again as
its **derivative** along `l`, and what the body makes of its own — a foot point, a lift — moves
with the contact by a tangent unknown of its own, solved with the rest: a set whose body declares
geometry is now tangent to a line (§6.21). Two sets may touch at a point, `S1 tangent(at: m) S2`:
their tangent spaces there are one, two conditions for two surfaces. `std.TangentCones` is that
word.

**[0.50] Predicates, applied.** A relation word and a set are one thing: a body over
**parameters**, given where it is called (`Sphere(c, r: 12mm)`, `above(d: 5mm)`), and **bound
variables**, written where it is defined (`{ p | … }`, `a above(d) b`) and filled where it is used
(`q coincident ball`, `r above(d: 5mm) q`) — applied by one rule (§9.9, §6.21). So a word's body
may hold several statements and declare the geometry they need (`l1 flush(d) l2 := { l1
parallel l2; l2.p1 distance(d) l1 }`), each use making its own; a set may be written where it is
used (`q coincident { p | p distance(5mm) c }`, `l tangent std.Cylinder(ax, r: 8mm)`); a word may
use a set and a set write a word, and a tangency differentiates through both. An operand that names
nothing is said once, where it is written.

**[0.49] Sets.** A shape may be written as the points that satisfy a predicate: `ball := { p |
p distance(12mm) c }`, and a family of them as a component whose body is one, `component
Sphere(center: point, r: Length) := { p | p distance(r) center }` (§6.21). A set adds nothing to
the drawing. `q coincident S` states the body with `q` for its point; `l tangent S` states the
body at a contact on `l` the solve finds, and the body's **linearisation** there along `l` — one
condition for a line touching a surface, derived from the definition rather than written as a
kernel per pair of kinds. `std.Sphere`, `std.Cylinder` and `std.Cone` are sets now; the line a
cylinder or a cone is about is its `.about` (`axis` is an element's word, which a body cannot
name bare), and `std.PointOnCone(p, k)` is retired: write `p coincident k`.

**[0.48] Relation words, and `use M (names)`.** A file may define a relation word at its top
level, as it defines a component: `a horizontal b := a level(up) b`, `flat l := l perpendicular
t`, `a above(d) b := b distance(d, along: up) a` (§9.9). A statement writing the word is its body
with the operands and parameters put in, closed over them as a component's body is, and an error
inside it is reported where the word is written. `use std (horizontal, vertical)` reads the
names listed bare in the file that says it (§14.4) — any name a module defines: a word, a
component, a value or a group — revising 0.30's "nothing is imported bare" to *nothing is
imported bare unless the `use` names it*. There is no prelude. `a horizontal b` and `a vertical
b` between points are `std`'s words now, not the language's: a file writing them says `use std
(horizontal, vertical)`; `horizontal l` of a line is the language's own and needs nothing.

**[0.47] `ring`, solved over one copy.** `ring N about C { … }` (§12.3) is implemented as §12.4
asks: one representative per ring-local entity, the other copies its turns, owning no unknown —
so a 30-tooth gear is one tooth's unknowns, and a regular polygon one vertex's. `C` may be a point
(the turn is in its view) or an axis whose direction is held (the turn is in space, right-handed
about it). The index of a ring is refused wherever its body reads it (**E015**, widened from
hints: a relation that varies with it is not a turn), a copy reached by index from inside is a
non-invariant reference (**E021**), and what a ring cannot turn is **E023**.

**[0.46] An ordinate along any direction, and `level`.** `a distance(d, along: t) b` states
how far `b` stands from `a` along a directed line `t` — `(Y − X)·t̂ − d = 0`, signed from the first
operand to the second along `t`'s own sense — and `t` may be any axis or drawn line, in a view or
in space (`along: std.x`, `along: hinge`). The words a direction was written in before are names
for particular ones: `x`, `y` (and with the sign said, `right`, `left`, `up`, `down`) the axes of
the view both points are drawn in — the run and the rise — and, against a plane, `u`, `v` and `n`
that plane's own axes and its normal, measured from its origin — its frame's: `v` is the plane's
up, square to `u` within it, which its `v` axis need not be, so an axis written outright is read
along itself. Its zero is a relation, **`a
level(t) b`**: the two have the same ordinate along `t`, one row, no number and no callout; the
direction stands in its parentheses as `symmetry`'s line does (`level(up)`, `level(std.z)`,
`p level(u) P`). **`a horizontal b` is `a level(up) b`** and `a vertical b` is `a level(right) b`;
both spellings are read. An ordinate written as a literal zero is **E040**, naming the level it
is; a direction written as a plane (`along: std.top`) is **E040**, since along a plane reads as
within it; and two points of one view along that view's normal are **E061**, the row being
identically nothing. One statement for what 0.45 said seven ways (§9.2, §9.3).

**[0.45] A vector is written `(a, b)` or `(a, b, c)`, and an entity's numbers are its
members.** A point *is* its place, so it is held and seeded whole by a vector — `fix((0, 0)) p`,
`p := point hint((3, 4))` — or by one of its own `x`, `y`, `z` (`fix(x == 3) p`, `hint(y: 7)`).
An axis has two vectors, `dir` (its direction) and `origin` (its point nearest the world origin),
and a plane one, `origin` (where it stands): `fix(dir == (1, 0, 0), origin == (0, 0, 0)) t`,
`back := plane hint(origin: (0, 0, 12))`, and a member of one is `dir.x`, `origin.z`. A vector
held whole is as long as the vector it holds — a point in a plane two, one in space three, `dir`
and `origin` three — so `fix((0, 0)) p` cannot leave a point in space's height free unsaid
(**E105**). `px`, `py`, `pz` and an axis's or a plane's bare `x`, `y`, `z` keys are gone. A vector
may be named, `o := (10mm, 20mm)`, and is read by its members, `o.x`; a member it has not (`o.z`)
is **E103**. A grouped expression, `(w + 2) / 3`, holds no comma and is no vector.

**[0.44] A line lies on an axis** (`l coincident t`, either way round: both ends on it, four rows,
placing the axis). `parallel` between a line and an axis was already the direction relation in
space.

**[0.44] Two shorthands.** `@` is `coincident` and `~` is `hint`: `p @ c` is `p coincident c`, and
`p := point ~((0, 0))` is `p := point hint((0, 0))`. Each is read as the word it stands
for and is that word everywhere the word may stand — an infix `@(t == 0.4)` pins as
`coincident(t == 0.4)` does, and a number inside `~(…)` is a seed (§4.3). Neither is a name. A
solve writes its seeds back inside the clause as written, so `~` stays `~`. The unimplemented path
grammar's arc segment, `a ~C~ b` (§10), loses its delimiter and wants another spelling when it
lands.

**[0.43] A plane's axes pass through its origin, and a bare plane is free.** `P.u` and `P.v` are
lines through `P.origin`, so where a plane's axes stand, the plane stands: `plane(u: std.x, v:
std.z)` is the front plane at the world origin, and a plane may no longer share another's axes
while standing off it — it takes its own, held or `parallel` (§6.7, §6.10). `P := plane` with no
slot written mints both axes, `P.u` and `P.v`, free: seven freedoms, three of place and two of
direction each; a slot left out of `plane(u: r)` is minted alone. **`a coincident b` between two
axes** says they are one line, either way round (four rows), so `u coincident P.u` places a plane
by its axes. An axis's place is held as its direction is, by `px`, `py`, `pz` — the point on it
nearest the world origin: `fix(dir == (1, 0, 0), origin == (0, 0, 0)) x`, which is how
`std`'s axes are held. A plane over two held axes stands where they meet; held axes that miss,
that run alike, or a plane held off its held axes, are **E067**. A drawn line serving as a
plane's axis is the line — parallel to it and through its start, or through the end it shares
with the plane's other line, where the plane then stands. **`P parallel Q`** says two planes face alike, either way round — their
normals parallel, two rows — and nothing of where either stands or how it turns within itself. A
child slot of a plane takes a seed, the direction its own axis starts from: `plane(u: hint(x: 0,
y: 1, z: 0))`.

**[0.42] A point stands in space, and a plane is two axes.** A point drawn in no plane stands in
space, with three coordinates — `hint((1, 2, 3))`, `fix((1, 2, 3)) p` — and a
2D drawing is drawn in a plane: `use std`, then `in std.front { … }`. There is no page.
`std.front` is the plane at the world origin with u = x to the right and v = z up, its viewer at
−y. A plane is `P := plane(u: r1, v: r2)` over two axes, a drawn line serving as the axis from its
`p1` toward its `p2`: right is `u`, out of the plane is `u × v`, and up is out × u, so `v` says
only which plane and which side is up (§6.7). The plane owns where it stands, three unknowns that
`fix(origin == (0, 0, 0)) P` holds, and it floats until a relation places it: `P.origin
coincident p`, `P coincident p`, `Q distance(d) P`. `P.u` and `P.v` are its axes and `P.origin` a
point drawn in it at (0, 0). A point drawn in a plane has that plane's own coordinates, and
`hint(at: P, (3, 4))` seeds a point at a place in them. A plane carries no place on paper:
the `.svd` puts each view on the sheet (`sketch top(m) from m.top at (X, Y)`). Retired, and
refused where written: `plane(origin:, toward:)`, `from:`, `fold:` (an angle, an unknown or
`along l`), `offset:`, `through:`, `attitude: free`, a basis written as `u:`/`v:`/`o:` triples,
`hint(u:, v:, offset:, fold:)` on a plane, `against` with its placed planes and **E083**,
`std.ThreeViews`, and the datum's rotor `.c`, `.s`, `.angle`. The role rule, the held page
placement and every refusal of "the page, which has no place in space" go with them. A circle, an
arc or a spline over a point in space is **E060**, and a face over one **E080**; two planes lying
on one another are permitted and said (**W113**).

**[0.42] An axis is a directed line in space.** `t := axis hint(dir: (0, 0, 1))` declares one:
a direction and a place, with no start, drawn in no view (§3.1). It is placed by the relations
every entity takes — `t parallel s`, `t perpendicular s` and `t angle(θ) s` against an axis or a
line, `p coincident t` for a point on it — and a direction is held by
`fix(dir == (0, 0, 1)) t`. An axis read only as a direction has two freedoms; a relation that
reads where it is gives it two more. An angle in space of 0° or 180° is **E040**, by value: it is
`parallel` read by a cosine that does not move there, and `parallel` with the seed picking the
sense is the regular statement.

**[0.42] Incidence is `coincident`.** `on` is retired: `coincident` relates two points, and a
point or a line to whatever `on` related it to — a line, a circle or arc, a spline, a curve, a
plane, a sphere, a cone, a cylinder — and a circle to a sphere (§9.2). What it means is the kinds
of its operands, as before; one word fewer, and no idea spelled twice. `on(t == 20)` is
`coincident(t == 20)`. (*The sphere, cone and cylinder are sets in 0.49, §6.21.*)

**[0.41] An unknown is declared.** `param` marks one of the document's inputs (§6.3): `param bore:
Length := 50mm` is a value a host may give another, and `param beta: Angle hint(30deg)`, with no
value, is an unknown of the solve — its type stated, its seed its own `hint(…)` — as a component's
unbound formal is. A name nothing declares is **E101** wherever it is read (a dimension, a fold, a
pin), where it was a free variable warned W111: a misspelling is no longer a degree of freedom. A
dimension's number defines nothing (named dimensions, `distance(w := 60)`, are gone: `w := 60`,
then `distance(w)`), and no expression defines a name. A fold over an unknown is seeded by the
unknown (`hint(fold: …)` is refused), a shared place (`t == s`) by `s`'s declaration, and a call
seeds a formal it leaves unbound with `hint(…)` in its place: `Wing(f, beta: hint(15deg))`.
(*The fold is withdrawn in 0.42: an angle between planes is an angle between axes.*)

**[0.40] What a pocket encloses at every pose, and a claim over a motion.** A curve that comes back
to where it started after whole turns (a Wankel bore, the apex's envelope over three) is closed,
and stands alone in a face as a circle does (§6.8). A body cut by a pocketed prism swept over a
whole period of a planar motion keeps what the pocket encloses at every pose — its inner envelope
carried through the prism: a rotor is its blank less its housing turned about it. `claim over
MOTION in (A, B) { … }` judges its claims at the poses of a motion's roll (§9.8).

**[0.39] Two contacts may share their place.** A pin to a name nothing defines — `path tangent(t ==
s) ground`, `path curvature(t == s) osc` — makes the contact's parameter that unknown, and every
contact on the same curve pinned to the same name owns it: a tangency and a curvature stated at one
place are one contact (§6.5, §9.2).

**[0.38] A prism's side generating in its view is a surface of the drawing.** `envelope(side,
under: m, …)` over `side := surface(prism, edge: e)` — a prism's side, which `surface` now names
beside a revolution's — under a motion keeping the prism's view is the planar envelope of `e`
extruded square to the view; it is built with the drawing, and a point drawn in any view is `on` it
by its place in the prism's view (§6.15.2).

**[0.37] Generation by motion in the plane.** `envelope(tool, under: m, from: a, to: b)` over a
tool of the sheet and a planar motion (`motion(about: point, …)` is a turn in a view) is a curve of
the drawing: the profile the tool cuts, which takes point, tangent and curvature contacts with the
tool and the motion's geometry solved for (§6.15.1). A locus supplies `C''` and `C'''` exactly, as
the implicit function's Taylor orders (§6.5), so `e curvature k` is stated against a traced curve.
A numeric formal a drawn instance leaves unbound is a column of its curve; a curve over a whole turn
that comes back is closed, and a contact wraps across its seam. A dimension reads geometry only in
a traced body.

**[0.36] A group's member may be a group in braces.** `design := {bar: {at: o, size: {length:
2cm}}}` nests groups in place (§8.1): `design.bar.size.length` reads the number, and
`design.bar` is a group a call may be given. A brace after a member's `:` opens a group too;
braces are not an argument in a call.

**[0.35] A group is its members in braces.** `dims := {width: 20mm, origin: o}` is a group (§8.1);
`group(…)` is retired. A group's braces hold a list, not a body: like an argument list it may run
across lines, and a brace is a group's exactly when it stands straight after `:=`.

**[0.34] A `fix` states what it holds.** `fix((0, 0)) p` holds a point at the numbers it
states, each pinned with `==` under the field it is, as any pin in a relation's parentheses is;
`fix(x == 0) p` holds one coordinate and leaves the other free, and `fix(r == 25) c` holds a
circle's radius. `ground` is retired, and so are `fix c.r` and a bare `fix p`, which held a number
at its *seed*: the one place a number in a `hint(…)` clause was not a seed (§4.3). A `fix` is still
a gauge, applied rather than solved for (§13), and is applied before the seeds that read geometry,
so a place read off a held point is where it is held (§6.4).

**[0.33] A place is a step from a point.** `hint(at: a, toward: b, by: f, turn: θ)` seeds a
point the fraction `f` (1 if unsaid) of the way from point `a` to point `b`, that step turned `θ`
about `a`; `along: l` in place of `toward:` steps by `f` times line `l`'s run. A midpoint, a
reflection, an extension or a quarter turn is one clause, where it had been coordinate arithmetic
on other seeds. A place drawn in another view than the point it seeds is read where it stands in
space and projected into that point's view, which is what `project` says of the pair (§6.4).

**[0.32] The body rule's union is `union`.** `boss union body` puts material on a body, beside
`cut` and `bound`; it is read by the word, as they are. `on` is a constraint word only, and between
two solids it is refused as any word is over operands it does not relate. 0.18–0.31 spelled the
union `on`, read as the body rule when both operands were solids, which put one word to two uses
whose only difference was the kinds of the names around it.

**[0.31] A swept solid takes features.** `hub on plate`, `bore cut plate` and `tip bound plate`
may name a swept solid as well as a body: the solid is then the body whose stock is its own sweep,
and its name means the whole object wherever it is read — a view, a `through:` extent, another
body's operand. Its faces keep their names (`plate.near`), and a feature's are reached through it
(`plate.hub.near`), as a body's are (§6.9). `body := solid(plate)` remains the way to name the
object and the primitive apart, and is no longer required to put a feature on a sweep.

**[0.30] Nothing is imported bare.** A used module's component, param or group is written with
the module's full path — `engine.parts.Crank(…)`, `hardware.nut14_af`,
`components.dims.vtwin_dims` — and only through a `use` the file wrote itself (§14.4). Two
modules may define one name.

**[0.29] One way to define a name.** `NAME := VALUE` is the only form that puts a name in scope, and
`(NAME := VALUE)` is the value itself, so a name may stand where its value does (§5). `w := 100` is
a param (§6.3); `c := circle(center: o)` a declaration (§6); `t := Tooth(…)` an instance (§8); `dims
:= {…}` a group (§8.1) **[0.35]**; `profile := (ab := line(a, b)) -> line -> close` a chain whose
first link is named in place (§6.6), since `:=` binds looser than `->`; `k := leg.toe over u in (a,
b)` a curve (§6.5); `p := point(x: e, y: e)` a computed point (§6.5). **[0.41]** A dimension's
number no longer defines a name (`a distance(w := 60) b` is gone), and `param` returns as a modifier
marking an input (§6.3). `param w = 100`, `group` before a name, `NAME: Component(…)`, `NAME =
CHAIN`, `curve NAME = …`, `point NAME = (…)` and a name after an element keyword are gone, and a
lone `=` is no token. A `label:` never defines a name: it fills a slot of what is being called or
declared — an argument, a child, a `hint(…)` key, a group's member, a formal. Modifiers stand before
the name (`private construction l := line(…)`); a prefix word's value is its operand, so `l :=
horizontal line(a, b)` names the line. `component Name(…) { … }` and the binders `as i`, `e in
CHAIN`, `over u in (…)` are unchanged.

**[0.28] A face runs along a stretch of a curve, and a loft pairs curved edges by their own
parameter.** `tooth := face(root, flank from p to q, tip)` bounds a face by the stretch of the curve
`flank` between two points held on it by their contacts (`p on flank`), so a traced or formula curve
stands in a loop (§6.8); a lone edge closed by `-> close` is a loop with its chord. A loft pairs a
spline or a stretch of a curve with the edge written in its place at the same fraction of its own
parameter, as it pairs an arc by angle (§6.9).

**[0.27] Equal angles, an arc's length, iteration over a chain, and measurements after the
solve.** `l1 angle(l3, l4) l2` states the angle from `l1` to `l2` equal to the angle from `l3` to
`l4`, with no number, and `length(L) a` an arc's length along itself (§9.3, §9.4). `repeat e in
CHAIN { … }` and `cycle e in CHAIN { … }` make a copy per edge of a named chain (§12.8). A
motion's `ratio:`, `phase:` and `advance:` may measure the solved drawing — `length(l)`,
`radius(c)`, `distance(a, b)`, `angle(l1, l2)` — and a measurement anywhere a number is needed
before the solve is refused (§6.14).

**[0.26] Cones and cylinders, `against` between solved views, and `o:`.** `cone k(axis: l)
hint(half: 30deg)` and `cylinder c(axis: l) hint(r: 10)` are surfaces in space built about a line
already drawn in a view, each owning one number (§3.1): a cone's apex is its axis's start. They
take `angle` (a cone's half-angle) and `radius` (a cylinder's), `p on k` and `p on c`, `c tangent
l` (a line touching a cylinder) and `k1 tangent(M) k2` (two cones with one tangent plane at M),
all in space (§9.2, §9.3). `F against G` with a solved view is a placement where the two faces'
planes turn together — one derived from the other, or both from one, with no fold — and a row
of the solve where the datum's offset is solved; two views that turn apart are E066 (§6.10). A
basis given outright may say where it stands: `u: (…), v: (…), o: (…)` (§6.7). (*`against`, E066
and the written basis are withdrawn in 0.42. The cone and the cylinder are library sets in 0.49
(§6.21), their numbers read by the instance's name; `k1 tangent(M) k2` is `k1 tangent(at: M) k2`
in 0.51.*)

**[0.25] A circle on a sphere, and the midpoint and the mirror in space.** `c on s` puts every
point of a circle (or an arc's circle) drawn in a view on a sphere: the sphere's centre on the
circle's axis and `√(‖S − C‖² + r²) = R` (§9.3). `s tangent c` is refused, since a circle and a
sphere may touch at a point or all the way round. `midpoint` and `symmetry` across views are the
midpoint of a line in space and the half turn about a line in space — on a page, the mirror.
(*The sphere is a library set in 0.49 (§6.21); a circle on one is `std.CircleOnSphere`.*)

**[0.24] A relation across views is a relation in space.** The same words, with no selector:
`gax angle(90deg) pax` between lines drawn in two views is the angle between them in space, and
`gax distance(17.5) pax` their common-perpendicular distance (§9.2). A plane's own datum points
are read by their **role** — beside other datum points they are sheet layout, beside a view's
points they are that view's — so no statement written before this changes meaning. A word with
no meaning in space across views is **E062**; `sense:` and `side:` there are E040. `p on P`,
`l on P` and `p distance(d, along: n) P` relate a point or a line to a plane in space whatever
view it is drawn in. A **sphere** (`s := sphere(center: p) hint(r: …)`, §3.1) takes `radius`, `on`
and `tangent` in space. A solved view's place on the sheet is held silently (§6.7). (*The role
rule and the held place are withdrawn in 0.42: a plane has no place on the sheet. The sphere entity
is withdrawn in 0.49: `std.Sphere` is a set, §6.21.*)

**[0.23] A view may be a workplane solved for.** A plane is fixed unless its brackets name an
unknown (§6.7): `fold: beta` over a name nothing defines solves the fold, `fold: along l` folds
the view square to its parent about a line drawn there, `attitude: free` and `offset: free` are
three and one unknowns, and `through: M` solves the offset that puts `M` in the view. Their
seeds are `hint(fold:, u:, v:, offset:)`, and a seed for a stated quantity is E040. A point drawn
in a view is that view's lift of it, and `project` between views either of which is solved is
the projector rule in space. New codes E064–E066 (§16.1). (*Withdrawn in 0.42, E064 and E066 with
it: a plane is two axes and a place, and is solved for whenever they are.*)

**[0.22] The body rule gains its third side.** `tip bound body` keeps of a body what lies
within `tip`: a solid is its stock, plus everything `on` it, minus everything that `cut`s it,
within everything that `bound`s it (§6.9). Union comes first; difference and intersection commute,
so the `cut` and `bound` sets need no order between them. Like `on` and `cut` it is
Declaration-class and contributes no residual. It replaces the `A − (A − B)` idiom through a
named intermediate, which was the only spelling of an intersection until now.

**[0.21] Explicit datum coordinates.** Signed ordinates use the existing distance operator:
`p distance(u, along: u) f` and `p distance(v, along: v) f`, where `f` is a plane used
as a datum. They are ordinary constraints in the shared solve. Components acquire no implicit
origin, orientation, or geometric result. Plane membership remains independent (`inst := Part(f)
in view`). The standard library has no coordinate-placement component; profiles are expressed
through geometric relationships, with datum ordinates used for design measurements.

**[0.20] Model/presentation split.** `.sv` specifies geometry, constraints, hints, and
assertions. All presentation belongs in [Solvent Drawing (`.svd`)](docs/solvent-drawing.md).
This supersedes earlier presentation clauses and examples, including `class`, `style`,
`view`, `section`, `dimensions`, and callout `at (t, r)` in model source. Components remain
scoped collections of statements and have no default geometric result.

*Draft 0.2 amends 0.1 in seven places, from implementing 0.1 end to end (bmander/geomsolver#2).
Marked **[0.2]** where they appear. In summary: the `=` / `==` seed mark (§4.3) that makes
Invariant H checkable by looking; seeds written inline (§6.4, §11); document state attached to its
statement, which is what makes P2 true rather than merely asserted (§13.1); the identity of a
statement under expansion (§12.7); curve families as a document declaration, promoted out of §17
(§6.5); a reporting duty on an implementation that unrolls a `ring` (§12.3); and `Line` cut back to
a segment whose infinite carrier is what constraints read (§3.1).*

*"Solvent" is a placeholder name; nothing in this document depends on it.*

---

## 1. Overview

Solvent is a declarative language for describing geometry under constraint. A Solvent program does not construct geometry step by step; it declares a set of entities, relations among them, and structural facts (symmetry, connectivity), and delegates the discovery of coordinates to a solver. The language borrows its module discipline from hardware description languages: designs are built from **components**, component bodies are **unordered**, and connection between components is **aliasing** (net formation), not constraint.

### 1.1 Design principles (normative)

These principles are binding on the rest of the specification. Where a detailed rule appears to conflict with a principle, the principle governs and the conflict is a defect in this document.

**P1 — Connection is aliasing; constraint is explicit.**
Passing an entity as an instance argument makes two names refer to *one* entity **[0.13]** (a port once did too; §7). Aliasing has zero solver cost and cannot be violated. A constraint always relates *distinct* entities and always contributes residual equations (or inequalities) to the system.

**P2 — Component bodies are unordered.**
Statements within a component body form a set. Reordering the statements of a body MUST NOT change the meaning of a program. Statements interact only through the entities they name. The only ordered contexts in the language are the interiors of single statements where order is semantic (path traversals, argument lists).

**[0.2]** P2 binds the *document* as well as the program text: every piece of state a document carries MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index (§13.1). Without that rule P2 is an assertion an implementation can satisfy in the parser and lose in the file format — and it fails silently, which is the worst way for it to fail.

**P3 — Hints are semantically inert.**
Deleting every seed from a program MUST NOT change its solution set. Seeds may change which solution a solver finds, or whether it converges, but never what counts as a solution. Any annotation that changes the solution set (orientation predicates, arc branch selection, tangency side) is a **constraint** and is classified as such by the implementation, regardless of surface syntax.

**[0.2]** The two are told apart by one mark, not by analysis: a number written with `=` is a seed and a solver may rewrite it (**[0.7]** the mark is now the `hint(…)` clause: a number inside one is a seed, §4.3); a number written with `==` is a constraint and a solver MUST NOT (§4.3). This is what makes Invariant H (§11) checkable *syntactically*, as that section already requires.

**P4 — Component boundaries are decomposition structure.**
A component is solvable against the entities it is written over. Implementations SHOULD exploit the component instance tree as a decomposition plan (solve interiors against the formals; solve the inter-component system over them).

**P5 — Symmetry is a claim, not a macro.**
The `ring` construct asserts cyclic symmetry. Its solution set contains exactly the symmetric solutions of the corresponding unrolled system. Implementations SHOULD solve in the fundamental domain, and one that does not MUST say so (§12.3).

### 1.2 Scope of this draft

**[0.1]** This draft specified **planar (2D) geometry only**. The entity vocabulary, constraint library, and group actions are two-dimensional. Section 17 lists the known lifting questions for 3D. This draft also excludes curve entities beyond lines and circles (no involutes, splines, or conics); see §17. **[0.2]** Curves are settled in §6.5 and splines in §6.1; **[0.42]** points, axes and planes stand in space (§6.7).

**[0.18] More precisely: the draft specifies planar geometry *solved*, with solids evaluated over it.** A document may say what object its drawing is of — a face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), and a view or a section of one is a picture the sheet asks for (§6.11) — and **nothing past the sketch is ever an unknown**. **[0.42]** The sketch reaches into space: a point may stand there, and a plane and an axis (§3.1, §6.7) are unknowns of the solve, solved with the rest of the sketch before anything below reads them. An extent is an expression (§6.9), and neither a face nor a solid owns a parameter, appears in a residual, or is reached by a constraint. The strata run one way and there is no edge back: the sketch solves, the extents are worked out, the terms are ordered, the outputs are read. So everything §15 says about a solver, and every count in §16.3's ledger, is unchanged by the presence of an object — which is the whole of what makes the addition affordable.

### 1.3 Conformance keywords

MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are used as in RFC 2119. Text marked *non-normative* is explanatory.

---

## 2. Lexical structure

- **Identifiers:** `[A-Za-z_][A-Za-z0-9_]*`. Component names are conventionally capitalized; this is not enforced.
- **Keywords:** `component`, `param`, `point`, `circle`, `line`, `frame`, `path`, `repeat`, `cycle`, `ring`, `about`, `as`, `next`, `prev`, `hint`, `at`, `fix`, `ccw`, `cw`, `rev`, `true`, `false`, **[0.2]** `curve`, `over`, `spline` (and `ellipse`, until **[0.15]** made the ellipse a library component — `Ellipse` in `std`, a computed point on a datum traced as a curve, whose contacts are the curve's; an implementation keeps the word only to refuse it). **[0.7]** `unit`, `class` and `style` in, `construction` out; every constraint is a prefix or an infix operator (§9.2), so `coincident`, `equal`, `tangent`, `curvature`, `symmetry` and `distance` are the words a statement is written with — it is a class now, and the base sheet is what draws it dashed (§13.2). **[0.4]** In a chain (§6.6) the word `close` is meaningful *contextually*; it is not reserved, and an entity may bear it as a name. **[0.19]** It is read the same way after `->` inside a `face`'s brackets (§6.8), which is the other place the language draws a loop. **[0.8]** `to` is retired: the plain corner is the `->` marker, and threading is stated at the joint rather than inferred from the operands. **[0.5]** A coordinate seed is written `hint at` (§6.4). **[0.7]** Every seed is written in one `hint(…)` clause (§4.3, §6.4); `hint at REF` kept its own form inside a trace block (§6.5.1) until **[0.14]**, when a place became the `at:` and `bearing:` keys of the same clause — `hint(at: REF, bearing: β)` — so `at` after `hint` is refused, and `bearing` is a key and no keyword. **[0.10]** `plane`, `in`, `project` and `fold` in (§6.7); `from` is contextual there as it is in a trace family. **[0.42]** `axis` in; `frame`, `fold`, and `from`, `offset` and `through` as a plane's labels, out — a plane's brackets take `u:` and `v:` and nothing else. **[0.13]** `port` is retired (§7); an implementation keeps the word only to refuse it. **[0.18]** `face` and `solid` are element keywords (§6.8, §6.9) and `view` and `section` open a statement (§6.11); `cut` is the body rule's own word and `on` gains a reading over two solids (§9.2), so both join the operator words a name may not be. **[0.32]** `union` is the body rule's too, and joins them; `on` loses its reading over two solids. **[0.42]** `on` is no constraint word: `coincident` says incidence (§9.2). **[0.20]** `view` and `section` leave model source for the drawing (§6.11). The eight labels a solid's brackets take — `from`, `to`, `depth`, `through`, `along`, `about`, `sweep`, `sense` — are **contextual**: they are read as labels inside the brackets that take them and are reserved nowhere, so a `param` or a point may still bear any of them as a name (`face := -(fw + D / 2)` is idiomatic). A declaration's *name*, however, may not be an element keyword, and three shipped examples renamed a line that had been called `face`.
- **Literals:** decimal numbers with optional unit suffix (`10`, `2.5mm`, `30deg`). The constant `tau` (= 2π) and `pi` are predefined.
- **Comments:** `//` to end of line; `/* ... */` nesting not required.
- **Operators and punctuation:** `== + - * / ( ) { } [ ] , : . := -> ~ @`
- **Shorthands [0.44]:** `@` is the word `coincident` and `~` the word `hint`, read as those words wherever they may stand.
- Whitespace and newlines are insignificant except as token separators. Statements are newline- or `;`-terminated; implementations MUST accept either.

---

## 3. Types and dimensions

### 3.1 Value types

| Type | Meaning | Free DOF when unconstrained |
|---|---|---|
| `Int` | compile-time integer (parameters, counts, indices) | — (elaboration-time) |
| `Scalar` | dimensionless real | 1 |
| `group` | named bundle of numeric values, geometry aliases, or nested groups (§8.1) | 0 of its own |
| `Length` | real with length dimension | 1 |
| `Angle` | real with angle dimension | 1 |
| `Point` | a position in the plane it is drawn in, or **[0.42]** in space where it is drawn in none | 2 in a plane; 3 in space |
| `Line` | segment between two `Point`s; its infinite carrier is what constraints read **[0.2]** | 0 of its own (4 through its ends) |
| `Circle` | center + radius | 3 |
| `Arc` | center + radius, its start and end on that circle | 5 |
| `Axis` | a directed line in space with no start, drawn in no view **[0.42]** | 2 (its direction), and 2 more once a relation reads where it is |
| `Plane` | **[0.42]** a plane in space: its attitude two axes `u`, `v` through its origin **[0.43]**, and where it stands; a **view**, the points drawn in it having its own coordinates (§6.7) | 3 of its own (where it stands), beyond its axes'; a bare `plane` is 7 with the axes it mints **[0.43]** |
| `Path` | directed piecewise boundary curve | 0 (derived object) |
| `Face` | a planar profile (§6.8) or an ordered boundary on an explicit spatial support (§6.20) | 0 — it owns no parameter |
| `Solid` | a face swept, or a term over other solids **[0.18]** | 0 — it owns no parameter |
| `Surface` | an analytic boundary patch read from a solid (§6.13) | 0 — it owns no parameter |
| `Motion` | a rigid-motion family about solved geometry (§6.14) | 0 — it owns no parameter |
| `Envelope` | the zero-normal-velocity locus of a moving surface (§6.15) | 0 — it owns no parameter |
| `Patch` | a spatial region of a surface or envelope, selected by material sides (§6.16) | 0 — it owns no parameter |
| `Seam` | a shared generating characteristic or envelope/boundary intersection (§6.17) | 0 — it owns no parameter |
| `Vertex` | a named spatial corner at two seams (§6.18) | 0 — it owns no parameter |
| `Edge` | a finite seam extent between named spatial vertices (§6.19) | 0 — it owns no parameter |

**`Face`, `Solid`, `Surface`, `Motion`, `Envelope`, `Patch`, `Seam`, `Vertex` and `Edge` are evaluated after the solve.** None owns a coordinate, allocates an unknown, or contributes a planar residual; none may be an argument of a planar constraint of §9.3, be dragged, be dimensioned, or be put on a plane with `in`. What each *is* is settled once the drawing has been solved and every extent worked out (§1.2, §6.9). A kind of this stratum therefore adds nothing to §16.3's ledger.

**[0.2] `Line` was a 2-DOF undirected infinite line with a `.dir` field in 0.1.** It is now a segment between two points, and every constraint that reads a line reads the infinite carrier through those points — which is what `parallel`, `perpendicular`, `coincident` and `angle` mean by a line anyway.

The change is a concession to cost. A 2-DOF line is a second representation of a line alongside the one every constraint already works in, so it needs its own column layout and its own kernel for roughly fourteen constraint types, and the whole return is that §16.3's ledger comes out differently. Where a drawing wants a line with no ends — a datum, an axis — it says so with a construction segment, or **[0.42]** in space with an `axis` (§6.7), which has none.

### 3.2 Sub-entities

Compound entities expose sub-entities by field access. Sub-entities are ordinary entities and participate in aliasing and constraints.

| Entity | Field | Type |
|---|---|---|
| `Circle` | `.center` | `Point` |
| `Circle` | `.r` | `Length` |
| `Arc` | `.center`, `.start`, `.end` | `Point` |
| `Arc` | `.r` | `Length` |
| `Axis` | `.x`, `.y`, `.z` | its unit direction — seeded by `hint(dir: (…, …, …))` (normalised) and held by `fix(dir == (…, …, …))` **[0.42]** **[0.45]** |
| `Axis` | `.px`, `.py`, `.pz` | `Length` — where it stands: its point nearest the world origin, held by `fix(origin == (…, …, …))` once a relation reads it **[0.43]** **[0.45]** |
| `Plane` | `.u`, `.v` | `Axis` — its axes, through its origin **[0.42]**; minted where the slot is not written (`P.u`, `P.v`) **[0.43]** |
| `Plane` | `.origin` | `Point` — drawn in the plane and held at its `(0, 0)` **[0.42]** |
| `Plane` | `.x`, `.y`, `.z` | `Length` — where it stands in space: read by a seed, seeded by `hint(origin: (…, …, …))` and held by `fix(origin == (…, …, …))` **[0.42]** **[0.45]** |
| `Line` | `.p1`, `.p2` | `Point` **[0.2]** |
| `Face` | its edges; `on:` for a spatial support | aliased boundaries; `face.on` reaches a spatial support (§6.20) |
| `Solid` | `.near`, `.far` | a prism's two caps, at the higher and the lower ordinate **[0.18]** |
| `Solid` | `.<edge>` | the side swept from the edge the source called `<edge>` **[0.18]** |
| `Solid` | `.start`, `.end` | a partial revolution's two caps **[0.18]** |
| `Solid` | `.volume`, `.area`, `.bounds.x0`… | what a report measures off it (§16.3) **[0.18]** |

**[0.18] A solid's derived names are *paths*, and a path is not a sub-entity.** A face of a solid carries no coordinates, joins no alias class and takes no constraint: the path is what a report writes a number under (`block.side_l.area`) and what a derived picture labels a stroke with. A guided loft/sweep has `.start` and `.end` caps and walls named by the start section’s source edges, including hole boundaries. Through a body the operand keeps its own name, so a face of `block` inside `body` is `body.block.near` — a path never renames, which is the naming problem every history-based kernel has and this one does not, because a boolean cannot renumber a name. The *sub-entities* a solid does have are the ones its declaration is written over: the face it is swept from, a revolution's axis line, and the solids of its term. Those are ordinary entities, and deleting one takes the solid with it.

**[0.42] A plane is two axes and a place.** `P := plane(u: r1, v: r2)` aliases two axes (or
drawn lines, each read as the axis from its `p1` toward its `p2`) and owns three numbers of its
own, where it stands. Its attitude is its axes': right is `u`, out of the plane is `u × v`, toward
its viewer, and up is out × u, so `v` need not be square to `u` — it says which plane, and which
side is up. **[0.43]** The axes pass through its origin, so two planes share an axis only where
both stand on it. No plane has a rotor, an angle or a `toward`: 0.6's datum `plane(origin: o,
toward: q)`, its intrinsic unit rotor `(c, s)` and the derived `.angle` are retired, and so is
0.15's reading of a plane with no attitude written as the page's, and 0.15's `frame`, which names
nothing now.

### 3.3 Dimensional analysis and units **[0.7]**

Implementations **MUST** check dimensions in expressions and constraints, and MUST NOT silently coerce `Angle` to `Scalar` or vice versa. Angle arithmetic is mod 2π where the operand is a bearing and signed where it is a turn; see §9.4. *(SHOULD in 0.1–0.6, and never implemented; MUST from 0.7.)*

**Two base dimensions**, because two is what the language has: a **length** and an **angle**. A quantity is a rational power of each — rational, because `sqrt` halves one and `sqrt(area)` is a length.

- `*` and `/` **derive**: the exponents add and subtract.
- `+` and `-` **demand agreement**, and so do `min`, `max`, `hypot` and `atan2`'s two arguments.
- `^` takes a plain number, and a **dimensioned base takes a whole power**: `x ^ 2.5` where `x` is a length is not a dimension anybody meant, and `sqrt` is how a half is written.
- The dimension an expression comes to is checked against the **slot** it is written in.

**A bare number is dimensionless, and a *context* may take one.** That is what "drawing units" means: `a distance(80) b` is a length because the slot says so, and `sin(30)` reads 30 as degrees because the function does. A context may **not** speak for a second operand: `90 / N + ivp` is a plain number added to an angle, and an implementation MUST report it rather than choose. The asymmetry is the design — a slot and a function say what they want; two operands are not a context, and neither of them is authoritative.

A **name** is worth a number, and where that number is *used* decides what it is: `w := 80` in a `Length` slot does not make `w` a length, since the same 80 may be a run, a rise or an angle. A unit on the literal (`w := 80mm`) says otherwise, and so does a component formal's declared type (§8) — which is what catches `x := w + phi`.

**[0.16] One namespace for a number's names; [0.41] an unknown is declared.** A number is named one way, `w := 60` — `param w := 60` when it is one of the document's inputs — and a dimension's number reads names and defines none. A name resolves by one rule (§5): a definition declares its name in the body it is written in, and a second definition of a name in one body is **E001**. An **unknown** — a number the solve answers for — is declared too: an input with no value (`param beta: Angle`, §6.3) or a formal no call binds (§8). A name nothing in scope declares is **E101**, wherever it is read; it is never made an unknown of its own accord, which turned a misspelling into a degree of freedom (0.16–0.39 called it a free variable and warned, W111). A value reading an unknown is that unknown scaled (`q := 2 * beta`), carried as the affine form it is.

| function | |
|---|---|
| `sin`, `cos`, `tan` | `Angle → Scalar` |
| `asin`, `acos`, `atan` | `Scalar → Angle` |
| `atan2` | `(D, D) → Angle`, arguments agreeing |
| `sqrt` | `D → D^½` |
| `abs`, `min`, `max`, `hypot` | `D → D`, arguments agreeing |
| `exp`, `ln`, `log` | `Scalar → Scalar` |
| `floor`, `ceil`, `round` | `Scalar → Scalar` |

`floor`, `ceil` and `round` are `Scalar`-only **deliberately**: rounding a dimensioned quantity depends on which unit you round in, and a language that silently picked one would be wrong half the time.

#### 3.3.1 The literal

```
80mm     3.5in     45deg     0.5rad     12
1' 6 3/16"
```

A number MAY carry a unit. The length units are `mm`, `cm`, `m`, `km`, `in`, `ft`, `thou`; the angle units are `deg`, `rad`, `grad`. `'` and `"` are the foot and inch marks.

**Feet-and-inches is one literal**, and it is a rule the language already had: *a space is what tells the readings apart*, exactly as it does in a mixed fraction, where `3 1/2` is three and a half and `31/2` is a division. So `1' 6"` is one length for the same reason.

**The language therefore has no string literal.** A `"` is the inch mark and there is nothing else for one to be; every `Str` argument is written as the word it is (`at: start`), and a raw branch key is written bare (§13.1).

#### 3.3.2 `unit`

```
unit mm
```

`unit` names the document's **length** unit. A bare number in a `Length` slot is that unit, so every existing document keeps working with one added line, and a suffixed literal converts to it.

**Without a `unit` line the document is in drawing units** — a length dimension with no name. Everything still checks: `a distance(45deg) b` is still an error and `Length + Angle` is still an error. You simply cannot write `mm` or `"`, because there is nothing to convert to, and an implementation MUST report that rather than guess.

**Storing the document's unit costs an implementation nothing for lengths**, because a well-formed kernel is homogeneous in length: scale every length in a sketch by a constant and no residual, no tolerance and no rank moves. **Angles are the exception, and it is not a choice**: `cos θ` is not homogeneous, and there is no consistent unit it works in other than radians. So an angle is stored in radians and converted at the text seam, which is exactly where it converts anyway; what units remove is not the conversion but the *guess*.

Where a document may be copied into another, a paste between documents in different units **SHOULD convert**: a figure is the same figure in either, and two inches is 50.8 millimetres.

#### 3.3.3 `pi` and `tau`

`pi` is the dimensionless mathematical constant. `tau` and `turn` are a full **turn**, which is an `Angle`. They were 3.14159 and 360 side by side with nothing saying why; units settle it, and `tau == 2 * pi * 1rad` now holds dimensionally where it used to be a coincidence of digits.

A conversion written out — `* 180 / pi` — is `* 1rad`, and the `1rad` that remains is not noise: it is the fact that `inv φ = tan φ − φ` **holds only in radians**, which the formula never said and which the check now makes it say.

---

## 4. Program structure

A program is a set of component definitions. One component is designated the **root** (by tool invocation, not by language syntax); the elaborated model is the root's instance tree.

The file tools use the file's top-level statements as an anonymous root. A file containing only
component definitions has an empty root: opening it does not implicitly instantiate its last
definition. An instance statement at file scope supplies the arguments and draws that component.

**[0.21] An instance name is optional.** `Part(args)` contributes the same statements as
`part := Part(args)`, with an independent expansion for each call. Use the named form when
constraints or drawing files need to refer to its members. An unnamed instance's internal keys
MUST NOT be exposed as writable source names. Operator calls such as `ccw(a, b, c)` retain
their existing meaning.


```
component Name(p1: Type1, p2: Type2, ...) {
    <statements>
}
```

### 4.1 Parameters

Parameters are passed by name or position at instantiation. A parameter of entity type (`Point`, `Circle`, `Line`, `Plane`, **[0.42]** `Axis`) is **bound by aliasing** (P1): the formal name and the actual argument denote the same entity. A parameter of value type (`Int`, `Scalar`, `Length`, `Angle`, **[0.17]** `Side`) is a compile-time or definitional value; it contributes no unknowns. A `group` formal bundles values and geometry references (§8.1). A `Side` is one of the words `left` and `right` (§9.2) and is not a number: it may be passed on to another instance and written in a selector, and nothing else.

**[0.17] Which way is a word, not a sign.** A distance measured **from a line** — a point to a line, a line to a line — is a **magnitude**: its solution set is *both* sides, and which one a solver finds is the seed's business (P3), as it is in every other sketcher. A negative one is an error (**E040**) wherever the number comes from, including a component's argument, since the kernel cannot tell one side from the other and the minus therefore said nothing a drawing could show. A statement that must pin a side writes the word — `p distance(12, side: left) ax`, left being of the line's own `p1 → p2` — and so does a tangency (`side: left | right`, which was `side: -1`). Where a sign is *arithmetic* rather than a convention — the run and the rise, signed from the first point to the second, and the directed angle of §9.4 — it stays a sign, because a component computes it from coordinates it is given; each gains the word that says the same thing in the open (`along: right | left | up | down`, `sense: cw | ccw`), and a document SHOULD prefer it. A component takes a side as a value of the type **`Side`** (`s: Side`, `side: s`), which is a word and not a number: encoded as ±1 it would put the unreadable idiom back one level down, inside every helper.

**Labels are mandatory for numbers and sides.** A positional argument MUST bind an entity
or group formal and MUST precede every labelled argument; either violation is **E004**.
An argument MUST NOT be supplied twice. For example, `Frame(layout, dims: vtwin_dims)` passes
a layout positionally and a dimension group by label, while `Cylinder(axes, fw: 12mm,
dims: vtwin_dims)` labels the numeric face-wall thickness.

### 4.2 Statement classes

Every statement belongs to exactly one class. The classification is normative because P3 depends on it.

| Class | Statements | Affects solution set? |
|---|---|---|
| **Declaration** | entity declarations, `param`, groups (`{…}`, §8.1), curves (§6.5), instance declarations, **[0.18]** the body rule (`union`, `cut` and `bound` over two solids, §6.9) **[0.32]** | introduces entities/aliases |
| **Constraint** | relations (prefix and infix words, §9.2), `==` equations, **pinned seeds (`==`, §4.3) [0.2]**, orientation predicates, arc-branch and tangency-side decorations, `ring` symmetry, gauge statements | **yes** |
| **Seed** *(was Hint)* | the `hint` statement (§11), **and every seed written inline in a `hint(…)` clause (§4.3, §6.4) [0.2] [0.7]** | **no (P3)** |
| **Structure** | `repeat`/`cycle` blocks, `path` declarations (net of their derived constraints, §10.4), **[0.18]** `view` and `section` (§6.11), which ask for a picture and declare nothing — **[0.20]** in a `.svd` drawing, never in model source | organizational |

### 4.3 Seeds and pins: `hint(…)` and `==` **[0.2]** **[0.7]**

> **A number inside a `hint(…)` clause is a seed. Every other number is not.**

This is the only distinction between the two classes, and it is lexical on purpose. §11 requires an implementation to verify Invariant H *syntactically*; a mark you can see does that, and no analysis of what a number "is really doing" does.

```
p := point hint((0, 0))          // seed: a solve may move it
c := circle(center: o) hint(r: 25)   // seed: a solve may move the radius
a distance(80) b                     // constraint: a solve must not move it
p coincident s hint(t: 0.37)         // seed: the contact may slide along the curve
p coincident(t == 0.37) s            // constraint: the contact is pinned there
w := 100                             // neither: a number worked out while elaborating
```

**[0.7] One clause, where 0.2–0.6 had three spellings.** A seed used to be written three ways depending only on what happened to carry it — `hint at (0, 0)` for a point's coordinates, a labelled `r: 25` inside the constructor for any other scalar, `t = 0.37` for a constraint's own unknown. Three spellings for one class of thing, and the middle one was the worst: it put a number the solver will move inside the same brackets as the structure it may not, so `circle c(center: o, r: 25)` read as though the radius were as much a part of what the circle *is* as its centre.

`hint(key: value, …)` says it once, keys in any order, on declarations and on constraints alike. **The brackets after the name are what the thing is made of; the `hint(…)` after them is where the solve begins.** It also mends the headline of this section, which was only ever approximately true: `r: 25` was a seed written with `:`, and `w := 100` is not a seed at all.

The three retired spellings — `point p at (0, 0)`, `point p hint at (0, 0)`, `circle c(center: o, r: 25)` — MUST NOT parse. A document written against 0.2–0.6 does not load; the change is small, mechanical and worth doing once.

The last pair is why the rule is needed at all. A curve contact carries its own parameter, and
whether that parameter is free is not a fact about how it looks — it is a fact about the solution
set. A curve fitted through *m* points whose contacts are unpinned keeps *m* degrees of freedom and
can slide along itself; pin them and it does not. In 0.1 both forms would have been written the
same way and classified by intent.

**Consequences, all normative.**

1. A solver MUST NOT rewrite a `==` number. It MAY rewrite a number inside a `hint(…)` clause, and doing so is how a
   drawing records where it ended up (§13.1).
2. A statement's class is decidable by inspection. An implementation MUST NOT need to consult the
   solution set, the solver, or the geometry to classify one.
3. Deleting every `hint(…)` clause from a program leaves its solution set unchanged (P3). Deleting a
   `==` number generally does not.

**Non-normative.** The mark also settles seed *writeback*, which is what an implementation needs
when the drawing is edited by drawing on it rather than by typing:

> A seed is writable iff it is inside a `hint(…)` clause, is a literal and not an expression,
> and is reached by exactly one instance path.

The first clause is this section. The second keeps a radius written as a component's parameter
(`hint(r: Rr)`) from being overwritten with the number it happened to come to — the author said what it
*is*, not where it starts. The third is §12.7: thirty instances share one statement, and there is
no one pose to record.

---

## 5. Names, scope, and resolution

- The scope of a name is the entire component body in which it is declared (P2). Forward reference is legal and idiomatic.
- **[0.29] A name is defined by `NAME := VALUE` and in no other way** (formals and block binders aside). The value decides what the name is: a number (a value, or under `param` an input, §6.3), an element (§6), a chain joined by `->` (§6.6), an instance (§8), a group (§8.1), a curve (§6.5). `(NAME := VALUE)` is the value, so it may stand where the value may: a chain link, `(ab := line(a, b)) -> …`. **[0.41]** A constraint's number defines no name. `:=` binds loosest. An input with no value, `param beta: Angle`, declares an unknown (§3.3, §6.3). A `label:` fills a slot and defines nothing in the caller's scope.
- Redeclaration of a name within one body is an error (**E001**), whether the two are values, inputs or one of each.
- Instance members are accessed by dotted paths: `t.lead`, `g.hub.origin` — an unknown an instance left unbound included: `t.w` (**[0.41]** a dotted name nothing made is **E101**, as a bare one is).
- **Components have closed model scope.** A body reads its formals and its own declarations. It may call visible component definitions and use built-in functions and constants, but cannot capture geometry, parameters or groups from the root, an importing module, or an enclosing component. Pass those dependencies as arguments. This includes `std.front` and the other standard datums.
- **[0.41]** An undeclared numeric name is **E101**. A component's unknowns are its numeric formals a call leaves unbound, each the instance's own (`t1.w`, `t2.w`); a call may seed one by leaving it unbound with `hint(…)` in its place (`T(a, b, w: hint(60))`). A reference to a known ambient value without an argument is an error, rather than an implicit capture. Inside a repetition, a name declared by the block belongs to each copy; other names resolve in the enclosing component. `next` and `prev` belong to the lexical cycle, and can be passed to a nested component as arguments.
- Inside `repeat`/`cycle`/`ring` blocks, the index binder (`as i`) and the pseudo-instances `next` / `prev` are in scope (§12).
- There is no shadowing: a block binder that collides with an outer name is an error (**E002**).
- **[0.17] A name that shadows a built-in is said.** The constants and functions of §3.3 (`pi`, `tau`, `turn`, `sin`, `min`, …) are known to every expression before the document is, so a `param`, a formal or a block binder of one of those names does *not* shadow it: a text carrying the name is substituted and reads the declaration, a number worked out reads the built-in, and the two answers differ silently — `tau := 35deg` passed to a `tau: Angle` formal arrived as a full turn. Each declaration — a value, an input, a formal, a block binder — is a warning at the declaration (**W112**), because the drawing is not wrong, the name is. An implementation MUST say it once per declaration, whether or not the component is ever instantiated.

---

## 6. Entity declarations

### 6.1 Free declarations

```
p := point
l := line                      // makes two points: l.p1, l.p2
c := circle                    // makes one:        c.center
a := arc                       // makes three:      a.center, a.start, a.end
```

A bare declaration introduces an entity all of whose coordinates are unknowns.

**[0.7] The children come with it.** A declaration that writes no argument list gets the ones its kind is built from, *unnamed*, and they are reached by dot access: `l.p1` is an ordinary point, and it constrains, drags and is picked like any other. Half the points in a drawing are named only because a line has to be written from something; a name earns its place when something says it twice.

**The dotted path is the name.** An anonymous child has no name in the source, so `l.p1` *is* its name, and everything that identifies an entity by name MUST agree — a constraint written against it, a selection that outlives a re-elaboration, a diagnostic that reports it.

**[0.9] The name itself is optional**, independently of everything after it: `line` alone is a syntactically valid statement — a line with no name, implicit children and no hint — and so are `line(p1, p2)`, `circle hint(r: 25)` and `arc(center: c)` — a line owns no scalar of its own, so its anonymous seeded form puts the seeds in the slots, `line(hint((0, 0)), hint((60, 20)))` (§6.2). The token after the kind keyword decides what the statement says next, so a trailing-clause word (`hint`, `knots`, `class`, `at`, `close`) can no longer be a declaration's name — the same reservation element keywords and operator words carry. A curve keeps requiring a name: its form is `NAME := REF over u in (a, b)` (§6.5), and the name is what the contact constraints address. **Identity is minted on demand.** Internally the statement suffices — an anonymous element parses, elaborates, draws, drags and deletes without ever being named. The moment the *source* must reference it (a constraint applied from a tool, a dimension stated on it), the implementation MUST splice a real name into the declaration — the same bargain §6.4's writeback strikes with an unwritten `hint(…)` clause. No hidden names: a name is what the source calls the thing, and an unnamed thing has none until the source needs one. Chains are in scope (§6.6): `line -> tangent arc -> tangent line` is a fully anonymous open contour, the corner-minting rule naming shared points by the parent's dotted path.

Where an unnamed and unseeded coordinate *starts* is not the language's business (§15). It is worth saying only that the obvious answer is wrong: two implicit endpoints both at the origin is a zero-length line, with no direction for `horizontal l` to bite on and a singular row for any tangency, so an implementation MUST NOT place them coincidently.

A **list** child slot — a spline's control polygon, a curve's arguments — has no arity to conjure children from, so a bare `s := spline` remains **E103**.

**[0.53] A free curve** **[0.55]** is written with the two points it runs between, `rope := curve(a, b)`: the curve runs from `a` to `b` and its shape is the drawing's to find — what an energy over it (§9.10) settles, the solution of that energy's Euler–Lagrange equation. It has no control points: how the implementation solves the equation is its own business (§15), so long as the answer is the curve the statements describe to the accuracy it reports. It owns one number, its **length**, held by `length(L) rope` or, with none stated, where the energy is stationary in it. It is drawn in the plane its ends are in (**E060** for ends in space). A free curve no energy is stated over has no shape (**E040**). **[0.55]** `spline(a, b)` is a spline of two control points, refused as too short.

### 6.2 Constructor declarations

```
pitch := circle(center)
f0 := plane(u: spoke, v: std.y)   // [0.42]: two axes, the line `spoke` read as one
l := line(hint((0, 0)), hint((60, 20)))   // [0.7]
```

Constructor arguments have two behaviors, by type:

- An argument of **entity type** *aliases* the corresponding sub-entity (P1). `pitch := circle(center)` makes `pitch.center` and `center` one entity.
- **[0.7]** A child slot may hold a **`hint(…)`** instead of a reference: an anonymous point, and where its solve begins. It is the same clause as §4.3's, standing in for a child rather than qualifying the declaration it follows, so one construct means "where this begins" wherever it appears.

**[0.8] Every slot is a name, a seed, or implicit.** A written slot carries a reference or a `hint(…)`; a slot the list leaves out — by a label naming a later one, or by a chain's marker filling only the ends it speaks for (§6.6) — is an **implicit child**, minted exactly as a declaration that writes no list at all mints them, and reached by its dotted path. There is no bare `hint` meaning "anonymous and unseeded", because an empty slot already says it. (Through 0.7 a partial list was **E103** — "all the children, or none" — a rule the `->` marker retired: `(l1 := line) -> (l2 := line)` fills one slot of `l2` and leaves the rest the drawing's own. E103 remains the refusal for *more* children than the kind has slots.)

**[0.2] The value-type rule is reversed from 0.1**, which made a value argument definitional — `pitch := circle(center, R)` introducing `pitch.r == R` as a substitution. That cannot stand beside §4.3, and it is wrong on its own terms: a radius is a coordinate a user drags. Under 0.1's reading, taking hold of a rim and pulling would be an *edit of what the program means* rather than a move within its solution set, and every direct-manipulation gesture on a scalar would be a different kind of event from the same gesture on a point. **[0.7]** A value argument is no longer written there at all; it is §4.3's `hint(…)`.

A computed point, `p := point(x: XEXPR, y: YEXPR)`, is wholly defined by expressions and is drawn only as a curve (§6.5).

A radius that is meant to be *held* says so:

```
c := circle(center: o) hint(r: 25)   // 25 is where it starts
radius(25) c                      // 25 is what it is
fix(r == 25) c                    // 25 is what it is, without a dimension on the drawing
```

### 6.3 Values and inputs: `NAME := EXPR`, `param NAME` **[0.29]** **[0.41]**

```
R := m * N / 2                      // a value
param bore: Length := 50mm          // an input: a value a host may give another
param beta: Angle hint(30deg)       // an input nothing binds: an unknown of the solve
```

A definition whose value is a number introduces a named definitional value. Values are evaluated at elaboration time when all inputs are `Int`/literal, otherwise they are definitional scalars — affine in an unknown when they read one (`q := 2 * beta`).

**[0.41]** `param` before a definition marks one of the document's **inputs**: the numbers it is drawn from, and the one kind a host may give another value. Its type (`Length`, `Angle`, `Scalar`, `Int`) is optional where it has a value, which is then checked against it and carries it (`param w: Length := 60` is a length); a mismatch is **E103**. An input with no value is an **unknown** of the drawing, as a component's unbound formal is: it MUST state its type, which is `Length`, `Angle` or `Scalar` (a count is never an unknown) — **E040** otherwise — and its seed is its own `hint(E)`, the one keyless hint clause, which a solve writes back as it writes a point's. A seed on an input with a value is **E040**. `param` stands at the top of a document or its `preview` (a component's inputs are its formals; a block's copies share the document's — both refused where written), and a module's input MUST have a value (**E040**): a module's numbers are read by documents that do not draw it. A value reads an input as it reads any value.

A value or an input is visible throughout the body that declares it and its repetition blocks. A file may read a used module's top-level parameters and groups (§14.4) in its root body, by the module's path (`engine.dims.bore`); a component receives external values through arguments (§8). A value MUST NOT read geometry (`a.x`): a value feeds constraints, and a number read off a seed would make the solution set depend on where a solve began (P3); a *seed* may (§6.4).

### 6.4 Seeds written inline **[0.2]**

A declaration MAY carry the starting values of its own scalars, in a trailing `hint(…)` clause:

```
p := point  hint((0, 0))
p := point  hint(y: 12)                     // an omitted scalar is 0
q := point  hint((0, 12, 5))         // [0.42] a point drawn in no plane stands in space
t := point                                  // no clause at all
c := circle(center: o) hint(r: 25)
```

These are seed-class (§4.2, §4.3) and semantically inert (P3). They are the primitive form; §11's `hint` statement remains, for the case it is actually good at.

A driving radius dimension already records a circle or arc's radius. Editor writeback does not
add an omitted `hint(r: …)` in that case; `rim := radius(size) circle(center: o)` is sufficient.
Authored hints remain editable, and free radii retain pose hints. A claimed radius or a radius
expressed in terms of a free unknown does not determine that scalar for this purpose.

**[0.7] One clause for every seed.** `hint(…)` joins the trailing-clause loop, so it is order-free against `knots` exactly as those two already are against each other. A constraint's own unknown is written the same way — `p coincident s hint(t: 0.4)` — and the *pin* stays in the argument list, `p coincident(t == 0.4) s`: `hint` marks what a solve revises, and a pin is precisely what it does not.

Two things that look like seeds and are not, and stay where they are. **`knots [...]`** is document data no solve moves. **A curve instance's values** — `e := Involute(base, phase: 0).p over u in (u0, u1)` — are numbers the family takes, not numbers the solve revises.

**What `hint` marks is that a solve revises the number — not that the number is seed-class.** The two are not the same set, and the difference decides where the word belongs. Seed-class is §4.3's classification: inert under P3, so deleting it changes no solution set. That is true of a coordinate seed *and* of a callout placement (§13.1), which **[0.20]** lives in the `.svd` drawing (§13.1) and needs no `hint` even though it is every bit as inert. What separates them is who writes them. A coordinate seed is an input a solve overwrites, every time, which is the whole of what §6.4's writeback does. A placement is never touched by a solve at all: it is derived by the layout until somebody drags the callout, and from then on it records where that person put it. So a placement is not a guess about anything, and `at` there says what it means.

*Non-normative:* an implementation can therefore read `hint` as the mark of "this is the solver's to answer", which is a narrower and more useful claim than "this is inert". A reader wanting to know what may be deleted without changing the drawing should ask §4.3, which answers for both.

**Why inline is the primitive.** A seed's job is to say where a coordinate starts, and the place a reader looks for that is the declaration of the thing that has the coordinate. It matters more than taste once a drawing is edited by drawing on it: a solve that wants to record where a point ended up rewrites six characters of a declaration that already exists, where under 0.1 it would have to locate that point's `hint` statement among the body's statements, or synthesise one and decide where to put it. The first is a splice; the second is a program transformation, and it is performed on every drag.

`hint` keeps the cases inline cannot express — seeding an entity declared elsewhere, and seeding from an expression over other geometry (`hint t.lead(x: center.x + root.r, y: center.y)`).

**[0.12] A seed may read geometry, and reads its seed.** The text in a `hint(…)` clause — a declaration's or a child slot's (`l := line(hint((p.x + 10, p.y)), …)`) — MAY name another entity's scalar by its dotted path: `p.x`, `p.y`, `k.center.x`, `k.r`, `e.b`. What it reads is that scalar's **own seed**, never a solved value, so the clause stays seed-class: delete it and the solution set is unchanged, as §4.3 requires. The same clause names a place outright **[0.14]**: `hint(at: REF)` — where another point starts — and `hint(at: K, bearing: β)`, the point on circle `K`'s edge at bearing `β` from the x axis of the plane the seeded point is drawn in. **[0.33]** A place may also be a **step** from the point `at:` names: `hint(at: A, toward: B, by: f, turn: θ)` is `A + f·R(θ)(B − A)`, the fraction `f` of the way to point `B` (1 if `by` is unsaid) turned `θ` about `A` (0 if unsaid); `hint(at: A, along: L, by: f, turn: θ)` steps by `f` times line `L`'s run from its `p1` to its `p2` instead. So `hint(at: a, toward: b, by: 0.5)` is the midpoint, `by: -1` the reflection of `b` through `a`, and `hint(at: o, toward: rim, turn: 90deg)` the rim turned a quarter about `o`. `toward` and `along` are one or the other, neither stands beside `bearing`, and `by`, `turn`, `toward`, `along` and `bearing` each need `at` — each refused at its key. **[0.33]** A place drawn in another view than the point it seeds (§6.7) is read **where it stands in space**, projected into the seeded point's view: a point of the pitch plane named in another view starts at its image there. An implementation settles such seeds once the memberships and the views' seeded poses are known, again in statement order. An arc's radius left unwritten is its centre to its start, read once the places are settled. `at`, `bearing`, `toward`, `along`, `by` and `turn` are keys beside `x`, `y`, `r` and `t`, and the rule of §4.3 is then lexical with no exception: a seed is what is inside `hint(…)`. A clause naming a place carries no coordinate (`hint(at: p, x: 3)` is an error at the key) **[0.42]** except a plane's: `hint(at: P, (3, 4))` is the place `(3, 4)` in plane `P`'s own coordinates, read in space and seen in the seeded point's plane, which is how a part seeds a point against the plane it is measured in. `bearing` without `at` is an error, and both are refused where they are written, as an unknown key is. (In 0.7–0.13 the place had a grammar of its own, `hint at REF [bearing (β)]`, which MUST NOT parse now; an implementation SHOULD say what the spelling became.) Both forms were a trace block's words (§6.5) and mean the same thing on the sheet. Seeds are settled once every declaration has one, in statement order, so a seed reading a seed that was itself read from a third comes out right when the three are written in the order they depend on; written the other way round it reads the earlier one's provisional seed (an unseeded point's scatter, an unwritten radius's default), which an implementation MAY warn about and MUST NOT refuse. A read of a scalar the entity has not (`l.r` of a line), or of nothing (`nobody.x`), is **E103**. A geometry read is a `Length` where the document names a `unit` (so it adds to `150mm` and not to `10`) and a bare number where it does not, since there no literal can be a length. Inside a component the names resolve as references do — a formal reads as the entity it aliases, a name inside a block's copy as that copy's. A seed written this way is an expression and is never written back by a solve.

### 6.5 Curves **[0.11]**

A curve is **a point of a component, as one of the component's numeric formals runs over an interval**:

```
NAME := REF over FORMAL in ( A, B )                    an instance's point
NAME := Component(ARGS).REF over FORMAL in ( A, B )    an instance written in place
```

`REF` names a point the component places — a declaration of its body or a nested instance's; a formal the component is written over does not move with the swept formal and is refused (**E103**). `FORMAL` is a numeric formal of that component, `Angle` or `Length` (**E040** otherwise); the interval's ends are expressions over the parameters in scope. A curve declares an entity and takes contacts like any other curve, each owning the curve's parameter — spelled `t`, as a spline's is, whatever the swept formal is called **[0.15]** (0.2–0.14 spelled it `u`, which an implementation SHOULD name when refusing the old key): `p coincident e hint(t: …)` says `p − C(t) = 0`, two residuals and one new unknown; `e tangent l` holds the line through `C(u)` along `C'(u)`, two residuals against the one unknown; `e curvature k` makes the circle the curve's osculating circle at `u`, three residuals against it. **[0.39]** Each contact owns its own parameter unless it is pinned to a name nothing defines (`e tangent(t == s) l`, `e curvature(t == s) k`, §9.2): then every contact on the curve pinned to that name owns the one unknown, so a line and a circle touching the curve *at the same place* are five residuals against one parameter. Stated instead as two contacts tied through other geometry — the circle tangent to the line — the condition holds to third order in the distance between their two parameters, and the solve is a degenerate root. A tangency needs `C'` and its derivatives in the geometry — second order — and a curvature `C''` and `C'''`: a computed point supplies them exactly from its expressions, and a locus supplies `C'` exactly (the implicit function theorem) and `C''`, `C'''` exactly as the implicit function's Taylor orders — each one more linear solve with the same Jacobian of the body, its rows read in truncated Taylor arithmetic — with their derivatives in the geometry by difference **[0.37]**. A body is read that way only through relations whose kernels an implementation has written in Taylor arithmetic; a curvature stated against a locus with any other relation in its body is an error (**E103**) naming the relation, since a residual by difference would solve to a slightly wrong circle and call it right. There is no separate curve family: 0.2's `curve NAME(FORMALS)(PARAM) = …` and 0.3's `trace POINT where { … }` are retired, and an implementation MUST refuse them with a message naming this form.

**Two ways a component places the point.** A **computed** point, `p := point(x: XEXPR, y: YEXPR)` **[0.13]** (`port p = …` in 0.12), gives the coordinates as expressions over the formals and the params; a component with one is drawn only as a curve, and an instance of it on the sheet is an error (**E103**), since nothing on the sheet holds a point to a formula. Any **other** point is placed by the body's statements — the locus form: `C(u)` is where the constraints put the point, given the formal's value and the geometry the component is written over. Traced, the body MUST determine its own coordinates — as many equations as coordinates of its own — or the curve is an error (**E103**): an under- or over-constrained locus is a curve that does not exist, and it must not elaborate quietly. Drawn, the same component may be closed from outside like any other.

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point
  p := point
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c                                       // the string leaves the circle...
  datum angle(u + phase) rad                   // ...at bearing u — directed, so this side
  rad perpendicular s                          // perpendicular to the radius there,
  p distance(-(c.r * u / 1rad)) rad            // and taut: let out == arc unwound
}
e := Unwind(base, datum, phase: a0).p over u in (u0, u1)
```

**Over a drawn instance.** `path := leg.toe over theta in (0, 360)` names a point of an instance the drawing holds. The trace is **anchored at the drawing**: the pose the instance stands in on the sheet is where evaluation begins, and the value the instance gave the swept formal is the anchor's parameter. An instance that leaves a numeric formal **unbound** makes it an unknown of the drawing (§3.3), named under the instance (`leg.theta`) so two instances leaving the same formal unbound have two unknowns — and the anchor then follows that unknown wherever the solve puts it. This is the form a mechanism is written in: drawn once with its crank free, and traced from the same statements. **[0.37]** Every other numeric formal a drawn instance leaves unbound is an unknown of the drawing in the same way (`leg.h`), and the curve is written over it as over the entities' coordinates: it is a **column** of the curve, so a contact against the curve may solve for it — a rod's length chosen so the stride touches the ground. Only the unknown itself: a formal given an expression in an unknown (`h: 2 * k`, inside an instance that left `k` unbound) is **E103**, and so is an unknown nothing on the sheet reads, which no solve could allocate.

**Closed curves [0.37].** A curve run over a whole turn of an `Angle` formal (`over theta in (0, 360)`) that comes back where it started — a crank's coupler curve, a cam's profile — is **closed**. A contact on it has no end to stop at: a parameter carried past the seam wraps round onto the interval and stays free, since the seam is only where the interval was written to begin. A curve that does not come back (an involute unwound a whole turn) is open, and a contact on it is held to its ends.

**Over an instance written in place.** `Involute(base, phase: a0).p` binds the arguments as an instance statement would and draws nothing: the curve is the only thing made of it. The anchor is the value it gives the swept formal, or the interval's start when it gives none. **[0.42]** A traced component is solved in one plane, so a relation in space in its body (§9.2) is refused where it is written.

**Requirements.** An implementation MUST differentiate `C` with respect to the swept formal *and* with respect to every coordinate the component reads. `∂C/∂u` is which way a contact may slide; `∂C/∂θ` is how the curve moves when the geometry it is written over moves, and an implementation that computes only the first will solve a contact once and drop it the moment that geometry is dragged. For a computed point both are mechanical from the expressions; for a locus both come from the implicit function theorem at the body's solution. A name a computed point's expressions cannot reach is an error (**E103**), not a free variable: a curve is written over geometry that exists, and a misspelling there would quietly add a degree of freedom to every point on the curve. A dimension in a traced body MAY read a number of the geometry the component is written over (`c.r`, `k.center.x`): it is a column of the curve, and is differentiated with the rest. **[0.37] Outside a trace a dimension MUST NOT read geometry** (**E103**): the expression graph has no column for it, and minting a free variable named `k.r` would make a drawn instance of a component state something other than its own trace.

**Branches.** A locus generically has several solutions, and a component states its way onto one — three instruments, in order of strength:

1. **A signed constraint,** wherever the vocabulary can say it. Above, *neither* choice is a branch at all: `angle` is directed (§9.4), so `t` sits at the bearing and not opposite it, and a `distance` from a line whose number runs with the swept formal is read signed in a traced body (on the sheet it is a magnitude, §4.1), so one equation unwinds the string one way for positive roll and the other for negative.
2. **An orientation predicate.** `ccw(a, b, x)` / `cw(a, b, x)` in the body is §9.6's statement doing §9.6's job: it contributes no residual and *selects among the discrete solution components*. Traced, its third point MUST be one the component places. A predicate is read **at the anchor** — the drawn pose, or the value the instance gave the swept formal, chosen where the predicates read unambiguously — and an implementation MUST enforce it there (reflect the placed point across the oriented line and solve again) and MUST NOT re-enforce it elsewhere: away from the anchor, continuity governs, and the component the predicate picks at the anchor is the component the whole curve is on, even where the curve has since wound to where the predicate no longer reads true. A body with predicates needs no seeds at all: an implementation MUST fall back to deterministic restarts, scaled by the geometry the component is written over and by nothing else, when the seeds (or their absence) leave the anchor solve nowhere to start. Drawn, the same predicate records the root choice the drawing is on (§9.6).
3. **A seed.** What neither an equation nor a predicate says, a seed says: the body's seeds are places over the formals, evaluation of an instance written in place starts from them, and away from the anchor continuity governs — an implementation MUST evaluate the curve as one continuation along the parameter, so the branch picked at the anchor is the branch everywhere. A curve over a drawn instance starts from the pose on the sheet and reads no seed.

**Places, not coordinates.** Inside a component that is only ever traced, a seed may be a *place*: `t := point hint(at: c, bearing: u + phase)` is the point at the edge of circle `c` at that bearing from the x axis of the plane it is drawn in, and `p := point hint(at: t)` is wherever `t` starts (a point already named must be declared first). Both lower to exactly what the coordinate spelling would, so `hint((xexpr, yexpr))` remains available and means the same thing. On the sheet a seed is a number a solve writes back, which a place named by reference is not, so a drawn instance of a component with a geometric seed is an error (**E103**).

**Bearings are measured in a plane.** A bare bearing was page-fixed, and a body posed against a datum with page-fixed seeds went quietly stale the moment the datum tilted (bmander/geomsolver#10); 0.6 answered with the datum's derived `f.angle`. **[0.42]** A bearing is measured from the x axis of the plane the seeded point is drawn in, so geometry drawn in a part's own plane turns with it and needs no correction; `f.angle` is retired with the rotor (§3.2), and reading it is **E103**, naming the scalars a plane has.

**Why a point of a component and not a family of its own.** 0.2 gave a curve a construct of its own — a family with formals, a parameter and a body — and 0.3 a second body form, and every mechanism was then written twice: once as the drawing and once more inside the family, with the formals passed to themselves (bmander/geomsolver#46). A component already has formals, a body, params and instances, and a curve is one question asked of it. The gear in §18 is the case that makes the difference plain: its flanks are involutes because the document says what an involute is, and nothing in the solver knows the word; the walking leg of `jansen.sv` is drawn once, and its stride is the same leg asked where its toe goes.

### 6.6 Chains **[0.4]**

A chain writes a run of declarations and the constraints *between* them in one ordered breath. Its geometry and constraints elaborate to exactly the statements a person would otherwise write out. **[0.20]** An optional `NAME :=` binds the resulting ordered traversal: an open chain, or a closed loop when the expression ends in `-> close`. Naming the traversal adds no coordinates or equations.

```
horizontal (bottom := line(b1, b2)) -> tangent
(a_br := arc(center: c_br) hint(r: r)) -> tangent
…
vertical (right := line(r1, r2)) -> tangent close
```

```
CHAIN  ::= [NAME ":="] LINK (JOINT LINK)* ["->" INFIX* "close"]
LINK   ::= PREFIX* (DECL | "(" NAME ":=" DECL ")") | REF
PREFIX ::= a constraint name whose spec is one entity slot     // horizontal, vertical
JOINT  ::= "->" INFIX* ["->"] | INFIX+ ["->"]                  // at least one marker or word
INFIX  ::= "tangent" | "equal" | a constraint name whose spec is two entity slots
```

**A chain is a value you can name [0.20].**

```solvent
component Box(/* parameters */) {
  // Points and dimensions are declared in this scope as usual.
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
}
boss := Box(/* arguments */)
iboss := solid(boss.profile, depth: 8mm)
```

The binding is an ordinary name in the enclosing scope. A component may expose several named
chains, reached through its instance (`boss.profile`); no name is special and no default is
inferred. The individual declarations remain in that same scope (`boss.ab`), rather than moving
under the traversal. Anonymous links are allowed: `profile := line -> line -> line -> close`.
In a swept loop, anonymous links name their sides `edge0`, `edge1`, … in traversal order,
skipping names already used by explicit links. Source offsets never become public side names.

A named chain MUST traverse lines or arcs and MUST thread every adjacent pair with `->`;
relations without a threaded joint can still form an unnamed chain of statements. Its links use
all the existing prefix, joint, endpoint-aliasing and forward-reference rules. A named expression
MUST finish before a block's closing brace; it cannot end in a joint awaiting the next copy.
Bindings within repeated components and blocks are independent, just like their geometry.

A named closed loop is usable directly wherever a sweep takes its section (§6.9), and is checked
by the ordinary face rules (§6.8). The closure marker records topology, not geometric validity:
planarity is checked during elaboration and the swept profile's geometric validity after solving,
so a poor hint is not rejected as the final shape. An open chain is not a sweep section (**E080**).
A `face` list may name either kind of traversal, expanding its links in place and in order;
`face(trail, -> close)` explicitly closes an open chain using the face's existing closing rule.
Naming a chain never closes an endpoint gap by itself.

**[0.8] Threading is a statement, not an inference.** The `->` marker on a joint says the two links beside it share a boundary point, threaded left-to-right along the traversal below; its absence says they do not. `->` alone is the plain corner — this is what `to` said through 0.7, and `to` is retired into the marker. `-> INFIX` is a corner that also states the relation, at the point just threaded. `INFIX` alone is the relation and no corner: `a_br equal a_tr equal a_tl` says three arcs are the same size and nothing whatever about where they meet, and welding them would be an invention. Because each joint states its own threading, a chain may mix declarations and names freely — `(l1 := line(a, b)) perpendicular (l2 := line(c, d))` declares two separate lines at a right angle, and `(l1 := line(a, k.start)) -> tangent k` extends a fresh contour onto geometry that is already there.

- A **prefix** desugars to that constraint applied to the declaration it stands before: `bottom := horizontal line(b1, b2)` is `bottom := line(b1, b2)` plus `horizontal bottom`. Eligibility is registry-derived — one entity slot and nothing else — so a new unary constraint joins the grammar without the grammar changing.
- **[0.8] A joint may state several relations.** `A -> equal angle(30deg) B` states both between the two links, at the corner the marker threads; each word desugars to a statement of its own, with its own identity and span. The marker may stand on either side of the words, or both — `A -> equal -> B` is the one joint `A -> equal B` is — and words with no marker state their relations and weld nothing. The words need no punctuation because fixity sorts them: a word is read at the joint until one opens the next link — an element keyword, or a prefix word standing before one — so in `-> tangent horizontal line b` the tangency is the corner's and the levelling is the link's. Deleting one relation splices its word out and leaves the rest standing; the whole joint deleted at once falls back to what a single word's deletion would be — the corner stays, or the statement breaks. **[0.20]** A trailing placement is the drawing's (§13.1), refused in model source.
- A **joint** stands between two links and says how they meet. Each joint constrains its two neighbours without changing which links the chain traverses; the optional binding names that traversal. Joint words have no operator precedence. `a equal b equal c` is therefore two statements, not three, and n operands give n−1 — the same rank as any other spanning set over the same elements, stated as a path rather than a star. An INFIX word is the two-argument counterpart of PREFIX, derived from the same registry: it desugars to that constraint over the pair, positionally, and MUST fit both — a word whose slots the pair cannot fill is an error, not a guess.
- **[0.5]** `equal` is **polymorphic**: `equal_length` between lines, `equal_radius` between circles or arcs, and an error between one of each, since no constraint equates a length to a radius. Like `tangent` it is drafting vocabulary rather than a constraint name, so no registry lookup can resolve it — the pair it stands between does. Where a chain declares its elements the keywords settle it as the program is read; where a chain names them it cannot be settled until the names are resolved, since a name may be declared further down the body (P2) or come from a component, so the word travels to elaboration and is settled there. Both report the same error.
- **Threading.** **[0.8]** A link a marker reaches is a line or an arc — an element with an entry and an exit, read left to right (`p1 → p2`; CCW, `start → end`); a kind with no boundary points — a circle — cannot be reached by a marker, though it may stand in a chain no marker touches. At each `->` the shared point may be named by one side, by both in agreement (two different names are an error), or — between two declarations — by nobody: the chain then mints it, the earlier-built side's boundary being an anonymous child with a name (its dotted path, §6.1), which fills the later side's slot. `(l1 := line) -> (l2 := line)` is therefore two lines and three points, one shared. A link that only *names* an element offers no list to read or fill and no kind to read a boundary field off — so at a corner with one, the declared side MUST name the shared point, usually by the existing element's own child (`k.start`). An end no marker reaches is an implicit child like any other unwritten slot (§6.1).
- `-> close` after the last link seals the loop: the last exit threads to the first entry, and a word beside the marker says how they meet there. A loop is a thread, so a `close` without the marker is an error.
- A statement otherwise ends at its line's end (§2); a line ending in a joint — the marker or a word — continues its chain onto the next.

**Every threaded joint is the regular form.** A threaded joint knows the shared point, so `-> tangent` between a line and an arc is `tangent_arc_line(arc, line, at: start|end)` — tangent *at* the point just threaded — and never the bare tangency over a coincidence, whose Jacobian is rank-deficient at every solution; `-> tangent` between two lines is collinearity (`parallel` over the shared point), and a pair the vocabulary has no regular at-form for (two arcs meeting at a corner) is an error, never a silently degenerate statement. An **unthreaded** `tangent` is the plain pair (`tangent_line_circle`, `tangent_circle_circle`), which is the correct and well-conditioned statement exactly when the two are separate — the `at:` argument is only ever supplied by a threaded joint. A bare `->` states nothing beyond the corner: the shared point is the whole of it.

**[0.8] A block body may end mid-joint.** Inside a `repeat`, `cycle` or `ring` body (§12), the body's final chain may end in a *threaded* joint — the marker, or the marker with words — standing at the body's `}`: the trailing joint threads the chain onto the **next copy's** first link, and is stated between copy i's last link and copy i+1's first exactly as an in-chain joint would be — the weld a shared point, every worded tangency the regular at-form. Which pairs are stated is the block's kind: `cycle` and `ring` wrap, so every copy states it and the trailing joint is the loop's closure — `cycle N { distance(d) line -> angle(a) }` is the dimensioned N-gon, with no `close`, no names and no written points — while `repeat` does not wrap, so the *last* copy's trailing joint is simply not stated and `repeat N { line -> angle(a) }` is an open polyline of N sides and N−1 corners, the natural reading rather than an error. Both boundary links MUST be declarations of kinds with ends (a name-link's boundary is §6.6's ordinary rule: the point must be named where it stands), and at most one of the two boundary slots may name its point — both named are two *different* points across the copy seam, and that coincidence is stated longhand. Where the construct can mean nothing — a `component` body, a trace block, the top level — it is an error, and an unthreaded trailing word still wants its right operand. The joint is one written joint however many copies state it: each stated copy keeps the one statement identity, told apart by the instance path (§12.7). A statement inside a braced body also ends at the body's `}` as at a line break, so the whole of a block may sit on one line.

Each desugared statement keeps an identity of its own and a span into the chain's text, so a caret, a diagnosis culprit and a splice land on the word that stated the thing. (§12.7 is many instances from one statement; a chain is several statements from one *line*, each still its own.) Deleting a chain-borne constraint is therefore a splice — a threaded joint steps down to the bare corner `->`, a prefix word goes where it stands, and **[0.8]** an unthreaded joint becomes a statement break, its span grown over a terminal name-link that a break would leave dangling (inside a chain that closes there is no safe break, and the deletion is refused). Deleting a link is refused: no splice takes one link out and leaves a chain behind, so that edit belongs to the source.

*Non-normative:* chains and paths (§10) answer different questions. A path is a traversal of geometry that already exists — vertices, the circles its arc segments lie on, orientation and branch rules, for boundary composition and export. A chain *declares* the geometry: it is how a contour's elements, their meetings and the levels on its straight runs are written down in the first place. The case library's fillet rectangle is the canonical chain; its longhand form states the same sketch in thirty statements.

### 6.7 Planes and projection **[0.10]** **[0.42]**

A drawing's geometry is drawn in planes, each a plane in space, and **[0.42]** a point drawn in
none stands in space. A multiview drawing is several planes with one object's corners drawn in
each, related by projection: the draughtsman's descriptive geometry, stated as such. A plane is a
place to draw and nothing else; where its picture goes on paper is the drawing's (§6.11).

**A point in space** **[0.42]** is a point no `in` reaches. It has three coordinates in the
world's axes, seeded by `hint((…, …, …))` and held by `fix((…, …, …)) p`; a
`z:` on a point drawn in a plane is **E040** at the key. A line between two points in space is a
line in space. A circle, an arc and a spline are drawn in a plane, and over a point in space each
is **E060**, naming the point; a face is **E080** (§6.8). A 2D drawing is drawn in a plane — `use
std`, then `in std.front { … }` — and there is no page: `std.front` is a plane like any other,
the one at the world origin with u = x to the right and v = z up, its viewer at −y.

**A plane is a set, and `in` is membership of it** **[0.54]**. `p coincident P` (either way
round), said of a point standing in space, **draws the point in `P`**: it is `p := point in P`
said as a relation, and the two are one statement with the same rows — the point has `P`'s two
coordinates, and every relation over it reads as one of `P`'s points' (`p distance(25) a`, with
`a` drawn in `P`, is `P`'s own distance). `l coincident P` draws each end of `l` not yet in `P`
there. A point is drawn in the first plane a statement puts it on; put on a second, it is held
there by a row in space, as a point of one view put on another plane is. A point whose own
statements say it stands in space stays there and the plane holds it by a row: one seeded with a
height (`hint(z: …)`), one whose three numbers a `fix` holds, the point a tangency differentiates
at (§6.21), a `ring`'s (§12.4), and any under a `claim`. Which representation stands is the
elaborator's; what the drawing means is the statement's either way.

**A point may be drawn in several planes** **[0.57]**. `O := point in P, G` draws `O` in `P` and
in `G`, so on the line where they meet: one freedom along it where both planes stand. Each
plane's geometry reads `O` as its own point — `arc(center: O)` drawn in `G` is centred there,
`line(O, T)` in `G` ends there, `O distance(5) q` with `q` in `G` is `G`'s distance — so what a
descriptive layout draws twice and ties by `project` is one point. A plane named twice is
**E040**; two planes that are parallel meet on no line, **E061**. Only a point is drawn in
several planes (a line is, when its ends are; **E040** otherwise). Where a plane is itself built
along a line `O` is on — an end of it, or stated on it — the plane already holds `O`, and the
elaborator states nothing more.

**An axis** (§3.1) is a directed line in space with no start. `t := axis hint(dir: (0, 0, 1))`
seeds its direction, and relations place it: `t parallel s`, `t perpendicular s` and `t angle(θ)
s` against an axis or a line, `p coincident t` for a point on it, and against a plane `t coincident
P` (it lies in `P`), `t parallel P` (it runs along `P`) and `t perpendicular P` (it is square to
`P`); each reads either way round. **[0.43]** `t coincident s` between two axes says they are one
line, either way round: four rows, the seed choosing the sense as it does for `parallel`.
**[0.44]** `l coincident t` between a drawn line and an axis says the line lies on it, either
way round: both its ends on the axis's line, four rows, placing the axis as `p coincident t` does.
`fix(dir == (0, 0, 1)) t` holds its direction, and **[0.43]** `fix(origin == (0, 0, 0)) t`
where it stands — its `origin` **[0.45]** is its point nearest the world origin. An axis read only
as a direction has two freedoms, and one whose place a relation reads has two more. **A drawn line
is an axis** wherever an axis is asked for, from its `p1` toward its `p2`, as its points stand.
**[0.43]** A plane over a line holds a hidden axis that is the line: parallel to it, and through
its `p1` — or, where the plane's two lines share an end, through that end, the plane's origin
standing there (three rows, not four). So `plane(u: hinge, v: …)` stands on the hinge.
There is no vector arithmetic: a direction is a thing constrained, never a value computed.

**A plane is two axes through a place.**

```
right := plane(u: std.y, v: std.z)        // looked at from +x: y to the right, z up, at the origin
t := axis hint(dir: (0.87, 0, 0.5))
t perpendicular std.y
side := plane(u: t, v: std.y)             // turns about y as the solve turns t
side.origin coincident std.front          // on std.y already: at the world origin
p := plane                                // [0.43] its axes p.u and p.v minted, free
```

`plane(u: r1, v: r2)` takes two axes or drawn lines and nothing else; a plane's 0.10–0.41 labels
— `origin:`, `toward:`, `from:`, `fold:`, `offset:`, `through:`, `attitude:`, and a basis written
as triples of numbers — are refused where written. Its **basis** is read off the axes'
directions: `û = u/‖u‖`; the normal `n̂` is `u × v` normalised, out of the plane toward its
viewer; and `v̂ = n̂ × û`. So `v` need not be square to `u`: it says which plane, and which side
is up. **[0.43]** The axes pass through the plane's origin: each plane carries two rows, its origin
on each axis, so where the axes stand the plane stands, and `u coincident P.u` places a plane by
an axis. Two planes share an axis only where both pass through it. A slot left unwritten mints an
axis, `P.u` or `P.v`, free and seeded as the front's (u along x, v along z), or from a seed the slot holds (`plane(u: hint(dir: (0, 1, 0)))`): a bare `P := plane`
has seven freedoms, three of place and two of direction for each axis. 0.42's planes sharing
axes while standing apart are withdrawn: a plane parallel to another takes its own axes, held or
`parallel` to the other's.

**Where a plane stands** is three unknowns, its origin's place in space, read by a seed as `P.x`,
`P.y`, `P.z`, seeded by `hint(origin: (…, …, …))` and held by `fix(origin == (0, 0, 0)) P` **[0.45]**. A plane
floats along its axes until something places it. `P.origin coincident p` puts its origin at `p`,
all three. `P coincident p` passes it through `p`, and `Q distance(d) P` stands it `d` off `Q`
(§6.10): each takes the one freedom across the plane and leaves it to slide within itself, which
the rest of the document settles or leaves free. **[0.43]** `P parallel Q` says two planes face alike, either way round — their normals parallel, two rows — and nothing of where either stands or how it turns within itself. **[0.43]** A plane over two axes wholly held
stands where they meet, and adds no row; held axes that do not meet or run alike, and a plane
held where its held axes are not, are **E067**.

Every plane has three members. `P.u` and `P.v` are its axes, axes through its origin. `P.origin`
is a point drawn in `P` and held at its `(0, 0)`, which a component may be handed and a relation
may place. An axis square to `P` is `t perpendicular P`; there is no `P.n`.

**A point drawn in a plane has the plane's own two coordinates.** The point `(a, b)` drawn in `P`
stands at `o_P + a·û_P + b·v̂_P` in space, its *lift*, which is what a relation in space reads
(§9.2). A plane whose axes' directions and place are all held is **constant**: its basis is
settled at elaboration and a lift over it reads it as constants. Any other plane is solved with
the sketch. *Non-normative:* an implementation places the planes and axes first, solving the
statements about them alone (and about points of planes already placed) round by round, so the
main solve starts where the planes stand rather than where their seeds put them; the geometry
drawn in a plane never places the plane.

**Two planes lying on one another** — turned alike up to a turn within themselves, and standing
in one place — are permitted. Where a relation reads points drawn in each, an implementation
SHOULD say so once, at the later plane's declaration, judged on the solved pose (**W113**: "`B`
lies on `A`, and a relation reads points drawn in each: one plane in space, so the relation is
read in space where one plane would read it on the plane"). A part drawn in its own plane turned
within `std.front` is the common case; a component drawing in the plane its caller hands it never
meets it.

**The standard planes** (§14.4). `use std` gives the axes `std.x`, `std.y`, `std.z` and
`std.back` (x reversed), all held; the planes `std.front` (`u: x, v: z`), `std.top` (`u: x, v:
y`, looked at from above), `std.side` (`u: y, v: z`, looked at from +x) and `std.up` (`u: z, v:
back`, the front turned a quarter), each held at the world origin; and `std.origin`, a point drawn
in `std.front` held at its `(0, 0)`. `std.Turned(o, t)` is a frame turned within the plane `o`
and `t` are drawn in: `axes := std.Turned(o, t) in std.front` makes `axes.axes`, the plane through
`o` whose u runs toward `t`, and `axes.u`, the line from `o` to `t`.

**[0.21] Coordinates relative to a plane.** For a point `p` and plane `f`, `p distance(d, along:
u) f` states how far `p` stands along `f.u` from `f.origin` — **[0.46]** the ordinate `f.origin
distance(d, along: f.u) p`, spelled against the plane (§9.2): drawn in `f`, that is its own
coordinate, `p.x − d = 0`; drawn in another plane or standing in space, **[0.42]** it is read in
space over its lift, `(X_p − o_f)·û_f − d = 0`. `along: v` is the same along `v̂_f`, and **[0.24]**
`along: n` the signed distance along `n̂_f`, in space, of a point not drawn in `f` (one drawn in
it is **E061**: every point of a plane is on it). Each contributes one
length-valued residual, owns no unknown, and accepts a signed length expression, including a free
dimension. **[0.46]** Zero written as a literal is **E040**: it is `p level(u) f`. Either ordinate may be stated independently; `claim` has its ordinary
assertion meaning. The operands MUST be a point followed by a plane, and `along: u`, `v` or `n` is
required for this pair. These selectors are not the `x` and `y` of a run and a rise. An ordinate
read in space is drawn with no callout.

Reading a plane for coordinates MUST NOT assign plane membership to either entity, transform
incoming aliases, hold geometry, or create a component placement phase. Passing `f.origin`, `f.u`
or `f.v` to another component aliases it, including through nested components; a face may
likewise name `f.origin` as a corner.

A seed may read `f.x`, `f.y`, `f.z` and the coordinates of `f.origin`, and place a point in `f`'s
coordinates with `hint(at: f, (…, …))` (§6.4). These read the starting pose, including
geometric seeds already settled. A hint over an unbound numeric formal uses zero for that
unknown's provisional value; an affine expression retains its constant offset. This substitution
applies only to hints, never to constraints, and traced components retain parameterized hints.
Hints still select starting configurations only. Signed ordinates explicitly exclude reflections
across the axis being measured.

**A point says which plane it is on with `in`.** `a := point in top` is a trailer of the declaration, order-free against `hint` and `knots`, and it applies to **every point the declaration mints or names**: `l := line(a, b) in top` puts `a` and `b` on `top`, `c := circle in right` its centre, `arc` and `spline` likewise. A membership moves nothing; it says whose coordinates the point's numbers are, and **[0.42]** a point with none stands in space. A point put on two different planes by two declarations is **E060**; agreement is not an error. `plane`, `axis` and `curve` have no points of their own to put anywhere, and `in` on them is refused. Inside a `ring` (§12.5) a plane is invariant: a membership referencing one is true of every copy alike.

**`a project b` says two points are images of one point in space.** It is an infix operator over two points (§9.2), each `in` a plane; the two planes are **inferred** from the memberships and are never written — an implementation MUST refuse (**E061**) a point on no plane (one standing in space), two points on one plane (a view relates nothing to itself), and two planes that are parallel (they share no fold line), each at the statement. With `d = (n̂_A × n̂_B)/‖n̂_A × n̂_B‖` the fold line the planes share, `d_A = (û_A·d, v̂_A·d)` its direction in A's own coordinates and `d_B` likewise, the residual between two constant planes is

`d_A · p − d_B · q + d · (o_A − o_B) = 0`

— one equation: two images of one point agree on their coordinate along the fold line their planes share, and on nothing else, each measured from its own plane's origin. **[0.42]** Where the two origins coincide the last term is zero, and a plane moved square to the fold line — along its projectors — leaves the row as it was, carrying everything drawn in it: the free spacing between a drawing's views is a floating plane's origin. Where either plane is solved for, the row is the projector rule in space over the two lifts, `(n̂_A × n̂_B)·(X_A − X_B) = 0`, and two planes that *come out* parallel are **E065** after the solve. `project` is claimable (§9.7): `claim a project b` asks whether two views are consistent. It is not commutative.

**The block form writes the clause once.** `in top { … }` marks every declaration in its body `in top`; **[0.12]** it stands at the top level of a document and inside a *component* body — over a plane the component was handed, which is how a part carries its geometry for each view in one place, its own `project` statements tying them — and not inside a root block, where a header buried in another statement's span would be a splice no deletion could compose; a `repeat`, `cycle` or `ring` inside it marks the declarations of every copy, so a contour drawn as a chain round a cycle is drawn in the view. The statements are ordinary statements of the enclosing body, and an implementation MUST treat them exactly as if each had written the clause itself — they splice, diagnose and delete as themselves, only the header and the closing brace are the block's, and deleting the plane removes exactly those, leaving the statements standing in space. The block stands at the top level of a document or of a component body (inside a root block, the clause says it one declaration at a time); a declaration inside that writes its own `in`, and a kind with no points of its own, are refused where they stand.

**An instance joins a view whole.** `t := Tooth(…) in top` puts every point-bearing declaration the instance's expansion makes — through nested components and blocks — on the plane: the block's rule, over the statements one statement stands for. A plane, an axis or a curve inside is left alone, having no points of its own to put there. A point aliased in through an argument joins through any body declaration that names it, and one already on another plane is E060; an expansion given two planes — a clause of its own under an enclosing `in` — is refused. An instance inside an `in { … }` block takes the block's plane the same way.

*Non-normative:* the front, top and right views of a part are then three planes — `std.front`, `std.top`, `std.side` — with the part's corners drawn `in` each and tied across them by `project`; an auxiliary view standing on an axis along an inclined face (`aux := plane(u: incline, v: std.y)`) shows that face true-size, and its corners can be placed by projection alone. Each is drawn in its own coordinates, and its drawing places it on the sheet (`sketch aux(m) from m.aux at (10mm, 110mm)`). `rust/examples/bracket.sv` and `bracket.svd` are the worked case.

### 6.8 Faces **[0.18]**

A **planar face** is a region of a plane: an outer loop of existing edges and straight runs between named corners, optionally containing holes. An explicit `on:` support instead declares a spatial face boundary (§6.20); the planar rules below do not turn that support into a plane drawn in.

```
sec := face(mouth, side_r, lid, side_l)     // four lines, walked in order
hole_f := face(hole)                        // one circle, which is a loop by itself
bore_f := face(b_mouth, bore_r, b_head, b_axis)
```

A face is a **Declaration** (§4.2). It adds no coordinate, unknown, equation or freedom, and its existing edges keep their own names: naming them is aliasing (P1), not constraint, so deleting an edge deletes the face. A named chain (§6.6) in its list contributes its ordered edges. Its list is a `List` slot like a spline's control polygon, so there is no arity to conjure children from and a bare `f := face` is refused (**E080**, the face's own code saying what a face is, where a bare `s := spline` is §6.1's E103).

**A face has one closed outer loop.** Consecutive edges — and the last with the first — MUST share an endpoint, and sharing is asked of the **points**, which is aliasing and cannot be argued with: two neighbours that share none are **E080** naming both. The loop is walked in the order it is written. A `circle` is a whole loop by itself and MUST stand alone in one (E080); an edge that is neither a line, an arc, a circle, a spline whose knots are clamped (its ends are its first and last control points), a stretch of a curve nor a point is E080 at the edge.

**A stretch of a curve [0.28].** A curve (§6.5) has no end points of its own for a walk to meet, so it stands in a face as the stretch between two points: `flank from p to q`, each point held on the curve by a contact (`p coincident flank`), whose parameter is where the stretch ends. The stretch follows the solve, as the contacts do; it is walked from `p` to `q`, against the curve's own sense where `q` comes first. A curve named without `from … to …` is E080, as is a point not held on it, a stretch from a point to itself, or one curve bounding one face twice. The implementation mints the stretch as a curve of its own carrying `.closure` (it is the curve already drawn), and it names a face of whatever is swept from the loop by the curve's name.

```
tooth := face(t.r.e from t.r.lo to t.r.hi, t.crown, t.l.e from t.l.hi to t.l.lo, -> close)
cap := face(k from a to b, -> close)                  // a stretch and its chord
```

**A face closes itself [0.19].** The list is a *walk*, and an item may be a **point** as well as an edge: a point is a corner the walk goes straight to and straight on from. Where two neighbours do not already share an endpoint the implementation MUST mint the straight run between them, from where the walk leaves one to where it enters the next; a run between two items standing at the same point is nothing and is not minted. The gap between the **last** item and the first is minted only where the list ends in the marker **`-> close`** — the chain's own word (§6.6), in the other place the language draws a loop — which MUST be the last thing in the brackets and is refused on any other kind. `-> close` on an already-closed loop, including a lone circle, mints nothing.

```
pist_f := face(crown, pL0, pL1, pL2, pL3, pL4, s0.p, -> close)
quad := face(x0.p, x1.p, x2.p, x3.p, -> close)      // four corners, no line drawn at all
```

A minted run is an ordinary line of the drawing carrying the class **`.closure`**, whose shipped rule is `display: none` (§13.2): it bears no design, nothing drew it, and it exists so that a region has a boundary. It names a face of whatever is swept from the loop, as `close0`, `close1`, … in traversal order, skipping names already used by existing edges in the loop.

Three restrictions keep the shorthand from swallowing a mistake. An interior gap between two **edges** is still E080: a point in a list can mean nothing else, while two edges that do not meet are edges listed out of order. And **an edge MUST meet at least one of its neighbours**, since that is what says which way it is walked — `bad := face(a, bc, d, -> close)` has two readings and states neither, and is E080 naming the edge. A straight loop with fewer than three corners is E080 for the same reason a prism swept nowhere is. A single item is a loop only as a circle, or as an edge with two ends closed by `-> close`, which mints its chord **[0.28]**.

**A face may have holes.** After its outer boundary, `holes:` introduces one or more circles or named closed loops, separated by commas. Each entry is a complete inner boundary; an open chain is refused (E080). All boundaries MUST lie in the same plane. After solving, every hole MUST be simple, nonzero and strictly inside the outer boundary, and holes MUST be mutually disjoint: touching, crossing, overlapping and nested holes are refused. Loop direction does not determine whether it is a hole; `holes:` does. The final `-> close`, when present, closes only the outer boundary.

```solvent
annulus := face(barrel, holes: core)
perforated := face(outline, holes: bolt0, bolt1)
groove := solid(annulus, from: z, to: z + width)
groove cut body
duct := solid(face(outline, holes: inner.profile), depth: length)
```

A section's holes travel through its entire sweep, including `depth:`, `from:`/`to:`, `through:` and full or partial revolutions. The declaration creates the swept region; applying it to another body remains a separate `cut` or `union` statement. A circular boundary names its wall by the circle’s path relative to the component containing the face (for example `duct.left.bore`). A hole supplied by a named loop qualifies its side names by that loop's path (for example `duct.inner.profile.wall`), relative to the component containing the face. References to repeated holes keep their indexed source paths (for example `duct.bore[0]`), without internal expansion IDs. Boundary names MUST be distinct; sweep cap names remain reserved. The source edges retain their identities, so holes do not require names derived from Boolean-generated topology.

**A face lies in one plane, and does not say so.** Its plane is read off the **memberships** (§6.7) of every point of every edge; those MUST agree, and a dissenting edge is **E080** naming it and both planes. Nothing is written on the face itself. A face bears no points of its own, so an `in` clause on one is refused where it stands — but a face **inside** an `in … { }` block is left *unstamped* rather than refused, exactly as a plane and a curve are: a block stamps the geometry the face is written over, the face is on the plane its edges are on, and refusing it would put a design and the region taken from it in two different blocks.

### 6.9 Solids **[0.18]**

A **solid** is a face swept, or a term over other solids.

```
block := solid(sec, from: face, to: back)         // a prism between two ordinates
boss := solid(boss_f, depth: 10mm)                // `depth: d` is `from: -d, to: 0`
bore := solid(bore_f, about: ax)                  // a full turn about a line in the face's plane
lug := solid(lug_f, about: ax, sweep: 90deg, sense: cw)
body := solid(block)                              // a body, whose stock is `block`
bore cut body                              //   ... less the bore
boss union body                                //   ... plus the boss
```

**The brackets are what the thing is made of** (§4.3, §6.2), so the sweep stands in them beside the face: `from:`, `to:`, `depth:`, `through:`, `along:`, `about:`, `sweep:` and `sense:` are labels of the constructor and are neither seeds nor constraints. A mixture of sweep forms, a half-written prism (`from:` with no `to:`), `from:`/`to:` beside `depth:`, and `sweep:` or `sense:` with no `about:` are each refused where they are written, with the shapes a solid has.

**A face may be written where it is used.** A swept solid MAY take an anonymous `face(…)` in place of a face reference. Its boundary follows every rule of §6.8, including points and `-> close`, and resolves names in the solid's enclosing scope. It introduces no public name and owns no unknown; the solid owns the section and any closing lines it generates. Every sweep form accepts it. An `along:` solid takes one or two faces; other sweeps take exactly one face, and a body takes solids. A section used by several sweeps can retain a named `face` declaration.

```
block := solid(face(mouth, side_r, lid, side_l), from: face, to: back)
```

**Every numeric extent is an expression, and MUST NOT be an unknown.** A number in a solid's brackets is settled by the flattener over the numbers in scope — a value, an input, a formal (§5) — and is then document data no solve moves, checked against its slot's dimension (`Length` for a prism's ordinates, `Angle` for a sweep; **E103** otherwise). A solid allocates no parameter, so P3's other half holds without a rule of its own: there is nothing here for a solve to rewrite.

- **A prism** runs `from:` one signed ordinate `to:` another **along the face's own plane normal**. Those signs are arithmetic and not a convention (§9.2 **[0.17]**) — they are ordinates on an axis, and a document writes both. `depth: d` is the draughtsman's spelling of `from: -d, to: 0`, the material *behind* the face the view shows, and is therefore a **magnitude**. A prism swept nowhere (`from` equal to `to`) is **E080**.
- **A through prism** is written `tool := solid(section, through: target)`. Its target MUST be a solid (**E080**); it spans the target in both directions along the section's own plane normal. The extent is evaluated after the sketch is solved: recursively collect the target's stock and additions, ignoring all cuts, project a conservative bound of that material along the normal, and pad both ends outside the material at the kernel's tolerance. The target may be a swept solid or a body. Changes to its material geometry or placement change the extent; subtractive tools do not enlarge it. `through:` cannot be mixed with any other sweep label, and does not itself subtract anything. Apply the tool separately with `tool cut body`.
- **A guided loft/sweep** is written `duct := solid(section, along: guide)` or `transition := solid(start_section, end_section, along: guide)`. The guide MUST be a directed line (`p1` to `p2`) or open circular arc (`start` to `end`, counter-clockwise in its own plane); other kinds are **E081**. The start section MUST lie in the plane through the guide start perpendicular to its tangent. An explicit end section MUST likewise lie at the guide end perpendicular to its tangent. These are checked after solving; the constructor does not reposition or constrain the sections. With one section, the end section is implicitly the transported start. A line transports by translation; an arc transports by rotation about its center and plane normal. With two sections, corresponding boundary points interpolate linearly in that transported frame. Sections must have equal numbers of holes, paired in written order, and corresponding loops must have equal numbers of source edges. Edges pair in counter-clockwise traversal order starting with the first source edge; circle/arc tessellation counts may differ. Each point of an edge pairs with the point of its partner at the same fraction along it: a line's by length, an arc's by angle, and a spline's or a curve stretch's by its own parameter **[0.28]**. Whole circles pair by radial direction in the transported frame, independently of the end plane’s drawing axes. Crossed or collapsed sampled sections, intersecting holes, and sections touching or crossing the bend axis are refused. `along:` cannot be combined with any other sweep label. It owns no unknown: editing the guide or either section changes the evaluated solid.
- **A revolution** turns a face about `about:` a line, which MUST be a line (**E081**) and MUST lie in the **face's own plane** (E081) — a line drawn in another view names a direction this face knows nothing about. `sweep:` is how far and is a **magnitude**, a full turn where the document writes none; **which way is a word, not a sign** (§9.2, §9.4): `sense: cw | ccw`, right-handed about the line's own `p1 → p2` unless `cw` is written, and a negative `sweep:` is **E040** at the value, in the words of the selector that replaces it.
- **A body** names its **stock** in the brackets and takes its features from the **[0.32]** `union`, `cut` and **[0.22]** `bound` statements that name it. A solid whose brackets hold solids and no face is a body. **[0.31]** A swept solid that a `union`, `cut` or `bound` statement names is a body too, whose stock is its own sweep: the name means the sweep with its features wherever it is read, the sweep's faces keep their names (`plate.near`, not `plate.plate.near`), and a feature's are reached through it (`plate.hub.near`). A document wanting the bare sweep and the finished part under two names writes `body := solid(plate)` and puts the features on `body`.

**The body rule, and the whole of it.**

> **A solid is its stock, plus everything in `union` with it, minus everything that `cut`s it, within everything that `bound`s it.**

As a point set, with `union(s)`, `cut(s)` and **[0.22]** `bound(s)` the three **sets** of statements naming `s`, and `S(s)` its stock:

```
B(s) = ( S(s) ∪ ⋃ { B(x) : x union s } ) ∖ ⋃ { B(y) : y cut s } ∩ ⋂ { B(z) : z bound s }
```

Union first, and none of the three groups is ordered; difference and intersection commute, so the last two need no order between them either. `tip bound blank` keeps of the blank what lies within the tip cone, which is what a rim inside a cone is, and what `blank ∖ (blank ∖ tip)` used to spell in two statements and an intermediate. `union`, `cut` and `bound` are Declaration-class (§4.2, §9.2): each says what its right operand *is*, contributes no residual, and enters no solve. A solid that reaches itself through its operands is **E041** — "made of itself".

**Extent dependencies are separate from Boolean operands.** A through prism reads only the target's additive material sources for its bounds; it does not read the target's final Boolean result. Therefore `tool := solid(section, through: body)` together with `tool cut body` is valid and unordered. Genuine material/extent cycles, such as using that tool as the body's stock or adding it in `union` with the same body, are **E041**. A cutter remains a finite named solid with its own reports and faces. Its padded caps are numerical extent boundaries, not design datums.

**`cut` replaces the old Boolean `through` spelling.** `pocket cut body` subtracts the pocket, whether its depth is blind or through the part. `X through B` is refused with a diagnostic directing the author to `X cut B`; `through:` is only an extent label. Exact blind-cut endpoints retain their declared values: this feature does not silently extend cuts that end on internal walls.

**Why a term and not a feature tree.** A feature tree is imperative because it is *stateful*: step *n* acts on the anonymous body as of step *n−1*, and names faces by the order they were made in. Solvent names everything, so the order lives **inside the term**, over names, exactly as it lives inside `h = w / 2`; between statements there is none, which is P2. `bore cut body` says what `body` is and may stand anywhere in the file — above the declaration it qualifies, below it, or in a component beside it — and moving it changes nothing.

A design that wants the other order **names the intermediate**, and then there are two solids because there are two things:

```
recess := solid(pocket)      // the pocket, less the boss standing in it
boss cut recess
body := solid(block)
recess cut body       // block − (pocket − boss), which the flat rule cannot spell
```

**A solid's faces are reached by path, never by index** (§3.2). A prism's caps are `.far` and `.near`, the lower and the higher of its two ordinates along the normal, so `depth: d` leaves `.near` the face the view shows; each side is named by the edge it was swept from, in the name the *source* wrote (`block.side_l`). A revolution names its wall by the edge likewise and its caps `.start` and `.end` where the turn is partial. A guided loft/sweep has `.start` and `.end` caps and walls named by the start section’s source edges, including hole boundaries. Through a body the operand keeps its own name (`body.block.near`). An implementation MUST NOT name a face by its position in anything: that is §13.1's rule, and a boolean is exactly the operation that would renumber one.

What a report says about a solid (§16.3) is therefore `NAME.volume`, `NAME.area`, `NAME.bounds.{x,y,z}{0,1}`, and `PATH.area` for each of its faces that survived — a bore that ate a cap leaves a name the document still writes and no area behind it, which is a fact and not an error. Those numbers MUST be taken at a faceting the **document** fixes and never at the screen's: a volume that changed with the zoom is a number nobody could quote.

**A solid stands on no plane.** It bears no points, so `in` on one is refused where it stands and an `in … { }` block leaves it alone (§6.8). It is not picked, dragged or dimensioned on the sheet, and it is evaluated after the drawing is solved (§1.2, §3.1).

*Non-normative:* `rust/examples/vtwin/components/cylinder.sv` is the worked case — one section in the plane of swing, the body swept from it, and the bore, the port, the bolt hole and the head's slot four more solids that `cut` it. The part had been written three times, the same body redrawn in two more views as page-aligned rectangles re-tied by `project`, with every depth ordinate related to the section by no statement at all.

### 6.10 Planes stood off **[0.18]** **[0.42]**

**`P distance(d) Q` stands `Q` off `P`**: `Q`'s origin is `d` along `P`'s normal, `(o_Q − o_P)·n̂_P
− d = 0`, signed, positive on `P`'s out side. It is one row, and says nothing of `Q`'s attitude;
over planes whose axes are parallel it is two parallel planes `d` apart, which is what 0.18's
`offset:` said:

```
deck := plane(u: std.x, v: std.y)         // [0.43] at the world origin, where its axes meet
lid := plane hint(origin: (0, 0, 12mm))
lid.u parallel std.x
lid.v parallel std.y
deck distance(12mm) lid
n := axis
n perpendicular deck
n coincident deck.origin
n coincident lid.origin
```

The relation takes `lid`'s one freedom across `deck`, and `lid`'s origin could still slide within
`lid`; the axis `n`, square to both and through both origins, stops it. The language adds no sugar
for a stack: a library component says it in one line.

**[0.42] `from:`, `offset:` and `against` are withdrawn.** 0.18 derived a plane `from:` another,
stood off it by a constant `offset:`, and placed a plane written `from: P` alone by a mate between
two faces of solids, `F against G`, refusing a stack that contradicted itself as **E083**. A plane
is now two axes and a place, and a place is stated by relations, so all three are refused where
written and E083 is gone. A contact between parts is stated over floating planes with the
relations of §6.7 and this section, or by a library component over them.

### 6.11 Views and sections **[0.20]**

Views and drawing sections belong in [Solvent Drawing](docs/solvent-drawing.md), not in
model source. A model plane describes geometry. Viewing direction, page placement, and scale
belong to the drawing. Removing a drawing leaves the complete model and solution unchanged.
**[0.42]** A plane carries no place on paper: a drawing's `sketch NAME(m) from m.P at (X, Y)`
draws the geometry drawn in plane `P` in `P`'s own coordinates at that place on the sheet, and a
sketch with no `from` draws `std.front`'s.

### 6.12 Dimensions as output **[0.20]**

A model's dimensional constraints and assertions remain model statements. Selecting and
placing their annotations, or requesting measurements of solved geometry, belongs in `.svd`.
A drawing MUST NOT add model unknowns, equations, or branch choices.

### 6.13 Analytic surfaces

```
flank := surface(crown, edge: rack.outer)
root_transition := surface(crown, edge: rack.outer_round)
```

A `surface` names an analytic patch of an existing solid. Its arguments are `solid` and
`edge`, in that order; labels may be written as with other constructors. The initial supported
source is an unmodified revolution, with a line, circular arc or circle on its profile's
outer or hole boundary. An unrelated edge, a modified body, or an unsupported source MUST
be refused. This declaration does not create new coordinates or approximate the surface
with a mesh.

The normalized parameter `u` traverses the declared source edge from 0 to 1, and `v` traverses
the declared revolution from 0 to 1, including its sweep angle and sense. Evaluation returns
position and both first partial derivatives in world coordinates. Their cross product follows
the edge and sweep directions; it MUST NOT be interpreted as an automatically outward material
normal. Trim and material orientation remain separate properties.

An optional pair of angular bounds selects a source span:

```
flank := surface(crown, edge: rack.outer, from: 180deg, to: 360deg)
```

Both bounds are required when either is given. They are bound Angles measured from the
source revolution's starting meridian, along its declared sense, and MUST satisfy
`0deg <= from < to <= sweep`, using the source's effective sweep (at most one full turn).
Omitting them retains the whole source sweep. They restrict
the existing `v` domain to `[from / sweep, to / sweep]`; they do not renumber parameters,
reverse tangents, or wrap an excluded point into another part of the surface. A clockwise
revolution still uses positive angles measured along that clockwise sweep. Bounds and
source dimensions may depend on component parameters.

`RevolvedSurface::domain` and browser `surfaceDomain` expose the retained parameter bounds.
Evaluation and finite-incidence projection MUST respect them. Envelopes inherit this source
domain, exposed with their roll interval by `GeneratedEnvelope::domain` / `envelopeDomain`.
This can select a generating semicircle without embedding chart bounds in an external
verifier. It does not alone prove the regularity or connectedness of the resulting envelope.

Surface references follow the current solved source geometry. An evaluated snapshot retains
the state it was read from and MUST be read again after that state changes. A surface may be
passed through a `surface` component formal, tagged as construction, or made private. Its
`solid` and `edge` fields alias its dependencies. Deleting a dependency deletes its dependent
surface; copying a surface carries its defining geometry.

An incidence evaluator may use the continued meridian's line or circle equation for a
line/arc/circle revolution whose meridian stays on one side of its axis. Its oriented residual
is in length units, and its gradient follows the source tangent orientation. Near a regular
interior point this is a signed normal distance; it is not a global minimum-distance claim.
The continued equation alone does not establish incidence with the finite source patch.
Evaluation also returns a point on the finite patch and the Euclidean error to that point.
Only that error, compared with an explicit length tolerance, establishes bounded incidence.
An axis-crossing meridian or a query with no unique normal is refused by this evaluator.

This declaration provides named generating geometry. Motion families, envelopes and material
trims are specified below; solids assembled from surface boundaries remain under development.

### 6.14 Rigid motion families

```
crown := motion(about: crown_axis)
blank := motion(about: blank_axis, ratio: 2, phase: 10deg)
generating := motion(crown, relative_to: blank)
```

A `motion` defines a family of rigid transformations over one shared angular parameter. It
owns no solver unknowns and does not move the sketch. `about:` names a directed solved line
in world space. Rotation is right handed about that line, through its first endpoint, by
`phase + ratio * angle`. `ratio` is a bound scalar (default 1), and `phase` is a bound Angle
(default 0deg). Reversing the line reverses the rotation. Axis geometry follows the current
solved model, read where it stands in space. `advance` is a bound Length (default 0) travelled along the line per full turn of the parameter, so a rotation with an advance is a screw. `motion(along: l, advance: a)` is a slide: a translation along the directed line `l` by `a` per full turn, with no rotation, `ratio:` or `phase:`.

`relative := motion(source, relative_to: observer)` expresses the source motion in the moving
observer's frame: `inverse(observer) * source`. `of:` may label `source`. Both dependencies
read the same angular parameter. The derivative includes the observer's motion. This is a
relationship between motions, not a sequence of commands that changes entity coordinates.

Evaluation takes the shared angle in radians and returns an exact rigid pose and its first
derivative per radian. A sampled source point returns world position and velocity per radian;
velocity becomes velocity per second only after multiplying by an angular speed. Numeric
arguments must be finite, and the solved axis must be nondegenerate. Cycles, missing motion
references and dependency chains beyond 64 levels are errors. Forward references, `motion`
component formals, privacy, and dependency-aware copy/delete are supported.

These constant-ratio families provide relative generating kinematics. They do not yet specify
arbitrary motion laws or an assembly-joint solver.

**[0.37] A turn in a view.** `about:` may name a *point*: `cam := motion(about: o, ratio: 1,
phase: 10deg)` turns the view about `o`, by `phase + ratio * angle`, counter-clockwise as the view
is drawn — in space, the rotation about the line through `o` square to the view, toward its viewer.
It takes no `advance:`. A turn is how a planar motion is written: what it generates in its view is
a curve of the drawing (§6.15.1), with `o`'s coordinates among that curve's columns.

**[0.27] A motion's numbers may measure the solved drawing.** A motion is read after the solve,
so `ratio:`, `phase:` and `advance:` may call `length(l)` (a line's length, or an arc's along
itself), `radius(c)` (a circle or an arc), `distance(a, b)` (two points, or a point
and a line produced) and `angle(l1, l2)` (between two lines' directions, 0° to 180°), all in
space: `pinion := motion(about: axis, ratio: -length(wheel_r) / length(pinion_r))`. The arguments
are references, resolved as any reference in the statement's scope; a length reads as a `Length`
where the document names a `unit`, so the slot's dimension is checked as for any number. The
value is worked out from the solved geometry each time the motion is read and adds no unknown or
equation; a motion goes with what it measures under copy and delete. Every other context — a
`param`, a seed, a constraint's number, a solid's extent or placement angle — is needed before
the solve, and a measurement there is an error (the reference implementation's **E107**).

### 6.15 Generated envelopes

```
flank := envelope(crown_flank, under: generating, from: -35deg, to: 35deg)
```

An `envelope` is the implicit locus of a named `surface` under a named `motion` where the
moving surface normal has zero component along its relative velocity:

```
F(u, v, roll) = normal(u, v, roll) · velocity(u, v, roll) = 0
```

The source parameters `u` and `v` remain in `[0,1]`. `from:` and `to:` are required bound
Angles specifying a finite increasing interval of the shared motion parameter. They are
part of the geometric domain, not solver seeds. The interval is evaluated in radians;
`F` has units of length per radian. Argument labels `surface:` and `motion:` are accepted,
with `under:` an alias for `motion:`. The `surface` and `motion` fields alias the dependencies.

The declaration does not allocate coordinates or planar solver equations. It is an implicit
surface, potentially with disconnected branches, singularities or no regular envelope.
Evaluation of a trial `[u,v,roll]` returns its transformed position, oriented source normal,
velocity and `F`. A trial with nonzero `F` MUST NOT be reported as a point on the envelope.
The source normal orientation does not designate which side contains solid material.

A local intersection adds two section equations and solves them together with `F = 0`.
Numerical search bounds must lie within the declared domain. Equal bounds hold a parameter
exactly; the remaining free Jacobian columns must have full rank, and every original
residual must meet its tolerance. A search step may be limited to a domain boundary, but a
boundary point that fails an equation MUST NOT be accepted. A converged local intersection
does not establish uniqueness, global regularity, a material side, or solid closure.

Two named analytic boundary patches may supply the section equations. Supporting equations
can guide the iteration, but the result MUST meet the finite-patch incidence tolerance for
each boundary as well. A point on a continued cone or sphere outside its declared edge or
sweep interval MUST NOT be accepted merely because those supporting equations vanish.

`envelope` component formals, forward dependency paths, privacy, semantic tags, and
copy/delete dependencies follow ordinary component rules. A solved evaluator is a snapshot
and must be read again after the source geometry changes. Envelopes are not drawing glyphs
or completed solids. Declarative trim boundaries and closed solids assembled from bounded
surfaces remain under development.

#### 6.15.1 Envelopes in the plane **[0.37]**

```
blank := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 30)
cutting := motion(rack, relative_to: blank)
flank := envelope(rack_flank, under: cutting, from: -25deg, to: 25deg)
profile := envelope(roller, under: follower_on_cam, from: 0deg, to: 360deg, side: near)
```

The same word over a **tool of the sheet** — a point, line, circle, arc, a curve from a computed
point, or a profile itself generated by a point, line, circle or arc — and a planar motion (built from turns about points, `along:` slides and `relative_to:`)
is a **curve of the drawing**, not a spatial envelope: the profile the tool cuts in the moving
frame, as the roll runs over `[from, to]`. It is a curve like any other (§6.5): it is drawn, and
`p coincident f`, `f tangent l` and `f curvature k` hold against it, each owning the roll at its contact
(`hint(t: …)`, degrees). The tool is drawn where it stands at roll 0. At roll `t` its point `T(s)`
stands at `X(s, t) = M(t)·T(s)`, and the cutting point is where `F(s, t) = X_s × X_t = 0`: the
tool's tangent runs along the velocity of the material under it, which in the plane says the
tool's normal there passes through the instant centre (the law of gearing). A point tool has no
`s`: its envelope is its path.

**What it is written over is solved for.** The tool's coordinates and the motion's — its turns'
centres, its slides' lines — are columns of the curve, so a contact on a generated profile moves
the tool and the motion to suit: a rack's pressure angle chosen so its cut passes a point, a
cam's roller placed so the profile it cuts bends at a stated radius. The motion's ratios, phases
and advances are numbers of the curve; a number written as a measurement (§6.14) is read after the
solve and is an error here (**E080**), as are a rotation about a line in space, a traced tool, and a
generated tool whose own tool is not a point, line, circle or arc. A generated tool must be
declared before the profile it cuts.

**Exact orders.** `C`, `C'` and `C''` are exact: the root `s` and the cut are read in truncated
Taylor arithmetic along the roll, each order of `s` one division by `F_s`. A point, line, circle or
arc is exact at every order, so a profile it cuts is exact to `C'''` too — and so a profile that
profile cuts in its turn (a rack-cut tooth cutting its mate) is exact to `C''`, its tool read as
its Taylor series. An implementation MAY take a `C'''` it cannot read exactly, and the gradients
along the columns, by difference: they enter only a Jacobian, never a residual.

**Which cut.** A line cuts once. A circle or arc cuts twice, on the line from its centre through
the instant centre: `side: near` (the default) takes the point nearer the instant centre, `side:
far` the farther; any other word is **E080**. A curve tool may cut in several places: the roots at
`from:` are found over the tool's interval, `side` picks among them by their distance from the
instant centre, and the choice is carried to every other roll by continuity. A root where the
tool stands still (the point a turn is about) is no cut **[0.40]**. A generated profile
over whole turns that comes back is closed (§6.5) **[0.40]**, and stands alone in a face (§6.8).

---


#### 6.15.2 A prism's side generating in its view **[0.38]**

```
side := surface(rack_tooth, edge: rack_flank)
flank := envelope(side, under: cutting, from: -30deg, to: 30deg)
p := point in cut
p coincident flank
```

`surface(S, edge: e)` names a side of a prism `S` (a face swept by `depth:` or `from:`/`to:`) as it
names a revolution's: `e` a line or arc of the face, the side what `e` sweeps along the view's
normal. A prism's side carries no angular bounds (**E080**). Under a motion that keeps the prism's
view — its turns about points of that view, its slides along lines of it — every section of the
side moves alike, so what it generates is the planar envelope of `e` (§6.15.1) extruded square to
the view: a surface that depends only on geometry the drawing solves. The envelope of such a side
is therefore **built with the drawing**, before any relation: it is the curve `envelope(e, under:
m, …)` would be, standing for the surface. Every point the curve is written over — the edge's and
the motion's — is drawn in one view, and that view is constant (**[0.42]** a plane the solve
places is **E080**).

`p coincident flank` holds a point drawn in **any** view to the surface: its lift, read into the prism's
view and put on that view's sheet, lies on the curve at the contact's roll — two rows against the
roll it owns (`hint(t: …)`, degrees), the one equation a point on a surface is worth. The rows are
the planar contact's, with the point's place for its coordinates, so the tool's and the motion's
geometry are columns as in §6.15.1. A side under any other motion is the spatial envelope of §6.15,
read after the solve.

### 6.16 Trimmed spatial patches

```
flank := patch(generated, inside: tip_body, inside: heel_body,
            outside: root_body, outside: toe_body)
```

A `patch` retains points of one named `surface` or `envelope` satisfying every material-side
condition. Each clipping operand is a **solid**: `inside:` retains its material and boundary;
`outside:` retains its exterior and boundary. All conditions intersect. The declaration
requires at least one condition; each operand carries its own `inside:` or `outside:` label.
The source may be positional or labelled `source:`. `.source` reaches that entity. The
`inside` and `outside` fields are lists, not singular child paths.

This is spatial geometry evaluated after solving, with no coordinates, hints or planar
constraint rows. Components, formal aliases, privacy and construction semantics apply.
Clipping does not replace the source parameter domain or the envelope equation. A patch may
be disconnected, singular or contain several envelope branches; it does not by itself
designate a topological disk, an outward normal, or a closed solid.

The current analytic evaluator supports unmodified **full revolutions** with line, arc or
circle profile boundaries and holes. Partial revolutions, lofts, prisms and Boolean bodies
are refused as clipping operands. Membership uses analytic meridian axis crossings and finite
curve distances, not the preview facets. Profile topology is checked by the existing solid
validator, whose topology checks currently use faceted loops. This is not an interval proof
of arbitrary analytic profile validity. Axis edges that disappear on full revolution are
not material boundaries: a sphere's diameter is interior.

`TrimmedPatch::named` reads a solved snapshot with an explicit absolute axis tolerance.
`surface_at` checks a surface point against the trims. `envelope_at` additionally requires
the normal-velocity equation within its explicit tolerance; arbitrary off-locus trials still
use `GeneratedEnvelope::evaluate`. Both use an explicit boundary tolerance in model length
units. `intersect` and `intersect_boundaries` solve on the untrimmed envelope and require the
final result to pass all trims; an excluded local solution is refused, not silently replaced
by another branch. Finite-boundary incidence checks remain in force. These floating-point
checks do not certify a global error bound or closed-solid topology.

### 6.17 Shared spatial seams

```
flank_join := seam(flank_region, transition_region)
root_join := seam(transition_region, root_region)
tip_edge := seam(flank_region, tip.wall)
```

A `seam` names an implicit curve shared by generating faces, or by a generating face and
a finite analytic boundary. `first:` and `second:` may label the operands; those fields
alias the dependencies.

With two generating operands, it names the characteristic shared at a tangent vertex of
their generating profile. Each operand is an `envelope` or a `patch` whose source is an
envelope. Both
envelopes MUST name the same generating motion and source revolution. Their profile edges
MUST have exactly one shared vertex identity. Coincident but separately named points do
not establish this relationship, and two copies of the same edge do not define one junction.

After solving, the seam evaluator checks coincident positions and a common tangent plane
at the source vertex with explicit tolerances. The common revolution and rigid motion
preserve this relationship over their shared domain. A corner with distinct tangent planes
is refused. The angular domain is the intersection of both sources' retained spans and
both envelopes' roll intervals; these intersections must have positive width.

The characteristic is still implicit. Its chart is the first envelope's original `[u,v,roll]`,
with `u` fixed at the shared vertex. `EnvelopeSeam::intersect` solves both envelope equations
and one spatial section equation using the existing local solver. `SeamIntersectionOptions`
and its seed contain only the actual unknowns `[v,roll]`; the shared endpoint cannot be freed
by a search option. Both face equations use one normal-velocity tolerance, and the section
has a separate tolerance in its own units. Returned residuals are the first and second
envelope equations followed by the section equation. The final result must
satisfy both envelopes and, when present, both patches' material trims. A rank-deficient
section intersection is refused. Reading a seam does not certify global regularity,
uniqueness or connectedness of its locus.

`EnvelopeSeam::named` reads a solved snapshot. `SeamTolerance` specifies absolute model-length
tolerances for the junction position and clipping-axis recognition, plus the difference
between unit normals for tangent-plane agreement (either orientation is allowed). Point
evaluation and intersections also take explicit normal-velocity and material-boundary
tolerances. These are floating-point checks, not a certified world-space export error bound.
The returned contact normal belongs to the first face, and its velocity is the generating
motion's velocity, not a derivative along the seam curve.

With an `envelope` (or patch of an envelope) first and a `surface` second, the seam names
their intersection. The boundary must support the finite analytic incidence evaluator of
§6.13. It need not share the generating motion, revolution or profile vertex. The operand
order is significant: the seam retains the envelope's original `[u,v,roll]` chart, and all
three parameters can vary. Two arbitrary surfaces or two unrelated envelopes are not
supported by this form.

`BoundarySeam::named` reads one generated-face snapshot with its material conditions and
one boundary projector, using an explicit clipping-axis tolerance. `evaluate` requires
the envelope equation, incidence with the finite boundary patch, and all material trims.
`BoundarySeamTolerance` supplies separate normal-velocity, incidence and trim tolerances.
`intersect` solves the envelope equation, boundary support equation and one spatial section
with `IntersectionOptions`; its residuals and tolerances follow that order. It checks finite
incidence at the retained root, so convergence on an undeclared support continuation is
refused. Local rank checks reject a nonisolated section. Neither a snapshot nor a local root
certifies global transversality, branch uniqueness, connectedness or closed-face topology.

Seams own no solver coordinates and have no planar drawing glyph. Component formals, privacy,
construction semantics, source printing and dependency-aware copy/delete apply. A seam
provides shared boundary geometry; oriented face loops and checked closed-solid assembly
remain separate work.

### 6.18 Shared spatial vertices

```sv
tip_toe := vertex(tip_edge, toe_edge)
join_toe := vertex(flank_join, toe_edge)
```

A `vertex` names the intersection of two distinct seams. Optional `first:` and `second:`
labels name its operands. Two boundary seams must use the exact same named generating
face and different boundary surfaces. Alternatively, one generating junction may meet
one boundary seam whose generating face is exactly one of that junction's operands.
The two seam operands may be reversed. Two generating junctions, unrelated faces and
duplicate boundary surfaces are refused. Separate declarations retain separate identity,
even if their evaluated coordinates coincide.

`BoundaryVertex::named` owns one generating-face snapshot with its material conditions
and two finite boundary evaluators. Its local solve checks the envelope and both boundary
equations over the original `[u,v,roll]` chart, followed by finite incidence and material
membership. `IntersectionOptions` residuals and tolerances follow that equation order.
`JunctionVertex::named` owns one checked generating junction and one boundary evaluator.
It uses `SeamTolerance` for the junction and `SeamIntersectionOptions` for its two unknowns
`[v,roll]`; the first face's endpoint `u` is fixed structurally. Its residuals are the two
envelope equations and boundary equation. Final acceptance checks both material trims and
finite boundary incidence on the particular face named by the boundary seam. Coincidence
within the seam tolerance does not transfer a finer incidence tolerance between faces.

Both snapshot types provide `position` with explicit normal-velocity, finite-incidence and
material-trim tolerances. Results use the canonical generating chart (the junction's first
face for a junction vertex). A vertex has no unique face normal or generating velocity;
its position accessor returns only three coordinates. Local solves check rank, but neither
a declaration nor a local root certifies global uniqueness or regularity. Search bounds
and seeds must select the intended root when there are multiple intersections.

Vertices are spatial, have no planar solver coordinates or drawing glyph, and cannot be
grounded as planar points. Component formals, privacy, construction roles, source printing
and dependency-aware copy/delete follow ordinary entity rules. They establish shared
corner identity; finite edge branches, oriented face loops and analytic solid assembly
remain separate work.

### 6.19 Finite spatial edges

```sv
tip_span := edge(tip_edge, from: tip_toe, to: tip_heel, along: shaft_axis)
toe_span := edge(toe_edge, from: join_toe, to: tip_toe, along: shaft_axis)
```

An `edge` names a seam, two distinct spatial vertices, and a line defining its slicing
direction. Its fields are `seam`, `from`, `to` and `along`, in that positional order.
The endpoint vertices must be incident through declared identities. For two-boundary
vertices, the edge's seam is one of the vertex's operands. At a generating junction,
the same finite boundary also meets the junction's other face: an edge on that face is
incident when it names that exact face and boundary surface. Coincident lookalikes do
not establish this relationship.

The `along` line is read in solved world coordinates and must be finite and nondegenerate.
The endpoints must have distinct projections along it. Edge fraction `s` ranges from zero
at `from` to one at `to`, and specifies the plane perpendicular to `along` at the linearly
interpolated endpoint projection. It is not arc length. Reversing the line's direction
preserves the slices; reversing the endpoint order reverses the fraction.

`edge::SpatialEdge::named` reads an owned snapshot from the declaration, endpoint parameter
witnesses and explicit `EdgeTolerance`. Witnesses use each vertex's canonical chart and
are checked against its full defining geometry and material trims. The reader maps them
into the edge seam's chart by face identity: a generating junction's two endpoint `u`
coordinates need not agree. The snapshot retains one seam and the shared corner positions.
`sample` solves the seam's original equations plus the spatial slicing equation through
the existing intersection solver. It checks local rank, finite incidence and material
trims in the interior, and preserves the exact stored endpoint positions checked by their
full vertex definitions. It does not re-solve an endpoint with a potentially tangent
slicing equation: a regular curve can have zero axial derivative at its endpoint.
The returned parameters belong to the edge seam; its position has no unique face normal.

This evaluator establishes checked local slices of a finite extent. It does not establish
that every slice has exactly one root, that separate local solves stay on one connected
branch, or that all intermediate slices are regular. Endpoint witnesses select local
roots; they are numerical evaluation inputs, not planar coordinates or model hints.
A branch with a turning point along the chosen line needs another chart or subdivision;
rank-deficient requested interior slices are refused. Global branch validation and tolerance-controlled
curve approximation remain required before these edges can bound a certified analytic solid.

Edges own no solver coordinates and have no planar drawing glyph. Formals, private member
geometry, construction roles, printing and dependency-aware copy/delete are ordinary entity
semantics. A spatial edge is distinct from a planar line and cannot be grounded or used as
a planar constraint operand. Oriented face uses are described in §6.20.

### 6.20 Spatial face boundaries

```sv
working := face(toe, tip, heel, join, on: flank_region)
transition := face(round_toe, join, round_heel, root, on: fillet_region)
```

An `on:` operand gives a face an explicit analytic support: a named surface, envelope
or material patch. Every loop operand MUST be a distinct named spatial `edge`; its
seam MUST name that exact support. Coincident but separately declared supports do not
establish incidence. Consecutive edges MUST meet at the same spatial vertex identity,
and the last MUST return to the first without revisiting another vertex. The written
order determines directed edge uses; for a two-edge loop, the first edge's declared
direction chooses between the two possible traversals. The current implementation
accepts one loop and refuses holes, repeated edge uses and `-> close` on spatial faces.

`face.on` reaches the support through an ordinary child reference. Faces retain their
existing entity kind, own no solver coordinates, and follow ordinary component formals,
privacy, construction roles and dependency-aware copy/delete. Spatial faces build after
their finite edges. They are not planar sweep profiles: extrusion, revolution and loft
MUST refuse them rather than infer a default plane.

`spatial_face::SpatialFaceBoundary` reads an immutable support and finite boundary edges
from canonical vertex witnesses and explicit `EdgeTolerance`. Sampling follows the
directed use, retains the edge position and verifies incidence on this particular support.
A generating junction maps its canonical first-face parameter to the second face when
needed. The result contains xyz, this support's parameters and the finite incidence error.
Envelope samples check the envelope equation and material conditions; surface samples
use the finite projector. A coarse junction tolerance does not replace fine face incidence.

A closed loop with valid support incidence is not yet proof of a disk interior, outward
orientation, geometric embedding or a valid solid. Those checks remain separate, as do
global branch selection and tolerance-controlled curve/surface export. In particular,
two different named edges can retrace the same curve and enclose no area.

### 6.21 Sets **[0.49]**

```sv
ball := { p | p distance(12mm) c }

component Cylinder(about: line, r: Length) := { p | p distance(r) about }

component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(half) g
}

shaft := std.Cylinder(ax, r: 8mm)
q coincident ball
l tangent shaft
```

A **set** is the points that satisfy its body (a predicate with one bound variable, §9.9): `{ NAME | STATEMENTS }`, a name for the point —
**bound**: it names whatever is put on the set — then one or more statements about it, on the
line or on lines of their own up to the closing brace. Written as a value, `S := { p | … }`, it
reads the names of the body it stands in. Written as a component's definition, `component
Name(FORMALS) := { p | … }`, it is a **family**: an instance is the set, its formals bound as a
call binds them (§4.1) and read by its name as an instance's are (`shaft.r`, `shaft.about`), a
numeric formal left unbound an unknown of the drawing (§6.3). A set with no statement is a syntax
error.

**A set adds nothing to the drawing**: no unknown, no equation, nothing drawn. What it means is
said where it is used, by two words:

- **A set may be written where it is used** **[0.50]**, on the right of `coincident` or
  `tangent`: `q coincident { p | p distance(5mm) c }`, `l tangent std.Cylinder(ax, r: 8mm)` —
  and is described as written.
- `q coincident S` (either way round) states the body with `q` for the bound point, expanded
  where the statement stands as a component's body is (§14.1). What the body declares privately
  — the cone's generator — is made once for each use. `q` MUST be a point (**E040**).
- `l tangent S` (either way round), `l` a line: a **contact** point standing in space, on `l`,
  is minted for the use and the body is stated at it; and each relation of the body is stated a
  second time as its **derivative** there **[0.51]** — the relation's rate as the contact moves
  along `l`'s direction, the numbers the set was given held. What the body declares of its own —
  a point, or a line not between points it was given — moves with the contact, by a **tangent
  unknown** of its own (its rows, a lift's among them, differentiated too), solved with
  everything else. So `l`'s direction lies in the set's tangent space at the contact: for a
  surface, one condition (the contact's three unknowns against its two rows on `l`, the body's
  rows against the geometry it declares, and the derivatives' against the tangents), the same
  count `distance(r)` from a sphere's centre states, and regular, the contact being an unknown
  of its own rather than a double root of the line's distance. The contact starts at `l`'s
  middle. `tangent` to a set from anything but a line or a set is **E040**, and so is a claimed
  tangency.
- **A set whose body is a circle's is the circle** **[0.54]**: `k := { p | p coincident P; p
  distance(r) o }` — the two statements in either order and either way round, and nothing else —
  with `P` a plane and `o` a point drawn in it, is drawn as `k := circle(center: o)` with `radius(r)
  k`: the circle's entity, rows and topology, its radius drawn `R…` where the body writes the
  number (for a family's instance, where the call gives it, `Round(o, std.front, r: 25)`), dragged,
  dimensioned and swept to a solid as that circle is, `face(k)` included. `q coincident k` is a
  point on the circle; `l tangent k` still reads the body's rows (above), whatever the set is drawn
  as. Where `P` is no plane or `o` stands off it, the body says something else — a line's points
  at a distance, a sphere cut off its centre — and the set is a set, drawn as nothing. Whether `o`
  is drawn in `P` is read once every statement has said where its points are, a use of another
  set included (`o coincident on`, `on := { p | p coincident P }`). *Non-normative:* which of the
  two a set is drawn as changes its rows and its callout, never the drawing's solutions; an
  implementation that drew it as a set and then found it a circle may keep it a set.
- `S1 tangent(at: m) S2` **[0.51]**, two sets: their **tangent spaces at `m` are one** — every
  direction along `S1` there is along `S2`. Two directions are solved for at `m`, in a chart the
  elaborator picks once (each rises along the world axis the sets' normal there is most along),
  and each body is stated as its derivative along both: two conditions for two surfaces, the
  directions' two unknowns against four rows. `m` on each is said beside it (`m coincident S1`);
  the word reads each body only at `m`, so a body that declares geometry of its own is **E040**
  here, and the word without `at:` is **E040**.
- `P tangent(at: m) S` **[0.56]**, a plane and a set: `P` is **`S`'s tangent plane at `m`** —
  `m` on `P`, and the body at `m` stationary along each of `P`'s directions. `m` on `S` is said
  beside it, as for two sets. Anything but a plane before it is **E040**, as is the word without
  `at:`. Where the drawing already says what the body says along `P` — a cone's apex drawn on
  `P` (a cone's body is stationary along its generator), a cylinder's axis parallel to it — one of
  the conditions is the other: the statement is no surplus, and the diagnosis reads the
  dependency as how the tangency is made, said nowhere. *Non-normative:* an implementation finds
  that dependency by rank, not by knowing the shape; a set's library body carries no shape.

Every relation a use states is **described as the statement wrote it** (`l tangent shaft`, §9.9),
its placement and classes the statement's; a dimension the body states is drawn as a component
body's is, and its derivative draws nothing. A set named anywhere else a reference stands —
`radius(5) ball`, a face's loop — is **E040**; `coincident` between two sets is **E040**, with
parentheses **E040**; a set whose body uses itself, however indirectly, is
**E003**. A set may be handed to a component as a `group` formal, as an instance is, and used
there by the formal's name (`std.CircleOnSphere(k, ball, view)`).

**Set algebra** **[0.54]**. A body's statements all hold of its point, so **intersection is
conjunction**: `on_both := { p | p coincident ball; p coincident shaft }` is the points on both, and
the circle above is a plane met with a sphere. **Union is a choice**: a point on `A` or on `B` is
on one of them, which a root choice says (`ccw`, `branch`, §13) and a body cannot. **Difference
is not expressible**: a set says what its points satisfy, never what they do not.

*Non-normative:* `std.Sphere`, `std.Cylinder` and `std.Cone` are sets. A set drawn by tracing its
points, other recognised shapes (a line as two planes met), and sets bounded by inequalities (arcs,
segments, rays) are not part of this draft (#101). A derivative row has no flat spelling, so a
program lifted from a sketch (`to_program`) keeps a tangency's contact and says nothing of the
tangency.

## 7. Ports **[0.13]**

**Retired.** Everything an instance makes is reachable by its dotted name — `inst.p` for a point of the body, `inst.sub.p` for a nested instance's, `inst.s[0].p1` for one inside a block's copy — so a port was a second name for a thing that already had one (bmander/geomsolver#47). Its three forms are written as what they were:

| was | is |
|---|---|
| `port lo: point hint((0, 0))` | `lo := point hint((0, 0))` — a declaration of the body |
| `port hub = c` | nothing; the caller writes `inst.c` |
| `port p = (xexpr, yexpr)` | `p := point(x: xexpr, y: yexpr)` — a computed point (§6.5) |

An implementation MUST refuse `port` with a message naming these forms. Aliasing is untouched, being a property of argument passing and not of ports (P1): passing one instance's entity to another as an argument still merges the two into one alias class.

*Non-normative:* joints between components are ordinary constraints written at the assembly site, e.g. `gear.hub coincident shaft.j3`. The language deliberately has no "connect with joint" primitive.

---

## 8. Instances

```
t := Tooth(root, tip, slot: tau/N)
```

Instantiation elaborates the named component's body into the current scope with formals bound per §4.1. Instance elaboration is recursive; cyclic instantiation is an error (**E003**).

### 8.1 Groups of arguments

```solvent
unit mm
use std

sizes := {length: 20mm}
layout := {frame: axes.axes, ax: axes.u, origin: o}

component Bar(layout: group, dims: group) {
  tip := point hint(at: layout.frame, (dims.length, 0mm))
  ax := line(layout.origin, tip)
  ax parallel layout.ax
  distance(dims.length) ax
}

in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((30, 40)) q
  axes := std.Turned(o, q)
  bar := Bar(layout, dims: sizes)
}
```

**[0.35]** `NAME := {member: VALUE, …}` bundles named numeric values, entity references, and
nested groups. A brace straight after `:=` opens a group, never a body: its members are separated
by commas and may run across lines, as an argument list's may. **[0.36]** A member's value may
itself be a group in braces, `design := {bar: {at: o, size: {length: 2cm}}}`, to any depth: its
members are the enclosing group's under the member's name (`design.bar.size.length`), and the
member is a group in its own right, which a call may be given (`Bar(design.bar)`). Written in
place or defined by name and referred to (`bar := {…}`, `design := {bar: bar}`), a nested group
is the same group. Braces are a definition's and its members', never a call's argument. Members are required to have labels; duplicate members and cyclic definitions are
errors. Numeric members retain their dimensions. A group creates no geometry, solver variable,
or constraint. Geometry members are aliases to existing geometry, including subentities and
members of repeated instances; grouping does not copy or solve them separately.

A `group` formal is required and accepts a group or a component instance. Passing an instance
exposes its geometry by the same dotted paths available at the assembly site: `layout.origin`,
`layout.bank[0].pivot`. Parameters inside the layout instance are local; use an explicit group
for numeric values. Groups may be passed positionally before labels or by label. A component
may forward a group or select a nested group for another call. A missing member is an error,
not an implicit solver unknown. The component's uses determine the member types required;
there is no nominal record type or new solve boundary.

Curves compiled over a component still require fixed scalar or entity formals. A group has no
fixed coordinate layout, so its needed members must be passed individually to a traced component.


---

## 9. Constraints

### 9.1 Equational form

```
<expr> == <expr>
```

The reference implementation does not yet parse an equation statement; a relation is written in §9.2's form. Both sides are dimension-checked. The residual is `lhs − rhs` (componentwise for future vector expressions; in this draft `==` applies to scalar-dimensioned expressions and to `Point` via `coincident`, §9.3).

### 9.2 Operator form **[0.7]**

> **Every constraint is written as a prefix or an infix operator.  `name(args…)` is retired.**

Where a constraint needs more than its two operands — a number, a selector, a third entity — those go in parentheses **on the operator**:

```
horizontal line1                    point1 horizontal point2
radius(25) circle1                  point1 distance(1' 3") point2
distance(6) line1                   point1 symmetry(line1) point2
distance(w / 2) line1               point1 distance(60, along: x) point2
fix((0, 0)) p1             l1 angle(30) l2
fix(r == 25) c                      line1 tangent(side: left) circle1
```

**The type system already has this shape.** Every constraint a person writes has 1 or 2 entity slots, always first in spec order: 1 for the prefix words (`horizontal`, `vertical`, `radius`, `length`, a cone's `angle`, a line's `distance`, `fix`), 2 for everything else, and 3 for symmetry alone — which the parentheses absorb exactly as proposed. So "two operands, the rest in parentheses" is a *description* of the library rather than a rule imposed on it.

What goes in the parentheses is a short list:

- **the number**, which may be an expression exactly as elsewhere: `distance(80)`, `distance(w / 2)`, `distance(1' 3")` — **[0.41]** it names nothing; a name is `w := 60` (§6.3);
- **a selector** — `side: left`, `at: start`, `external: true`, `along: x`. **[0.17]** A selector's *key* must be one the word has (a slot of the settled kind), and its *value* must be one of the words that slot takes — **[0.46]** `along:` takes a word or a reference to an axis or a line, a name outside its words being a reference (§9.3) — both **E040**, at the key. Neither was checked through 0.16, and both failures were silent: a mistyped key was dropped and the statement settled without it, and a word outside the set fell through to whichever reading the implementation tested for last, so `at: banana` meant `end`. An implementation MUST publish each slot's vocabulary in its registry, so that a front end offers what the core accepts rather than keeping a second list;
- **the third entity**, for `symmetry`, and **[0.46]** the direction of `level`, which may be a word of `along:`'s (`level(up)`);
- **a pin**, `t == 0.4`, for a slot the constraint owns. Its *seed* is the trailing `hint(t: 0.4)` where every seed in the language is (§4.3). **[0.39]** A pin to an unknown — `t == s` over `param s: Angle` (§3.3, §6.3) — does not hold the slot at a number: it makes the slot's unknown **that one**, as a dimension reading it is written in it, and every contact pinned to the same unknown owns it. Inside a component the unknown is a formal the call left unbound (`leg.s`). **[0.41]** A pin to a name nothing declares is **E101**. Only a bare name is shared (an expression over one would need an equation the slot does not have), only a curve contact's parameter (`t`) can be, and only by contacts on the **same** curve — two curves have two intervals, seams and speeds — so a contact on another curve pinned to the name is **E040**, and so is a `fix` pinned to one, since `fix` holds a number. **[0.41]** The shared unknown is seeded where it is declared; a `hint(t: …)` beside a pin to it is a second seed, **E040**.

| word | fixity | operands → constraint |
|---|---|---|
| `coincident` | infix | **[0.42]** (point, point); (point, line \| circle \| arc \| spline \| curve) — incidence, where 0.41 and before wrote `on`; **[0.24]** (point, plane), (line, plane) in space — **[0.54]** of a point standing in space, its membership of the plane, as `in` (§6.7); **[0.49]** (point, set), the set's body at the point (§6.21); **[0.42]** (point, axis), (axis, plane); **[0.43]** (axis, axis), one line either way round; **[0.44]** (line, axis), both ends on it; **[0.38]** (point, the envelope of a prism's side), from any view (§6.15.2). Read either way round: `P coincident p` is `p coincident P`. **[0.32]** Not (solid, solid): the body rule's union is `union` (§6.9) |
| `cut` | infix | **[0.18]** (solid, solid) — the body rule's other half (§6.9), and no constraint at all |
| `distance` | infix | (p, p); **[0.46]** +`along:` an ordinate — an axis or a line, or the view's `x`/`y` for the run and the rise (`right`/`left`/`up`/`down` with the sign said); (p, line); (line, line); (circle, circle); (p, plane) with `along: u`/`v` for signed ordinates from the plane's origin, or **[0.24]** `along: n` for the signed distance along the plane's normal, in space; **[0.42]** (plane, plane), the second's origin along the first's normal (§6.10) |
| `distance` | prefix | on a line: the distance between its own ends |
| `tangent` | infix | (line, circle); +`at:` for a tangency at a named end; (circle, circle); (arc, line); (spline, line); (curve, line), §6.5 — **six**; **[0.49]** (line, set), a contact on the line in the set and the body's derivative along the line; **[0.51]** (set, set) with the point in the parentheses, `k1 tangent(at: M) k2`, one tangent space there (§6.21) |
| `equal` | infix | (line, line) a length; (circle, circle) a radius |
| `curvature` | infix | (spline, circle), (curve, circle) |
| `horizontal`, `vertical` | prefix | a line. **[0.48]** Between a pair of points they are the standard library's relation words (§9.9), `level(up)` and `level(right)`: `use std (horizontal, vertical)` |
| `level` | infix | **[0.46]** (p, p), the direction in the parentheses — an axis, a line, or the view's word; (p, plane) with `u`/`v`, from the plane's origin |
| `angle` | infix | (line, line); **[0.27]** (line, line) with a second pair in the parentheses, `l1 angle(l3, l4) l2` (§9.4); **[0.42]** (axis, axis \| line), (line, axis), the unsigned angle in space |
| `radius` | prefix | a circle or an arc |
| `length` | prefix | **[0.27]** an arc: its length along itself; **[0.53]** a spline: its whole length |
| `midpoint`, `parallel`, `perpendicular`, `symmetry` | infix | one each; **[0.42]** `parallel` and `perpendicular` also take an axis beside an axis or a line, in space, and (axis, plane) — along the plane, and square to it — either way round; **[0.43]** `parallel` takes (plane, plane): the two face alike, either way (two rows over their normals; neither's place nor its turn within itself) |
| `project` | infix | (point, point), each `in` a plane — the two planes are read off the memberships and never written (§6.7) **[0.10]** |
| `fix` | prefix | the gauge (§13): an entity, and its own numbers pinned whole or by member — `fix((0, 0)) p`, `fix(x == 0) p`, `fix(r == 25) c`, `fix(dir == (0, 0, 1)) t` **[0.34]** **[0.45]** |
| `ccw`, `cw` | call | three points, all in the parentheses (§9.6) |

The collapses are where the saving is: **`coincident` is fifteen constraints, `distance` nine, `tangent` six** (in space included, and a set's uses besides, §6.21), and `horizontal`/`vertical` are two each with the **fixity** doing the work — a line prefixed, a pair of points infixed, the second **[0.46]** a `level` along the view's own axis. `angle` and `radius` keep their own words rather than folding into `distance`, because over two lines a length means a parallel distance and an angle means an angle, and nothing but the number's unit could separate them.

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end; `line tangent circle` is the ordinary one. Each named itself before and the order was decoration; as an operator, which side the arc is written on picks the constraint.

**What a word means is the kinds of its operands, and a name does not carry its kind until elaboration** — a name may be declared further down the body (P2) or come from a component. So an implementation MUST settle the word after resolution, and MUST report a pair a word does not relate rather than guessing at one.

**The surface word and the wire name are separate.** An export format's constraint identifier is unaffected by this section; the operator is information beside it.

**`ccw` and `cw` keep a call.** Under the general rule they would be `a ccw(c) b`, which reorders three points that are symmetric: the predicate is about the *triangle*, not about a pair with a decoration. The call is a third fixity of the same table — every operand in the parentheses — and not a statement kind of its own. **[0.15]** The gauges and the orientation predicates are entries of the operator table like every other constraint: read by the one relation grammar, so a class, a placement and the chain's lookahead treat them as any other word, and settled by the word alone, since `fix(r == 25) c` names a number and `ccw(a, b, c)` has no operand outside its parentheses. They hold parameters or record a root choice rather than adding an equation, so a `claim` on one is refused (E040): a claim is judged by rank, and they add no row.

**[0.18] The body rule is written in this grammar and is not a constraint.** **[0.32]** `boss union cyl`, `bore cut cyl` and **[0.22]** `tip bound cyl` are **Declaration**-class (§4.2): each says what its right operand *is* (§6.9), contributes no residual, and takes no part in a solve, a decomposition or any partition of work. None relates geometry or has a residual to be settled into, so each is read by the word alone. None is in the constraint library of §9.3, and none may be `claim`ed: a claim is judged by rank and these add no row, which is the rule already stated for the gauges. `claim a cut b` and `claim a union b` are refused where they are written, neither being a constraint word. (0.18–0.31 spelled the union `on`, settled by the kinds of its operands; `coincident` between two solids is now refused as any constraint word is over operands it does not relate.)

**[0.24] Across planes, the same word is the relation in space.** A relation's operands are read in the planes their points are drawn in (§6.7). Within one plane it is the 2D relation, and the lift is rigid, so the 2D relation *is* the relation in space. **[0.42]** Where they differ, or where any point stands in space, the word means its relation between the points' lifts: `coincident`, `distance` between two points (the true length), a point and a line (to the infinite line) or two lines (the common perpendicular — a magnitude, its side the seed's), `coincident` a line or a circle, `angle` (unsigned, 0 to 180°), `parallel`, `perpendicular` and `equal` (true lengths), and **[0.25]** `midpoint` (of the line in space) and `symmetry` (the half turn about the line in space, which on the line's own plane is the mirror). No selector says so. `sense:` and `side:` name a turn and a side *in a plane* and are **E040** there; a word that has no meaning in space — `horizontal`, `vertical`, a run or a rise (**[0.46]** a direction named by the view's word: name the axis, `along: std.x`), `tangent` between drawn figures, a curve's contacts — is **E062**. A radius and a ring's width read only radii, which the lift carries unchanged, so they mean the same in any plane; `along: u`/`v`/`n` are measured from a plane in space wherever the point is (§6.7); and `project` relates two planes by definition. A relation over an axis or a plane is in space whatever plane its points are drawn in.

**[0.42] The role rule is withdrawn.** 0.24 read a plane's own datum points — its origin and its `toward` — by what they were related to, because they placed the view on the sheet. A plane has no `toward` and no place on the sheet now; its origin is a point drawn in it like any other.

A chain (§6.6) is the same grammar: a **lone infix statement is a one-joint chain**, and what a chain adds is the corner — which end two links meet at — that an operator between two names cannot know.

### 9.3 Standard constraint library

Residual conventions: points are ℝ²; `×` is the scalar 2D cross product; `∠(u, v)` is the signed angle from `u` to `v` in (−π, π]. In a row in space, capitals (X, A, o_P) are lifts in ℝ³ and `×` the vector cross product; a plane's û, v̂, n̂ are §6.7's.

| Predicate | Residual(s) | Eq. count | Notes |
|---|---|---|---|
| `coincident(C: Circle, p: Point)` | ‖p − C.center‖ − C.r | 1 | |
| `coincident(L: Line, p: Point)` | n(L)·p − d(L) | 1 | |
| `coincident(p, q)` | p − q | 2 | for *distinct* entities; see **W100** |
| `distance(p, q) == e` | ‖p − q‖ − e | 1 | |
| `angle(a, b, c) == e` | ∠(a−b, c−b) − e | 1 | signed; see §9.4 |
| `angle(L1, L2) == e` | wrap(∠(dir(L1), dir(L2)) − e) | 1 | directed, mod 2π; see §9.4 |
| `angle(L1, L2) == angle(L3, L4)` **[0.27]** | wrap(∠(dir(L1), dir(L2)) − s·∠(dir(L3), dir(L4))) | 1 | written `l1 angle(l3, l4) l2`; s = −1 under `sense: cw`; §9.4 |
| `length(A: Arc) == e` **[0.27]** | A.r·θ − e | 1 | θ the sweep counter-clockwise from `start` to `end`, in (0, 2π]; a magnitude |
| `parallel(L1, L2)` | sin(L1.dir − L2.dir) | 1 | |
| `perpendicular(L1, L2)` | cos(L1.dir − L2.dir) | 1 | |
| `tangent(C1, C2)` | ‖c1−c2‖ − (r1 + r2) *or* ‖c1−c2‖ − \|r1 − r2\| | 1 | branch by decoration, §9.5 |
| `tangent(C, L)` | dist(C.center, L) − side·C.r | 1 | `side: left \| right` **[0.17]** |
| `equal(e1, e2)` | e1 − e2 | 1 | any matching dimension |
| `midpoint(m, a, b)` | m − (a+b)/2 | 2 | |
| `ccw(a, b, c)` | (b−a) × (c−a) > 0 | 0 | inequality; selects a connected component |
| `cw(a, b, c)` | (b−a) × (c−a) < 0 | 0 | |
| `coincident(p, P: Plane)` **[0.24]** | n̂_P·(X − o_P) | 1 | X the point's lift; in space whatever plane `p` is in; `p` drawn in `P` is E061 |
| `distance(p, q, along: t) == e` **[0.46]** | (q − p)·t̂ − e; in one view along its own axes q.x − p.x − e (q.y − p.y); across views (Y − X)·t̂ − e | 1 | t̂ the unit direction of axis or line `t`, signed along it; `x`, `y` name the points' view's axes |
| `distance(p, P, along: n) == e` **[0.24]** | n̂_P·(X − o_P) − e | 1 | the ordinate from `P.origin` along `P`'s normal |
| `distance(p, P, along: u) == e` **[0.21]** **[0.42]** | p.x − e; across planes (X − o_P)·û_P − e | 1 | the ordinate from `P.origin` along `P.u`; `along: v` likewise; §6.7 |
| `level(p, q, t)` **[0.46]** | (q − p)·t̂ | 1 | the ordinate's zero, written `p level(t) q`; `p horizontal q` is `p level(up) q` |
| `distance(P, Q) == e` **[0.42]** | (o_Q − o_P)·n̂_P − e | 1 | signed; parallel only where their axes make them so (§6.10) |
| `coincident(t: Axis, p)` **[0.42]** | two components of (X − A) × d̂ across the axis | 2 | A the axis's place, d̂ its direction |
| `coincident(l: Line, t: Axis)` **[0.44]** | two components of (X − A) × d̂ across the axis, for X each of the line's ends | 4 | A the axis's place, d̂ its direction; a line of no length is still on it |
| `coincident(t: Axis, P: Plane)` **[0.42]** | n̂_P·(A − o_P), n̂_P·(A + L·d̂ − o_P) | 2 | the axis lies in `P`: two of its points, L the drawing's extent apart |
| `parallel(t: Axis, P: Plane)`, `perpendicular(t, P)` **[0.42]** | n̂_P·d̂; (n̂_P × d̂) across n̂_P | 1; 2 | along `P`, and square to it |
| `coincident(t: Axis, s: Axis)` **[0.43]** | two components of (d̂_s × d̂_t) across `t`, scaled by L; two components of (A_s − A_t) × d̂_t across `t` | 4 | one line, either sense; L the drawing's extent |
| `parallel(P: Plane, Q: Plane)` **[0.43]** | two components of n̂_Q × n̂_P across n̂_P | 2 | either way round; neither plane's place nor its turn within itself |
| `coincident(L, P: Plane)` **[0.24]** | n̂_P·(A − o_P), n̂_P·(B − o_P) | 2 | A, B the line's two ends, lifted |
| `coincident(q, S)` **[0.49]** | the rows F of S's body, with q for its point | as many as the body states | §6.21; the geometry the body declares is made once per use |
| `tangent(L, S)` **[0.49]** **[0.51]** | F at a contact X on L (two rows putting X on L), and F_x·ẋ: ẋ the direction B − A in X's columns, a **tangent unknown** in the columns of what the body made, 0 in the set's own | 1 for a surface | the derivative read exactly from each row's form (its Jacobian, and its Hessian along ẋ); the body's own rows (a lift) differentiated too |
| `tangent(S1, S2, at: m)` **[0.51]** | F₁,ₓ·wₖ and F₂,ₓ·wₖ at m, k = 0, 1, with w₀ = e_a + s₀ e_c and w₁ = e_b + s₁ e_c | 2 for two surfaces (four rows over s₀, s₁) | e_c the world axis the sets' normal at m runs most along, chosen once; `m coincident S1`, `m coincident S2` beside it |
| *across views* **[0.24]** | the relation over the lifts X, Y of its points: X − Y (3); ‖X − Y‖² − e² (1); the point–line magnitude (1); the signed common perpendicular (1); two components of (X − A) × (B − A) across the line (2, `coincident` a line); â·b̂ − cos e (1); â·b̂ (1); (â × b̂) across â (2, `parallel`); ‖B − A‖² − ‖D − C‖² (1, `equal`); ‖X − C‖ − r and n·(X − C) (2, `coincident` a circle); **[0.25]** X − (A + B)/2 (3, `midpoint`); Q + P − 2F, F the foot of P on the line (3, `symmetry`) | as listed | §9.2 |
| `project(p, q)` **[0.10]** **[0.42]** | d_A·p − d_B·q + d·(o_A − o_B); over a plane solved for, (n̂_A × n̂_B)·(X_A − X_B) | 1 | the planes A, B inferred from `p`, `q`'s memberships; d the fold line they share, §6.7 |

Implementations MAY extend this library. Extensions MUST document residuals and equation counts, and MUST classify each decoration as hint or constraint per P3.

### 9.4 Signed angles

*Not yet spelled:* §9.2's table has no form for an angle at a vertex of three points, and the reference implementation has none.

`angle(a, b, c)` is the signed turn at vertex `b` from axis `b→a` to axis `b→c`, positive counterclockwise, in (−π, π]. Equating it to an expression is a 1-equation constraint. Programs that need the unsigned angle write `abs(angle(...))`; implementations MUST warn (**W102**) that `abs` introduces a branch (two solution families) unless an orientation predicate elsewhere disambiguates.

**[0.17]** `sense: cw` turns the number a statement writes: `l1 angle(30, sense: cw) l2` states −30° and is the spelling a drawing SHOULD use, the minus being a coin a reader cannot check. An implementation MUST draw the figure from the number the statement *makes* — the arc sweeping the way the label reads.

`angle(L1, L2)` between two lines is likewise directed: `∠(dir(L1), dir(L2))`, the signed turn from `L1`'s direction (p1→p2) to `L2`'s, positive counterclockwise, in (−π, π] as everywhere else in this section. It is NOT a statement mod a half turn — the residual pins which side, so a bearing needs no orientation predicate beside it — and it is therefore sensitive to the order of the two lines and to the endpoint order each was declared with. Equating it to `e` compares the two mod 2π, so `e` may be written on any lap: 270° and −90° state the same thing, and an implementation MUST NOT treat a stated angle outside (−π, π] as an error.

**[0.27]** The number may be another such angle: `l1 angle(l3, l4) l2` states `∠(L1, L2) = ∠(L3, L4)`, both directed, compared mod 2π, and `sense: cw` equates the first with the second's negative — its mirror image. A statement holds at most one number, so two unlabelled items in `angle`'s parentheses are the second pair or an error. It relates turns in a plane and has no reading across views (**E062**).

### 9.5 Branch decorations are constraints

Several predicates have discrete solution branches. Branch selection changes the solution set and is therefore constraint-class (P3), written as a selector on the word:

```
c1 tangent(external: true) c2     // external tangency: ‖c1−c2‖ = r1 + r2
c1 tangent(external: false) c2    // internal tangency: ‖c1−c2‖ = |r1 − r2|
```

A selector left unwritten is read off the seed (§9.2): the branch the seeded pose stands on is the one stated, as a side is for a distance from a line. (0.1 wrote the branch as a decoration, `tangent.ext(c1, c2)`, and refused an undecorated `tangent` as **E010**.)

### 9.6 Inequalities

Orientation predicates (`ccw`, `cw`) are the only inequalities in this draft. They contribute no equations; they select among the discrete solution components of the equality system. Solvers MUST verify them on candidate solutions and MUST NOT report a solution violating one.

### 9.7 Claims **[0.5]**

```
claim <relation>                  // any relation statement of §9.2: claim a distance(5) b
```

A **claim** is a relation stated as *expected to add no rank*: an assertion about the drawing
the rest of the document determines, not part of what determines it. The altitudes of a
triangle concur; the trace of a Peaucellier cell is straight — a claim is how a document says
so out loud and has the statement checked, rather than smuggling the theorem in as one more
constraint and hoping the diagnosis reads the intent.

A claim MUST NOT act. Solvers MUST exclude claims from the equation system, from
decomposition, and from any connectivity used to partition work (a claim spanning two figures
does not join them): the solution set, the degrees of freedom, and every diagnostic class of
the surrounding document are exactly what they would be with the claim deleted. In
particular, a claim never makes a document over-constrained or in conflict.

A claim MUST be judged. At a solution, diagnosis classifies each claim as exactly one of:

* **theorem** — the claim holds and its residual rows add no rank to the system: the document
  already implies it;
* **violated** — the claim does not hold at this solution;
* **consuming** — the claim holds, but its rows add rank: only the pose satisfies it, and
  enforcing it would have removed a freedom. The claim claims too much.

A claim MUST NOT introduce unknowns: a relation whose signature carries a solver-owned
parameter (a curve contact's `t`) is an error as a claim (**E040**), since its unknown would
appear in no equation. A claim's dimension may be an expression, but MUST NOT bind a free
variable, for the same reason.

`claim` qualifies a single longhand relation statement; it does not enter chains (§6.6).

### 9.8 Claims about solids **[0.18]**

A claim is a statement judged and never solved (§9.7).  Everything §9.7 says of a claim about the drawing holds of a claim about the **object**, and the reason is one stratum out rather than new: a solid is evaluated after the drawing is solved, so a statement about one compiles no row and can no more act than a `claim a project b` can.

Three words relate two solids.  Each is an operator like any other (§9.2), and each settles to no constraint kind, because a constraint kind is a thing with a kernel.

| written | asks |
|---|---|
| `a clear(d) b` | `a` and `b` are disjoint, and no point of one is nearer than `d` to the other |
| `a inside b` | every point of `a` is a point of `b` |
| `a fits(d) b` | `a` is inside `b`, with no point of it nearer than `d` to `b`'s boundary |

Both operands MUST name solids (**E040** naming the kind that arrived).  `clear` and `fits` take a `Length` in their parentheses and one written without it is **E040**; `inside` asks about containment and takes none.

**A verdict is a measurement, not a yes or no**, and it carries its own uncertainty.  An implementation MUST report, for each such claim: what was measured — a distance, negative where the two overlap — and how far the answer could be wrong.  A claim decided within that margin is reported **undecided**, which is a third answer and not a failure.  Where an implementation evaluates a solid by reducing its round surfaces to flats (§6.9), the margin is the sagitta of that reduction, and a faceted solid lies inside the true one; an implementation that computes exactly reports a margin of zero.  Two claims that fail, one by a hair and one by a hand's breadth, are different drawings, and a reader is owed the difference.

*Implementation note:* the kernel measures interference as **common-material thickness**: the diameter of the largest ball contained in the intersection, reported with a negative sign. Thus two 10 mm cubes overlapping by 5 mm report −5 mm, and identical 10 mm cubes report −10 mm. A bounded spatial search includes its remaining measurement error in the reported uncertainty. This convention applies to disconnected and nonconvex intersections too; disjointness and containment remain required predicates, whatever sign is written for a gap.

The trichotomy of §9.7 does **not** apply here.  *Consuming* asks whether enforcing a claim would take a freedom, and there is no rank to take: a solid claim holds, is refuted, or is undecided.

**A claim over a sweep.**  Every claim in the language is judged at one pose, and a fact about a *cycle* — a disc clearing a cylinder's mouth all the way round, a port open through mid-stroke — is not one of those.

```
claim over crank.theta in (0deg, 360deg) {
  crank.disc clear(1mm) bankA.cyl.body
  bankA.pis.body inside bankA.cyl.bore
}
```

`claim over NAME in (A, B) { … }` judges every claim in its body as the drawing runs along `NAME`, and reports the **worst** pose reached.  It is Structure-class: it says how the claims inside it are judged and asserts nothing itself.

- `NAME` MUST be an **unknown** of the drawing (§3.3, §6.3) — an input with no value, or a formal an instance left unbound — or **[0.40]** a named motion (§6.14), whose roll the claims run along: every solid placed under it (`solid(S, under: NAME, at: A)`) is read at its angle advanced by the roll, and nothing is solved again.  A value or an input with a value is a number the document already fixed and sweeping a constant is not a question; naming one, or naming other geometry, is **E040**.
- `A` and `B` are read in the units the unknown's readers are written in: an interval of an angle is an angle, and one of a length is a length.
- An implementation MUST state that its answer is by **sampling**, and how many poses it took. A pose that did not solve or produced invalid solid geometry cannot certify the claim; the report identifies those failed parameter values, and a sweep with unresolved poses cannot be reported as holding.  A claim that holds at every sample is a claim that held at every sample; a swept claim is honest about that in the way a faceted one is honest about its margin.

*Non-normative:* the two together are what make a drawing's claims a test suite for the *object* rather than for one picture of it at one moment.  The loop an author works in — write, run, read the verdicts — needs the verdicts to be about the thing being made.

### 9.9 Relation words **[0.48]**

```
a horizontal b := a level(up) b
flat l := l perpendicular t
a above(d) b := b distance(d, along: up) a
```

A **relation word** is defined at the top level of a file — never inside a body — as a component is: the word between its two operand names (infix) or before its one (prefix), its parameters in parentheses after it, `:=`, and the **one** relation it stands for. A statement writing the word, `p horizontal q`, `r above(d: 7mm) q`, is that relation with the operands and the parameters put in, expanded where the statement stands (§14.1) as a component's body is.

- **Operands are entities**, aliased as a component's entity formals are (P1); their kinds are checked by the relation they reach, where the word is expanded (`p above(d: 1) l` of a line is **E040**, as `l distance(…, along: up) p` would be). A body may read an operand's members (`a.p1`).
- **Parameters are numbers or selector words**, given **by label** where the word is written, as a call's numbers are (§4.1): a value given by position is **E004**, a label the word has not is **E040**, and a parameter left out is **E040**. Whether a parameter is a number or a word is how its body reads it: a name read in a dimension, a pin or a seed is a number (written in as the text it was given, read where the word is written), and one standing where a selector's word or a direction does (`side: s`, `level(s)`, `along: s`) is a word; one read both ways is **E040**.
- **The body is closed** over its operands and parameters (§5): a reference or a name it reads that is neither — another entity, a value of the file, a datum — is **E101** at the definition. A word whose body states another defined word is that word's expansion in turn, resolved from the file the body is written in; a word reached again while it is being expanded is **E003**.
- **An error inside the expansion is reported where the word is written**: every place in the body is the word's own place in the statement that writes it, and a dimension that is one of the word's parameters is the argument the statement gave it — so its callout shows that argument, and editing the callout edits it.
- A word that is a word of the language's own — an element keyword, a trailing clause, a modifier, a body word, a word opening a statement — or a constraint word of the language **of the same fixity** is **E071**; so is a word defined twice in one file with one fixity. An infix word and a prefix word of one spelling are two words: `std`'s `a horizontal b` stands beside the language's `horizontal l` of a line.
- A word nothing defines where it is written is **E102**, naming the `use` that would import it where a module defines it.
- The statement keeps its identity: its placement, classes and `claim` are the statement's, and a constraint the body states is **described in the word as written** (`p horizontal q`), not its expansion. A body states its relation alone: a placement or a class in it is a syntax error.

**A body may be several statements** **[0.50]**: `l1 flush(d) l2 := { l1 parallel l2; l2.p1 distance(d) l1 }`, in braces, one per line, for one fact about the operands that takes several rows. It may declare the geometry its relations need (`private ab := line(a, b)`), which each use makes anew, under a name of the use's own, and read only what it declares besides its operands and parameters. Every relation a use makes is the statement's: deleted with it, described as it, a placement on it going to the one relation when it makes one. A body may write other words and use sets (§6.21), a set written in place included (`a orbits(r) o := a coincident std.Sphere(o, r: r)`). An infix word is a relation **between** its operands — distance, symmetry, flushness; that each stands on a third thing is said of each (`a coincident P`), not made a word.

**A word and a set are one predicate** **[0.50]**: a body over parameters (in parentheses, given at the call) and bound variables (a word's operands, a set's point, written where the definition shows them and filled at the use), applied by one rule — under a name of the use's own, each bound variable the operand the use wrote, the body closed over what it was given. A component instance is the case with no bound variable, stated where it is written. An operand that names nothing is **E101** once, where it is written, and not again at each place in the body that reads it. *Open, non-normative:* a word as a value (#80). The standard library defines `horizontal` and `vertical` between two points as words (§14.4), and where one point stands from another (`b offset(dx: 30, dy: 12) a`, `b right_of(d: 30) a`, `left_of`, `above`, `below`), where a point stands in a plane's frame (`p coords(du: 20, dv: 5) P`, `p on_u(d: 20) P`, `p on_v(d: 5) P`), two lines crossed (`a skew(theta: 90deg, e: 20mm) b`) and one line turned from another (`a turned(theta: 180deg) b`). Each is two dimensions or a dimension and a level, said as the one fact they are. **[0.53]** A body may state an energy over an operand (§9.10): `std.hangs(L: 150mm) rope` is a curve that long, hanging.

### 9.10 Energies: `minimizes` and `maximizes` **[0.53]**

```
rope minimizes integral(p.y over p)
strip maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t))
rope minimizes 2 * integral(p.y over p) - integral(p.x over p)
```

`k minimizes E` is a statement about the curve `k`, as `horizontal l` is about a line: the word is indicative, never an instruction. Its **energy** `E` is a sum of terms `c * integral(EXPR over p)`, each a constant `c` (1 where none is written) times an integral along `k` of `EXPR`, read at the point `p` running along it — and, written `over (p, t)`, at its unit tangent `t` there — **weighted by arc length**. `EXPR` reads `p.x`, `p.y`, `t.x`, `t.y` (the point's coordinates in the view `k` is drawn in) and numbers in scope; any other name is **E101**. Where the document names a unit the point is a length and `EXPR` is dimension-checked: a whole power of length, every term of one energy the same (**E103**).

`k minimizes E` states that **`k` has the shape that makes `E` stationary** among every shape from its start to its end of its length: **[0.55]** the solution of `E`'s Euler–Lagrange equation — in arc length `s`, with the tangent at angle `θ`, the Hamiltonian `H = f(p, t) + λ·t` stationary in the direction, `p' = t`, `λ' = −f_p`. `k maximizes E` is `k minimizes -E`. The curve's **ends** are what it hangs between, so they keep the freedom they have: the curve is a function of its ends and its length, and adds no freedom of its own to the drawing (§16.3). Several energy statements over one curve are **one energy**, their terms added: order-free (P2), as a body's operations are. What is solved for is stationarity; that the stationary shape is the extremum the word names is **judged**, as a claim is (§9.7, below).

- **What it is over.** **[0.55]** An energy is over a free curve (§6.1); over anything else it is **E040**.
- **Its length.** Held by `length(L) k`, the length is that. With none, it is where `E` is stationary in it too — transversality, `H = 0` at the end: two points and an energy with no length is a geodesic of the weight `f`.
- **A peg presses; a drawing relation yields.** A relation between the curve and other geometry that is still free is satisfied by that geometry: a line drawn tangent to a hanging rope moves onto it. **[0.55]** A held point the curve is stated to pass (`peg coincident rope`, `peg` held) is a **peg**: the curve's problem passes it, its costate free to jump there — a point force — and its direction with it: the rope drapes over the peg in a corner, the place along the rope where it does solved too. Any other relation whose every operand but the curve is held is **E040**: a held line pressed against the curve meets it at a corner, so no smooth tangency to it is stationary.
- **The solution set is the stationary points (P3).** A stationary curve may be a minimum, a maximum or a saddle, and the seed decides which the solve reaches. The diagnosis MUST say which — **[0.55]** by Legendre's condition (the Hamiltonian's sign in the direction along the curve), Jacobi's (no point conjugate to an arc's start before its end), and, with pegs or a free length, the energy's Hessian in where they are — and SHOULD say it where the statement is written, as a claim's verdict is (§9.7): `minimum`, `maximum`, `saddle`, `degenerate` where the second order cannot tell, or `unsolved` where no stationary shape was found. A `maximizes` answered `maximum` found what it asked for. A non-minimum is reported, not refused.
- A relation word's body may state an energy over its operand: `std.hangs(L) k := { length(L) k; k minimizes integral(p.y over p) }`.

*Open, non-normative:* an integral as a constraint of its own (`integral(…) == A`), an integrand reading other geometry or the curvature (the elastica, #136), a natural end condition (an end free to slide, varied with the curve), a held line touched at a point the curve chooses (a corner on the line), and a free curve in space.

---

## 10. Paths

Paths are the one ordered construct (P2, exception). The reference implementation does not parse them yet. A path is a directed traversal of vertices connected by segments.

### 10.1 Grammar

```
path outline: ccw = lead -> tl ~tip~ tr -> trail
```

- `path NAME : ORIENT = PATHEXPR` declares a named path with orientation `ccw` or `cw`.
- A bare `PATHEXPR` statement is an anonymous **path fragment** (used for splicing, §10.5).
- Segments: `a -> b` is a straight segment; `a ~C~ b` is an arc on circle `C`; `a ~C rev~ b` is the reversed-branch arc (§10.3). **[0.44]** `~` is `hint` (§4.3), so the arc segment wants another delimiter when paths are implemented.

### 10.2 Orientation

Every path or fragment containing an arc segment MUST have an orientation, either declared (`: ccw`) or inherited (§10.5). A closed path's declared orientation MUST match the winding of its solved vertex sequence; mismatch is a solve-time error (**E011**).

### 10.3 Arc branch rule (the consistent default)

> **An arc segment traverses its circle in the direction of the path's orientation.**

In a `ccw` path, `a ~C~ b` is the counterclockwise arc on `C` from `a` to `b`; in a `cw` path, the clockwise arc. The decoration `rev` selects the opposite branch. `rev` is constraint-class (P3): it changes the solution set of the *shape* (and, where arc-length or containment constraints reference the path, of the coordinate system too).

*Non-normative:* this rule makes convex-ish boundaries annotation-free. Tracing a gear outline counterclockwise, every tip arc and every root gap arc is counterclockwise on its own circle; no segment needs `rev`.

### 10.4 Derived constraints

An arc segment `a ~C~ b` implies `coincident(C, a)` and `coincident(C, b)`. These derived incidences enter the constraint store subject to deduplication (§14.3), so restating them explicitly is legal and free.

### 10.5 Fragment composition

Path fragments compose by **endpoint identity**: two fragments whose end and start vertices are the same alias class concatenate. A fragment without declared orientation inherits the orientation of the (unique) named path it composes into; if composition is ambiguous or orientations conflict, error **E012**. A set of fragments whose composition closes (every vertex has in-degree = out-degree = 1) forms a closed boundary; implementations MUST report boundaries that fail to close when a closed boundary is demanded by export (**E013**).

*Non-normative:* this is how the gear outline is assembled: each `Tooth` contributes `lead → tl ⌒ tr → trail`, and the `ring` body contributes the gap arc `t.trail ~root~ next.t.lead`. Under ring elaboration the fragments chain around and close at instance N−1 → 0 with no seam case.

---

## 11. Hints

**[0.2]** A hint is one of the two seed forms; the other, and the primitive, is the inline seed of §6.4. Everything in this section applies to both, and "hint" below should be read as "seed". Use `hint` when inline cannot say it: seeding an entity declared elsewhere, or seeding from an expression over other geometry. The statement form below is not implemented; the reference implementation seeds inline only (§6.4).

```
hint t.lead(x: center.x + root.r, y: center.y)
```

`hint REF(key: EXPR, …)` seeds the entity `REF`'s named scalars at the values of the expressions (evaluated with whatever definitional values and previously seeded values are available; a quantity with no seed of its own reads its provisional one, as an inline seed's does (§6.4 **[0.12]**)).

Normative invariant (**Invariant H**): *for every program P, sol(P) = sol(P minus all seeds).* Implementations MUST maintain a statement classification sufficient to verify Invariant H syntactically — i.e., the seed class is closed under everything the grammar allows in a seed, and nothing in the seed class can generate residuals or alter aliasing.

**[0.2]** §4.3 is how that classification is meant to be maintained: a number inside `hint(…)` is seed-class and every other number is not, so the check is a look at the clause rather than an argument about the statement.

Hints on entities inside a `ring` seed the fundamental-domain representative (§12.4). Hints MAY use block indices in `repeat`/`cycle` (where each instance is a distinct variable) and MUST NOT use them in `ring` (**E015**: there is only one representative to seed). **[0.47]** Nor may anything else in a ring's body: a relation, a count or an argument that varies with the index is not a turn of the representative's, and a quotient solve would state only the representative's (**E015**).

---

## 12. Repetition

Three constructs, three meanings. All take a compile-time `Int` count and an optional index binder.

### 12.1 `repeat` — open array

```
repeat N as i { ... }
```

Pure elaboration: N copies of the body, index `i` ∈ 0..N−1 available in expressions and hints. `next`/`prev` are illegal in `repeat` (**E020**); cross-instance references use explicit indexing `name[k]` from outside or arithmetic indexing patterns from inside. **[0.8]** A body ending mid-joint (§6.6) states its trailing joint between consecutive copies and leaves the last copy's unstated — the joint is the block's own statement, so this does not put `next` in the body's scope.

### 12.2 `cycle` — structural closure, no symmetry

```
cycle N as i { ... }
```

Elaborates N copies; `next` denotes instance (i+1) mod N and `prev` instance (i−1) mod N. Instances are independent variables; nothing forces them to resemble one another. Use for closed chains of unequal links. **[0.8]** A body ending mid-joint (§6.6) states its trailing joint at every pair — the wrap included, so the trailing joint is the loop's closure.

### 12.3 `ring` — cyclic symmetry claim

```
ring N about center as i { ... }
```

**Semantics.** Let g = Rot(center, τ/N), the rotation by τ/N about the point entity named in the `about` clause — counter-clockwise in that point's view — or **[0.47]** about the axis it names, right-handed about the axis's direction, which MUST be held (a free direction makes g a function of an unknown; **E023**). A ring about a point turns points drawn in its view; one about an axis turns points in space (**E023** otherwise). Define the unrolled program U = the same body under `cycle N`. Then:

> sol(`ring`) = { x ∈ sol(U) : instance i+1 of every ring-local entity equals g · (instance i) }.

That is, `ring` ≡ `cycle` + symmetry constraints. `ring` is constraint-class: it restricts the solution set to the C_N-symmetric solutions. This is a normative equivalence — an implementation MAY literally elaborate to `cycle` plus per-instance rotation equalities and MUST get the same solution set as one that solves in the quotient.

**[0.2] An implementation that unrolls MUST report that it did**, per `ring`, wherever it reports the DOF ledger (§16.3). The solution sets match; nothing else does. A `ring` states symmetry so that an implementation can *exploit* it, and an unrolled one gives every bit of that back: measured on a 30-tooth gear, unrolling put the wheel outside the cluster vocabulary entirely (so it fell to a numeric residual), past the size at which a drag can be answered by moving rigid bodies, and past the size at which the numeric rank cross-check runs at all — so the dependency reporting of §16 silently switched off. None of that is visible in the solution set, and all of it is visible to a user. 0.1 let an implementation take the licence in §12.3 and skip the SHOULD in §12.4 without ever saying so; that is the gap this closes.

**The `about` clause is mandatory.** The axis point MUST be an entity invariant under g — which for a rotation means the axis point itself (trivially) — and MUST be declared outside the ring.

### 12.4 Fundamental-domain solving (SHOULD)

Implementations SHOULD solve a `ring` in the quotient: one representative per ring-local entity name; a reference `next.e` inside the body denotes g·(representative of e); `prev.e` denotes g⁻¹·(representative of e); an external reference `name[k].e` denotes gᵏ·(representative of e). Every body constraint is instantiated once over representatives with group-element annotations. Solutions lift by orbit expansion xᵢ = gⁱ·x₀.

*Non-normative:* this is N× fewer unknowns and structurally excludes asymmetric spurious roots and permutation-collapsed roots. It is why one hint seeds one tooth and the gear cannot stack its teeth.

**[0.47]** The reference implementation solves in the quotient: a copy is an entity like any other — named `name[k]`, drawn, an operand of a relation outside the ring, an edge of a face — whose numbers are its representative's turned, a linear function of the representative's and the centre's (`model::Turn`). A copy's relations, holds and claims are the representative's turned and are not stated; its curve is the representative's, turned. The ring's own turn is the representative's freedom, gauged by a statement outside the ring (`hub horizontal tip[0]`). A copy may not be held (**E023**: hold the representative), and a relation reading a turned curve is refused (**E023**) until a contact can be stated through the turn.

### 12.5 Invariance of external references

An entity declared **outside** a ring and referenced **inside** it MUST be invariant under g. Implementations MUST verify this by the following syntactic criterion, and MAY additionally prove invariance semantically:

- the axis point itself: invariant;
- a `Circle` whose `.center` is (an alias of) the axis point: invariant;
- any value-typed entity (`Scalar`, `Length`, `Angle` used as magnitude): invariant;
- **[0.47]** a plane a ring-local point is drawn `in` (a place, not a position), and anything a seed's place reads (a seed is no constraint, §11): not judged;
- everything else: **not** established — error **E021** ("entity referenced in ring is not C_N-invariant"). **[0.47]** So is a copy of the ring's own reached by index from inside it (`tip[2]`): it reads the same copy from every turn; a neighbour is `next.tip` or `prev.tip`. A curve written in place inside a ring is judged by the entities its instance is given.

*Non-normative:* E021 is one of the language's best diagnostics; it converts "the solver produced something weird and asymmetric" into a precise compile-time message.

### 12.6 Nesting

Nested `repeat`/`cycle` inside `ring` (and vice versa) is legal. Nested `ring` inside `ring` requires the inner axis to be invariant under the outer generator; implementations MAY reject nested `ring` in this draft (**E022**, "nested ring not supported") and MUST NOT silently mis-solve it. Full nested-group semantics is deferred (§17).

### 12.7 The identity of a statement under expansion **[0.2]**

> **A statement inside a `repeat`, `cycle` or `ring` body is ONE statement, however many things it makes.** What tells its instances apart is the **instance path**, not the statement.

A `cycle` of thirty makes thirty entities from one line of source. That line is the statement: it is what a span points at, what a caret lands on, what a diagnostic names and what an edit rewrites. The thirty are distinguished by the sequence of block indices reached to get to each — outermost first — which an implementation MUST record alongside whatever it records about where an entity came from.

An implementation MUST NOT give each expanded copy a statement identity of its own.

**Why this is normative rather than an implementation detail.** It decides whether the language can be edited at all. Give each copy its own identity and every entity a `cycle` or a component produced names a statement that appears nowhere in the source — so a caret in the text cannot find what it draws, a diagnostic cannot point at the line that caused it, and an edit computed against a span has nothing to splice. It also hides exactly the fact a seed writeback needs (§4.3): a statement reached thirty times has thirty poses and no single one to record, and that is visible only if the thirty agree on which statement they are.

An implementation MAY still need a per-instance key for its own tables. That key is `(statement, path)`, and it is not a statement.

### 12.8 Repetition over a chain's edges **[0.27]**

```
repeat e in CHAIN as i { ... }
cycle e in CHAIN as i { ... }
```

One copy per link of a named chain (§6.6), in traversal order, with `e` a reference to that copy's edge — a constraint operand, a field (`e.p1`), an argument — and `as i` optional as before. The count is the chain's, so the chain MAY be declared later in the body (P2) or reached through an instance or a group. The copies are §12.1's (or §12.2's, with `next`/`prev`) in every other respect, and §12.7 holds. `cycle` over an open chain, and iteration over what is not a named chain, are errors at the reference (**E103**); a body declaration named as the edge is **E001**.

---

## 13. Gauge fixing

Well-posed models are typically invariant under rigid motion; the Jacobian is rank-deficient by design. The language names this freedom rather than letting the solver pick:

```
fix((0, 0)) center            // holds a point: removes 2 DOF
fix(x == 0) p                         // holds one coordinate: removes 1 DOF
fix(r == 25) c                        // holds one of an entity's own numbers: removes 1 DOF
```

**[0.34]** A `fix` names an entity and states each number of its own it holds, pinned with `==`
under the field it is (`x`, `y` of a point, **[0.42]** and `z` of one standing in space; `r` of a
circle or arc; **[0.42]** `x`, `y`, `z` of a plane's
place and of an axis's direction, **[0.43]** and `px`, `py`, `pz` of an axis's place). The values are expressions over the parameters in scope, with units, and MAY
NOT read geometry. A `fix` is a gauge: it takes the numbers out of the solve at the values stated
and adds no equation, so it never takes part in a conflict set. **[0.52]** Two holds of one number
are one gauge where they state one value; where they differ, each statement is **E031** and the
number is not held — no order of statements may decide which wins (P2). Two orientations of one
triangle (§9.6) choosing opposite roots are **E031** alike. It is applied before the seeds
that read geometry (§6.4), which read a held number where it is held and never move one; a seed
for a held number is never read. A `fix` that states no number, writes one as a selector
(`fix(x: 0) p`) or unnamed (`fix(5) c`), or names a field the entity does not have is **E040** /
**E105** at what was written; so is one stating a number twice (`fix((3, 4), (1, 1)) p`,
`fix((3, 4), x == 1) p`), as any relation giving one slot twice is **E040** at the second. Implementations MUST report residual gauge freedom (rank deficiency
whose null space is spanned by rigid motions) with the suggestion to add a `fix` (**W103**), and
MUST distinguish it from genuine under-constraint.

### 13.1 Document state travels on its statement **[0.2]**

A document generally carries more than the program: where an annotation was dragged to, which of several solutions the drawing is on, and whatever else a tool needs to reopen a drawing as its author left it.

> **Every such datum MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index.**

This is P2 applied to the document rather than to the program text, and it is stated separately because it is the half implementations get wrong. Reordering a body is required to preserve meaning (P2); a body can be reordered by an editor, by a code formatter, or by an implementation's own printer. Two keys make that reordering destructive:

- **position in a list.** An annotation stored as "the 7th constraint's placement" follows the 7th position when the 7th statement moves, so it silently reappears on some other statement's annotation.
- **an entity's index.** A recorded solution branch stored as a triple of point indices goes *inert* when the points are renumbered: the document still carries it, a reader still loads it, and fewer of them apply. Nothing reports anything.

Both failures are silent, and both are invisible to any test that checks only the solution set — which is why 0.1 could assert P2 in the parser and lose it in the file format. Where a datum has no statement to ride on, it MUST name what it qualifies (an entity by name, a solution branch by the names of the points that orient it) rather than by index.

**[0.17] A recorded solution branch is one record of one triangle.** Three points can be named six ways, and a chirality named in two of those orders is the same fact with the sign turned. An implementation MUST therefore record a branch **canonically** — one order of the three points, with the sign read against that order — and MUST normalise a record it reads into that form. Keyed by whichever order each writer happened to use, a document's stated orientation and the same choice as the implementation constructs it are two records that never meet: the stated one matches nothing and decides nothing, and the constructed one, written back out as a statement, names a different triple. Both are silent, which is this section's subject.

**[0.20] Annotation placement is drawing state.** A `.svd` drawing identifies a named
model dimension or measures named geometry. Model source MUST NOT serialize a callout's
placement, and a drawing reference MUST NOT use a statement's position or an entity's internal
index. Missing references MUST be diagnosed. Solver hints and recorded solution branches
remain model state.

### 13.2 Presentation **[0.20]**

Styles and presentation classes are defined in [Solvent Drawing](docs/solvent-drawing.md).
Model source MUST reject presentation statements and clauses. A renderer may assign implicit
roles such as hidden edges or closure lines without adding source-level classes to geometry.
Model serialization and editor reconciliation MUST omit presentation state. An editor MAY
provide an automatic preview for a model without a drawing file.

---

### 13.3 Geometry roles and private members

`private`, `construction`, and `centerline` are optional prefixes on geometry declarations,
named chains, and component instances. They may be combined in any order, once each:

```solvent
private construction layout := Polygon(center, ref, n: n, r: pitch_r, phase: phase)
construction centerline ax := line(a, b)
```

Members are public unless marked `private`. A private name is accessible from the enclosing
component's body and its repetition blocks. A private instance hides its member paths from
outside that enclosing component. Its own body can still use its members. Private members of
a nested component remain private to that nested component. Forward references obey the same
access rules. An inaccessible member is **E101**, including in hints and component arguments.
A root-level private name is local to that model and unavailable to `.svd` paths.

Explicitly passing private geometry to another component grants access through that formal;
it does not copy the geometry or change its original visibility. Passing an instance as a group
grants its public interface, not access to private members inside it. Public geometry can expose
its constituent entities (for example, `hole.center`) even when their declaration names are private.
Privacy restricts names, not geometric identity, solver participation, or editor inspection.

`construction` designates supporting geometry; `centerline` designates an axis or center path.
They are independent roles, not arbitrary presentation classes. Both remain ordinary geometry
for constraints and explicit face/solid construction. Neither adds parameters, residuals, or
geometric relationships. Roles on an instance apply to geometry created inside its expansion,
including unnamed children, but never to borrowed arguments. Referencing a construction point
as a hole's center does not make the hole construction geometry. A modifier on a chain applies
to its named traversal and declared links, not to links borrowed by reference.

The editor displays supporting geometry. Authored sheets hide construction geometry by default
and expose `.construction` and `.centerline` selectors for drawing styles. Centerlines default
to a long/short dash pattern. Privacy alone does not hide geometry. Selecting a public component
or the whole model can render its private geometry; a drawing cannot name a private member directly.
Model source edits preserve these semantic annotations; flat geometry export preserves roles.

## 14. Elaboration semantics

Elaboration lowers a program to the **kernel form** consumed by solvers. The pipeline is normative in effect, not in mechanism.

### 14.1 Phases

1. **Instance expansion.** Recursively inline component bodies for the root's instance tree, freshening names by instance path. `repeat`/`cycle` unroll. `ring` either unrolls to `cycle` + symmetry constraints (§12.3) or lowers to quotient form (§12.4); the two MUST be solution-equivalent.
2. **Alias resolution.** Union-find over all names, merging classes for: entity-typed arguments and constructor entity-arguments. Each class gets one representative entity. Type mismatch within a class is an error (**E040**).
3. **Definitional substitution.** Definitional equalities (constructor value-arguments, and values and inputs with a value — `NAME := EXPR`, `param NAME := EXPR`, §6.3) are substituted, METAFONT-style: they are not residuals and consume no solver iterations. A cyclic definitional dependency is an error (**E041**).
4. **Constraint collection.** Predicate statements, `==` equations, derived path incidences (§10.4), symmetry constraints, and gauges are collected into the constraint store.
5. **Path assembly.** Fragments compose per §10.5 into boundary curves attached to the model as derived objects.

### 14.2 Kernel form

The kernel is a **bipartite entity/constraint graph, quotiented by group actions**:

```
Kernel := {
  groups:      [ { id, order N, ax: EntityRef } ],
  entities:    [ { id, type, dof, orbit: Fixed | Orbit(group, size N), seed? } ],
  constraints: [ { pred, args: [ (entity, power) ], params, class: Eq|Ineq, span } ],
  gauges:      [ ... ],
  paths:       [ ordered segment lists over (entity, power) refs ],
}
```

- `orbit: Fixed` marks entities invariant under the relevant group (axis, on-axis circles, scalars).
- Constraint arguments carry a **group power**: `(e, +1)` means "the image of e under the generator" — this is how `t.trail ~root~ next.t.lead` appears with one Tooth's worth of variables.
- `span` is a source location; every kernel object MUST be traceable to source for diagnostics.

*Non-normative:* with all groups trivial this degrades to exactly a SketchGraphs-style bipartite graph, which is the intended interchange representation for external tooling.

### 14.3 Deduplication

The constraint store is a **set**: two constraints identical after alias resolution and definitional substitution (same predicate, same argument classes and powers, same parameters) are one constraint. This makes derived path incidences free when also stated explicitly, and it is the reason redundancy diagnostics (§16) report *semantic* redundancy rather than syntactic repetition.

### 14.4 Modules **[0.12]**

```
use engine.dims
use engine.parts
use std (horizontal, vertical)
```

A **module** is a Solvent document read for its components and relation words. `use NAME` at the top level of a document — never inside a body — asks for one; `NAME` is a dotted path, and **what it resolves to is the host's question**: an implementation with a working directory resolves `engine.parts` to `engine/parts.sv` beside the document, one without a filesystem resolves it against whatever library it carries, and both fall through to the other in that order. The core takes text and never opens a file. A module contributes exactly its **component definitions**, its **relation words** (§9.9) and its top-level **values, inputs and groups** (§6.3, §8.1), available at the importing root; its own loose statements — its drawing — are not drawn, so a document that is also a library (`gear.sv`) is a module as it stands. A module's own `use`s are followed the same way, each module linked once however many times it is asked for, so a diamond is one copy and a cycle terminates. A module a host cannot resolve is **E070**, at the `use`.

**[0.30] [0.48] Nothing is imported bare unless the `use` names it.** A file reaches a module's component, relation word, param or group by the module's **full path**, as its `use` spells it: `engine.parts.Crank(…)`, `engine.dims.bore`, `components.dims.vtwin_dims` — or bare, where its `use` lists the name in brackets:

```
use std (horizontal, vertical)
use engine.parts (Crank)
```

A value or group so named is read in the file's root body (components are closed, §5); a component may be called from anywhere in the file, and a relation word written anywhere in it. A module names its *own* definitions bare. Only a module the file itself `use`s may be named — one reached through another module's `use` is that module's business, and naming it is an error that says which `use` to write. So two modules may define one name, and only two definitions in one file are **E071**. A relation word is reached only bare: there is no `a std.horizontal b`, so a file writing one imports it.

**[0.48] Imports.** Any name a module defines may be listed — a relation word (both fixities of its spelling), a component, a value or input, a group. The full path still works beside the bare name. Imports are **per file and not transitive**, as `use` is: a module's imports are its own, and a file importing from a module that imported a name has not imported it. There is **no prelude**: `use std` alone imports nothing bare, and a file writing `a horizontal b` says `use std (horizontal)`. A listed name the module does not define is **E101**, at the name; the same name imported twice, or from two modules, is **E071**; an import of a name the file defines itself at its top level — a component, a word, a value, a group, an instance or a declaration — is **E071**; and an import of a name that is a word of the language's own (a keyword, a constraint word, a built-in function or constant) that is not a relation word of the module's is **E071**, read by its path instead.

The standard datums follow the rule: `std.front` needs the file's own `use std` (with or without names). A drawn callout of a dimension written `engine.dims.D` shows `D`. A module's own errors — a parse error, a faulty `param`, a faulty relation word or import — are reported to a reader of the document *at the `use` that brought the module in*, with the module's name, line and column in front of the message, since that line is the one the document can edit.

**[0.21] Standard datums.** **[0.42]** `use std` makes the standard axes `std.x`, `std.y`,
`std.z` and `std.back`, the standard planes `std.front`, `std.top`, `std.side` and `std.up`, and
`std.origin` available (§6.7). Every one is held: the axes outright, the planes at the world
origin, and `std.origin` a point drawn in `std.front` at its `(0, 0)`. `std.front` is `u: x, v:
z`, the plane a 2D drawing is drawn in. They are one shared expansion of `std`'s ordinary
`StandardDatums` statements, present whenever the document says `use std`, named or not, so a
workspace can offer the planes as places to draw; being held, they add no freedom. An explicit
root declaration or instance named `std` takes precedence over the standard binding.

A plane argument does not assign membership. `Part(axes, …)` draws its geometry in space unless
something puts it in a plane; `Part(axes, …) in std.front` assigns membership explicitly, as does
calling it inside `in std.front { … }`. A part measured against a turned frame is handed one,
`axes := std.Turned(o, up) in std.front`, which carries its plane (`axes.axes`) and its axis line
(`axes.u`). With the dimension module used, the upright cylinder's preview is:

```solvent
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

The name `cyl` is retained because the part drawing references `cyl.body`; its bore follows the
frame's u axis.

**[0.21] A component file may provide its own preview.** At most one `preview { … }`
block may appear at file scope. Its body contains ordinary model statements, including units,
parameters, datums, instances and constraints:

```solvent
use std

component Part(f: plane, width: Length) {
  // the reusable design
}

preview {
  unit mm
  part := Part(std.front, width: 20mm) in std.front
}
```

When the file is opened as a model, including through a drawing's `model … from`, the preview's
statements join the file's ordinary root body and solve together. A preview adds no namespace,
implicit coordinate system, execution order, or separate solve. When the file is imported with
`use`, the whole preview is omitted: its geometry, values, inputs and unit
statement are not exported. Component definitions and imports remain at file scope; a preview
cannot be nested. Presentation still belongs in `.svd` files. Source editing in a preview MUST
preserve the block and place newly drawn geometry inside it.

Hosts resolving files SHOULD try the model's directory and then its ancestor directories,
nearest first, before falling back to the shared library. Thus opening
`components/cylinder.sv` directly can resolve `use components.dims` from the same project as
an assembly beside the `components` directory. All transitive imports use the same root model.

*Non-normative:* an implementation may parse a module with its spans offset past everything linked before it, so that every span in a linked program is one integer into one virtual text and no consumer learns a second coordinate; a splice on the document then walks the root body alone, which no module span is ever in.

---

## 15. Solver contract

The numerical method is unspecified. Whatever the method, a conforming solver:

- MUST treat alias classes as single variables (no residuals for binding — P1).
- MUST apply definitional substitutions before iteration (§14.1 phase 3).
- MUST use hints as initial values only (Invariant H).
- MUST verify all inequalities and declared path orientations on any reported solution.
- MUST NOT report a solution to a `ring` program that is not symmetric (§12.3).
- SHOULD decompose along component boundaries (P4) and solve `ring` in the quotient (§12.4).
- MUST report the diagnostics of §16 with source spans; "solver did not converge" without a structural diagnosis is a nonconforming failure mode when a structural diagnosis is computable (rank analysis at the seed or at the failure point).

---

## 16. Static and solve-time diagnostics

### 16.1 Errors

| Code | Condition |
|---|---|
| E001 | redeclaration within a body |
| E002 | a block's index or edge binder over a name already in scope |
| E003 | cyclic component instantiation: a component reached again while it is being expanded |
| E004 | a positional argument that binds a value parameter, or that follows a labelled one (§4.1) **[0.17]** |
| E010 | withdrawn: an undecorated `tangent` between circles reads its branch off the seed (§9.5) |
| E011 | closed path winding contradicts declared orientation |
| E012 | ambiguous or conflicting path-fragment composition |
| E013 | boundary fails to close where closure demanded |
| E014 | withdrawn: a hint expression reading an unseeded quantity reads its provisional seed (§6.4, §11) |
| E015 | a `ring`'s index read inside it (**[0.47]** any read, not only a hint) |
| E020 | `next`/`prev` where no `cycle` closes the copies: in a `repeat`, or in no block |
| E021 | external entity referenced in `ring` not provably invariant |
| E022 | nested `ring` (if unsupported) |
| E023 | **[0.47]** what a `ring` cannot turn: a centre neither a point nor an axis held in its direction, a plane, axis or motion declared in it, a point outside its centre's view (or not in space, about an axis), a hold on a turned copy, a relation reading a turned curve |
| E030 | retired **[0.34]**: a `fix` states its numbers (E040 where it does not) |
| E031 | two gauges that disagree: a number held at two values, or a triangle oriented both ways (§13) **[0.52]** |
| E040 | type mismatch within an alias class |
| E041 | cyclic definitional dependency (**[0.18]** a solid made of itself, §6.9; a plane folded from itself until 0.42) |
| E050 | inconsistent system (no solution); report a minimal infeasible subset when computable |
| E060 | a point put on two different planes (§6.7) **[0.10]**; **[0.42]** a circle, an arc or a spline over a point standing in space |
| E061 | `project` refused: a point on no plane, both on one plane, or parallel planes (§6.7) **[0.10]**; a point drawn in a plane asked whether it is on it, or how far off it along its normal **[0.24]**; two points of one view, or a point with itself, measured where the row is identically nothing **[0.46]** |
| E062 | a word across planes, or over a point in space, with no meaning in space (`horizontal`, a run or rise, `tangent` between drawn figures, a curve's contacts) (§9.2) **[0.24]** **[0.42]** |
| E064 | withdrawn **[0.42]**: was a solved view the model cannot hold (`fold: along`, `through:`) **[0.23]** |
| E065 | a relation in space degenerate at the solve: views a `project` relates that came out parallel, lines whose skew distance is stated that came out parallel (§6.7) **[0.23]** |
| E066 | withdrawn **[0.42]**: was `against` between planes that turn apart **[0.23; narrowed 0.26]** |
| E067 | a plane that cannot stand on its held axes: two wholly held axes that do not meet or run alike, or a plane held where its held axes are not — a plane's axes pass through its origin (§6.7) **[0.43]** |
| E070 | a `use` nothing resolves (§14.4) **[0.12]** |
| E071 | a component or a relation word defined twice in one file (§14.4, §9.9) **[0.12]** **[0.30]**; **[0.48]** a relation word defined over a word of the language's own, and an import of a name twice, from two modules, over the file's own definition, or over a built-in word (§14.4) |
| E080 | a face or a solid the model cannot build (§6.8, §6.9) **[0.18]**: a loop that does not close, an edge that is not a line, an arc, a circle or a point, a circle standing *in* a loop rather than being one, **[0.19]** an edge that meets neither neighbour and so is walked two ways, a straight loop with fewer than three corners, edges in two planes, a swept solid written over anything but one face, a body made of what is not a solid, a prism swept nowhere, a feature written into a solid that is a face swept rather than a body; **[0.42]** a face over a point standing in space, and a prism's side generating in a plane the solve places (§6.15.2) |
| E081 | invalid revolution axis or guided sweep path (§6.9) **[0.18]** |
| E082 | a face of a body that the body no longer has (§6.9) **[0.18]** |
| E084 | a section whose cutting plane is not parallel to the view it is drawn in (§6.11) **[0.18]** |
| E101 | a name nothing in scope declares, or a member a scope cannot reach (§3.3, §5, §13.3) **[0.41]** |
| E102 | **[0.48]** a relation word nothing defines where it is written (§9.9), naming the import that would where a module defines it |
| E103 | a shape or a number the model cannot build: more children than the kind has slots (§6.1), an input whose value is not its type (§6.3), a seed reading a scalar that is not there (§6.4), a curve the model cannot trace (§6.5), an extent of the wrong dimension (§6.9), iteration over what is not a named chain (§12.8) |
| E105 | a `fix` of a number the entity does not have (§13) **[0.34]** |
| E107 | a measurement of the solved drawing where a number is needed before the solve (§6.14) **[0.27]** |

**[0.18]** The reference implementation raises all four (**[0.42]** E083 is gone, with `against`). A negative sweep and a picture asked of the wrong kind are **E040** by the rules those codes already state (a magnitude written with a sign, §9.2 **[0.17]**; a kind mismatch, §14.1), and an unresolved name in any of these statements is **E101**, so no new code is spent on either.

### 16.2 Warnings and lints

| Code | Condition |
|---|---|
| W100 | `coincident(p, q)` where making `p`,`q` one entity would suffice — "consider binding instead of constraining" |
| W101 | withdrawn: frames fully welded by constraints — `Frame` was folded into `Plane` in 0.15, and there is no `weld` |
| W102 | `abs(angle(...))` without a disambiguating orientation predicate |
| W103 | rank deficiency spanned by rigid motions — "add a fix" |
| W104 | under-constrained: report the number of residual DOF and, when computable, a basis of unconstrained motions attributed to source entities |
| W105 | consistent redundancy: constraints dependent on others; report the dependent set with spans |
| W110 | an expression that cannot be computed: the last number stands |
| W112 | a `param`, formal or block binder declared over a built-in name (§3.3, §5) — the built-in is what an expression reads **[0.17]** |
| W113 | two planes lying on one another, with a relation reading points drawn in each: one plane in space, read twice (§6.7) **[0.42]** |

### 16.3 DOF ledger

Implementations SHOULD emit, on request, a degrees-of-freedom ledger: per alias class, its free DOF after definitional substitution; per constraint, its equation count; totals per component and for the model; gauge accounting. (§18.3 shows the gear's ledger.)

**[0.17]** They SHOULD also emit, on request, **where each name landed**: the value of every scalar of every entity the source names, keyed by that name and the field (`hinge.x`, `base.r`, **[0.42]** `side.z`, `t.x`). A report that says how many freedoms a drawing has and which statements disagree, and never where anything *is*, leaves its reader to recover coordinates by stating a `claim` and reading whether it is refuted. The names and the numbers are both already in hand; this is a serialisation and imposes no analysis.

---

## 17. Deferred and open issues (non-normative)

1. **3D lift.** **[0.10]** Multiview drawing is settled *without* one — §6.7: a `plane` is a frame with a constant attitude, a point is `in` a view, and `project` is the one equation two images of a point share; nothing three-dimensional is solved for, no true length is measured, and a view's attitude is never an unknown. (**[0.23]**–**[0.42]** revised this: a relation across planes is read in space, and planes, axes and points in space are unknowns of the solve, §6.7, §9.2. The stratification below holds: nothing past the sketch is an unknown.)

   **[0.18] The object is settled too, and on the same terms.** A face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), a plane may be stood off another (§6.10), and a view or a section of a solid is a picture the sheet asks for (§6.11) — so a part is written once and every drawing of it is a question, with no depth kept in step by hand. The stratification is what makes it affordable and is the thing to preserve: **nothing three-dimensional is an unknown**, every extent is an expression, and a solid owns no parameter, so the solver contract of §15 and the ledger of §16.3 are untouched.

   What remains open is the lift itself, and it is now a shorter list (**[0.28]** lofts are §6.9's). **Fillets and chamfers**, which need a name for an *edge* rather than for a face: the spelling is reserved, `body.block.side_l.near` — the edge where two named faces of one solid meet, in the path vocabulary §6.9 already uses, so that a boolean cannot renumber one. A **rigid-body mate solver**: joints between two solids in space, at which point something three-dimensional does become an unknown and P4's decomposition question is asked again one stratum out. Still open from before: the arc-branch rule needs a replacement (no global winding in 3D); and from §6.7, a solved-for fold (`fold: along l`) — **[0.23]** answered by `fold:`, and **[0.42]** by planes over axes.

   **[0.18]** One item of the old list is answered by §6.9 rather than deferred: "`ring` generalizes to rotation about a line" is what `about:` does for a **sweep**, and it needed no group action to do it, because a revolution is one solid and not *N* congruent copies. `ring` itself is still the cyclic-symmetry question of §12.3 and is untouched by this — the reference implementation goes on refusing the word until it can hold its copies congruent. The two were only ever adjacent.
2. ~~**Curve entities.**~~ **[0.2] Settled — see §6.5.** **[0.11]** A curve is a point of a component as one of its numeric formals runs (0.2's family of two expressions is retired), rather than an entity kind per curve. Involute, cycloid and trochoid are library code, and `tangent` and `curvature` against such a curve are §6.5's. What remains open is the path grammar's slot for a curve segment.
3. **Nested symmetry groups.** Semantics of `ring` in `ring` beyond the reject-or-elaborate rule of §12.6 (planetary sets are the motivating case: carrier symmetry ≠ sun symmetry).
4. **Constraint strengths.** A Cassowary-style required/strong/weak hierarchy for graceful over-constraint. Interacts with P3 (a weak constraint changes the solution set; it is constraint-class) and with diagnostics (W105 becomes resolution, not warning).
5. **Reflection symmetry.** `mirror about <line>` as a second group kind; the kernel's group table already permits it.
6. **Inequality vocabulary.** Beyond orientation: `inside`, `min_distance`, clearance constraints.
7. **Assemblies and motion.** A `mechanism` layer where some constraints are joints with time-varying free coordinates; out of scope for the static solve contract.

---

## 18. Worked example: spur gear with revolute teeth

**[0.2]** The tooth below is drawn with straight `->` flank segments, because 0.1 had no way to say
what an involute is (§17.2). §6.5 now does, and a flank written that way is a curve rather than a
chord across one:

```
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
             y: c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)))
}

component Flank(base: circle, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  e := Involute(base, phase: phase).p over u in (u0, u1)
  lo := point
  hi := point
  lo coincident e hint(t: u0)
  hi coincident e hint(t: u1)
  lo coincident root
  hi coincident tip
}
```

Nothing there says *where* the flank goes. It says the curve is the involute of the base circle at
this bearing, that it begins where it crosses the root circle and ends where it crosses the tip —
and the solver finds the two rolls that satisfy it. There is no closed form for either, which is
the point: the shape of the tooth is a solve, not arithmetic performed in a `param` block.

One caution the geometry imposes and the language cannot: **below the base circle there is no
involute.** A textbook 1.25·m dedendum on a small tooth count puts the root circle inside the base
circle, where `u0` has no real value; a conforming implementation reports that rather than fudging
it.

The 0.1 source is kept below as written, since §18.2 and §18.3 walk through it. None of its spellings parse now: `port` (§7), `on(…)` and `angle(…) == e` as calls (§9.2), `param R = …` (§6.3), a radius as a constructor argument (§4.3), `frame` (§6.7) and `t: Tooth(…)` (§5) are retired, and `ring`, `path` and the `hint` statement are specified and not implemented.

### 18.1 Source

```
component Tooth(root: Circle, tip: Circle, slot: Angle) {
  port lead:  Point
  port trail: Point
  point tl, tr

  on(root, lead, trail)
  on(tip,  tl, tr)

  path outline: ccw = lead -> tl ~tip~ tr -> trail

  angle(lead, root.center, trail) == slot / 2
  angle(lead, root.center, tl) == angle(tr, root.center, trail)
  ccw(lead, tl, tr)
}

component Gear(N: Int, m: Length) {
  param R = m * N / 2
  point  center
  circle pitch(center, R)
  circle tip(center, R + m)
  circle root(center, R - 1.25*m)

  frame f0(origin: center, toward: t[0].lead)
  port hub = f0

  ring N about center as i {
    t: Tooth(root, tip, slot: tau/N)
    t.trail ~root~ next.t.lead
    angle(t.trail, center, next.t.lead) == tau/(2*N)
  }

  hint t.lead(x: center.x + root.r, y: center.y)

  fix((0, 0)) center
  t.lead horizontal center
}
```

### 18.2 Elaboration walk-through

- **Aliasing:** `pitch.center`, `tip.center`, `root.center`, `f0.origin`, and `center` form one class. `Tooth`'s formals `root`, `tip` alias the gear's circles (entity-typed arguments). The three circles' radii are definitional (`R`, `R+m`, `R−1.25m` substitute out).
- **Ring lowering:** group G = C_N about `center`. Ring-local free entities: one representative each of `t.lead`, `t.trail`, `t.tl`, `t.tr` (orbit size N). `center` and the circles pass the §12.5 invariance criterion (`Fixed`). The gap constraints reference `(t.lead, +1)`.
- **Paths:** each Tooth contributes `lead → tl ⌒tip tr → trail` (ccw); the ring body contributes `t.trail ⌒root (t.lead,+1)`, inheriting ccw. Composition closes after N teeth and N gaps: one closed ccw boundary. All arcs take the default branch (ccw on their circles) — zero `rev` decorations, per §10.3.
- **Deduplication:** the arc-derived incidences `coincident(tip, tl)`, `coincident(tip, tr)`, `coincident(root, trail)`, `coincident(root, (lead,+1))` duplicate the explicit `coincident` constraints (the last after symmetry transport) and merge in the store.

### 18.3 DOF ledger (quotient system)

| Item | DOF / equations |
|---|---|
| `center` | +2 |
| circle radii | 0 (definitional) |
| `t.lead`, `t.trail`, `t.tl`, `t.tr` (representatives) | +8 |
| **Unknowns** | **10** |
| `coincident(root, lead)`, `coincident(root, trail)` | −2 |
| `coincident(tip, tl)`, `coincident(tip, tr)` | −2 |
| tooth span angle `== slot/2` | −1 |
| flank symmetry angle equality | −1 |
| gap angle `== tau/(2N)` | −1 |
| `fix((0, 0)) center` | −2 |
| `t.lead horizontal center` | −1 |
| **Equations** | **10** |
| `ccw(lead, tl, tr)` | 0 (inequality) |

Square system, full rank at the hinted seed; one Newton basin per §12.4, lifted to N teeth by orbit expansion. Expected solution: lead at bearing 0 on the root circle, trail at bearing τ/2N, tip corners inset symmetrically — a gear.

---

## 19. Grammar (EBNF)

```ebnf
program        = { use_decl | component | word_def | preview | statement } ;
preview        = "preview" "{" { statement } "}" ;                  (* at most one, §14.4 *)
use_decl       = "use" IDENT { "." IDENT }
                 [ "(" IDENT { "," IDENT } ")" ] ;          (* a module, and names read bare, §14.4 *)
word_def       = ( IDENT IDENT [ "(" IDENT { "," IDENT } ")" ] IDENT  (* infix, §9.9 [0.48] *)
                 | IDENT [ "(" IDENT { "," IDENT } ")" ] IDENT )      (* prefix *)
                 ":=" relation ;
component      = "component" IDENT "(" [ formals ] ")" "{" { statement } "}" ;
formals        = formal { "," formal } ;
formal         = IDENT ":" type ;
type           = "Int" | "Scalar" | "Length" | "Angle" | "Side"      (* §4.1 [0.17] *)
               | "group"                                             (* §8.1 [0.35] *)
               | ekw | "curve" ;  (* an entity, in either case: `f: plane`, `p: Point`; `Frame`
                                     was folded into `Plane` in 0.15 *)

statement      = definition | input | relation | gauge | branch | block | in_block
               | unit_decl | body_rel | claim_over | energy | hint | path_decl | frag ;
energy         = ref ( "minimizes" | "maximizes" ) term { ( "+" | "-" ) term } ; (* §9.10 [0.53] *)
term           = [ "-" ] [ NUMBER "*" ] "integral" "(" expr "over"
                 ( IDENT | "(" IDENT "," IDENT ")" ) ")" ;
in_block       = "in" ref "{" { statement } "}" ;         (* membership, written once: §6.7 *)
unit_decl      = "unit" IDENT ;                                           (* §3.3.2 *)

(* §5 [0.29]: a name is defined by `NAME := VALUE` and in no other way, and what stands after
   `:=` says what the name is.  An element or an instance may also be written with no name. *)
definition     = { role } [ IDENT ":=" ] ( chain | instance )
               | IDENT ":=" ( expr | group | vector | curve ) ;   (* a value, a group, a vector, a curve *)
role           = "private" | "construction" | "centerline" ;      (* once each, §13.3 *)
input          = "param" IDENT [ ":" num_type ] [ ":=" expr ] [ hint_clause ] ;
                 (* §6.3 [0.41]: with no value an unknown, its type stated, seeded by `hint(E)` *)
num_type       = "Int" | "Scalar" | "Length" | "Angle" ;
group          = "{" member { "," member } "}" ;                  (* §8.1 [0.35] [0.36] *)
member         = IDENT ":" ( expr | group ) ;
vector         = "(" expr "," expr [ "," expr ] ")" ;      (* [0.45]: members x, y, z *)
member_key     = IDENT { "." IDENT } ;                     (* `x`, `dir`, `dir.x` [0.45] *)
chain          = link { joint link } [ "->" { infix } "close" ] ;
(* link, joint and infix follow §6.6: a link is an `element` with any prefix words before it, or
   a named link `(NAME := element)`; a lone element is a one-link chain. *)

(* §3.3.1: a number may carry a unit, and feet-and-inches is ONE literal — a space is what
   tells the readings apart, exactly as it does in a mixed fraction. *)
number         = digits [ "." digits ] [ ("e"|"E") [ "+"|"-" ] digits ]
                 [ " " digits "/" digits ]                                (* mixed fraction *)
                 [ unit_suffix ] ;
unit_suffix    = "mm" | "cm" | "m" | "km" | "in" | "ft" | "thou"
               | "deg" | "rad" | "grad"
               | "'" [ " " number "\"" ]                                  (* feet, and inches *)
               | "\"" ;

element        = ekw [ "(" ctor_arg { "," ctor_arg } ")" ] { trailer }
               | "point" "(" "x" ":" expr "," "y" ":" expr ")"   (* a computed point, §6.5 [0.13] *)
               | face_decl | solid_decl ;
ekw            = "point" | "line" | "circle" | "arc" | "spline" | "plane" | "axis"  (* axis [0.42] *)
               | "face" | "solid"                                            (* §6.8, §6.9 [0.18] *)
               | "surface" | "motion" | "envelope" | "patch"                (* §6.13–§6.16 *)
               | "seam" | "vertex" | "edge" ;                               (* §6.17–§6.19 *)
(* the trailing clauses are order-free: `hint(…)`, `knots […]`, `in REF`.  A
   place — `hint(at: t)`, `hint(at: c, bearing: …)` — is the same clause with `at:` and
   `bearing:` for keys, §6.4 [0.14] *)
(* §6.1: no list at all is the anonymous form — the kind's children are minted and reached as
   `l.p1`.  A slot is a name, a seed, or implicit; only an overfull list is E103. *)
trailer        = hint_clause
               | "knots" "[" number { "," number } "]"
               | "in" ref { "," ref } ;      (* membership of a plane, §6.7; several: a point [0.57] *)
hint_clause    = ( "hint" | "~" ) "(" ( expr | hint_item { "," hint_item } ) ")" ;
                 (* SEEDS, §4.3; `~` [0.44]; one keyless expr seeds an unknown, §6.3 [0.41] *)
hint_item      = member_key ":" ( expr | vector ) | vector  (* `(3, 4)`, `dir: (1, 0, 0)` [0.45] *)
               | "at" ":" ref | "bearing" ":" expr                  (* a place, §6.4 [0.14] *)
               | ( "toward" | "along" ) ":" ref | ( "by" | "turn" ) ":" expr ;   (* a step [0.33] *)
ctor_arg       = [ IDENT ":" ] ( ref | hint_clause )       (* what the thing is made of, §6.2;
                                                              a plane's are `u:` and `v:`, §6.7 *)
               | IDENT ":" expr                            (* a motion's or an envelope's numbers,
                                                              §6.14, §6.15 *)
               | sweep_arg ;                               (* how a solid is swept, §6.9 [0.18] *)
(* [0.42] a plane's `from:`, `fold:`, `offset:` and a basis of triples are retired *)

(* §6.9 [0.18]: the sweep stands in the brackets with the face, being what the solid is made
   of; numeric extents are expressions and never unknowns. `through:` instead names a solid
   whose material bounds determine an extrusion after solving. `about:` is a revolution.
   `under:` a motion places a solid at its pose `at:` an angle, or sweeps it over `from:`/`to:`. *)
sweep_arg      = ( "from" | "to" | "depth" | "sweep" | "at" ) ":" expr
               | ( "about" | "through" | "along" | "under" ) ":" ref
               | "sense" ":" ( "cw" | "ccw" ) ;

(* §6.8, §6.9 [0.18].  Both are ordinary elements — `face` and `solid` are element
   keywords — and are written out here for what their one list slot holds. *)
face_decl      = "face" "(" face_arg { "," face_arg } [ "," "->" "close" ] ")" ;
face_arg       = ref | ("edges" | "holes" | "on") ":" ref ;  (* planar/spatial restrictions: §6.8, §6.20 *)
solid_decl     = "solid" "(" solid_term ")" ;
solid_term     = ref [ "," ref ] { "," sweep_arg }                     (* a face swept *)
               | ref { "," ref } ;                         (* a body: its stock, then solids *)
body_rel       = ref ( "union" | "cut" | "bound" ) ref ;  (* the body rule, §6.9 — each its own
                                                     word: `bound` [0.22], `union` [0.32] *)

(* §6.10: `against`, the 0.18 mate, is retired [0.42]. *)

(* §9.8 [0.18]: a claim about the object.  The three words are operators like any other, and a
   sweep says how the claims in its body are judged. *)
solid_claim    = [ "claim" ] ref ( "clear" "(" expr ")" | "fits" "(" expr ")" | "inside" ) ref ;
claim_over     = "claim" "over" ref "in" "(" expr "," expr ")" "{" { solid_claim } "}" ;

(* [0.20]: solid views, sections, and annotation requests belong to .svd. *)

(* a curve, §6.5 [0.11]: a point of a component as one of its numeric formals runs.  0.2's
   family form and 0.3's `trace … where { … }` are retired and MUST NOT parse. *)
curve          = ( ref | ref "(" [ args ] ")" "." ref )
                 "over" IDENT "in" "(" expr "," expr ")" ;
instance       = ref "(" [ args ] ")" [ "in" ref ] ;   (* a used module's component by its full
                                                          path, §14.4 [0.30]; `in`, §6.7 *)
args           = arg { "," arg } ;
arg            = [ IDENT ":" ] ( expr | hint_clause ) ; (* `w: hint(60)` leaves the formal unbound
                                                          and seeds it, §5 [0.41] *)

(* §9.2: every constraint is a prefix or an infix operator; `name(args…)` is retired. *)
relation       = [ "claim" ] ( prefix_form | infix_form | orientation | solid_claim )
                 [ hint_clause ] ;  (* §4.3; presentation placement belongs to .svd *)
prefix_form    = IDENT [ op_args ] ref ;
infix_form     = ref ( IDENT | "@" ) [ op_args ] ref ;   (* `@` is `coincident` [0.44] *)
orientation    = orient "(" ref "," ref "," ref ")" ;      (* §9.2: a call *)
op_args        = "(" op_arg { "," op_arg } ")" ;
op_arg         = expr                                      (* the number it states *)
               | ref                                       (* a third entity: `symmetry` *)
               | IDENT ":" ( IDENT | number )              (* a selector, §9.2 [0.17] *)
               | IDENT "==" expr ;                         (* a pin, §4.3 *)

(* §4.3: a number inside `hint(…)` seeds; `==` inside an argument list pins.  The two are the
   whole of the seed/constraint classification, and they are told apart by the clause alone. *)

gauge          = "fix" "(" pin { "," pin } ")" ref ;       (* §13 [0.34] *)
pin            = vector                                    (* the point itself [0.45] *)
               | member_key "==" ( expr | vector ) ;       (* `x`, `r`, `half`, `dir`, `origin`,
                                                             `dir.x`, … [0.45] *)
                 (* the entity's own fields: `z` [0.42]; an axis's place `px`, `py`, `pz` [0.43] *)
branch         = "branch" "(" KEY "," [ "-" ] digits ")" ; (* a recorded root choice, §13.1; KEY is
                                                              the raw text up to the comma *)

path_decl      = "path" IDENT ":" orient "=" path_expr ;  (* §10; unimplemented, and its `=` is no
                                                              token since [0.29]: to respell *)
frag           = path_expr ;                               (* statement-level fragment *)
orient         = "ccw" | "cw" ;
path_expr      = ref seg ref { seg ref } ;
seg            = "->" | "~" ref [ "rev" ] "~" ;           (* `~` is `hint` since [0.44]: to respell *)

hint           = "hint" ref hint_clause ;                  (* §11; unimplemented *)

block          = ( "repeat" | "cycle" ) ( expr | IDENT "in" ref ) [ "as" IDENT ]
                 "{" { statement } "}"                (* IDENT "in" ref: §12.8 [0.27] *)
               | "ring"   expr "about" ref [ "as" IDENT ] "{" { statement } "}" ;

ref            = ( IDENT | "next" | "prev" ) { "." IDENT | "[" expr "]" } ;
expr           = expr addop term | term ;
term           = term mulop factor | factor ;
factor         = NUMBER [ UNIT ] | "tau" | "pi" | ref | call
               | "(" expr ")" | "-" factor ;
call           = IDENT "(" [ args ] ")" ;                  (* polar, direction, angle, abs, ... *)
addop          = "+" | "-" ;
mulop          = "*" | "/" ;
```

Parsing note: a statement is told by its first tokens. `NAME :=` is a definition, and the token after `:=` settles which: `{` a group, `over` in the statement a curve, a call an instance, an element keyword or a prefix word a chain, anything else a value (§6.3). `param` opens an input and `in REF {` a block of membership. A constraint word opens a prefix relation, or a call for `ccw`/`cw`, which also appear as orientation keywords after `:` in `path`, where context disambiguates. Otherwise a `ref` comes first, and the word after it, read by a lookahead past the dotted path, is a constraint word, `@`, or `cut`, `bound` [0.22] or `union` [0.32], which relate no geometry and have no residual for the operator machinery to settle them into. **[0.18]** `view`, `section` and `dimensions` open a statement and are therefore not names; **[0.20]** model source refuses them as presentation (§13.2).

**[0.2] Note on a number read verbatim.** After the `:=` of a value (0.2–0.6: the `==` after a predicate's closing parenthesis, which §9.2 retired), an implementation MAY take the rest of the logical line verbatim rather than tokenizing it, and hand that text to whatever evaluates dimension expressions. This is not laziness: `3 1/2` is three and a half and `31/2` is a division, and that rule belongs to one tokenizer. Two copies of it are two rules the moment one is edited. An `==` *inside* an argument list is the pin of §4.3 and is lexed normally; the two never meet. **[0.7]** A trailing `hint(…)` ends the verbatim region, since it is a clause of the statement and not part of the number.

---

## 20. Conformance checklist for a first implementation

A minimal conforming implementation provides:

1. Parser for §19; classifier assigning every statement to §4.2 classes **by the `hint(…)` / `==` mark (§4.3)**; Invariant H enforced by construction of the parser.
2. Elaborator: instance expansion **preserving statement identity (§12.7)**, union-find aliasing, definitional substitution, dedup store, `ring` lowering (quotient form or cycle-plus-symmetry — either, per §12.3, **and reported if unrolled**).
3. Invariance check §12.5 (syntactic criterion), gauge analysis (W103), DOF ledger (§16.3).
4. A numeric backend satisfying §15 — Newton on the quotient system seeded by hints is sufficient — with rank-deficiency reporting attributed to source spans.
5. Path assembly and closed-boundary export (the solved gear outline as a polyline+arc sequence).
6. **[0.2]** Document state attached to its statement, never to a list position or an entity index (§13.1).

Deliberately *not* required for v0: 3D, nested rings, constraint strengths, decomposition planning (P4 is a SHOULD). **[0.2]** Curve families (§6.5) are not required either, but they are no longer deferred: an implementation that wants involute or cycloid geometry has a way to say it, and needs no new entity kind to do so.
