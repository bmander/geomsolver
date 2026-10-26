# Solvent: language changelog

What each draft of the [specification](../solvent-spec.md) changed, newest first. A **[0.n]**
mark in the specification points at the entry here that introduced or amended the rule. Entries
keep the spellings of their day.

**[0.188] An instance's numbers, read from outside it.** `b.n` names a number the body of the
instance `b` defines — a value, an input with a value, a group's member — worked out from what the
call gave it, as `ball.r` names a formal (§8): `shield_z := uno_z + shield.top` reads `Shield`'s
`top` rather than a second copy of its formula. A path reaches through an instance the body writes
(`st.base.top`). Either is read in a definition as well as by a statement, the body writing the call
worked out in dependency order with it: through 0.187 a definition reading even a formal,
`y := b.w + 1mm`, was **E103**, and a body's own number was not read at all (`` `shield.top` is not
a number here — nothing in scope gives it one ``). A definition the call reads that reads the number back is
**E041**; a number the call leaves unable to work out is **E103** where it is read. A formal a call
leaves to its default reads as that default from outside, where it read as an unknown. Every solved
position is unchanged.

**[0.187] Faces that meet.** A `when` may ask whether two faces share a point: `a meets b`,
`a` and `b` faces of one solid a face repeat runs over, holds where they share an edge or a corner
of its boundary as made by the terms that do not read its faces (§12.9). It is decided where the
block is elaborated, so where the drawing is seeded; a pair it reads apart that meets where the
drawing solves is **E080** at the document's statement that made the block. Faces of two solids,
and a face a round made that the faces it reads have not, are taken to meet; a name that is no
face is **E103**. `std`'s `A round(r) B` and `A chamfer(d) B` now pair only faces that meet, and
round a corner only of three that each meet the other two, so a round over a body's faces makes
copies for its edges and corners rather than for every two and three of the faces it lists: a disc
whose web four turned spokes cut was past the expansion's limit of 200000 statements, and builds in
under a second. An expansion past that limit is said once, at the document's statement that
reached it, and nothing of it is built, where before each body the walk was in said it, at the
`use std` line, and every name the rest would have made was then said missing. Every solved
position is unchanged. (Issue #625.)

**[0.186] An orientation word may be a name.** `ccw` and `cw` are calls and nothing else (§9.6),
so a definition may bear either as a name (`cw := ConjugateWheel(…)`, a conjugate wheel) and the
word with no `(` after it is that name wherever it stands (§1.3): `claim cw.body clear(1mm)
ccw.body`, `cw.rim.center coincident hub`, `hub coincident cw.rim.center`. A definition was already
admitted and the name read mid-statement; a statement opening with it was taken for the predicate
missing its brackets (**E001**, then **E040** or **E103** about an orientation), and one reading it
after a constraint word took it for a second joint word. `ccw(a, b, c)` reads as before. (Issue
#616.)

**[0.185] A mate said once.** A relation word's body may read the datums of an instance it is
given, as an operand or an entity parameter (`g.std.origin`, §9.9), and a statement writing the
word reads them from outside as one writing `d.std.origin` would (§8): the instance stands in a
frame of its own, placed by the body's relations — through any word the body uses in turn, an
instance whose body reads no datum **E111** where the word is used, a culprit read as the word
(`d seats(turn: 15deg, up: std.z, east: std.x) b`). Through 0.184 such a use was refused, each of
the body's relations saying it did not know what its operands were (**E040**). The standard library
gains two words: `g seats(turn, up, east) p`, the mate of an instance drawn about its own origin —
its origin on `p`, its z along `up`, its x turned `turn` from `east` about `up` — and `q raised(d,
along) p`, `q` the distance `d` from `p` along `along` on the axis through `p` that runs that way,
the station a part's origin seats on. The language gains no keyword or statement form, and every
solved position is unchanged. (Issue #595.)

**[0.184] A group is a value.** `small := sizes`, where `sizes` names a group (the file's, a
group formal, a nested one or a used module's, `engine.dims.handle`), names the same group: its
members read through either name, either handed to a `group` formal, nothing made. A group may be
written in braces where a `group` formal takes it, `Bar({length: s.length + 2 * o, origin: o})`,
its members read in the caller's scope and an interface typing the formal judging it at the
argument; braces given to a formal that takes no group are **E103**, where before every braced
argument was. A body hands on a group among its file's constants, as it reads its members:
`Disc(sizes)` inside a component, by the file's name for it, unless a formal or a name of the body
hides it; one naming the file's geometry is **E103**, the drawing out of a body's reach as before.
A group read as a number is said to be a group (`` `engine.dims.handle` is a group, and a group is
not a number``) rather than as a member its module lacks. The language gains no word. Every solved
position is unchanged. (Issue #594.)

**[0.183] A claim over values.** `claim` before a comparison of the document's numbers —
`claim handle_stall_torque >= 2Nm`, `claim 2 * ratio > 30 and teeth != 12`, a condition as a
`when` writes one — asserts it of the values it reads (§9.7). A value is worked out before the
solve, so the claim is judged there, a **theorem** or **violated** and never consuming, each
comparison kept within 10⁻⁹ of rounding; in a component's body it is judged once per instance.
One reading an unknown of the drawing, or a value written over one, is **E040**; a name that is
no number **E101**, two dimensions compared **E103**. A statement after `claim` is a condition
where it opens with a number, a bracket, a sign or a function of numbers, or where its first name
is followed by arithmetic or a comparison; every relation claim reads as it did. *Non-normative:*
the reference implementation carries every number a document names out of elaboration under its
absolute name, and says one in the unit it reads in — a power of length and angle in the
document's unit and degrees, a figure with a mass, a time or a current in SI (`torque = 0.0864
N·m` in any document's lengths); `solventc --where ratio` reads one, `--json` gives them as
`values`, and each claim is said with what it reads (`claim torque >= 2Nm — refuted: torque =
0.0864 N·m`), as `valueClaims` under `--json`. (Issue #596.)

**[0.182] A block's word may be a name.** `repeat`, `cycle`, `ring` and `when` open a block only
where a header follows them (§1.3): `ring := Gear(…)` names a ring gear, and a statement starting
with the name is about it where the next token is `.` or a constraint word with no `{` opened on the
line — `ring.std.origin coincident hub`, `ring coincident hub`. A definition was already admitted;
a statement reading it was taken for the block's header and refused with the block's message, at a
column inside the statement (`a \`ring\` names what its copies turn about`). A header always runs on
its line to its `{`, so no block is read otherwise than before. (Issue #598.)

**[0.181] A module's parameter is its drawing's.** A file that declares a parameter that runs
(`param spin: Angle over (0deg, 360deg)`, §6.3) beside the drawing that reads it — a part opened on
its own to scrub its joint — is now a module as it stands (§14.4): `use` leaves the parameter out
with the drawing, as it leaves the drawing and the `preview`, where through 0.180 it was **E040**
at the `use`. A component never saw one, so nothing a module contributes changes; a module's
parameter is named by nothing outside it (`part.spin` names nothing, and `use part (spin)` is
**E101**). A file opened as the document reads its parameters as before. (Issue #593.)

**[0.180] A filled body drives.** A driving `deflection(p, along: d) == v` of a point of a solid
read in three dimensions that is no prism — turned, united, cut — is a row of the drawing, as a
prism's is: the solid is meshed once where the drawing stands, the point naming the node nearest
it, and within the solve its boundary's nodes move with the surfaces its faces lie on — a prism's
caps and sides, a full turn's faces, each a function of the points, radii, planes and axes that make
it, a boolean trimming it and never changing it — while its other nodes follow them. A `mass(m)` or
`volume(v)` stated of such a solid is a row too, its boundary moved the same way. Through 0.179 the
deflection compiled nothing and was judged after the solve, and the measure was **E040**. A face of
a placed copy, a sweep, a round or a region, a drafted prism or a partial turn moves with nothing
yet: the requirement is **E040**, naming the face. An implementation MAY solve such a drive on a
coarse mesh first and finish it on its own; one that cannot afford a drive (a page solving on the
thread it draws on) MAY judge it after the solve, as a claim, and SHOULD say where it is driven. The
engine's connecting rod takes its web's thickness as a formal, `ConRod(pin, bore, wt: Length =
5mm)`, its default the rod it was; `sized_conrod.sv` sizes it by its small end's sag. (Issue #567,
from #510.)

**[0.179] A corner round is a joint.** `A -> round(r: R) B` rounds the corner where the lines `A`
and `B` of a chain meet with an arc of radius `R` tangent to both (§6.6): the arc (named where it
stands, `-> (k := round(r: R))`), its radius, a tangency at each end and the corner kept on both
lines as construction, each line now ending at a point of its own where the arc touches it. The
arc lies inside the corner, on the side the walk turns toward there — counter-clockwise at a left
turn, drawn from its end at a right one — held there by three stated orientations, so no seed and
no `ccw`/`cw` chooses which of the four tangent circles is meant; its points start where the
corner puts them. A trailing round at a `cycle`'s or `ring`'s `}` rounds the corner into the next
copy, and `-> round(…) close` the corner that closes a loop. A round at a straight-through corner,
or at a `repeat`'s brace, is **E040**. The language gains no keyword: `round` is read so only
beside a marker, and a bare `A round(r) B` is still `std`'s round of solids. *Non-normative:* the
reference implementation says a round that does not fit after the solve (`` `k` (r 9mm) runs past
`up`'s far end; it fits up to r 6mm``, two rounds overlapping on one line). The star handle of the
issue is four arms written as their corners: DOF 0, one root. Every solved position is unchanged.
(Issue #576.)

**[0.178] A mirrored placement.** A closed instance's call may end in `mirrored(front)`,
`mirrored(top)` or `mirrored(side)` (§8.2): the instance is its binding reflected in that datum plane
of the frame it stands in. Its body is solved as the preimage, where the same instance unmirrored
stands — the same rows, freedoms and diagnosis — and what it publishes, read from outside, is the
image: each entity its body makes reflected, its datums `b.std.D` the image's (`b.std.x × b.std.y =
−b.std.z`), its solids reflected; every frame stays right-handed. A mirrored instance that is not
closed is **E112**, new, and so for now is one in a `ring` or in another mirrored body, and a body
declaring a plane, an axis, a frame, a curve or a motion. The language gains one trailer and no
keyword or statement form; `mapped(…, mirror: true)`, which mirrors a profile within a drawing (a
loft's handed section), stays. *Non-normative:* the reference implementation derives the image from
the preimage with weights ±1, draws the image, and evaluates a left and a right of one binding once,
the reflection an improper rigid motion of the B-rep and the mesh. The hub motor's `Cover` and
`Revolved` are closed, drawing their frame's datum and axle from their own `std`: the brake-side
cover is `Cover(…) mirrored(front)`, and the left bearing, spacer and locknut are the right ones
mirrored, so `sg: Scalar`, its two `when` blocks, the corpus's only `mapped(…, mirror: true)`, the
`min`/`max` round the tapped holes and the six hand-negated stations go — 961 lines where there
were 964, 311 labelled arguments where there were 329, 14 solids built for its objects where there
were 17, and every solid the same to 5·10⁻¹² mm. Each cover draws its clearance holes, and the ring
its tapped holes, as a ring of its own from the table's numbers, and the assembly claims they meet. (Issue #578, from #523, #562
and #566.)

**[0.177] One figure, many placements.** A closed instance — one whose only geometric input is
the frame it stands in: a definition whose formals are numbers (or groups carrying numbers alone),
each given a constant, its call drawn in that frame or one of its datums — has a **binding**, its
definition and its arguments, and what it makes is a function of its binding read in that frame
(§8.2): two closed instances of one binding are one figure under two placements. An instance
declared in a `ring` stands in its copy's frame, the representative's turned: a relation of its
body may read the standard datums of the frame it stands in (`std.origin`, `std.x`, …), and a
relation word's operand there may too where the application declares nothing, where 0.47–0.176
refused both (**E021**, narrowed, §12.5). A declaration built on such a datum, a datum read by the
ring's own body outside any instance, an argument and every other outside reference are refused as
before. So a pattern of parts is `ring N about C { p := Peg(r: 3mm) }`, one binding under N
placements. The language gains no statement form, keyword or label. *Non-normative:* the reference
implementation evaluates a binding's solids once — exact B-rep and mesh — and places each
instance's by its frame, keyed by provenance; the solve is untouched. The hub motor's magnets
(`Magnet(theta: θ)`) and slot cutters (`Slot()`) are each one instance in their ring, where the
ring held the figure's statements and a `repeat` swept each copy's faces again: two bindings under 95
placements, 243 B-rep nodes built for the assembly before and 97 after, and every solid the same.
Every solved position is unchanged. (Issue #566, from #523 and #562.)

**[0.176] One definition form.** A braced definition is a function of its arguments and of the
frame its instance stands in, and every one is held to it (§8.2): E108, E109, the diagnosis read
through instances (D1–D3) and W207 apply to every drawn instance of every `component Name(…) { … }`,
where 0.162 held only those written `part`. `part` stops being a keyword: `part Name(…) {` is
**E100**, saying to write `component`, and `part` is a name like any other. Set families and
relation words are untouched. The frame an instance stands in is one of its inputs: the rows that
tie what it makes to that frame (a lift, a plane's axes through its origin, a twin's tie) are its
own, and a row placing what another instance made is that one's. A numeric formal the call leaves
unbound may be determined by the instance's own rows, as its output, or left to the parent. The
corpus's 56 `part` definitions are components; the definitions that broke the promise are fixed in
their text: the flows (`rates/`) say their home `initially` from formals, `ngon.sv`'s `Ngon`
exports its turn as `heading`, the hypoid's steps leave the numbers they set (`mn`, `rp`, `pa`, the
face mill's two settings) to the pair's own rows, the spheres and the grinding wheel say each fact
once, and `std.Frenet` holds its plane's `u` along its normal line rather than building the plane
over it. `std.Turn`, `std.Spin` and `std.Slide`, kept beside the joint words since 0.154, are
retired; `std.Rolls` is the word `a rolls(at: p, along: d) b`; `std.Screw(toward, m)` makes its
axis, `s.isa`, running the way a held `toward` does. `std.Along(g, up: u)` and `std.AlongFrenet(g)`
make the frame they hold, `f`, where they were given one; `std.CircleOnSphere(k, s, view)` is the
word `k on_sphere(view: v) s`. A `fix` of a joint's ratio must be a plain number and of its advance
a length (**E103**), the check the retired components made through their typed formals. The Rect
tool's `Rectangle` stands at a corner it is given, its first side level, the corner a point of
the document beside the instance. A circle carried whole by `mapped` holds its centre once. Every
solved position is unchanged. (Issue #562, from #523.)

**[0.175] What a body is made of is the drawing's.** A mass and a volume answer no load, so
neither is a scenario's: `mass(m) B` and `volume(v) B` are relations of the drawing, prefix words
(§9.1, §9.11) over a beam, a line a beam is drawn on, or a solid. Stated, each is a row the solve
holds — a beam's or a member's `ρ A L` (its `A L`), a prism's density times its face's area
through its depth — so it sizes a section's area read over an unknown, places a joint, or moves a
prism's points and radii, its face meshed once and moved with the drawing as a driving
deflection's region is; it reads no support and no load, so a drawing with no scenario at all is
sized by its mass. Claimed, each is judged on the solved drawing: `claim mass(m) B`, and — a claim
alone, since a bound on a mass chooses no root — `claim mass(<= m) B` and `claim mass(>= m) B`,
each value reported with its error beside the scenarios' answers (`mass(arm) is 10kg: driven ==
10kg, held`); a volume in the document's length cubed (`42000mm³`), written `42 * 1cm^3`. A mass of
a material stating no `rho` is **E040**, as is a bound not claimed and a measure of a solid that is
no prism stated. **`mass(B)` is no scenario output now**: through 0.174 a scenario claimed it, read
only where the scenario's supports held the body; `mass(…)` (or `volume(…)`) in a scenario is
refused as it is parsed, naming the drawing's statement, so a document that claimed a mass in a
scenario writes it out of it — `claim mass(B) <= 1kg` in `scenario s { … }` is `claim mass(<= 1kg)
B` beside it. The library's and the corpus's mass claims are moved so. `mass` and `volume` are
operator words, so no relation word may be defined as either. (Issue #510.)

**[0.174] A directed angle about an axis.** `angle` and `perpendicular` take `about: a` (§9.4):
`d1 angle(θ, about: a) d2` is the turn about axis `a`, by the right-hand rule and on the full turn,
from `d1`'s shadow on the plane square to `a` to `d2`'s, and `d1 perpendicular(sense: ccw, about:
a) d2` that turn at a quarter (`cw` the other). The directions are lines or axes, need not be square
to `a`, and the row has one root, where the unsigned `angle` and the bare `perpendicular` in space
hold either way round. `about:` is a label the language already reads as an axis (`solid(…,
about:)`, `std.Turn(f, about:)`), and no keyword is added. In a plane `about:` is the plane's normal,
unsaid: naming it is the plane's own statement, and naming anything else is **E040**, as are a
`perpendicular` with `about:` and no `sense:` and `about:` on any other word. A `sense:` in space
with no `about:` is refused as before, its message now naming `about:`. The V-twin's bank mates its
cylinder's x a quarter turn clockwise about its z from the plane of swing's normal
(`cyl.std.x perpendicular(sense: cw, about: cyl.std.z) pin.plane.normal`), where 0.173 left which
way round its y stood to the seed; every position is unchanged. (Issue #558, from #523.)

**[0.173] Which way an axis runs.** `parallel` takes `sense: same|opposite` (§9.4): the second
direction runs the way the first does, or the other way, where `parallel` alone holds either way
and leaves which to the seed. Between two lines in a plane it is the directed angle at no turn or a
half; between directions in space, the turn carrying the first onto the second, whose size is the
angle between them, so the other way round is no root. `coincident` between a line and an axis, or
two axes, takes the same word. None states a number or draws a callout. And **the rule**: where the
language holds an axis to a direction without a statement saying so, it runs the way that
direction does — a plane built on a drawn line (`plane(u: across, v: up)`) from the line's `p1`
toward its `p2` (§6.7), and a frame's joint (`turns`, `screws`, `slides`, `spins`, §6.14) along
the line, the axis or the view's normal it is held to. Through 0.172 the seeds put them so and the
solve kept whatever sense a start had, so a drawing that solved from its seeds is unchanged; one
whose plane or joint the solve had turned round now comes back. `--lint`'s W207 reads a sense as a
stated root. The engine's detailing drops the eight bounds that pinned those senses by ordinates
with nothing to do with the design (`Valves`, `Pulley`, `StarterRing`), and the V-twin's bank mates
its cylinder's z along the rod the way the rod runs (`cyl.std.z parallel(sense: same) rod`) and its
x in the plane of swing, where 0.172 left which way the z ran to the seed. Which way round its y
stands is still its seed's, one of the two: a sense on a single turn needs a directed angle about
an axis, which space does not have yet. The hypoid layout's rolls, indexes and normal sections had
been solved running against their lines; every point stands where it did, coordinates drawn in
those sections change sign, and the tip relief's section `N` is now built on `to_inner`, the trace
normal run away from `M`, the way its toppings were cut in it. (Issue #535, from #523.)

**[0.172] Solids read in three dimensions; every output's error.** A solid a requirement names,
made of a material and read in no plane (§9.11), is read as the solid it is, in quadratic
tetrahedra: a prism as its face's mesh stacked through its depth, any other solid by refining its
exact boundary. Its supports hold along the world's axes, its loads fall on the faces, points and
volume they name, and it answers all six outputs — `buckling` among them, the least factor on the
loads, and `safety` from von Mises at the nodes — each judged as a frame's is; a load that presses
nothing buckles nothing, a sharp re-entrant edge leaves `safety` unanswered, a support along a
direction off the world's axes and a line naming a part of a body (not the body) have no answer
yet, and each says why. A driving deflection of a point of a prism read so is a row of the drawing,
its face's nodes moved with the drawing and its own stacked over them, refused where a region's
is (**E040**). Every output of a frame, a region or a solid now has its error estimated beside it,
the change from the same reading twice as coarse, said beside the value (`± 0.026mm`). Before,
a requirement on a solid read in no plane had no answer and said nothing; a document that wrote
one now has it answered. No syntax is added. (Issue #510, S3.)

**[0.171] An instance is a frame.** From outside, an instance's datums are `b.std.x`, `b.std.y`,
`b.std.z`, `b.std.front`, …, `b.std.origin` (§5, §8), the spellings its body reads them by, where
0.170 refused the path (no such member). Once the body declaring the instance, or one around it,
writes such a path, the instance stands in a frame of its own: declared beside the call in the
`in`s around it, so it starts where the frame around the call stands and rides it; square, so it
has six freedoms and its body cannot be sheared; and placed by relations on those datums, a mate,
or reported free under the instance's name (`cyl.attitude`, `cyl.origin`). An instance whose `std`
nothing outside reads stands as before, so nothing that elaborated changes. Reading the datums of an
instance whose body reads none is **E111**: it is placed by its arguments. No syntax is added. The
V-twin's bank mates its cylinder by `cyl.std.origin`, `cyl.std.y` and `cyl.std.x`, the frame it
named and its `perpendicular` row gone, its positions unchanged. (Issue #548, from #523.)

**[0.170] Solids read in their plane.** A scenario's line `plane stress B` or `plane strain B`
(§9.11) reads the solid `B` — a prism, its face swept square to its plane with no draft — over
that face, its depth the thickness: plane stress for a thin plate, plane strain for a long prism.
Its supports, loads and requirements fall on the region where they name it (a wall of the prism
is a side of the face, the solid or a cap the whole), and it answers `deflection`, `compliance`,
`mass`, `mode` and `safety` (von Mises at the nodes), each judged as a frame's is; `buckling`, a
load across the plane and a `safety` at a sharp re-entrant edge have no answer and say why. A
driving deflection of a point of such a region is a row of the drawing, its mesh made once where
the drawing stands and moved with it, the region's points and radii its columns. Refused: a target
that is no solid or no such prism (**E040**), a solid read twice in one scenario (**E001**), a
driving row over a region bounded by a spline or a curve, in a plane the solve turns, or left free
(**E040**). `plane` stood at a scenario line's head as a syntax error before; the corpus's
Kirsch, LE1 and FV32 now state the reading they were read in. (Issue #510, S2.)

**[0.169] A frame has the standard datums.** Inside a definition's body whose instance stands in
a frame `f`, `std.*` names `f`'s own datums (§5, §8, §14.4) where 0.167 refused them (**E101**):
`std.x`, `std.y` and `std.z` are `f.u`, `f.v` and `f.normal`, `std.back` is `f.u` reversed,
`std.top`, `std.front`, `std.side` and `std.up` are `plane(u: f.u, v: f.v)`, `(f.u, f.normal)`,
`(f.v, f.normal)` and `(f.normal, back)`, and `std.origin` is a point in that front at its `(0, 0)`,
at `f.origin`. They agree with the frame's coordinates — a point `in f` at `(x, y, z)` stands at
`std.origin + x·std.x + y·std.y + z·std.z` — so a frame at the world's pose has the world's
datums, and a body in a frame moved by a rigid motion is the root's moved by it. They are made once
a frame, when a body standing in it first reads one, stand where the frame does and add no freedom;
reading one names the frame's axes, so a bare frame is placed by relations on `f.u`, `f.v` and
`f.origin` (a mate) or reported free. What the body declares rides the frame as anything under
`in f` does. No syntax is added, and nothing that elaborated changes. The V-twin's cylinder stands
in a frame its bank mates to the rod, reading its own `std.up`, its positions unchanged.
(Issue #542, from #523.)

**[0.168] A frame's deflection drives.** `deflection(p, along: d) == v` in a scenario (§9.11),
`p` a joint of a frame, is a row of the drawing, solved for: the frame's deflection under the
scenario's supports and loads, its rate the adjoint's. A beam's section may now be read over the
drawing's declared unknowns (`param h: Length hint(40mm)`, `Iz: b * h^3 / 12`, any function of
them), which the row sizes; or it places a joint, a length an unknown. An unknown a section reads
is a seeded length or plain number (**E040**), and one no row binds is **E040**; a driving
deflection its frame cannot be read under (left free) or along a direction the solve moves is
**E040** where it is written. Other driving requirements compile nothing yet. Before this, a
section reading an unknown was **E040**; no document wrote one. (Issue #510, S1.)

**[0.167] A definition's standard datums are those of the frame it stands in.** Inside a
definition's body (`component` or `part`), `std.front`, `std.top`, `std.side`, `std.up`, `std.x`,
`std.y`, `std.z`, `std.back` and `std.origin` name the datums of the frame the instance stands in
(§5, §8): the innermost frame of the `in` blocks around its call or of an `in f` trailer on it,
else its parent instance's — at the root, the world. A plane is no frame (`Tooth(…) in top`
changes nothing about what `std` names inside it). The datums of a frame other than the world are
not drawn yet, so a body reading one while its instance stands in such a frame is **E101**, saying
so. The body's file must say `use std`, a module's by its own, and the datums are made whenever
any file read says it. A relation word still reads only its operands. Through 0.166 a body could
not read `std` at all (**E101**, `no such entity: std.front`), so nothing that elaborated changes;
the corpus drops the views it handed its definitions where every call handed the world's — the
engine's block and head lose all six of theirs. (Issue #524, from #523.)

**[0.166] Frames.** `arm := beam(span, post, A: b * h, Iy: …, Iz: …, J: …) of steel`
(§9.11) makes lines of the drawing members of a frame: Euler–Bernoulli beams rigid at their ends,
of the section stated in each member's own axes (z the normal of the plane its line is drawn in,
so `Iz` bends it in that plane) and the material `of` names. `A`, `Iy`, `Iz` and `J` are
required, `Sy` and `Sz` optional (a factor of safety reads them); each a positive power of
length the drawing states (**E103**, **E040** reading an unknown, **E001** twice). A target that
is no line, or a line standing in space, is **E040**. A beam adds nothing to the drawing, and a
scenario names it as a beam (a support or a pressure on one is **E040**). The drawing is the
mesh: members join where their ends meet or an end lies on another, and a point the scenario
names cuts a member there. **A scenario's requirements on a frame are answered and judged**,
never solved for — each output's value in the document's units, a claim kept or refuted — and
where there is none it says why: the supports leave the frame free, or its material states too
little. `beam` is a word only as a definition's call; no document used it as one. (Issue #510,
S0.)

**[0.165] What a solid is made of.** `steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3,
yield: 250MPa)` (§6.9) declares a material, a group whose members are the properties of an
isotropic linear-elastic solid: `E` and `yield` pressures, `nu` a plain number in (−1, 0.5), `rho`
a density, each optional. `arm := solid(sec, depth: 4mm) of steel` says the solid is made of it.
A property of the wrong dimension, out of its range, reading an unknown, given twice or not one a
material has, and a material stating none, are **E103** (twice **E001**) where they are written,
and a **bare number for a dimensioned property is refused** — the document's unit of pressure is
nothing anyone writes. An `of` naming something that is no material is **E103**. A material is
held in SI whatever the document's unit, and its members read as a group's (`steel.E / 1GPa`).
Nothing reads a material yet: it is the first step of design intent as structural performance.
`of` becomes a reserved trailer, as `in` is, and `material` a word only as a definition's call; no
document used either. (Issue #510, P3.)

**[0.164] Load scenarios.** `scenario NAME { … }` (§9.11) states a load case at the top of a
document or in its `preview`: supports (`fixed X`, `pinned X`, `along: d` for a roller), loads
(`load X (fx, fy, fz)`, `load X f along: d`, `pressure F p`, `accelerate B (ax, ay, az)`) and
requirements, an output (`deflection`, `safety`, `mode`, `buckling`, `compliance`, `mass`)
compared with a number the drawing states — a claim (`claim …`), a driving row (`== v`) or a
bound (`<=`, `>=`). Targets are read by the selectors every operand is: points, edges, faces,
solids and face paths (`arm.base`); every number is held to its dimension (**E103**). `scenario B:
A { … }` begins with `A`'s supports and loads; a load combination is written out. A scenario adds
no geometry and no row, so the drawing solves as it does without one; one scenario's requirements
may drive dimensions (**E106** at a second's). Nothing is analysed yet. `scenario` is reserved;
the line words are contextual. No document changes. (Issue #510, P4.)

**[0.163] A perpendicular that says which way.** `l1 perpendicular(sense: ccw) l2` (§9.3, §9.4)
states that `l2` is a quarter turn counter-clockwise of `l1`, as `angle` counts, and `sense: cw`
the other quarter; `perpendicular` with no sense is square either way, as before. It is a
relation and states no number, so it draws no callout. In space it is **E040** at the key.
`std.Turned` states its turn this way, so its `v` is a quarter turn *anticlockwise* of its `u`
from any start: through 0.162 it was square and equal to `u`, which has the mirrored frame as a
second root, and a datum turned a quarter or more in one solve could land on it (`t` from `(14, 23)` to `(7,
24)` about `o = (10, 20)`, and further). A drawing whose `Turned` frames were right-handed is
unchanged. (Issue #497.)

**[0.162] Parts.** `part Name(…) { … }` (§8.2) is a component declared a function of what it is
given. It is held to two rules, judged on each instance's rows with its inputs held:
- its rows place everything it makes, or **E108** at the definition names what they leave free;
- no row is about its operands alone, or **E109** at the row.

A freedom a part leaves is a formal the call leaves unbound, an unknown of the parent. A parent's
statement over a part's outputs is the parent's row and never completes it. A part is solved flat
as a component is. `part` is a word only before a definition's shape, `part Name(…) {`, so the
documents that name a thing `part` are unchanged. No document changes. (Issue #461.)

**[0.161] A stated orientation is held.** `ccw(a, b, c)` and `cw(a, b, c)` written in a
document (§9.6) are checked on the solution and the solve is steered to them, as a bound is
([0.60]): a triangle whose seeds turn the other way has a point carried across the line through
the other two before the solve, everything seeded from it following; a solution that turns the
other way is steered to the root that turns the stated way; and where none is found the solve
fails (`a stated orientation does not hold`) and the diagnosis names the statement. Three points
the drawing puts on one line turn neither way and break it. A point 5 from each of `(0, 0)` and
`(6, 0)` under `ccw(a, b, x)` is `(3, 4)` from any seed; through 0.160 a seed below the line gave
`(3, −4)`, reported solved, and the statement was read only by a plan replaying its closed-form
construction. What the document states is kept apart from the root choices a plan records of its
seeds, and only it is held. Inside a traced body an orientation is read at the anchor, as before
(§6.5.1). A drawing whose solution kept its orientations is unchanged. (Issue #157.)

**[0.160] A definition reads the constants of its file.** A component's or a relation word's
body (§8) reads the numbers its file states at the top and those of the modules the file `use`s,
by the names the file reads them by (`engine.dims.L`, `L` under `use engine.dims (L)`), where it was
**E103** ("outside this component; pass it as an argument"). Only constants: a value that is or
reads an unknown (`param c: Length`) stays out of reach, **E103** saying so; so do a document's
`preview` values and its root instances' numbers, and every number of a file that uses the module.
A formal, or a number of the body, by the same name hides the constant. So a dimension table is
read where it is needed rather than threaded through every call as a `group`. No document changes.
(Issue #461.)

**[0.159] A body drawn in the plane its operand is drawn in.** `in pin.plane { … }` (§6.7) reads
the plane a point is drawn in (`o.plane`, [0.152]) as the frame of an `in` block, through any path
that reaches a point (`in pin.end.plane`, `in l.p1.plane`): a component draws where what it is
handed is drawn, with no plane formal, and the plane is read once that point is drawn, wherever
either is declared. A point standing in space is drawn in no plane, **E040** at the reference.
Through 0.158 any path in an `in` was **E040** ("`o` is a point, and `in` names a plane"). No
document changes. (Issue #461.)
**[0.158] An envelope in space generated under a drive, held.** A relation holds to an envelope
in space (§6.15) generated under a driven frame (§6.14, [0.143]) as to one generated at a constant
rate, the joint standing at the law's value of the roll: with `tube := envelope(skin over t, under:
follower, seen_from: cam)`, `P coincident tube` holds `P` on the tube the follower's ball sweeps as
the cam sees it, and the ball's radius solves through it; its normal, a plane or a line touching it
and its curvature at `P` read the same law. The law is a constant of the relation, read in Taylor
arithmetic, so the rows and their Jacobian are exact through it. A turn about a point of a view
(`std.Spin(cam, about: std.origin, on: std.front)`) is read in such a motion as the turn about the
line through the point square to the view, as a carried point reads it. Through 0.157 such a
relation was **E061** ("a relation held to what is generated under it reads no drive", and a spin
in a view "an envelope in space is generated under turns about lines"); a meshing on a drive still
is. No document changes. (Issue #398 G.)

**[0.157] One copy of a pattern, by its index.** An operand the document wrote in a `repeat`'s or
a `ring`'s body is a step of its body's faces' names with its copy's index, as a reference to the
copy is written (§6.21): the hole the second drill of `repeat 3 { … drill := solid(face(hole),
through: plate) … }` cuts is `plate.drill[1].hole`, so `plate.near chamfer(d: 1mm) plate.drill[1]`
chamfers that hole alone (`drill[i + 1]`, any whole number the scope reads), and `plate.drill`
names every copy's faces, as it did. An edge drawn in the copy is named as the copy names it:
the hole's side is `hole`, not `hole[1]`. A ring's copies' caps, one name through 0.156
(`pegs.peg.near`), are each the copy's (`pegs.peg[3].near`). Through 0.156 every copy's faces had
one step (`plate.drill.hole[1]`) and an index on it was **E101** ("no such entity"). A word's
solids are no copies a document wrote and keep their names (`plate.taken.flat`). Reported face
names of copies change; no document's text does.

**[0.156] A point carried under a drive, held.** A point carried to a roll by a driven frame
(§6.14, [0.143]) is held where the joint stands at the law's value of the roll: `X coincident r0
at(under: follower, seen_from: cam) 30deg` is the follower's roller seen from the cam at 30° into
the turn, and `Z coincident(under: follower, seen_from: cam) r0` with `Z distance(23mm)
std.origin` solves for the instant the follower has risen 3 mm. The law is a constant of the
relation, compiled over the numbers it names as they are stated and read in Taylor arithmetic, so
the rows, their Jacobian and `rate(X over t)` are exact through it; a law naming a number the
drawing does not state is **E080**, now saying so. Through 0.155 the point and its rate were
**E080** ("a relation the solve holds reads no drive"). No document changes. (Issue #398 G.)

**[0.155] A profile in the plane generated under a drive.** A joint driven by its frame's value
(§6.14, [0.143]) carries a tool of the sheet into a planar envelope (§6.15.1) as a constant-rate
joint does, standing at the law's value of the roll: `fix(value == 180deg - abs(180deg - t))
follower` and `envelope(roller over t in (0deg, 180deg), under: follower, seen_from: cam, side:
near)` cut a heart cam's rising half from one law. The law is a constant of the curve, compiled
over the numbers it names as they are stated (one the solve moves is **E080**) and read in Taylor
arithmetic, so the profile is exact to the order it was at a constant rate and `p coincident f`,
`f tangent l` and `f curvature k` hold against it. Where the law turns a corner each profile reads
it on the side its own interval runs over. Through 0.154 the profile was **E080** ("a profile in
the sheet is generated under joints at a constant rate"). `generation/heart_cam.sv` is rewritten
over `param t` with one law in place of its two constant-rate slides, and no solved value moved.
(Issue #398 G.)

**[0.154] A frame's joint is said in words.** `std.sv` states a joint as relation words (§6.14),
the frame the subject and what it moves about or along the object, each of one number given bare
(§9.9, [0.152]): `f turns(r) l` about a line, `r` radians a radian of the roll; `f screws(a) l` a
screw about it, a radian a radian, `a` along it a turn; `f spins(r) o` about the point `o`, square
to the plane it is drawn in (`o.plane`; a point standing in space is **E040**); `f slides(a) l`
along a line, `a` a turn. Through 0.153 they were components called for their effect,
`std.Turn(f, about: l, ratio: r, advance: a)`, `std.Spin(f, about: o, on: P, ratio: r)` and
`std.Slide(f, along: l, advance: a)`, which are kept with the same meaning for one release. A ratio
the solve finds is given a declared unknown (`param k: Scalar` then `f turns(k) l`), where
`std.Turn` left it out. The library's 67 calls are rewritten (`counter spins(-1) centre`,
`rack slides(2 * pi * r1) pitch_line`, `climb screws(pitch) spindle`) and no solved value moved;
a `fix` a word's body states is held where the word is written, so an edit does not write it again
at the root. (Issue #464.)

**[0.153] A line square to a plane.** `perpendicular` relates a drawn line and a plane, written
either way round: `P perpendicular l` turns `P`'s normal along `l` in space, whatever view the line
is drawn in — two rows across the normal over the line's ends' places, as an axis square to a
plane is (`LinePerpendicularPlane`). Through 0.152 only an axis was (`t perpendicular P`), and a
line was **E040** ("`perpendicular` does not relate a plane to a line"). No document changes.

**[0.152] A word's one number bare, and the plane its operand is drawn in.** A relation word of
one parameter may be given it unlabelled, as a constraint word of the language takes its one
number (§9.9): `r above(7mm) q` is `r above(d: 7mm) q`, and passed on bare inside a body (`a
twice(d) b := a above(d * 2) b`) it is the inner word's one parameter; through 0.151 it was
**E004**. A word of two or more keeps its labels (`b offset(dx: 30, dy: 12) a`; given in order,
**E004**), and a bare value given a word of none is **E040**. A point drawn in a plane has that
plane as its member `p.plane` (§6.7), read wherever a plane is, its `normal` included — so a word
about a point drawn in a view reads the view from its operand and stays closed over what it is
given: `f spins(ratio) o := { o coincident f.axis; f.axis parallel o.plane.normal; fix(ratio ==
ratio, advance == 0mm) f }` is `std.Spin` with no `on:`. A point standing in space is drawn in no
plane: its `plane` is **E040**, where the word is written. No document changes. (Issue #463.)

**[0.151] A claim with witnesses of its own.** `claim { … }` declares geometry made only to
state a claim, its **witnesses**, and states relations of them (§9.7): `claim { x := point; x
coincident upper; x coincident lower; x coincident centres }` claims that the runs of a belt meet
on the line of centres. The drawing holds every witness while it solves, so the block restricts
nothing — the document solves, counts and diagnoses as it would without it — and after the solve
the witnesses are solved by the block's relations, the drawing held, and the block judged as
one claim: violated where no witness exists (the parallel runs of equal pulleys, where the drive
still solves), a theorem where one exists for every nearby solution of the drawing, consuming
where one exists only at this pose. A statement outside the block that names a witness is
**E040**; a block holds declarations, relations and `when` blocks, and a condition on the
drawing's numbers is a `when` around it. Through 0.150 the same point was drawing geometry, and
over-constrained or broke the document. No document changes. (Issue #460.)

**[0.150] A frame the drawing moves.** A frame with axes and no joint, placed by relations on
geometry the drawing moves as a declared parameter runs — `coupler := std.Turned(b, c) in
std.front` on a four-bar's coupler — is a motion over that parameter (§6.14): a reading over the
parameter (`sweep(tool over t, under: coupler.axes)`, `envelope(k over t, under: coupler.axes)`,
either's `seen_from:`) and a placement at an instant where the document declares one parameter
(`tool at(under: coupler.axes) 90deg`) carry what they read from where the frame now stands to
where the drawing followed along `t` puts it; through 0.149 such a frame stood still and `under:`
it was refused ("does not move"). The parameter is an angle; a relation the solve holds reads no
march (**E080**), and nothing is held to a profile generated under one. `trace(m over t)` of a
point in space is a curve in space (§6.5), drawn in the workspace (through 0.149 **E060**). No
document changes. (Issue #398, item F.)

**[0.149] Rounds that know of one another.** A defined word's use may be named (§9.9): `rim :=
plate.c0 round(r: 3mm) plate.far`, what its body makes standing under the name, so a round's
faces are `plate.rim.taken.round` (the name alone every face it made) and two named rounds' faces
stay apart, where written bare they share the body's `taken.round`. A round that names another's
faces by that name reads the body as that one leaves it and acts after it (§6.9, the stratum in
layers): `bevel := plate.rim chamfer(d: 4mm) plate.far` and `plate.bevel round(r: 1mm) plate`
round the two edges the chamfer made, in either order of the lines — what `part :=
solid(plate)` then `part.plate.bevel round(r: 1mm) part.plate` said by hand; rounds that name each
other are E041. A face a body's round made is among those `repeat a in X` runs over only where
`X` names that round, and a block over a solid's faces is run again while its solids give them
otherwise, so a round of a body whose operand is rounded reads that operand's rounds' faces
(through 0.148 it read the faces of the first pass, which had none, and made nothing). A face's
`F.smooth` is its surface carried on across every seam where a face of its body goes on from it
tangent (§6.21), and the standard library's `Run`, `Fill`, `Flat` and `FlatFill` stand their ball
and flat against it: a chamfer whose leg runs past the far face of a plate onto its rim's round
rests on the round, where through 0.148 its flat stood on the plane run on into the air and left
a ledge over a gap. Where no face goes on tangent within the section's reach of its edge nothing
changes; where one does the region is read point by point (the lowering table builds the round
or the flat of the face's own surface, and refuses one that reaches across such a seam). No
document changes but those that relied on either reading. (Issue #183, X0's findings 2 and 5.)

**[0.148] A family the same set in any order.** A set family may say, after its formals, that it
is the same set whichever order some of them are given in (§6.21): `component Run(a: face, b:
face, r: Length) symmetric(a, b) := { p | … }`, an item a formal or a tuple that changes places
whole (`symmetric((a, la), (b, lb))`). An instance given what one before it was given, in an order
the family allows, is that instance. The standard library's `Run`, `Fill`, `Corner`, `CornerFill`,
`Flat` and `FlatFill` say so, so `round(r) X` and `chamfer(d) X`, which reach every edge in both
orders of its faces and every corner in six, state each once: a body of nine faces rounded all over
is 240 regions where it was 1152, and the same body. The word is contextual, read only there. No
document changes; one defining such a family of its own may now say so. (Issue #415.)

**[0.147] Parts in frames of their own.** A drawing's `sketch` of a plane shows what is drawn in a
plane lying in it and facing the same way, where it stands (§13.2): a part drawn `in` a frame of
its own (`in f.axes { … }`) is drawn where the frame puts it; through 0.146 a sketch showed what
was drawn in its own plane only. An overall extent `dimensions in VIEW` reads of a solid reaches a
round outline's own edge. The V-twin is written so: each moving part — the crank disc, the
cylinders, the pistons — drawn in its own frame, the frames placed by the kinematics, and the
crank's angle a declared `param t` the drawing is followed along (`crank.theta`, the free
unknown a drag turned, is gone, and with it the table's `theta0`); its sheet shows it `at t =
180deg`, the drawing it showed before, to the number. The model language is unchanged. (Issue
#398, item K.)

**[0.146] The instant shown.** A drawing shows a model over a parameter at an instant of its
march (§13.1): `model m from "four_bar.sv" at t = 90deg` (or `t=90deg`), the value in the model's
units and of the parameter's dimension, stood on the branch the march carries; the same file at two
instants is two snapshots, and an instant the march did not reach, a parameter the model does not
declare or a value of another dimension is refused, saying why. The app's timeline shows the model
it edits at an instant the same way, as view state; no instant is written into model source, and no
seed is written back from one. The model language is unchanged. (Issue #398, item L.)

**[0.145] Faces swept over a declared parameter; frames along a path.** A face family runs over a
declared parameter (§6.9): `param s: Scalar over (a, b)` and `sweep(Station(…).f over s)`, the
station's formal that runs the one the call gives the parameter (`Tube(g, s: u)` over `u`) or else
the one named as it, of the parameter's kind (E103 otherwise); `sweep(Station(…).f over s in (a,
b))` is retired, refused naming the new spelling, and a component's body, which sees no parameter,
holds no face family. A face of the drawing itself is swept as a declared parameter of any kind
runs, `sweep(sec over s)`: a face family whose station is the drawing, drawn where the parameter
stands (its start) and solved again at every other station; a solid swept over a parameter that is
no angle is E103. What a jointed frame with a value draws in its `in` block stands where the value
puts it (§6.14; through 0.144, E080): each plane the block draws in is read as that plane drawn in
the frame (`P in f`), and a relation in the block reads it so, so the drawing at rest is carried
whole — the epic's helix, `in screw { in std.front { … } }` swept over `s`, is written as shown; a
declaration drawn in the valued frame and no plane of it is E080, and a frame declared in the block
still is. A frame's joint held along a path (§6.14), `g coincident f.axis` beside `fix(ratio == 0,
advance == 0mm) f` (`std.Along(f, g, up: …)`), or ratio 1 (`std.AlongFrenet(f, g)`), makes a frame
along the path: its pose where the path's parameter is `u` read off the path after the solve, the
rotation-minimising frame (ratio 0) or the path's own (ratio 1) carried from where the frame is
drawn, its home; a face drawn in it and swept over a declared parameter stands where the frame
does at every station. Any other ratio, an advance, a frame drawn off its path, and a relation the
solve holds reading such a frame are refused (E080). The case library's face families are written
over declared parameters, `lofts/thread.sv`'s thread as the epic's helix, and `twist_drill/`'s
roll over a declared one; no solved value moved. (Issue #398, items B and C.)

**[0.144] Relations held to a followed trace.** `trace(m over t)` (§6.5) is a curve a relation may
hold to — a point on it at an instant, a line tangent to it, a circle osculating it — and a number
of the drawing that does not run with `t` (a declared unknown, an instance's unbound number) may
solve through it; through 0.143 a relation naming it was E061. What moves with `t` must place
every number it moves: where it leaves one free, or holds what such a curve cannot read yet, the
relation is E061, saying why. The case library's mechanisms solved through a traced path
(`stride.sv`, `osculating_stride.sv`, `dwell.sv`, `peaucellier.sv`) and `jansen.sv` are written
over `param t`, to the same numbers. (Issue #398, item A.)

**[0.143] General drives.** A joint's value that reads a declared parameter drives its frame by it
(§6.14): the value is a law, any expression of the parameter (through 0.142 a value over one was
read only by what is drawn in the frame, at the instant the drawing is held at), and every reading
of the frame's motion over the roll — a sweep, a placement, a roll through a body, an envelope in
space — reads the roll as that parameter, the joint at the law's value of it. A value reads one
parameter (E103 otherwise); one over a parameter that is no `Angle` is followed but not read over
the roll (E080); a relation the solve holds reading a drive, and a profile in the plane generated
under one, are E080. A face family's frame is recognised as a constant twist where its value is
affine in the running number as written, no longer where three samples of it are. A value of `t`
alone is the frame undriven, and no existing document changes meaning. (Issue #391, stage 5.)

**[0.142] Relations over a parameter.** A dimension of the document's own relations may read a
declared `param t: Angle over (a, b)` (§6.3; through 0.141, E103), and the relation holds for every
value of it, as every other relation of the drawing does: the drawing is a function of `t`, drawn
at the interval's start. It is followed along `t` by continuation, one branch carried from the
start, to the interval's end or to a singular instant, where the march stops saying whether the
motion turns back there or two branches cross. A drawing not held at the start is not followed:
the freedom left is a free function of `t`. The diagnosis counts the freedom each parameter drives
(`1 DOF over t` for a four-bar's crank) and names what is left beside it a free function of it. A
value reading a parameter is E103 (a value is worked out once). `trace(m over t)`, new, is a point
of the drawing itself as `t` runs (§6.5): a curve of its view, over the parameter's interval or a
stretch of it, ending where the march stops, drawn and held to by nothing (a relation naming it is
E061). A joint's value may read a parameter (§6.14), so what is drawn in the frame is a trajectory.
No existing document changes meaning. (Issue #391, stage 4.)

**[0.141] A joint's value.** A jointed frame has a value, the instant of the roll it stands at
(§6.14): `fix(value == τ) f`, beside its ratio and advance, turns its joint `ratio · τ` and raises
it `advance · τ / 360°` from where it is drawn — linear in `τ`, the ratio and the advance its
slopes — and a frame stating none stands at rest. `τ` is an Angle worked out as a dimension is, so
it may read a face family's running number or name an unknown, never a declared parameter (E103).
`n := x in f`, new, is `x` drawn in `f`, where its value puts it: the reading `x at(under: f) τ`
whose instant is the frame's, making a frame, a solid or a point as `x at τ` does. A frame with a
value rides on none, and what its `in` block would draw is E080, naming `n := base in f`. The
standard library's `Helix`, `OnHelix` and `Twisted` are frames whose value is written over `s`,
their sections drawn in them (`n := base in screw`); their readings at an instant are retired, and
every station is where it was, to the solve's tolerance, as are the two springs' one screw.
(Issue #391, stage 3.)

**[0.140] At an instant and over an interval.** Every reading of a frame's motion is one of two
operations (§6.14). `x at τ` is `x` where the roll `τ` puts it, one verb whatever `x` is: `n :=
base at τ` a frame carried (0.138's `n := base carried(at: τ)`), `s := tool at θ` a solid placed
(`solid(tool, at: θ)`), `X := x at τ` a point held there, new, and `X coincident x at τ` a point
drawn elsewhere held so (`X coincident(at: τ) x`); the frames stand on the `at`, `x at(under: m,
seen_from: g) τ`. `over t in (a, b)` after the operand that runs is the interval of the roll a
reading runs over: `sweep(tool over t in (a, b), under: m)` a solid a motion sweeps (`solid(tool,
under: m, from: a, to: b)`), `sweep(tool, through: body)` one rolled through a body,
`envelope(tool over t in (a, b), …)` and `meshing(f1, f2 over t in (a, b), …)` (their `from:` and
`to:`); `over t` alone reads a declared `param t: Angle over (a, b)`'s interval, the first reading
of one. A face family is `sweep(Station(g).f over s in (a, b))` (§6.9, `solid(…)` through 0.139)
and a traced curve `trace(leg.toe over theta in (a, b))` (§6.5, written bare through 0.139). Every
retired spelling is refused naming its replacement; the corpus is rewritten and no solved value
moved. (Issue #391, stage 2.)

**[0.139] One parameter axis.** `param t: Angle over (a, b)` declares a number that runs over an
interval (§6.3): it states its type (`Length`, `Angle` or `Scalar`), has no value and no seed, and
its ends increase (E103 otherwise). Nothing reads one yet — a dimension reading it is E103. A face
family's running number (§6.9), a traced body's formals (§6.5) and the interval of the roll a
reading states (§6.14, §6.15) are represented as that one notion, with no change in what any
document means and no solved value moved. (Issue #391, stage 1.)

**[0.138] One notion of a moving frame: station words carried by a frame's motion.** A frame that
moves as a number runs is a frame's motion read at a roll (§6.14). `X coincident(at: τ) x` carries a
point to a roll the statement states, by what `x` rides on or the frame `under: m` names — an Angle,
so it may be written over a face family's running number and is read again at every station, or name
an unknown the solve finds; drawn in a plane, the point is held there by two equations; `seen_from:
g` reads it as `g` sees it. It is the reading `t == τ` is, the roll stated rather than the
relation's own unknown, and saying both is an error. `n := base carried(at: τ)` carries a plane
whole, its origin, axes and normal, by what `base` rides on or `under: m`. A frame may turn about
another frame's axis, `std.Turn(f, about: P.normal)`, as about a line. The standard library's
`Helix`, `OnHelix` and `Twisted` are written over frames so (`std.Turn` about `base`'s axes, with
`advance:` a screw; `std.Spin` a turn in a view), and contain no `cos` or `sin` of the running
number and no geometry drawn to hold an axis; their stations are what they were, but the frames of
`lofts/spring.sv`, `lofts/ribbon.sv` and `docs/lofts/l0/spring.sv`'s thread, which through 0.137
could stand turned against their own lines where the solve started them, now run with them. A face
family every relation of whose station reading its running number is a point one frame's motion
carries, at a roll affine in it, is that motion's sweep (§6.9): `lofts/spring.sv`'s wire and
`swept_spring.sv`'s ball are one tube on one screw, and both export exactly. (No measurement in a
motion's numbers, the issue's first stage, is 0.135's E107.) (Issue #366.)

**[0.137] Frames: joints in the solve.** A frame's ratio or advance no `fix` states — `std.Turn`
or `std.Spin` given no `ratio:`, a number pinned to an unknown — is an unknown of the solve, a
column of every relation reading the motion, and a joint nothing holds is freedom the diagnosis
reports. `std.Rolls(at, a, b, along: d)` holds two frames rolling on one another at a point
without slipping along a direction, so a gear pair's ratio follows its pitch radii, or a stated
ratio places the pitch point; `lantern_generation.sv` times its roll so. A profile generated in
the plane is generated over stated numbers. Through 0.136 a joint stating no ratio was E080. (Issue
#383, stage 5.)

**[0.136] Frames: riding.** `in` nests in `in` (§6.14): a declaration in nested blocks rides on
every frame of its path and is drawn in the innermost with axes (an outer one with axes is W115),
and a frame declared in a block rides on the block's frame, its motion its joint after theirs. A
reading at a roll that leaves `under:` out reads what its tool rides on — `X coincident(t == τ)
x`, `envelope(tool, seen_from: G, …)`, `meshing(f1, f2, …)`, `solid(tool, at: θ)`, and over a
riding solid `solid(tool, from: a, to: b)` and `through:` — and what it makes rides on its
observer. `under:` on a tool that rides is refused, as is a tool whose parts ride on different
frames. Through 0.135 an `in` block inside another was refused, and a plane given twice (a block
and a clause, or two blocks) was an error, now a nesting. (Issue #383, stage 4.)

**[0.135] Frames: a motion is a frame's joint.** `motion(…)` is retired (§6.14). A frame's joint
is three members minted where a reference names them — `f.axis`, `f.ratio`, `f.advance` — held
by relations, and the standard library states them: `std.Turn(f, about: l, ratio: r)` (with
`advance: a` a screw), `std.Spin(f, about: o, on: P, ratio: r)` (a turn in a view) and
`std.Slide(f, along: l, advance: a)`. A frame with no joint is the world. A reading names the
frame it is seen from beside `under:`, `seen_from: g` — on `solid`, `envelope`, `meshing`, a
carried `coincident`, `claim over m seen_from g in (…)`, `l reciprocal(seen_from: g) m`,
`std.Screw(isa, m, seen_from: g)` — where 0.134 declared `motion(m, relative_to: g)`. `phase:` is
gone: roll 0 is as drawn, and a phase φ at ratio r is the roll offset φ/r. A ratio or advance is
stated from the document's numbers; a measurement there, allowed in a motion's numbers since
[0.27], is E107. A formal `m: frame` takes a frame, a `frame` formal left out is the instance's
own frame at rest, and a number formal may state a default, `advance: Length = 0mm`. `motion(…)`,
`m: motion`, `relative_to:` and `phase:` are refused naming what they became. (Issue #383, stage
3.)

**[0.134] Frames: the bare frame.** A plane is a frame whose `in` draws flat, and `f := frame`
declares a spatial one bare (§6.7): it owns nothing until the document names its axes `f.u`,
`f.v` or its origin `f.origin`, which are then seven numbers placed by relations, as a plane's
are. A point `in` a frame with axes has three numbers along them (`u`, up, out) — `hint((x, y,
z))`, `hint(at: f, (x, y, z))`, `fix((x, y, z))` — and one in a frame with none is a point in
space. `in (f := frame) { … }` declares the frame in the header. A spatial frame is no set:
`p coincident f`, `f distance(d) g`, `f parallel g` and `p inside f` over one are E040. `frame` is
a declaration only as a definition's whole value, so documents keep the name (`hub_motor`'s
`frame := {…}`, `hypoid_drive`'s `frame := Frame(…)`); 0.15's `frame f(origin: o, …)` is refused
naming `f := frame`, and a formal `f: frame` is the plane's kind. Through 0.133 `frame` was
refused at a statement's start and as a formal's type. (Issue #383, stage 2.)

**[0.133] Cauchy data on a drawn curve.** A body marched over two formals (§6.5, 0.121) may start
its moved point on a curve the drawing has: `initially { p coincident(t == u) k }`, `k` a `curve`
formal given a spline, a curve drawn in a held plane (a formula, a trace, an envelope) or a curve
traced in space, `t` pinned to an expression over the formals. The body's columns are the curve's,
so the march's sensitivities run through them. A cam's pitch curve is marched from its profile
(`rates/pitch.sv`), its undercut the fold; the Wankel bore is read off the apex's envelope
(`rates/bore.sv`). A body traced in space is written over curves now as well as points, lines and
motions, and a `curve` formal of one takes a spline or a curve traced in space. Through 0.132 the
Cauchy data were relations and expressions over the formals and points a motion carries (0.131),
and a curve given to a body traced in space was E103. (Issue #376.)

**[0.132] A dimension that reads the body's own points.** In a body traced in space a dimension
may read a coordinate of a point the body declares (`p.x`), not only the formals, the numbers it
was given and the geometry it is written over (§6.5): a number the body places, held to the
expression by an equation of its own, so a rate row's speed may read where the point is — a front
moving at `1 / n` with `n` read at its own point, the eikonal equation in a graded medium
(`rates/fisheye_fronts.sv`, Maxwell's fish-eye). Over two formals it stands in a rate row or a
relation placing a point the rates do not move, never in `initially` (E103). Through 0.131 such a
text was E103 ("`p.x` is not something this curve can read"). (Issue #378.)

**[0.131] A point a motion carries, in a body that states a rate.** `X coincident(under: m, t ==
τ) x` (§9.3) is a relation without a rate in a body that states one (§6.5), over one formal or
two: a row of the body, held everywhere, or in `initially { … }` a row of the home's alone — so
a marched surface's Cauchy data may be a drawn point turned about an axis by the sampled formal
(`rates/geodesic_circles.sv`'s starting circles, `p coincident(under: spin, t == u) s`) or the
point a planar motion carries (`rates/housing.sv`'s trochoid, the apex `wankel.sv`'s rotor motion
carries). Its rate is the row differentiated along the formal, and its sensitivities to the
motion's axes and to the point it carries are exact. Through 0.130 such a body was E103 ("a body
that states a rate reads no point a motion carries"), and each document wrote its starting curve
as its ordinates. Both documents march the fronts they did, to the march's tolerance. (Issue
#377.)

**[0.130] The rate of a contact; a motion's twist read by rates.** `rate(p over t)` runs over the
slot of the contact holding `p` (§6.5) — a curve's `t`, a traced curve's `u`, a meshing's phase,
the roll of `p coincident(under: m, t == …) q` — where `t` names no formal in scope: the derivative
of the point the contact holds in that slot, every other unknown of the drawing held (a meshing's
lag and places following its rows). It is held `parallel` or `perpendicular` to a line or an axis,
and measured `distance(d, along: l)` (`d` stated or free); a rate along a
`param`, a slot no contact holding the point owns, or one owned by two contacts is refused, naming
it. Every statement at a point a contact holds is the contact's frame there, read by one kernel
whatever the contact: `g tangent(at: m) l` of a spline, a curve, a traced curve or a meshing is that
rate held along the line — the path of contact is `rate(Q over t) parallel l` — and a surface's
normal (`S perpendicular(at: p) l`, a meshing's common normal too), a plane or a line touching it
(`S tangent(at: p) P`, `S tangent(at: p) l`), its section's circle (`S curvature(at: p) k`) and a
curve's centre of curvature (`k curvature(at: p) c`, 0.125) are the same frame's, solving to what
they did. A curve's rate in its view may now be held square to
a line and any contact's measured along one, where its tangent was held only along a line. A motion's
twist has no words of its own: `reciprocal` and `screw` (0.105) leave the core, and the standard
library says them with rates of carried points — `L reciprocal m` (import it, `use std
(reciprocal)`) and `std.Screw(isa, m)`, whose `pitch` is the axis's advance a radian of the roll
(Ball's pitch times how fast the motion turns about it), a component's unknown read as any is. A
claimed word makes what it declares all the same: the `coincident` placing it is stated, the rest
judged. `screw(pitch == …)` is `std.Screw(isa, m, pitch: …)`; a twist is read of any motion, a
relative of a relative too. Through 0.129 the twist was read by two relation words of its own and
a rate ran only along a running formal. Every example solves to the values it did, to
rounding: bit for bit but `phillips.sv`'s screw axis (its last bit), `meshing_problem.sv`
(8e-9 mm, a solve that stops at its iteration limit, through `reciprocal`) and zeros at 4e-22.
(Issue #365.)

**[0.129] An arc walked from its end.** A chain may enter an arc at its end and walk it to its
start, `-> tangent (k3 := arc(center: c2, start: f, end: e)) ->` entered at `e`, or the arc named
there: the same arc, drawn from its end and turning the other way, so a family runs it forward and
L0's S-bend is one chain with its right turn written as a counter-clockwise arc. `k3.start` is
still `f`. An arc two chains walk opposite ways is E080. Through 0.128 a chain entered every link
at its start, and an arc entered at its end was E080 ("does not share the preceding link's
endpoint"); an arc in a plane turned right only said `sense: cw`. (Issue #363, S4.)

**[0.128] An arc's sense.** An arc drawn in a plane may turn clockwise from its start, `arc(center:
c, sense: cw)`, so a chain in a plane turns right at it — L0's S-bend is one chain in `std.top`.
Through 0.127 an arc in a plane ran counter-clockwise from its start always, and a chain entering
each link at its start could only turn left at an arc. (Issue #363, S4.)

**[0.127] Arcs and circles in space.** An arc over a point standing in space is drawn in the plane
through its centre and its ends, the short way round; three points in a line are refused. A circle
over a centre in space takes `normal:`, the axis or line it stands square to, and is drawn in the
plane through its centre square to it. Each is the arc or circle it is in a plane — its radius,
its contacts, a chain's joints, a face swept from it — so a bent pipe needs no plane written, and
a chain drawn in space turns either way at an arc (L0's S-bend is one chain); in a plane's block a
chain still turns left at every arc.
Through 0.126 each was E060. (Issue #363, S6.)

**[0.126] Splines in space.** A spline over a point standing in space is a curve in space:
`Σ Rᵢ(u)·Pᵢ` over its control points where they stand, its parameter over its own domain. Its
contacts are a curve in space's (a point held at a place, `t == u`; the tangent; the centre of
curvature), it is a `path` a family runs along, and `spline(through: …)` over corners in space is
the cubic through them, natural or along the tangents its ends are given — so a loft's smooth rail
may leave its plane. It bounds no face. Through 0.125 a spline over a point in space was E060.
(Issue #363, S3.)

**[0.125] A curve's centre of curvature, and Frenet's frame.** At a point a contact holds on a
curve traced in space, `k curvature(at: p) c` puts the point `c` at the centre of its osculating
circle. The standard library's `Frenet(g, s:)` is the station whose `u` runs toward that centre
and whose `v` is the binormal, refused where the curve has no curvature. Through 0.124 a curve in
space read no curvature (`curvature` was a surface's), and Frenet's frame had no word: the side
the centre is on could not be said, `parallel` being sign-free. (Issue #363, S5.)

**[0.124] Chains across planes.** A line touching an arc or a circle at a point of it, the line
not drawn in the round's plane, is the tangency in space: its direction in the round's plane and
square to the radius there (`TangentCircleAt3`). So a chain's straight between a bend in one plane
and a bend in another touches both, and a bent pipe — the most common sweep — is one chain a
family runs, its frame carried across every joint. A station that states a rate, drawn on a link
past the first, is no longer solved by the drawing, which cannot know the frame carried to it.
Through 0.123 a chain was one plane's: the tangency at a joint across planes was E062. (Issue
#363, S4.)

**[0.123] `On` integrates its frame, and a rate runs over a chain.** `std.On(g, up:, s:)` holds
its plane square to the guide and turns it by `Minimal`'s rate, `up` read at the family's home
alone: over a guide in one plane with `up` its normal the solid is the one it was, and along a
twisted guide (a curve traced in space) the section stays square to it. A station that states a
rate runs over a named chain, its frame carried across each joint, and a station read off its
integration meets its relations, so a law in pieces is read as written. Through 0.122 `On` held
`v` along `up` at every station, which along a twisted guide leaned the section off square to it
with nothing said, and a rate over a chain was refused. (Issue #363, S2 and S4.)

**[0.122] A curve traced in space is a path.** A curve traced in space (§6.5) is given to a
`path` formal: `m coincident(t == s) g` pins its contact's place, the body's own formal, as a
spline's `t` is pinned — to a face family's running number in a station — and `hint(at: m,
along: g)` steps along it. So a station stands on a guide drawn in space, its frame stated by a
rate (`std.Minimal`'s), and a family is swept along it. A surface traced over two formals is no
path (**E103**). Through 0.121 a station could stand only on a guide in one plane or one written
into its own body (`std.Helix`, `std.OnHelix`): a traced curve given to a `path` was E103, `t ==`
on its contact E040, and a step along it E103. `std.On` still held its `v` along `up`, so over
a twisted guide its plane leaned off square to the tangent: there the plane was held square to
the tangent and turned by `Minimal`'s rate (0.123 made that `On`). (Issue #363, S1.)
**[0.121] Two formals marched by the method of lines.** A body traced over two formals may state a
rate along either (§6.5): the first is marched, the second sampled at the Gauss–Lobatto nodes of
32 pieces of its interval, a rate along it the derivative of the quintic through a piece's nodes,
and `initially { … }` the Cauchy data where the first starts, for every value of the second — a
first-order PDE, an ellipse's offset front (`rates/front.sv`). The body's points whose rate along
the first formal a rate row reads are integrated; the rest are placed at every node by the
relations that read them, exactly as many, and only such a relation or a rate row reads a rate
along the second formal, of an integrated point. A rate along an `Angle` formal over two is per
radian. The surface's regularity is checked at every node and step, and a fold — characteristics
crossing, an undercut's kind — is refused where it happens. A contact on the surface reads its
point, tangent plane and normal; its curvature is not yet read. A rate written to the right of
its word, `nl perpendicular rate(p over u)`, parses as the grammar has said since 0.119 (it was
read as a chain's link). Through 0.120 a body over two formals stating a rate was refused.
(Issue #339, D4.)

**[0.120] The B-rep read after the solve leaves the language.** `patch`, `seam`, `vertex`,
`edge(…, along:)` and `face(…, on:)` (§6.16–§6.20) are retired: they added no unknown, no solve row
read them and nothing exported from them, and the one check that needed them, the spiral bevel's,
already made each of its parts in Rust. They are Rust API now — `TrimmedPatch::of_envelope` and
`of_surface`, `EnvelopeSeam::new`, `BoundarySeam::new`, `SurfaceSeam::new`, `BoundaryVertex::new`,
`JunctionVertex::new`, `SpatialEdge::new` and `SpatialFaceBoundary::new` — over the surfaces and
envelopes a drawing names. `patch`, `seam` and `vertex` are no keywords, `edge` no constructor, and
a face takes no `on:`: every face is planar. `surface` (§6.13) and the spatial `envelope` stay,
filed with what the solve holds to, and `surface` says its two meanings, one per kind of solid
it is read from: a revolution's side, a held spatial envelope's source, and a prism's side, a
planar envelope's. The section numbers are kept, so §6.21 onward keep theirs. Through 0.119 each
was declared in the document and read after the solve by a `::named` reader. (Issue #364, settling
#76 item 6.)

**[0.119] Rates along a running formal, in a traced body.** A traced body may state a condition
on a rate along the formal it runs over (§6.5), as a face family's station may since 0.116 (§6.9):
`rate(x over s)`, a point's velocity, is an operand, and stands on either side of a word that reads
a direction — `parallel`, `perpendicular`, a directed `angle`, and alone before `distance(k, along:
l)`, its component along `l` — never as a place or a magnitude; any other word is a syntax error,
and two rates in one relation, or `angle` and `distance` in a face family, are refused. Such a body
is integrated from its home: a drawn instance as it stands on the sheet (its rate rows no rows
there, its coordinates columns of the curve), or, written in place, where `initially { … }` puts
it. Its rate rows and its other relations differentiated fix the rates, a rank checked at every
place reached; a body short of a rate row is refused naming the freedom it leaves, and a march is
refused where its rates lose rank. Through 0.118 every relation of a traced body held at each
place by itself, and a body one row short was a curve that did not exist. (Issue #339, D2.)

**[0.118] A face carried whole, and a mirror.** `sec := prof mapped(from -> a, to -> b) in n`
(§6.8) is the face `prof` carried into the plane `n` by a map: each corner carried as a point is,
each edge rebuilt between the images, the source's corner and edge names kept under the image's,
so its walls are named as the source's are. A line is carried to a line; under the similarity of
two pairs an arc to an arc and a circle to a circle, its radius scaled — and `k mapped(o -> a, x ->
b) disc` says the same of one circle — while under three pairs either is refused, naming it.
`mirror: true` reverses the turn, so the image is walked the other way round and winds as its
source does: a handed pair is one profile and its image. The standard library's `Affine` and
`Similar` are each that one statement, and `Twisted(n, twist:, reach:, s:)` is the one point a
section's frame turns to, where it was a plane of axes. Through 0.117 a section was drawn again
in each station's turned or scaled axes (`std.Twisted`, `scaled`, `scaled_radius`), and a handed
pair was drawn twice or signed by a number threaded through every station (`hub_motor`'s
`sg`). (Issue #357, M2 and M3.)

**[0.117] A point carried by a map.** `q mapped(from -> a, to -> b, third -> c) p` (§9.2) holds
`q` where the affine map taking the three source points to the three targets carries `p`: each
pair a point of the source, `->`, and where it goes, in the parentheses. Two pairs are the
similarity, keeping the turn or, with `mirror: true`, reversing it. Each side is read in its own
plane, so a point drawn in one view is carried into another, and the source points are columns of
the relation, so the map is exact wherever its frame stands. A source frame standing in a line is
refused where it is stated, naming its points. The standard library's `Affine` and `Similar` are
one `mapped` per corner. Through 0.116 the map was written out in coordinates — Cramer's rule over
`x` and `y`, the determinant twice in each quotient — each corner two `offset`s through a hidden
point over a hidden pair of unknowns per side of the target frame (`std.Framed`, `std.Spans`,
retired), and a collinear frame divided by zero. (Issue #357, M1.)

**[0.116] A station may state a rate.** A relation may read the rate of an axis's direction or of a
point along a formal of its body, `rate(x over s) perpendicular y` (or `parallel`), the formal
always named; `initially { … }` says what holds at the drawn station alone (§6.9). A face family
whose station states one integrates it from the drawn station, its home, along the running number,
instead of solving it again at each station. The standard library's `Minimal(n, up:, s:)` is the
rotation-minimising frame of a station. Through 0.115 a station was algebraic at every place:
nothing could be said of how it moves, only where it stands. `rate` and `initially` are reserved.
(Issue #339, D3.)

**[0.115] A flank regular within its blank.** `flank regular(by: tool) within blank` reads the
least **area factor** of a generated flank (§6.15.3) — how much the map from a tool point to the
point it generates scales the tool's area, nothing at a fold — over the part of the flank the tool
cuts in the blank, found by search: a bound (`>= 0` with no number, no undercut; `>= m` a margin),
a verdict that steers nothing, or an equation that solves the setting at which a stated margin
binds. Tool and blank are region solids read by their terms; the flank's source is a face of the
tool, its round ending the face. `SpecKind::Solid` and `SpecKind::Ratio` are the first solid
operand and the first number with no unit. Through 0.114 undercut was read only after the solve:
the sweep's admission refusing a fold (E3), a traced surface's regularity report, and
`hypoid.sv`'s `Balanced`, which balances two osculating radii and bounds nothing. (Issue #345, S3.)

**[0.114] Ease-off.** A meshing's first flank may be eased (§6.15.4): `meshing(…, ease: M,
along: l, profile: κₚ, lengthwise: κₗ)` relieves it along its normal by the parabola `½ κₗ s² +
½ κₚ r²` about `M`, `s` along `l` and `r` across it, each curvature a number per length or affine
in an unknown of the drawing. A conjugate pair so eased touches at a point, the gear lagging by the
turn that closes the relief, and lags stated of points held to the contact solve the curvatures.
Through 0.113 a mismatch was written only into how the pair is cut — a cutter grown or turned, a
modified roll (`spiral_bevel/contact.sv`) — and `std.Offset` moves a set by a constant. (Issue
#345, S2.)

**[0.113] A block walks a face, walks run in step, and two faces drawn apart are lofted.** `repeat e
in f` and `cycle e in f` run over a face's walk (§12.8): its edges in order, a face of corners by
the straight run from each corner to the next. `cycle (ea, eb) in (a, b)` runs several walks in
step, a link of each in every copy, walks of different lengths refused naming both. A face may be
written of a block's copies, `face(side)` for `side[0]`, `side[1]`, … (§6.8), the count the block's.
A number a face family's station gives a relation word or a component may be any expression of the
geometry the station was given, worked out at each station (§6.9). Over these the standard library
says CAD's loft of two sketches, `Ruled(a, b, s:)`, corner to corner by straight rails in written
order — a connector is where a walk starts, and faces of different corner counts are made alike by
naming a point on an edge — a loft to a point, `Apex(a, tip, s:)`, and a section held to its rails,
`Similar(prof, from, to, n, a, b)` (two rails, a similarity) and `Affine(…, third, …, c)` (three, an
affine map). Through 0.112 a block ran over a named chain or a solid's faces, a face listed every
edge it was made of, two faces drawn apart had no loft (since 0.100), and a station's numbers read
the geometry affinely. (Issue #243, L8.)

**[0.112] The common normal and the path of contact at a point.** At a point held to a meshing,
`contact perpendicular(at: q) l` holds a line or an axis along the two flanks' common normal there,
and `contact tangent(at: q) l` along the path of contact: the contact point's derivative along the
phase, in the fixed frame (§6.15.4). Each shares the phase, the lag and the places of the contact
holding `q`, as an envelope's relations at a point share its place ([0.90]), so either reads at any
phase, and a number solves from an angle stated of it. Through 0.111 the normal at a contact was
stated on the generated flank alone, in the pose the pair was drawn at (`pinion_flank
perpendicular(at: M) l`), and the path not at all. (Issue #345, S1.)

**[0.111] A loft through several sections is drawn on rails.** A spline may be written through
the corners of a named open chain of lines, `spline(through: rims, start: l, end: m)` (§6.1): the
cubic through them, a span a link, its ends along the tangents written, or else running straight —
short for its minted control points and ordinary contacts. A face family given several named open
chains runs them in step, link by link (§6.9), so a guide cut at each section and a chain of lines
through each corresponding corner is a ruled loft through several sections, piecewise in the running
number with an edge at each section; the spline through the same corners is the smooth one's rail,
its ends' tangents the loft's end conditions. The standard library's `Across(a, b, c, s:)` is a
station with no guide, the plane through three rails at `s`. Through 0.110 a family ran one chain,
and a smooth rail was drawn by hand, its control points and a corner on it. (Issue #243, L7.)

**[0.110] A screw is constraints.** The standard library's `Helix(base, pitch:, turns:, s:)` is
the frame of the plane `base` turned `360deg · turns · s` about its normal and risen `pitch ·
turns · s` along it (§6.9): `n` is `base` so carried, and a section drawn in it sweeps a twisted
prism; `axial` is the plane through the axis, and a section drawn in it sweeps a thread.
`OnHelix(base, r:, pitch:, turns:, s:)` is the plane square to the helix of radius `r`, for a
coil's wire. Each direction is an ordinate in `base`'s frame, so neither word is held by a seed,
and a negative pitch winds left-handed. Through 0.109 the screw was `Screwed(ax, base, …)`, a
plane through an axis given apart from `base`, turned by a reference line and its `u` and `v`
held by `parallel`, which a seed decided the sign of; `OnHelix` took the same axis. `Screwed` is
retired: `Helix(…).axial` is its plane. (Issue #243, L6.)

**[0.109] A flank stated by its contact.** A meshing's flank may be a surface traced in space
(§6.15.4): the contact element as its member sees it, a point of a component as the phase and a
place along the element run, carried back by the member's turn. Phillips' pair is written so in
`spiral_bevel/phillips_pure.sv` — each flank a line of its plane of action moving along the line of
action, seen from its member — with no rack anywhere, and meshes as the rack-cut pair does: a
point contact along the line of action with no transmission error. Through 0.108 a flank was an
envelope or a source standing still. (Issue #326, R6.)

**[0.108] A surface over two formals.** `S := C(…).p over (phi in (a, b), s in (c, d))` is a
point of a component as two of its formals run: a surface traced in space, held to by a point on it
and, at a point held there, a line in its tangent plane or along its normal, a plane touching it
and a circle osculating its section (§6.5). Its normal and its curvature are exact. `solventc`
says whether it is regular over its domain or where it folds. (Issue #326, R5.)

**[0.107] A curve traced in space.** A traced component whose relations do not read in one plane,
or which is given a motion, is solved in space: §6.5's curve, a point and a tangent held to it in
space. Through 0.106 such a body was refused, a traced component solved in one plane alone
([0.42]). A component may take a motion as a formal, and its body carry a point by it (§9.3).
(Issue #326, R3.)

**[0.106] A point as a moving frame sees it.** `X coincident(under: m, t == τ) x` holds `X` where
the motion `m` carries `x` at the roll `τ` (§9.3): three rows, the roll in degrees and the
relation's own unknown, pinned or left to the solve. Under a motion seen from a turn, at the
negated phase, it is a point of the fixed frame as the turning member sees it — a rolling circle's
point seen by the fixed one is the epicycloid, a follower's contact seen by its cam the cam's
profile. A turn about a point of a view reads as the turn about the line through it square to the
view. Through 0.105 a point was carried by a motion only as an envelope's tool, after the solve or
in one plane. (Issue #326, R4.)

**[0.105] A motion's twist: `reciprocal` and `screw`.** Two relation words read a motion as
the velocity field it is (§9.3). `L reciprocal mesh` says a force along the line `L` does no work
on the motion — its direction square to the velocity of its points — which for the mesh of two
members is the meshing law of a line of action: they move as fast along it. `isa screw mesh` puts
the axis `isa` on the motion's instantaneous screw axis, with the pitch the relation's own unknown
(`pitch == h` pins it, `solventc` reports it). Either reads a turn, a screw or a slide about a fixed
line, or one seen from another, whose twist is constant in the fixed frame; a relative of a
relative is refused. They are the first relations whose operand is a motion. Through 0.104 the
line of action of `spiral_bevel/phillips.sv` was stated point by point, its helix angle worked out
by hand; now its pressure angle is stated and `L reciprocal mesh` solves the rest. (Issue #326, R2.)

**[0.104] A plane is an envelope's source.** `envelope(std.HalfSpace(o, l, d: x), under: m, …)`
is held to by the solve (§6.15.3): the plane square to `l` at `x` along it from `o`, its place two
lengths in it and the roll, its normal `l`'s everywhere. Under a slide along its normal relative
to a turn its envelope is an involute helicoid, so Phillips' spatial involute gearing is two of
them — each member's flank the plane square to the line of action sliding along it at the
members' common speed, seen from the member — meshed point to point along that line with no
transmission error (`spiral_bevel/phillips.sv`). A plane may stand as a formed flank of a meshing
too (§6.15.4). Through 0.103 the solve read cones and cylinders only, and a plane's envelope was
E040. (Issue #326, R1.)

**[0.103] Twist and scale laws.** A twist or a scale of a face family's section is a dimension
reading the running number, and any expression in it is a law (§6.9). The standard library adds
words that take a total and run it linearly: `std.Twisted(n, twist:, s:)`, axes in the station's
plane turned `twist · s` about its origin; `p scaled(du:, dv:, scale:, s:) P`, a point of a
section scaled about the plane's origin; and `scaled_radius(r:, scale:, s:) k`. Any other law is
the same word given another fraction (`s: 3 * s^2 - 2 * s^3`, `s: sin(180 * s)`). Through 0.102
a turn was the section's own construction, a reference line and a turned one
(`docs/lofts/l0/twisted_duct.sv`), and a fraction that was not affine in `s` could not be given:
a component's formal given one became an unknown of the drawing (the station "not held"), and a
relation word's was E103.
A `when` whose condition reads the running number is E040, as one reading an unknown was; it
now says the number is the family's, that a `when` is made once for the whole run, and that
`min`, `max` and `abs` write a law in pieces. (Issue #243, L5.)

**[0.102] Any path, a chain too.** A face family's `path` formal may be given a named open chain
of lines and arcs (§6.9): the station runs over it link by link, link `k` while `s` is in `(k, k +
1)`, a contact pinned to `s` on the chain at the fraction `s − k` of that link's run and every
other reading of `s` the chain's whole number, so a law runs along the chain. Each link's stations
start where the last link's ended, and a wall keeps one name across the links. Through 0.101 a
chain was no value a call could pass. `std.On` stands on an arc too: a line touching an arc or a
circle at a point of it is drawn in the round's plane (§6.21), where through 0.101 the tangency,
its point read in space, was E062. A seed's `along:` steps along any path where through 0.101 it
stepped only along a line, so `std.On` and `std.OnSpline` start their tangent ahead along their
guide and their plane with its `v` along `up`, never on the mirrored branch. (Issue #243, L4.)

**[0.101] A step: one number per run.** `step(u: s) (ab := line)` says where a line's end
stands, one ordinate at a time — `du`, `dv` from its start, `u`, `v` from the frame's origin — and
keeps the ordinate it does not name, so a stepped outline is a chain of runs, each a station or a
radius as the part is dimensioned (§6.6). Absolute and relative are chosen per ordinate, not per
step: `step(u: s, dv: b)`. The frame is the view the line is drawn in, or `from: P`. A step is the
ordinates and levels it names, each number a dimension kept where it is written; a literal zero is
a level. The closing edge takes no step. A corner a step places, with no seed written, starts
where the walk puts it. Through 0.100 such an outline was a point per corner, each placed by two
ordinates and seeded by a `hint` that restated them, and then the lines threading them: the hub
motor's cover was 18 points, 18 lines and about 40 lines of source, now 18 links. `heading(θ)`
and `rev arc`, planned beside it, are not in this draft. (Issue #330, from #309.)

**[0.100] A loft is library words.** `solid(section, along: guide)` and `solid(start_section,
end_section, along: guide)` are retired: they no longer parse, and `along:` in a solid's brackets
says what to write instead (§6.9). A loft or a sweep along a path is a face family (0.97) whose
station stands on its guide through a word of the standard library: `std.OnLine`, `std.OnArc`,
`std.OnSpline`, `std.On` (any `path` whose tangent at a point is read in space: a line, a spline,
a traced curve), `std.Screwed` and `std.OnHelix`, each a plane `n` square to its path at the
running number, its `v` along `up`. The section is the document's own component drawn in that
plane, so what through 0.99 was the constructor's pairing rule (edges joined in written order, a
second section's corners mixed with the first's) is a dimension reading `s`. The reducer, the
elbow, the narrowing bend and the square-to-diamond duct of the case library say so, at the same
volumes (`rust/examples/solid_loft.sv`, `solid_elbow.sv`, `solid_bend.sv`, `solid_twist.sv`).
Pairing two faces drawn apart has no word until the language can walk a face (#243, finding 8).
(Issue #243, L3.)

**[0.99] An empty body.** `pegs := solid()` is a body with nothing in its brackets: its stock is
the first solid in `union` with it (§6.9), so a pattern of solids joins every copy alike,
`repeat 6 as i { peg[i] union pegs }`, where through 0.98 the first copy had to be written apart as
the stock (`pegs := solid(peg[0])` and a `repeat` over the other five). `solid()` was E080 ("a
solid is a face swept … or a body over other solids"); an empty body nothing is in `union` with
still is, at its declaration, naming the statement that would give it material. (Issue #313.)

**[0.98] A design table's figures carry their units.** Three base dimensions join length and angle
for values: **mass**, **time** and **current** (§3.3), and the units derived from them may suffix
a literal (§3.3.1): `745.7W`, `48V`, `255rpm`, `0.85T`, `11.9kPa`. `+`, `-` and a comparison
demand agreement and `*` and `/` derive, as for lengths, so `torque := power / (omega / 1rad)` is
a torque and `torque + power` is an error. No slot takes them: a figure reaches a constraint only
divided by its unit (`torque / 1Nm`). Their numbers are SI with lengths in the document's unit,
so a figure read back over its unit is the same in every document. Through 0.97 a design table's
electrical and mechanical figures were plain numbers with their units in comments, and nothing
checked them. (Issue #311.)

**[0.97] A face that runs: the face family.** `duct := solid(Station(guide, …).f over s in (a,
b))` is the solid a face of an instance sweeps as one of the instance's numbers runs — §6.5's
curve (`Involute(…).p over u in (…)`) one dimension up (§6.9). The instance is written in place
and never drawn; the running formal is left unbound by the call and is **held**, never an
unknown: the station stands at the middle of the interval, is solved there with the drawing, and
is solved again at every other station, each warm from its neighbour. Its body reads as a traced
body's: any expression in `s` (`twist * (3 * s^2 - 2 * s^3)`), and a number of the geometry it
was given (`rim.r`, `a.x`), never of its own. A station inside the interval must be held — solved,
no freedom, no equation lost — and keep the station's topology (no loop crossing itself, no edge
lost or turned over, no hole touching); only an end may lose an edge or the whole section, and is
read as the limit of the stations closing in (an apex). The swept volume may not cross itself.
Caps are `start` and `end`; walls are named by the section's edges under the instance
(`body.sec.ab`). Calling the instance with the running formal bound is **E103**, an interval run
backwards **E080**, and a drawn station (`solid(st.f over s in …)`) a syntax error until it is
written (L10). (Issue #243, L1.)

**[0.96] A free curve is an edge of a face.** `rope := curve(a, b)` runs from `a` to `b`, so it
stands in a face whole, between those two points, as a line or a clamped spline does (§6.8):
`face(rope, deck)`, `face(rope, -> close)` — a hanging rope and its chord swept into a solid.
Through 0.95 a free curve was read as every other curve, with no ends of its own, so naming it
alone was E080 asking for a stretch, and the stretch `rope from a to b`, asking `a` to be held
on the curve, could only be reached by pegging the rope at its own ends. A stretch from a free
curve's own end is now E080 naming the whole curve instead, and a stretch between two points
held on one (`rope from m to n`) is that curve cut, where it had been read as a second free curve
with no energy and so no shape.

**[0.95] A set's body holds of a point of its plane; a call gives every entity, or owns a
point.** A plane's membership stated by a set's body (§6.21) of a point drawn in that plane
already — or twinned there — holds by construction and states no row, so a body meeting a
sphere with a plane, `{ p | p coincident ball; p coincident std.front }`, takes a point of the
front onto the circle the front cuts. Through 0.94 the row was refused as asking nothing
(**E061**), which the same words said of the point directly still are (§6.7). And a call that
leaves out an entity formal is **E103** at the call (a missing point argument, say), as a set or
group formal left out already was (§5): before, it was said only where the body read the formal,
and nowhere for an instance of a set family nothing used, so `std.Sphere(r: 5)` was silently
nothing. The one exception is new: **a point formal drawn in a plane formal**,
`center: point in on`, may be left out, and the instance then owns a point of its own drawn
there (`k.center`), seeded by `center: hint((x, y))` and written back there by a solve — so a set
family's parameters are its formals whether the call gives them or not, and
`Circle(std.front, r: 25)` is a circle with a centre of its own.

**[0.94] Two flanks meshing are a locus.** `contact := meshing(pinion_flank, gear_flank, under:
mesh, from:, to:)` turns the first flank with the pinion — `mesh`'s observer, `mesh :=
motion(gear_turn, relative_to: pinion_turn)` — and the second with the gear, each an envelope in
space the solve holds to or a cone, a cylinder or a revolution's side standing still in its member
(a formed gear's flank), and is where they touch: at each phase of the pinion, the gear turned to
the ratio's roll plus a lag, the transmission error (§6.15.4). `p coincident contact` holds a
point to the path of contact, owning the phase `t`, the lag `lag` (degrees) and a place on each
flank, so a pin states either — `coincident(t == 0deg, lag == 0.01)` — and a number that makes a
flank solves from it. After the solve the contact is traced over the phase, and `solventc` reports
each meshing's contact, transmission error and path (`spiral_bevel/contact.sv`, #283). `meshing`
is an element keyword: `generation/spur_mesh.sv`'s motion called it is `mesh`.

**[0.93] A document naming no unit is in millimetres.** Without a `unit` line a document is
read exactly as under `unit mm` (§3.3.2): a bare number in a `Length` slot is a millimetre, a
suffixed literal converts to it (`a distance(2in) b` is 50.8), and a geometry read in a seed, an
integrand or a measurement is a `Length`. Through 0.92 such a document was in unnamed *drawing
units*: a suffix was **E103** ("this document names no unit") and a geometry read was a bare
number. So `pin.x - 10` is now refused as under `unit mm`, and `pin.x - 10mm` is the spelling;
the dimension tables written to be read through a `use` (`hardware`, the engine's and the
V-twin's) elaborate on their own. The line, where written, is still what a document saves and
prints back, so one without it gains none; glTF and CAD exports scale such a document as
millimetres, and a paste converts it as one.

**[0.92] One vocabulary for a view's directions.** A view is a plane (§6.7), so its own
directions are said as a plane's are: between two points drawn in one view, `along: u` is the
run and `along: v` the rise — the view's `u` and `v` — as against a plane they are that plane's
(§9.2). `right`, `left`, `up` and `down` stay, the same two with the sign said. Through 0.91 the
run and the rise were `along: x` and `along: y`, which named the view's `u` and `v` by the world's
letters and read as the world's where they were not: in `std.front`, `along: y` was world z while
`along: std.y` was world y. `x` and `y` are no longer direction words: a name in scope by either
is a reference like any other, and one naming nothing is **E040** at the key, saying what the word
became. (Issue #268.)

**[0.91] `when` is the one conditional.** A `when`'s condition compares two drawn things by
**identity** — `a != b`, they stand for different things (an entity or a face, through every
formal and binder), `a == b`, the same — and `c in B` asks whether `c` is one of the faces `B`
names, joined with numbers' comparisons by `and`, `or` and `not` (§12.9). So a block over a
solid's faces keeps the copies it wants by a `when` in its body: `repeat b in B { when b != a
{ … } }`. The filter after the face path, `repeat b in B if b != a { … }` (0.69), is retired and
is a syntax error naming the `when`. A number against something drawn is **E103**. A switch is
written `when flag == 1 { … }`, not `repeat flag { … }` over a count of zero or one, and a
conjunction `when a == 1 and b == 1`, not `repeat a * b`: what a `when` makes keeps its name
(`rod1.sm`, `crank.t1.pin`) where the `repeat` named a copy (`rod1.sm[0]`). The engine, the
V-twin and the twist drill are rewritten so.

**[0.90] An envelope in space is held to by the solve.** `flank := envelope(S, under: m, from:,
to:)` over a cone or a cylinder set (`std.Cone`, `std.ConeThrough` with its lean stated,
`std.Cylinder`) — a set is now an envelope's operand, read as what its body states at its point —
or over a revolution's side turned from a line, an arc or a circle, under a motion of turns about
lines, screws, slides and `relative_to:`, is no longer only read after the solve: `p coincident
flank` holds a point to it (owning the place `[u, v, t]`, four rows, net one), and at a point held
there `flank tangent(at: p) l` (a line or an axis in its tangent plane), `flank perpendicular(at:
p) l` (along its normal, the common normal of a mesh), `flank tangent(at: p) P` (a plane touching
it) and `flank curvature(at: p) k` (the osculating circle of its section by `k`'s plane, by
Meusnier from the shape operator, which the moving source's normal gives with no second solve)
(§6.15.3). The source's geometry, its number where that is an unknown (`half: hint(20deg)`), and
the motion's axes are columns, so a cutter's lean solves from a contact stated at the mean point
(`spiral_bevel/mean_point.sv`, #283). `at:` names a point after `perpendicular` and `curvature` as it
does after `tangent`. A hint over an unknown a call leaves unbound with a seed reads it at that
seed, no longer at zero (§6.4): `std.ConeThrough(ax, q, lean: hint(20deg))` places its apex where a
20° lean puts it.

**[0.90] `length` measures one thing; `distance` relates two.** A line's length is `length(d)
l`, the word an arc's, a spline's and a free curve's already were (§9.1), so a word over `g: path`
states `length(L) g` whatever kind `g` is given. `distance(d) l` on a line, the spelling through
0.89, is **E040**, naming `length(d) l`; the infix `a distance(d) b` is unchanged, and so is the
prefix in a chain: `length(d) (l1 := line) -> …`, `cycle N { length(d) line -> angle(a) }`.

**[0.89] A plane's normal is a member.** `P.normal` is the axis along `P.u × P.v` through
`P.origin`, read wherever an axis is: `l parallel P.normal`, `t perpendicular std.front.normal`,
`Q := plane(u: P.normal, v: P.u)`, `hint(dir: P.normal)` (§6.7). It adds no freedom and is never
declared; the plane holds it square to its axes, on the side `u × v` is, and through its origin.
Reading it before the plane's declaration is **E040**. Through 0.88 an axis along a plane's normal
was declared and held `perpendicular` to both of the plane's axes by hand.

**[0.88] A sweep may roll through what it cuts.** `removal := solid(tool, under: m, through:
body)` sweeps `tool` under the motion `m` over the whole roll about 0 during which it meets
`body`'s material — its stock and additions, never its cuts, as a through prism spans them — so a
generating sweep states no roll limit: within the window that clears the blank at both ends, the
cut does not depend on one (§6.9). The roll is found after the solve, conservatively (it may run a
little past where the tool first clears, never short of it), and is one interval for every reader.

**[0.87] Two sets touching at a point have one spelling.** `std.TangentCones(k1, k2, m)` is
retired: its whole body was `k1 tangent(at: m) k2` (§6.21), a second name for a relation the
language already says, and as a component it read as making something when it only states a
relation. Write the word; the hypoid's pitch cones are `gc tangent(at: M) pc`. A call to it is
**E103**, as any name the library does not have.

**[0.86] One spelling for a side grown.** `p inside(d) S` and `p outside(d) S` are retired: a
point at most `d` outside `S`'s surface is `p distance(<= d, side: outside) S`, and at most `d`
inside it `p distance(<= d, side: inside) S` — the bound on the signed distance each always was
(§6.21) — and the set `S` grown by `d`, where a set is passed or used, is `std.Offset(S, d: d)`.
`inside` and `outside` take nothing in their parentheses; a number there does not parse, and an
implementation SHOULD say the two spellings kept. Plain `inside` and `outside` stay the sides.

**[0.85] The body rule's intersection is `common`, and a half-space is `std.HalfSpace`.** `tip
common body` keeps what a body has in common with `tip` (§6.9), the word the Boolean is named by;
through 0.84 it was `bound`, which is also what a dimension that chooses a root is (§9.6), so a
region solid used to keep a body was a bound twice over. `X bound B` is a syntax error naming
`common`, and `bound` stays a word no name or relation word may be. `std.Level(o, along, d)`, the
half-space behind the plane `d` along a line from a point, is `std.HalfSpace(o, along, d)`: its
points are not level with `o`, which is what `a level(t) b` says (§9.2). Entries below keep the
spellings of their day.

**[0.84] A group's shape may be named.** `interface Gear { O: point, M: point, generator: line,
pitch: Length }`, written at the top of a file as a component is, names what a group must hold:
each member written as a formal is, `name: type`, separated by commas or line breaks, its type a
number type, an entity kind, `path`, `set`, `group` or another interface. A formal typed by one
(`gear: Gear`) is a `group` formal (§8.1) the call must give every member to, each of its type —
a number of the member's dimension, an entity of its kind, a set, a group of the nested
interface's shape — whatever else the group or instance given carries; a member missing or of
another type is **E103** at the argument, one private to the instance given **E101**. The formal
is a group of its own holding only what the interface declares: a member it does not is no name
in the body, **E101** where it is read, as is handing it on, and the formal handed on whole holds
no more, to a typed formal or a plain `group` one. So what a component relies on its argument for
is written where the component is, not found by reading its body. Structural, as everything in
the language is: an instance or a group is a `Gear` by having the members, never by saying so. A module exports its interfaces as it
does its components (`spiral.Gear`, or `use spiral (Gear)`). An interface defined twice in a
file, or over a component of the file, is **E071**; a member twice **E001**; a `Side` member
**E103** (a group holds no side); a type naming no interface **E101**; one holding itself
through its members **E003**. The spiral bevel's steps take `gear: Gear` and `trace: Trace`.

**[0.83] A relation word's parameter may be an entity.** A parameter read where an entity
stands in the word's body — an operand, an entity in a relation's parentheses, a declaration's
child — is given a reference where the word is written (`n square(up: std.z) g`) and is that
entity in the body, an alias as an operand is; given a direction word (`level(w: up)`), it is that
word, unless a name where the word is written is spelt so (a formal `up` given on is the formal).
One that names nothing is **E101** at the argument (§9.9). The standard library says with one such
word what a plane square to a line takes three statements to (`P square(up: d) l`), since a
plane's own axes are not held square to one another (§6.7).

**[0.82] A point at a fraction of a line's run or an arc's turn.** `coincident` between a point and
a line or an arc takes the contact's place `t`, as a spline's and a curve's contacts do (§9.2):
`m coincident(t == 0.25) g` is `m = p1 + t·(p2 − p1)`, and on an arc `t` is the fraction of its
turn from `start` to `end`, counter-clockwise, by angle. Pinned, it states the place; seeded
(`hint(t: 0.3)`), it is the contact's own unknown, starting where the point stands; with no `t`
written the statement is the incidence it always was and owns nothing. A line's fraction runs on
past either end, as a point `coincident` a line is on its whole extension (`t == 1.5` is half a
run past `p2`, an external division); an arc is only its turn, so a free fraction is held to
`[0, 1]` and one pinned outside it is **E040**; a circle has no start, so a fraction of one is
**E040**. Pinned to an unknown (`t == s`), one fraction is shared by contacts on any lines and arcs
— one place along every run, which is similar figures — and never with a curve's own parameter.
Across views, and for a point in space, it is the same relation between places in space; in a
traced body (§6.5) the pin may read the swept formal, `t == u * u`. A loft's station stands on its
path so (`docs/lofts`), handed no length and no sweep.

**[0.81] A direction seed may name geometry, and a twin is judged where its planes stand.** A
plane's axis slot is seeded along a direction read off the drawing, as a point is seeded at a
place (§6.4): `plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))` starts square
to the line `g`, its `v` along the axis `up`, whatever way `g` runs. A point drawn in two planes
(§6.7) is judged against where they stand once the statements that place them have placed them,
or else where the solve leaves them: a plane only seeded parallel to the other is no
contradiction yet.

**[0.80] A prism's sides drafted from its face's plane.** `solid(f, from: a, to: b, draft: θ)`
(and `depth: d, draft: θ`) tapers each side by `θ` from the plane `f` is drawn in, the parting
plane: the section at ordinate `h` is `f` moved `|h| tan θ` into its material — an outer loop
in, a hole out — so the sides lean away from the plane either side of it, as a forging's or a
casting's come out of the die (§6.9). A straight side stays a plane and a circle's a cone; every
face keeps the name it has undrafted, a side crossing the parting plane two pieces of that one
name. A draft of a right angle or more is **E080**, and one taking a side to nothing within the
prism's ordinates is refused where the solid is evaluated.

**[0.79] Any path, and the tangent at a point of it.** A component takes a formal of type `path`
— a line, an arc, a circle, a spline or a curve — given by position as an entity is, its words
settled by what the call gives it; anything else is **E103** at the argument (§4.1). `g
tangent(at: m) l`, `m` a point a relation already holds on `g`, says `l` runs along `g` there: one
equation. Of a spline or a curve it reads the place of the contact holding `m` on `g` — that
contact's own unknown, never a second — so beside `m coincident(t == s) g` it states the direction
where `g tangent(t == s) l` restated the point; with no such contact it is **E040**. Of a circle
or an arc the radius to `m` is square to `l`, of a line it is `parallel` (§9.2). A line and a
spline or a curve touch whichever is written first. One station word stands over any guide so.

**[0.78] Words choose every root; a document need seed no point.** A bound chooses the root of
the block of the equations it decides, not only of the whole drawing: where a block of the
solve's block-triangular order comes to no root, or to one a bound it decides refuses, the
solver looks for the one the words say — across the bound, the block's mirror across it, and
starts about what the blocks before it placed — and a document whose roots are all said so solves
with no seed at all (§9.6, §15). A skew distance between two lines in space (§9.2, the common
perpendicular) is a magnitude: both sides of the first line are roots, and its side is read off
the seed no more; a bound on a point of either line chooses it, or a pin of the side. The spiral
bevel is one file so, `spiral_bevel/hypoid.sv`, with no `hint` and no `project`.

**[0.77] A side's surface is a set a point is put on, and nothing is a distance.** `S.surface`
(the set with each bound made its equality, §6.21) stands wherever a set is used by a drawn point,
not only in a measurement: `kink coincident tooth.inner.surface` is on the surface of a side
grown across a cone, the offset cone. A distance from a set that a number comes to nothing
(`std.Offset(S, d: 0mm)`, an allowance of none) is the set's own side as a bound and `coincident`
as a point's equality; a zero written so is still **E040**. A bound on measurements may read one
unknown of the drawing in its number, as a region's other terms do (`std.Rounded(a, b, r: 0.3 *
mn)` with the module the solve constructs), and the faces its term makes are named by the set the
region's body used. `std.ConeThrough` holds its apex by the angle between its line and the
generator through its point, its spine from the apex to the point's foot on the line: as long as
the cone is, at any size. A seed's step from a point with no turn (`hint(at: c, along: l, by:
f)`) is the step in space, so a point in space steps along a line square to the plane its `at:`
point is drawn in (§6.4). The spiral bevel's crown tooth and gear space are regions so.

**[0.76] A family's own geometry, and the crown's sets.** What a set family's body makes and
states without reading its point — through what it declares — is the instance's, made once where
the instance is rather than at each use (§6.21): `std.ConeThrough(about, q, lean)`, the cone about
a line through a point, its radius growing `tan(lean)` along the line, owns its apex so. A
distance from a set that is one side of another (`{ q | q outside inner }`) is from that set's
surface, the sides turned round where it is the outside, so `std.Rounded` takes a region's side
of a set as it is written (§6.21). `std.Level(o, along, d)` is the plane `d` along a line from a
point, its inside behind it. A region's tip land is said on the land's two ends — where its
flanks meet its tip, the flanks drawn through them — by a bound between them, which chooses the
root as a drawn section's land does (§9.6).

**[0.75] A component may instantiate itself.** A call back into a component still being
expanded is a recursion, elaborated as any instance is (§8), each level named under the one that
made it (`d.s.s.m`); a `when` over the numbers it is given ends it (§12.9). Only what nothing
ends is refused, **E003**: a call given the numbers its own expansion was given, directly or
through other components, every choice of which comes out the same again, and one nesting past
the implementation's depth. A subdivision is written as itself — Penrose's tiling by Robinson
triangles is two components each calling the other.

**[0.74] A point said on another plane is drawn there.** `gear.M coincident q`, said of a point
its declaration draws in a plane, draws it in `q` too, as `point in p, q` draws it — the statement
that puts a point another component declares into a plane only this one knows. 0.54 to 0.73 held
it on `q` by a row, so a layout drew it in `q` again and tied the two by `project`, which still
reads each operand in its own plane. Said the other way round, `q coincident gear.M` puts the
plane through the point, the row it was (§6.7, §6.21).

**[0.73] A chamfer's flat is its sum of distances.** `std.Chamfered(a, b, la, lb)` is the points
whose distances from `a`'s and `b`'s surfaces, each over the other's leg, sum to one or more, and
nothing else: where both surfaces are flat in section that is the straight flat, and where one is
curved it bows with it — between a cone and a sphere about one axis a parabola's arc turned, never
the straight line between the legs' ends (§6.9). Its bound is written multiplied through by both
legs, so a leg of nothing cuts nothing; a bound on measurements weighing every measurement by
nothing holds everywhere or is **E040** (§6.21). The hypoid's end relief is two such bounds in each
member's blank, and `EndChamfer` is retired.

**[0.72] A block made or not by a condition.** `when COND { … }` makes its body once where
`COND` holds and not at all where it does not, decided at elaboration over the numbers in scope
(§12.9). It repeats nothing, so it has no index and no copy: what it declares is named as if
written beside it (`p_in`, `t.tip`). A condition compares numbers of one dimension — `<`, `<=`,
`>`, `>=`, `==`, `!=` — joined by `and`, `or` and `not` (§3.3); two dimensions are **E103**, and
a condition reading an unknown of the drawing is **E040**, since the block is made before there
is a solve. A switch is no longer a `repeat` over a computed 0 or 1: the hypoid's tip relief is
made `when design.tip_relief > 0mm`, `pinion_relief` no longer `pinion_relief[0]`.

**[0.71] A set formal, and a side is a bound.** A component takes a set as a `set` formal
(`component Rounded(a: set, b: set, r: Length)`), filled by a named set or a set family's
instance; a `group` formal no longer takes a named set, nor a `set` formal a group, each **E103**
at the argument (§4.1, §6.21). `std.Rounded`, `std.Chamfered`, `std.Offset`, `std.TangentCones`
and `std.CircleOnSphere` declare theirs so. A side of a set is a bound on the signed distance
from its surface, so the outside of a set of one side is that side turned round — outside `{ q |
q distance(<= 1mm, side: outside) S }` is beyond `S` grown by 1mm — and a set's surface is its bounds made
equalities, a grown side's the offset surface (§6.21).

**[0.70] A face path is read from its solid, and a member is no name.** A face path names a face
by the route through its solid's operands, segment by segment from the first: `pad.plate.near`
is the near cap `plate` gave `pad`, whatever `plate`'s own absolute name, and a placed copy's
faces stand under the placement's name (`pad.moved.near`); a path never names a face by a suffix
of it (`pad.near` is a cap of `pad`'s own sweep, not every operand's) (§6.21). The members
`S.ends`, `F.surface` and `A.body` are read so only past a set or a solid, and only where the
document declares nothing under that name: an instance's member called `ends`, `surface` or
`body` is itself (§5).

**[0.69] Rounds and chamfers are said in the language.** A region's body may read a solid's faces
(`p inside pad.plate.near`, `distance(p, block.near) <= 1mm`), bound a sum of measurements
(`distance(p, f1) + distance(p, f2) <= 10mm`), measure from a set (`distance(p, crease)`), bound
the material's angle at an edge (`angle(a, b, at: p) <= 180deg`) and stop where an edge ends (`p
inside crease.ends`) (§6.21). `repeat a in X` runs over a solid's faces, `if` keeping some copies
(§12.8), and a relation word's body may make solids (§9.9). A region whose terms share no axis is
read point by point (§6.9). On these the standard library says what a round and a chamfer are —
`A round(r: 2mm) B`, `round(r: 2mm) X`, `A chamfer(d: 1mm) B`, `chamfer(d: 1mm) X`, and in a
region's own body `std.Rounded(a, b, r: …)` and `std.Chamfered(a, b, la: …, lb: …)` — with no
feature of the kernel's own (§6.9).

**[0.68] A face is the face, and `.surface` the surface it lies on.** A face path (`b0.near`,
`body.b0.near`, `pad.boss`) means the face as the solid has it, bounded by its edges, wherever it
is read; `b0.near.surface` is the whole surface it lies on. What is drawn on a face is drawn on
its surface: sketching on a face is `v1.origin coincident b0.near.surface` and `v1 tangent(at:
v1.origin) b0.near.surface`, and a point or a plane the drawing draws on the face itself is
**E040**, pointing at `.surface`. A region's point reads either (§6.21): `p inside b0.near`,
`distance(p, b0.near)` from the face, `distance(p, b0.near.surface)` from its surface. One
meaning, two representations: the solve reads a prism's flat face's surface as rows over the
drawing's unknowns, and a region reads any face off the solid as it stands, after the solve —
so a sketch on a face and a round of it read the same face. This replaces 0.66, where the bare
path was the surface.

**[0.67] A face turns about an axis too.** `about:` takes an `axis` lying in the face's own plane
(`solid(f, about: std.z)` on `std.front`) as well as a line drawn there: a plane's own axes lie
in it, an axis nothing the solve moves is judged where it stands, and one the solve places is
judged where the solid is evaluated (§6.9). Right-handed about the axis's own direction.

**[0.66] A face of a solid is a set.** `b0.near`, `b0.far` and a straight side `b0.ab` (and
through a body, `body.b0.near`) name the surface the face lies on, written over the solid's own
extent and profile. So a point is put on one as on any set, and `v1 tangent(at: v1.origin)
b0.near` is a plane lying on it — sketching on a face (§6.9, §6.21). A tangency's point may be a
path (`at: v1.origin`).

**[0.65] A set's body is read in space wherever its point is drawn.** A set is points in space,
so what its body states is the relation in space even where every operand is drawn in one view
(§6.21): a cone about a line drawn in a view is both its generators there, its angle unsigned,
and a region over one is no longer refused. In one view a body's distance or incidence says what
it says in space; a side or a sense says nothing there and is **E040**. The hypoid's pitch cone is
about the member's axis, with no point held in space to make its angle unsigned.

**[0.64] A point's distance from a set, and offsets.** `p distance(d, side: outside) S` is `p` the
signed distance `d` from the surface of a set `S`, `std.Offset(S, d)` the set `d` outside it, and
`p inside(d) S` / `p outside(d) S` that side of `S` grown by `d` (§6.21; **[0.86]** written
`p distance(<= d, side: outside) S` / `side: inside`). A sphere's or a
cylinder's offset is its number moved; a region solid's cone offset is exact, a cone with its apex
rounded by a ball where it grows (§6.9); a drawn point stands at its foot on the set. A bound may
read an unknown the drawing's rows read (§9.6). The hypoid's blank is its pitch cone grown by
the addendum and the outside grown by the back depth, and the end chamfers put their corners on
the tip as a set.

**[0.63] A region's numbers may be the drawing's.** A region solid's bound may read an unknown of
the drawing — a sphere's radius left for a point stated on it to set, a cone's half its
generator's — and the solid reads it once the drawing is solved (§6.9). The hypoid's blank is
stated so, sized by the layout it stands in.

**[0.62] A region is a solid.** `blank := solid(R)`, `R` a set, is the solid of the points inside
it: a ball, a shell, a cone met with a ball, the hypoid's blank — `{ p | p inside tip; p inside
heel; p outside toe; p outside back }` — built exactly as one revolution of its meridian (§6.9).

**[0.61] Regions.** A set whose body bounds a number is a region, and `p inside S` / `p outside S`
put a point in or out of any set, its one number read as at most or at least what it says: inside a
sphere, a cylinder, a cone, a disc. Regions meet by conjunction, and `outside` says a difference
(§6.21, §9.6).

**[0.60] A bound chooses the root.** A dimension's number may be a bound, `distance(>= d)`,
`distance(<= d)` or `distance(in: (a, b))`, and `p inside P` / `p outside P` say which side of a
plane a point is on. A bound adds no row: it says which of the equations' solutions is meant,
checked on the solution, and a solve is steered to the one where it holds (§9.6).

**[0.59] A free curve touches a line where it chooses.** `rope touches rod` holds the rope to a
line somewhere along it, with nothing along the line holding it: the rope bends there in a corner,
and where the corner stands on the line is the rope's problem's, one more unknown with one more
condition — no force along the line (§9.10). The line's points are the rope's to read as its ends
are, so the line need not be held: drag it and the rope follows. A ring sliding on a rod; a rope of
an integrand of the point alone leaves the line at the angle it met it. Where it already crosses
the line, it presses nothing.

**[0.58] A point drawn in two planes.** `O := point in P, G` draws `O` in both planes, so on the
line where they meet: what a layout draws in two views and ties by `project` is said once, and
each plane's own geometry reads `O` there (§6.7).

**[0.57] A free curve's length is the drawing's first.** With no `length` stated, a free curve's
length is solved for wherever the rest of the drawing determines it — a point placed below the
span that the rope passes, a held deck it touches — and is where the energy is stationary in it
only when nothing else holds it (§9.10). So a span and a sag give the rope the length it needs.
A held line or circle touching a free curve whose length nothing states is what sets that length,
no longer refused; and a length that nothing holds and no stationarity settles is left a freedom
and said (**W114**).

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
assertions. All presentation belongs in [Solvent Drawing (`.svd`)](solvent-drawing.md).
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
