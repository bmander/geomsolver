# Solvent: language changelog

What each draft of the [specification](../solvent-spec.md) changed, newest first. A **[0.n]**
mark in the specification points at the entry here that introduced or amended the rule. Entries
keep the spellings of their day.

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
