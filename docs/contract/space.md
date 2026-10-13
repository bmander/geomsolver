# Space: planes, axes, points in space and membership

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `planes.rs`,
`views.rs`, `reading.rs`, axes or planes. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

**Points in space, planes from axes (0.42–0.43, #81, #84, [plan](../planes-plan.md)):** a point no `in`
reaches stands in space (`PointE.z`, given by `Sketch::give_place` once memberships are in; its
own lift, no row); a 2D drawing is `use std` + `in std.front { … }`. There is no page: a point
with neither plane nor `z` is only a 2D sketch's (JSON, hand-built tests), with no lift. An **axis**
(`t := axis hint(dir: (x, y, z))`, `AxisE` a[3], d[3]; intrinsics `axis_unit`, `axis_foot` once a
relation reads its place) is a directed line with no start; a drawn line reads as one. A plane
over a line holds a hidden axis that is the line (`entities::axes_along`: intrinsic `Parallel3`
and `PointOnAxis` at its `p1`, or at the end it shares with the plane's other line — each
line's own image of it, a twin's included (`Sketch::twinned`) — where the
plane's two `PlaneAxis` rows give way to one intrinsic `Coincident3` of origin and end — four
rows over three unknowns would be a dependency the structural count cannot see, #88). The lift
writes both rows (`r parallel l`, `p coincident r`).
A **plane** is `P := plane(u: r1, v: r2)` (`PlaneE { u, v, o, origin }`): right u, out u × v, up
out × u; it owns `o` (three unknowns); members `P.u`, `P.v`, `P.origin` (a point drawn in P held at
(0, 0)), and `P.normal` (below). **Its axes pass through its origin** (0.43, #84): `push_plane` mints two intrinsic
`PlaneAxis` rows (`point_on_axis` over `o`), placing both axes, standing one it is first to place
through `o`. A slot left out mints a free axis (`P.u`/`P.v`, `build_plane`): `p := plane` is 7
DOF. `views::origins_on_axes` (right after the `fix` pass) holds `o` where two wholly held axes
meet, so std's planes add no row; held axes that miss or run alike, or a plane held off its held
axes, are E067. A plane standing off another takes its own axes (held, or `parallel`): with
shared axes it would sit on them. `P.origin coincident p`, `P coincident p`, `P distance(d) Q`
(one row along P's normal); `P parallel Q` (`PlaneParallel`, normals alike either way, two
rows; no plane–plane angle yet); `fix(origin == (x, y, z)) P`. A plane's child slot takes a seed,
`plane(u: hint(dir: (x, y, z)))` (`KidSeed` carries x, y, z; a point's refuses z), and
`commit_seeds` writes a minted axis's direction back there. **Or read off geometry** (0.81, #252):
`hint(dir: up)` an axis, a line `p1`→`p2` in space, `P.n`; `turn: θ, about: g` turns it
(`KidSeed::dir`, `syntax::DirRef`; `Deferred::Dir`, `entities::direction_of`, read again after
`views::place` where it reads what that places, `crosses_views`); never written back. The loft
station `OnLine` seeds `n` so (`docs/lofts/l0/stations/lofts.sv`); `tests/seeds.rs` is the gate.
An origin already on one axis is placed in fewer rows than three (`P.origin coincident
std.front`), or the structural count sees a redundancy. W113 warns of two planes lying on one
another. `views::place`
settles axes and plane origins before the solve, round by round with seed settlement; they stay
unknowns. `p distance(d, along: u) P` is an ordinate from `P.origin` along `P.u` (`CKind::Ordinate`,
in space across planes with no callout); `hint(at: P, (x, y))` seeds in P's coordinates. Circles, arcs
and splines over a point in space are E060, faces E080. `use std` gives axes
`std.x/y/z/back`, planes `std.front` (x, z), `std.top` (x, y), `std.side` (y, z), `std.up` (z,
back), and `std.origin` (in front, fixed) — flattened after the document, present whenever the
document says `use std` (the workspace offers them as places to draw). `std.Turned(o, t)` is a
frame turned within a plane (`.axes` the plane, `.u` the line o→t). No coordinate-placement
helper: model contours with alignments, incidences, symmetry and dimensions — `std`'s words
`offset`, `right_of`/`left_of`/`above`/`below`, `coords`, `on_u`/`on_v` are pairs of those
dimensions (an ordinate and an ordinate or a level) said as one statement, still dimensions;
`skew` and `turned` are the same for two lines (#103's corpus pass), `P square(up: d) l` the three
statements a plane square to a line takes (#250). Components have no
implicit frame; `instance := Part(f) in view` supplies membership. Aliases keep subentity paths
(`f.origin`). Calls may omit `name :=`; anonymous keys stay out of user-facing names. E.g. `cyl
:= Cylinder(std.Turned(std.origin, up), fw: components.dims.fwA, dims:
components.dims.vtwin_dims)` (`vtwin/components/cylinder.sv`'s preview). Retired: the
datum rotor, `toward:`, `from:`, `fold:`, `offset:`, `through:`, `attitude: free`, quaternions,
hinges, `against`, the role rule, page placement, `std.ThreeViews`.

**A plane's normal, `P.normal`** (0.89): an axis along `u × v` through the origin, no field of
the declaration (`fields()` and the JSON keys stay three) but `PlaneE::normal`, minted only where
a reference reads it — the flattener records each (`Expansion::normals`, in `rewrite`'s `fix`)
and `build_plane` mints it with the plane (`Sketch::plane_normal`, bound as `P.normal`), so a later
declaration may name it and an earlier one is E040 (`follow_building`). Its rows are intrinsic:
`Perpendicular3` to each axis and `PlaneAxis` through `o` — three rows over its direction with the
unit row, two over its place, no freedom; `axes_along` keeps its `PlaneAxis` where it swaps the
plane's for a `Coincident3`. The rows leave its sense, so `Sketch::seed_normals` turns it along
`u × v` after `axes_along`, in `views::place` (which reads it off the axes, never solves it:
placed with them, its rows not taken) and after `settle_turns`; the solve keeps the sense by
continuity. `follow` resolves it; `hint(dir: P.normal)` reads the basis as `P.n` does. **Drawn
nowhere**: `Sketch::drawn` leaves it out (so the workspace's scene, picks and boxes) and
`nearest_axis` skips it. JSON writes a plane's `"normal"` (an axis index) only where one was read,
and the reader and `graft` give it its rows again (`stand_normal`); `lift` names it `vK.normal`
and declares no axis. `tests/plane_normal.rs` is the gate.

**A point drawn in two planes** (0.58, #145): `O := point in P, G` (`Membership::also`). The
flattener declares a hidden **twin** per further plane beside it (`Walk::twins_of`, key `O@1`,
`hint(at: O)`, `Expansion::twins`); the elaborator records them (`Sketch::twins`) and, after the
relations, ties each (`planes::tie_twins`, E040 a plane twice; `planes::parallel_twins` E061 after
`views::place`, of the planes it placed — `place` says which — and of a plane only seeded where the
solve leaves it, `views::degenerate`'s twin row, #252): `Sketch::tie_twin`'s two intrinsic rows, the twin on the point's plane and
`Project`, then `Sketch::hold_twins`' point on the twin's plane — left out where a line the plane is
built along (`axes_along`'s `Parallel3`) ends at the point or has it stated on it, a dependency the
structural count cannot see (#88). The crown's normal section N is square so. A point is read where its reader
is drawn: `planes::memberships` puts a twin in a drawn element's slot (`Sketch::twin_in`,
`replace_point`), `reading::read_twins` in a relation's before `in_space`. A twin goes by its
point's name (`SourceMap::twin_of`, read by `name_of`/`writable_name`; never filed in `names`, so
`--where` lists the point once), and a sheet's view selects a point with its twins
(`drawing/render.rs`). JSON `"twins"`, `graft`,
`lift` (prints `in P, G`, never a twin) and `edit::remove` (keeps the other planes) carry it.

**Said as a relation** (0.74, #211, #145 F11): `gear.M coincident q`, point first, of a point
its own `in` or an `in` block draws in a plane, is the same twin. `Walk::drawn_incidences`, at the
head of `resolve` so the memberships read first know it, makes the statement the twin's
declaration (`Walk::twin`, shared with `twins_of`), std's datums read from `StandardDatums`
(`Walk::datums`). The row stays for `q coincident gear.M` (the plane through the point: what
`edit::add_datum` writes, the plane perhaps lying on the point's own, where a twin is E061), a
point only its instance's `in` puts in a view (in space to its component:
`two_planes.rs::a_point_its_instance_puts_in_a_view_keeps_the_row`), a claim, a derivative, a set's application, a ring's statement, the point's own
plane and a line's minted end (`l.p1`); a point in space is `planes::incidences`'. `project`
never reads a twin (`relations::constrain`): it relates two planes, each operand in its own, so
`m coincident P; O project m` still elaborates.
`tests/two_planes.rs` is the gate.

**Membership is one rule (§6.7, §6.21, 0.54, #105):** a plane is a set and `in` membership of
it. `q coincident P` (or `l coincident P`, its ends) of a point standing in space is lowered to
`PointE.plane` (`planes::incidences`, after `memberships`, in statement order; the statement
then states nothing, `drawn` in `program.rs`), the rows `in P` makes. Kept in space, a row:
a point with a `Deferred::Height` seed, held by a `fix`, a dual's point (`along.point`), a
ring's, or claimed. **A point on a curve drawn in a plane is in it** (#249): `m coincident g` (pins
allowed), `g` a spline or a traced curve (`planes::drawn_plane`, not an extrusion) drawn in `P`, draws a
point in space in `P`, and `g tangent l` (either way round, `at:` too, #251) a line's free ends —
read off the kind `relations::settle` gives, so a word the operands do not take draws nothing; the
statement stays the 2D contact. A
kept point is E062 naming what keeps it (`kept`'s cause), its statement returned apart from the
lowered ones and skipped; a circle or an arc keeps `PointOnCircle3`. `tests/curve_membership.rs`. **A set whose body is a circle's is the circle**, the one entry so far of
**`lowering.rs`**, the table of what a set may be drawn as (#140): `lowering::shape` reads `{ p
| p coincident X; p distance(r) Y }` (the flattener's and `edit::set_dimension`'s one reader),
and `lower_circle` (in `set_made`, outside applications) emits `circle(center: Y)` under the
set's name (its statement's) and `radius(r)` on it (`Radius::at`: the body's number, or the
family call's argument); a use the element answers (`Form::answers`: plain `coincident`) stays
as written against it (`set_use`), `tangent` uses read the body. The elaborator judges each (`Expansion::lowered`,
`lowering::refused`: `X` a plane, `Y` drawn in it, after `incidences`) and one that is none is
walked again as a set (`elaborate` loops over `elaborate_in`, `flatten::expand_with`'s
`unlowered`). The judgment cannot precede the expansion — `Y` may be drawn by another set's use
(`o coincident on`) — so a pass ends at it, before any constraint; a refusal is final.
Lowering is a representation, never a meaning: the solutions are the same either way.
`tests/membership.rs` is the gate.

**Axes** (`docs/planes-plan.md`, #81): `t := axis hint(dir: (x, y, z))` is a directed line in space,
`AxisE { d, a, placed }`: a unit direction (`axis_unit`, intrinsic, held like `quat_unit` when `fix`
holds all three) and the point nearest the origin, fixed — no freedom — until a relation reads it
(`CKind::place_slots`: `PointOnAxis`, `AxisCoincident`, `PlaneAxis`), when `Sketch::place_axis`
frees it and mints `axis_foot` (`a·d = 0`); `remove` holds it again once nothing reads it. `fix(origin
== (x, y, z)) t` holds a place (placed first, so nothing later frees it); std's axes are held
so. `t coincident s` between axes is `AxisCoincident` (4 rows, either sense); `l coincident t`, a
line on an axis, is `LineOnAxis` (both ends, 4 rows, places the axis). JSON writes a placed
axis's held place as `a_fixed`; a load or `graft` puts places back after the planes
(`Sketch::place_restored`), elaboration's order. `parallel`/`perpendicular`/`angle` over an axis
are `Parallel3`/`Perpendicular3`/`Angle3` (slots `SpecKind::Direction`, a line or an axis): an
axis is handed to the line kernels as the segment
`(0, d)` from `Sketch::origin_param`, a fixed 0 (`Constraint::axis_columns`). An unsigned angle
of 0°/180° is E040 by value (`program/relations.rs`). `tests/axis.rs` is the gate.

- A **`plane`** (Solvent §6.7) is also a **view**: `Sketch::basis(P)` is `plane::Basis` `(u, v)`
  over its axes' directions, `n = u × v` toward the viewer, standing at `o`.  A point drawn in
  it is lifted into space by the intrinsic `lift` row (`kernels::lift`: X, p, o, d_u, d_v;
  `Sketch::lift_point`, minted on request, never serialized).  `frame` is refused by the parser.
  A point's **membership** is `PointE.plane`, set by `a := point in top` (every point the
  declaration mints or names; filled by `program::memberships` after every kind is built, before
  any constraint); it moves nothing — only `Project` reads it.  `a project b` is one row over 12
  columns (`kernels::project`) with the line the planes share as consts (`plane::fold_line`).  The two
  plane slots are real entity slots and **inferred**: `infers_arg` marks them, the registry
  publishes null, and **`io::seed_omitted` is the one seam** that fills them
  (`constraints::infer_entity`) and refuses (`constraints::validate`: no plane, one plane,
  parallel planes) — so the elaborator (E061), `from_json`, `gcs_constraint_add` and
  `Constraint::project` share one rule.  `operator_text` skips an inferred slot (`a project b`).
  A plane's minted label starts with `v` (`syntax::kind_initial`).  Its glyph (`plane::glyph`)
  is drawn under the *implicit class* `.plane` (`EntKind::implicit_class`, resolved in `style_of`; JSON
  never writes it); in the workspace it is a pane, chosen in the plane chooser rather than picked.
  Deleting a plane (`edit::remove`) splices the `in` clause out of every
  surviving declaration, and dooms every statement whose *elaborated* constraint named it.
  `commit_seeds` replaces the bracket list at `Decl::list_span`.
  **`in top { … }` is the clause written once**: the parser **hoists** the body into the
  enclosing body, stamping each declaration (`Decl::plane_from_block`, `stamp_plane` into
  `repeat`/`cycle` bodies), so writeback, carets, the DOF ledger and `edit::in_root` see root
  statements; only header and brace are the block's (`Program::in_blocks`, spliced by
  `remove`).  Printers spell no clause a statement did not write; a membership edit on a
  block-stamped declaration is refused with the cause.  Top level only.  **An instance joins a
  view whole** (`t := Tooth(…) in top`): the flattener stamps it (`Scope::in_plane`,
  `stamp_scope_plane`, the ref resolved with the prefixes of the scope it was written in, #45.4),
  skipping datum and curve kinds and refusing a plane given twice.  `add_rectangle` takes the
  plane.  Not commutative (`same_args` swaps only the first two entity slots).  `cgraph` leaves it
  unsupported.  `bracket.sv` is the case; `tests/plane.rs` and `tests/plane_lang.rs` the gates.
  **Across views a word means space** (`program/reading.rs`): `in_space` reads each operand's
  points' views by membership (a point in space is its own view) and where they differ maps the 2D kind
  to its twin in space (`Distance3`, `PointLine3`, `LineLine3`, `Angle3`, `PointOnLine3`,
  `EqualLength3`, …) or refuses (E062; E040 for `side:`/`sense:`).  Radii, ordinates (their
  form is the operands') and `project` are view-free, but a page word across views is E062;
  `coincident` to a plane is spatial from `infix_op`.  `tests/cross_view_audit.rs` asserts the corpus's
  cross-membership relations keep their 2D kinds.

- **A stack is planes and distances**: parts standing on one another are drawn in parallel
  planes, `P distance(d) Q` with an axis square to both through both origins (`against`, placed
  planes and E083 are gone, #81).  `hardware.Groove` states the O-ring rule once (10–20% squeeze,
  a groove a third wider), so `dims.sv` derives `grooveb` and `groovew`.
  **A component contributes a `cut` to a body it was handed**, which is the body rule being a
  set and not a sequence: the feature owns the void it cuts.
