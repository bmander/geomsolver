# geomsolver

**Model/drawing split (0.20):** `.sv` is geometry, constraints, hints and assertions only;
presentation belongs in `.svd` ([docs/solvent-drawing.md](docs/solvent-drawing.md)). This
supersedes older presentation instructions below: never write `class`, `style`, `view`, `section`,
`dimensions` or callout placements in model source. Model serialization and reconciliation omit
presentation. The drawing compiler and renderer stay in the core; hosts supply texts. Browser
examples are file or directory entries in `app/example-catalog.ts`, loaded via `app/remote.ts`;
V-twin is `rust/examples/vtwin/` (shared modules in `components/`). `.svd` shows the paper preview,
`.sv` the model canvas; `app/program.ts` keeps each file's source across switches; the `file` URL
parameter keeps the selected file. Every parsed file has an anonymous root; component definitions
are not instances. A file-level `preview { … }` adds root statements when opened as a model (also
via `.svd`) and is omitted entirely on `use` (units and params included). Each V-twin part's setup
is its component's `preview` block (the plate's in `vtwin/components/frame.sv`); only the assembly
has a top-level `.sv`. Source edits add geometry inside the preview, reusable definitions outside.
Hosts look for modules beside the model, then its ancestors, then the library.
`web/tools/copy-examples.js` packages sources for static hosts; live files override them.

**Points in space, planes from axes (0.42–0.43, #81, #84, [plan](docs/planes-plan.md)):** a point no `in`
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
(0, 0)). **Its axes pass through its origin** (0.43, #84): `push_plane` mints two intrinsic
`PlaneAxis` rows (`point_on_axis` over `o`), placing both axes, standing one it is first to place
through `o`. A slot left out mints a free axis (`P.u`/`P.v`, `build_plane`): `p := plane` is 7
DOF. `views::origins_on_axes` (right after the `fix` pass) holds `o` where two wholly held axes
meet, so std's planes add no row; held axes that miss or run alike, or a plane held off its held
axes, are E067. A plane standing off another takes its own axes (held, or `parallel`): with
shared axes it would sit on them. `P.origin coincident p`, `P coincident p`, `P distance(d) Q`
(one row along P's normal); `P parallel Q` (`PlaneParallel`, normals alike either way, two
rows; no plane–plane angle yet); `fix(origin == (x, y, z)) P`. A plane's child slot takes a seed,
`plane(u: hint(dir: (x, y, z)))` (`KidSeed` carries x, y, z; a point's refuses z), and
`commit_seeds` writes a minted axis's direction back there.
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
`skew` and `turned` are the same for two lines (#103's corpus pass). Components have no
implicit frame; `instance := Part(f) in view` supplies membership. Aliases keep subentity paths
(`f.origin`). Calls may omit `name :=`; anonymous keys stay out of user-facing names. E.g. `cyl
:= Cylinder(std.Turned(std.origin, up), fw: components.dims.fwA, dims:
components.dims.vtwin_dims)` (`vtwin/components/cylinder.sv`'s preview). Retired: the
datum rotor, `toward:`, `from:`, `fold:`, `offset:`, `through:`, `attitude: free`, quaternions,
hinges, `against`, the role rule, page placement, `std.ThreeViews`.

**A point drawn in two planes** (0.58, #145): `O := point in P, G` (`Membership::also`). The
flattener declares a hidden **twin** per further plane beside it (`Walk::twins_of`, key `O@1`,
`hint(at: O)`, `Expansion::twins`); the elaborator records them (`Sketch::twins`) and, after the
relations, ties each (`planes::tie_twins`, E040 a plane twice; `planes::parallel_twins` E061 after
`views::place`): `Sketch::tie_twin`'s two intrinsic rows, the twin on the point's plane and
`Project`, then `Sketch::hold_twins`' point on the twin's plane — left out where a line the plane is
built along (`axes_along`'s `Parallel3`) ends at the point or has it stated on it, a dependency the
structural count cannot see (#88). The gear's folds are square so. A point is read where its reader
is drawn: `planes::memberships` puts a twin in a drawn element's slot (`Sketch::twin_in`,
`replace_point`), `reading::read_twins` in a relation's before `in_space`. A twin goes by its
point's name (`SourceMap::twin_of`, read by `name_of`/`writable_name`; never filed in `names`, so
`--where` lists the point once), and a sheet's view selects a point with its twins
(`drawing/render.rs`). JSON `"twins"`, `graft`,
`lift` (prints `in P, G`, never a twin) and `edit::remove` (keeps the other planes) carry it.
`tests/two_planes.rs` is the gate.

**Nothing is imported bare unless the `use` names it (Solvent §14.4, [0.30], [0.48], #94):** a
used module's component, relation word, param or group is written by full path —
`engine.parts.Rod(…)`, `hardware.nut14_af`, `components.dims.vtwin_dims` — only through the
file's own `use`, or bare where it lists the name: `use std (horizontal, vertical)` (`Use::names`;
`Module::uses` keeps each module's `Use`s). `Program::imported` is the one lookup;
`resolve_component`/`resolve_word` consult it after the file's own; `Walk::imported_params` puts
values and groups under their bare names at a root (a group as an alias of the module's). Per
file, not transitive, no prelude. `program/words.rs` checks every import as written: undefined
E101, twice / two modules / over the file's own top-level name / over a built-in word E071; a
module names its own bare.
`Program::resolve_component(name, from)` resolves from the calling file (`Scope::module`);
`component_id` is program-wide identity (keys `CurveDef`s). `Walk::used_params` qualifies module
numbers (`module_params` keeps the module's own); groups register under their path.
**Relation words (§9.9, #94):** `a horizontal b := a level(up) b`, `flat(d) l := …`, `a above(d)
b := b distance(d, along: up) a` at a file's top level (`syntax::WordDef`, `Program::words`,
parsed by `parser/definitions.rs`: `word_definition_ahead`, `defined_prefix_ahead` before a call,
`word_args` keeping labelled texts). The flattener applies a use (`flatten/words.rs`,
`Walk::apply_word`, over `flatten/apply.rs`, #103): params by label (E004/E040), a number
parameter bound in the body's scope, a word parameter where a word stands, operands aliased,
every relation's span moved to the use's word (a parameter's dimension to its argument, so
`written` and `edit::set_dimension`'s `word_dimension` read it); nested words resolve from the
definition's file, a cycle is E003, an unknown word E102 naming the import. `Relation::word` (`Worded`) rides to
`Constraint::word` (`WordUse`, entities; `graft` keeps it while its operands survive) and
`io::describe_with` reads it (`p horizontal q`). `flatten::word_faults` (closure E101, a
reserved word or a builtin of the fixity E071 — `constraints::builtin_word` — names twice E001)
is asked of every definition (`program/words.rs`) and blocks its expansion. A body is one relation
or several with their declarations (#103); none is defined in a body. `horizontal`/`vertical` between points live in `rust/lib/std.sv`; the printer
writes `level(up)` (`print::ordinate_text`). `tests/relation_words.rs` is the gate. E071 is a
component defined twice in one file, the document's own included (a value twice is E001). `std.front` needs the document's own `use std`. A drawn callout drops a
module path (`relations::unqualified`).

**Spheres, cones and cylinders are library sets** (`rust/lib/std.sv`, 2026-10-06; sets #101): a
centre or a line and one number (`std.Sphere(c, r)`, `std.Cylinder(about, r)`, `std.Cone(about,
half)`, apex at the line's start; the line is `.about`, since `axis` is an element's word a body
cannot name bare), nothing drawn, no entity kind, kernel or `CKind` of their own.  A point on one
is `p coincident S`, a line touching one `l tangent S` (below); `std.TangentCones(k1, k2, m)` is
`k1 tangent(at: m) k2` (below, #104) and `std.CircleOnSphere(k, s, view)` stays a component.  An unbound number is the instance's unknown
(`pc.half`, an angle in degrees); `edit::unknown_seeds` writes it back into the call's `hint(…)`
(`InstVal::Hint` carries the span).  A lifted program declares it under a name it can write
(`lift::declarable`: `pc_half`, apart from every other unknown's) and every dimension reading
it says so (`flatten::substitute_with`).  `tests/spatial_surfaces.rs` and `tests/spatial_lang.rs` are the
gates.

**Sets (§6.21, 0.49, #101):** `S := { p | BODY }` (`StmtKind::Set`, `syntax::SetLit`; the lexer
reads `{ NAME |` after `:=` as a body, not a group) and `component Name(F) := { p | … }`
(`Component::set`, an empty body: the instance *is* the set).  Making one makes nothing
(`flatten::sets::Site`, `Walk::set_made`).  Uses are expanded once every name is known
(`Walk::expand_sets`, rounds, E003 past `MAX_DEPTH`): `q coincident S` walks the body under the
use's own prefix (`{use scope prefix}#{stmt}.0.`, path + `Instance(stmt)`), the bound name an
alias of `q`; `l tangent S` declares the contact at the bound name (in space, seeded at `l`'s
middle), states it `coincident l`, walks the body, then walks it again with `Scope::twin` set —
relations only, each with `Relation::along` (`syntax::Along { point, toward: AlongBy::Line,
key, made }`, `key` the use's derivative, `made` the prefix its geometry is under; refs resolved
in `rewrite`).  Every relation made carries
`Worded { sets: [(k, name)], span }`, so `describe` reads `l tangent shaft` (`WordUse::sets`).
**Differentiated at elaboration (#104):** `program::relations::constrain` registers one
`model::Dual` per key (`Sketch::add_dual`: the point, `Toward::Line(l)` or `Chart`, `owned` =
`SourceMap::ents_under(made)` but the point) and sets `Constraint::along` to its index, dropping a
row that reads nothing the use moves (`Constraint::differentiable`, entity-level, the one rule
`io::from_json` shares; refuses a kernel with no Taylor form).  The kernel is
`kernels::dual_kernel(inner)` (ids past the curve families, `kernel_id_in`; in `kernel_table`
when any row has one): columns `[x, ẋ, a, b]`, consts `[kid, row consts, mask]` (`Move`: 0 held,
1–3 the line's direction `b − a`, `TANGENT` its own tangent column), residual `J·ẋ` and Jacobian
— the Hessian along `ẋ` by polarisation of the ε² coefficient, then `J` routed by the mask — all
read from `taylor::residual`, exact.  The use's own geometry (its `own_params` and lifts) moves by
**tangent unknowns** `Sketch::mint_tangents` mints in `add` (`Dual::tangent`, `{name}.d`, seeded
0, never saved; `retire_tangents` on `remove`); its intrinsic rows (a lift) are stated as
derivatives too (`twin_intrinsics`, at `add_dual` and at each later intrinsic add) — never the
point's own.  **`S1 tangent(at: m) S2`** (`expand_pair`): each body applied under
`begin_nth` (`#{stmt}.{n}.`), walked `Pass::Made` (declarations only; `Scope::pass` is the walk's `Pass`) then
`Pass::Along` along `Chart(0)` and `Chart(1)` (keys `#{stmt}.t{k}.`, shared by both sets); a
chart's point tangent is `e_a + s·e_c`, two components held (`set_chart`), `c` the world axis
the rows' gradient at the seed runs most along (`Sketch::choose_charts`, after phase 3 and at
`from_json`).  Refused there: a body making geometry (`sets::makes_points`), no `at:`, a claim,
inside a derivative.  `along` and `duals` travel through `graft` (duals first, remapped; one
missing its point or line is dropped with its rows), JSON (`"duals"`, `"along": index`) and
`topology_key`; `cgraph` leaves it to the residual, the witness never jitters it, callouts skip
it, `to_program` drops it (a flat program cannot spell it).  Every static kernel but the drags
has a Taylor form (the spline contacts' exact to every order, `taylor::span_frame`:
the basis about `t₀` a cubic in the jet `τ`, over `Σ wB` by the quotient rule): the spatial `*_rows` are generic over
`kernels::Num` (`Dual<N>` for res/jac, `Jet` for the form, `kernels::num_form`), the
hand-written spatial kernels have generic twins held to them by `tests/taylor.rs`.  Refused: a
set named as an entity (E040 in `rewrite`), `coincident` between two sets, parentheses, a
non-point `coincident`, a non-line or claimed `tangent`.  `tests/sets.rs` is the gate.
**`P tangent(at: m) S`, a plane tangent to a set (#145, #148, 0.56):** `sets::expand_touch`
applies the body at `m` and walks it `Pass::Made` then `Pass::Along` toward `P.u` and `P.v`
(`model::Toward::Axis`: the axis's `d` columns fed where a line's ends go, `dual_kernel`
unchanged), plus `m coincident P`, dropped in `relations::constrain` where `m` is drawn in `P`.
**No shape in Rust:** where the drawing already says what the body says along `P` (a cone's apex
drawn on it, a cylinder's axis parallel to it), the two derivatives are dependent, and the
diagnosis finds it by rank — a derivative row counts as a relation, and a plane touch's own
derivative rows found removable are `Diagnosis::expected`, out of `implied`/`over`, never
painted. A non-plane or no `at:`, E040; a labelled reference fills its slot
(`Written::assemble`). `tests/plane_tangent.rs` is the gate.

**Membership is one rule (§6.7, §6.21, 0.54, #105):** a plane is a set and `in` membership of
it. `q coincident P` (or `l coincident P`, its ends) of a point standing in space is lowered to
`PointE.plane` (`planes::incidences`, after `memberships`, in statement order; the statement
then states nothing, `drawn` in `program.rs`), the rows `in P` makes. Kept in space, a row:
a point with a `Deferred::Height` seed, held by a `fix`, a dual's point (`along.point`), a
ring's, or claimed. **A set whose body is a circle's is the circle**, the one entry so far of
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

**Variational curves (§9.10, §6.1, 0.53–0.57, #121, #144, `variational.rs`, `extremal.rs`):**
`k minimizes E` / `k maximizes E` (`StmtKind::Minimize`, `parser/minimize.rs`; indicative, read by
the word past its ref, as the body words are) with `E` a sum of `c * integral(EXPR over p)` (or
`over (p, t)`, `t` the unit tangent) along `k`, ds-weighted; `flatten::values::settle_integrand`
writes the scope's numbers in and renames the binders `p`/`t`. Each term is a `CKind::Stationary`
(`[curve, weight, integrand, degree, maximize]`), only over a **free curve** `rope := curve(a, b)`
(`program/entities.rs::build_free_curve`: `EntKind::Curve`, `CurveBody::Extremal`, `CurveE::length`
its one own param, `CurveE::pegs`). **The curve is the solution of its Euler–Lagrange equation**,
solved inside it (`extremal.rs`): optimal control in arc length — state `z = (x, y, λx, λy)`, the
direction algebraic (`H_θ = 0` by Newton, `H_θθ > 0` Pontryagin, so a branch flip is refused, not
followed), `λ' = −f_p` — from `Lagrangian`'s three tapes over `(θ, x, y)` (`t.x → cos θ`; each
first in one variable, so second derivatives only). Integrated by Gragg–Bulirsch–Stoer
(`extremal/flow.rs`, seven columns) with the variational equations through the same substeps; the
BVP (`extremal/shoot.rs`) is multiple shooting, four pieces an arc, a **peg** (a held point the
curve passes: `peg coincident rope`, absorbed — `rows_in` 0, its place held) splitting arcs with
the costate free to jump and `H` unbroken (a corner); dense Newton, a long curve started as a
gentle sag (`EASY`) and walked out, a pegged one from the unpegged shape with each peg where it
passes, warm starts walked along `dq` (`walk`). The drawing sees a curve of columns `[a, b, L]`
(`entity_params`): contacts are the ordinary curve kinds (`kernels::EXTREMAL` body,
`extremal::kernel_eval`/`kernel_frame`, gradients by the implicit function theorem; `∇C''` and
`C'''` by differences), the shape memoised by its constants' content (`extremal::shape_for`,
shared with `Sketch::curve_shape` and drawing). `length(L) rope` is `CKind::CurveLength` (the
radius kernel on `L`); with none holding it, the length is the drawing's when the drawing
determines it (`Sketch::lengths_held`, a rank question at the pose, contacts seeded first: hold the
columns the rows reading no free curve determine and the curve's ends, and ask whether a null
vector still moves `L` — a structural count cannot tell a deck's slide from the length's
freedom), and only otherwise does the energy's first statement carry the transversality row
`H = 0` (`variational::rows`/`kernel`, `KernelKey::Stationary`) — when `stationary_length` finds
one (`Energy::stationary`); else the length is `unsettled`, stated no row (one sent the solve after
ever longer ropes) and left a freedom, W114. A held row pressing a curve of held length that is not a peg is E040 (a corner,
never a tangency); with the length free it is what sets it. `Sketch::
settle_variational` (at `add`/`remove`, the end of `graft`, `from_json`, elaboration) compiles each
curve's definition (keyed by its terms and peg count), its pegs and free length into
`Sketch::variational`; `seed_extremals` then seeds lengths (a row's number, else `H = 0` by
bracketing above the chord) and contacts left at 0. The verdict (`extremal::verdict`: Legendre,
conjugate points as sign changes of `det ∂p/∂λ₀`, the Hessian in pegs' places and a free length)
is `Diagnosis::extrema` (`unsolved` where no shape is found), reported by the CLI, the JSON
(`[id, verdict, asked]`) and the app's marks. `length(L) s` on a spline stays `SplineLength`
(`integral.rs`). `std.hangs(L) k` is a word. `tests/{extremal,catenary,minimize,spline_length}.rs`
are the gates.

**Predicates, applied (§9.9, §6.21, 0.50, #103):** a relation word and a set are one predicate —
parameters given at the call, **bound variables** (a word's operands, a set's point) filled at
the use — applied by one routine, `flatten/apply.rs`: `begin` (an `Application`: the closure
under the use's prefix `{scope}#{stmt}.0.`, and where its output starts), `bind_to_use` (each
bound variable an alias of the use's operand; one naming nothing is E101 once at the operand,
`use_aliases`/`failed` in `resolve`), `inherited_twin` (differentiated already where the use is),
`apply` (a dimension that is a parameter put at its argument for `written` and
`edit::word_dimension`, every other at an empty span; the body walked per `walks`; every relation
the application made stamped with the use's id/span/path, claim, placement when it makes one
relation, word spans at the use, and `Worded`, the outermost use's).  `syntax::Relation::of`
builds a relation with nothing but its form.  Words: `WordDef::body` is `Vec<Stmt>` (one relation, or `{ … }` of relations and
declarations), applied synchronously in the walk's Relation arm (`Walk::apply_word`; numbers
bound in the closure's `vals`, selector words substituted; `applying_words` for E003); sets:
`expand_use`.  A set may be written in place on the right of `coincident`/`tangent`
(`P::inline_set_use`, hoisted as a `Chained::Link` statement keyed `#s…`/`#i…`, described by its
text).  The lexer reads a brace after `:=` as a group only before `}` or `name:` (`group_ahead`).
`tests/applied.rs` is the gate.

**`ring N about C { … }`, solved over one copy (0.47, #96):** copies that are turns of the first
(the representative) about a point (in its view) or an axis held in its direction (in space).
The flattener expands it as a `cycle` (`BlockKind::Ring`, `Block::about`, `Scope::ring`) and marks
every statement of copy k ≥ 1 `ir::Statement::turned` (`Site::turned`): its declarations are
built, its relations, holds, claims and branches are not (`program.rs`'s `stating`).
`program/rings.rs` pairs each turned entity with the representative's by key
(`prefix#id.k.rest` ↔ `prefix#id.0.rest`) into `Sketch::turns` (`model/turns.rs`): a point's,
circle's or arc's numbers are then **derived** (`Sketch::derived`, linear in the
representative's and the centre's params), owning no column — `System::col_of` is `-2 - k` for
one, its Jacobian folded into its bases by weights (`ent_w`, 1 elsewhere so every other bit
stands), its value settled in `apply_z`/`full_x` and at `expr::sync_free`; a turned arc's
intrinsic rows are not compiled; a turned curve evaluates as its representative, turned
(`curve_turn`). Readers outside `System` are told: `cgraph` leaves rows on copies to the
numeric residual, `decompose` writes none, the diagnosis reads a copy as free as its bases,
`Part`/`graft`/JSON (`"turns"`)/`topology_key` carry turns. E015 the index read in the body
(parser finds the reads, `Block::index_reads`), E021 a reference the turn would move or a copy
by index (`judge_rings`, curves through their instances' arguments), E022 a ring in a ring, E023
what a ring cannot turn (`rings.rs`). The ring's own turn is gauged outside (`hub horizontal
tip[0]`). `std.Polygon`, `gear.sv`, `gear_trace.sv`, `ngon.sv` and the Wankel rotor are rings;
`tests/ring.rs` and `tests/turns.rs` are the gates.

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

**Closed components:** model dependencies enter through arguments, standard datums included.
Definitions and built-ins stay callable. A component scope holds only its formals and
declarations; repetitions share it. `dims := {width: 20mm, origin: o}` bundles values and
geometry aliases; `dims: group` is a required formal. Groups nest, by name or in place
(`{bar: {at: o}}`, `InstVal::Group`, flattened to dotted members by `members_of`, each nested
group registered so `Bar(design.bar)` binds); an instance may pass as a layout group. **An
instance's numbers are read by its name**: `Walk::instance_numbers` puts every numeric formal
of every instance a body writes into its `vals` as `inst.formal` (the number bound, else the
unknown `bind` names), before the statements, so `p distance(ball.r) ball.center` reads it and a
group argument carries it (`bind`'s group arm copies `{actual}.…`); body values are not. No
solver state; member units survive substitution; missing members are errors.
Curves need fixed scalar/entity formals. The V-twin and inline-four pass
`components.dims.vtwin_dims` / `engine.dims.engine_dims` explicitly; `Frame(layout, dims: …)`
takes a layout naming the front datum, origin and reference axis.

**Analytic surface references:** `flank := surface(crown, edge: rack.outer)` names a
line/arc/circle patch of an unmodified revolution: spatial, parameter-free, not 2D.
`solid::RevolvedSurface::named` snapshots it; re-read after model changes. `u` follows the edge,
`v` the revolution, so tangent orientation is not an outward normal. Browser `core/surface.ts`
(position, `du`, `dv`: nine doubles). `RevolvedSurface::projector` / `SurfaceProjector::named`:
`project` gives an oriented meridian residual, normal, support parameters, a bounded patch point
and incidence error. The residual is no global distance and cannot certify finite-patch
membership; use incidence error (`surfaceProjection`, `gcs_surface_project`). `line_on_sphere`
returns every finite root. `from:` / `to:` Angles restrict the `v` chart without renumbering or
reversing normals; read `RevolvedSurface::domain` / `GeneratedEnvelope::domain` (`surfaceDomain`
/ `envelopeDomain`), never assume [0,1]. Spans and roll bounds share `AngularSpan`; neither is a
hint or unknown.
**Named motions:** `turn := motion(about: axis, ratio: 2, phase: 10deg)`,
`relative := motion(turn, relative_to: observer)`. `motion::evaluate` takes radians, returns exact
pose and derivative via `Sketch::world_point`; relative is observer inverse times source. Spatial,
ABI kind 10. Cycles and depth over 64 refused (checked with cached subgraphs too). Browser
`core/motion.ts` (six doubles). `motion::Family::read` snapshots; re-read after axis edits.
**Measurements read after the solve** (`measure.rs`, `expr::Measure`): `length(l)`, `radius(c)`,
`distance(a, b)`, `angle(l1, l2)` only in a motion's `ratio:`/`phase:`/`advance:`; resolved by
`rescope_measures`, held as text in `MotionE::measured`, evaluated by `MotionE::rotation`/`advance`
on every read (`Family::read`, `solid::reads`), never stored. Elsewhere `expr::eval` refuses (E107).

**Solid placement:** `indexed := solid(source, under: indexing, at: 90deg)` places a solid at a
motion's pose; `at:` is a constant Angle, not a hint or sweep. Repeatable, nestable, Boolean
operands; copy keeps source and motions. `make solventc OCCT=1` enables native STEP via a C ABI
over OCCT; the core supplies construction data, the CLI owns shapes and catches exceptions. A
written STEP is parsed back by `native::step_check`; `--verify-step full`
(`SOLVENT_STEP_VERIFY=full`, slow tier) also reads it through the kernel. Native `--stl` defaults
to OCCT (`--stl-backend mesh` legacy). STEP+STL build once, check both, then replace each. Native
units are mm; STL 0.01 mm deflection, 0.2 rad — settings, not an error bound. Every CAD face must
be meshed; `mesh::stl_shells` checks every component, cavities included, unwelded. Failure keeps
old outputs; renames are atomic per file, not across files. Native-path tests must not hide legacy
mesh defects.

**Planar generation (§6.15.1, #61):** `envelope(tool, under: m, from:, to:, side:)` over a tool of
the sheet (point/line/circle/arc/formula curve) and a planar motion (`motion(about: point)` is
`MotionDef::Turn`, a rotation square to its view in space) is a **curve** (`CurveBody::Envelope`,
`program/generated.rs`, built with motions after the primitives, before constraints):
`F = X_s × X_t = 0` in the tool parameter, `C`..`C''` exact by `taylor::Jet`s, `C'''` and θ
gradients by difference (`generate.rs`); tool and motion geometry are its columns (shared ones
once; the motion's numbers ride past `tape::MAX_VARS` in `OUTER_MAX`).  Analytic tools are exact to
`C'''`, so a profile they cut may cut in turn (`generate::Tool::Envelope`, read as its Taylor
series); the encoding is `Generated::new` and `view` only.  Kernels are generic over
`kernels::{FORMULA,TRACE,ENVELOPE}` and ask a body for the orders they `need`.
`tests/generation.rs` is the gate.
**One engine for a prism's side (#70):** `side := surface(prism, edge: e)` names a prism's side
too; its `envelope` under a motion keeping the prism's view is built with the drawing
(`generated::extruded_envelopes`, after memberships, before relations) as `e`'s planar envelope
marked `CurveE::extrusion`, the face in a view that stands where drawn. `p coincident
flank` from any view is `CKind::PointOnExtrusion` (`FamilyKernel::Extrusion`, consts
`kernels::EXTRUSION_FRAME` then the contact's; `Sketch::extrusion_frame`). The export builds a
prism cutter's sheet the same way: `brep::sweep::extruded` (edges as lines/arcs, convex corners as
points over their fans, `generate::cut_at`, Hermite spans, extruded; `Family::in_frame`), else
traced (`sweep::traced` kept to compare). `tests/extruded_envelope.rs`, `rack_cut.rs`.
**Named envelopes:** `flank := envelope(source, under: generating, from: -35deg, to: 35deg)`: a
zero-normal-velocity locus over a finite increasing roll interval (ABI kind 11).
`GeneratedEnvelope` intersects it with two section equations via the shared DogLeg loop. A nonzero
trial residual is off the envelope; a local root certifies no global regularity or material side.
`TrustRegion::restrict_step` (default no-op) limits steps to the box; all equations and rank are
still checked. Browser `core/envelope.ts` (10 doubles). `intersect_boundaries` checks two
patches' finite incidence and refuses roots on undeclared continuations.

**Trimmed spatial patches:** `flank := patch(source, inside: tip, outside: root)`; repeat the label
per clipping solid. `patch::TrimmedPatch` has explicit axis, trim and residual tolerances; clips
unmodified full revolutions (line/arc/circle profiles, holes). `solid::RevolvedRegion` is analytic
meridian membership apart from the CSG kernel. Mind full-circle seams, arc shared vertices,
vanishing axis edges; never turn a sphere's construction diameter into a wall. The final root must
satisfy trim and envelope. `core/patch.ts`; ABI kind 12. No branch, face loop or closed solid yet.

**Whole-interval geometry bounds:** `interval::Interval`: private finite endpoints,
outward-rounded arithmetic; zero-straddling division, overflow and invalid domains fail closed.
`sin_cos` is Taylor with remainders on [-8,8], not libm. `generating_profile_bounds`
(`generatingProfileBounds`) bounds profile and du; `generating_profile_jet_bounds` also duu. The
circular-crown verifier and its rational checker establish nominal differential regularity, not
source-error transfer, injectivity or interference (`docs/interval-geometry.md`).

**Geometry validation contracts:** `docs/geometry-validation.md`; the spiral-bevel
[roadmap](docs/spiral-bevel-roadmap.md). `interval::minimum::enclose` refines with an enclosure
oracle; exhausted budgets keep bounds, never guess membership. `Family::bounds` / `MotionBounds`
enclose mathematical poses and point boxes (coefficients private), not float evaluation or
source error; past [-8,8] angles use `Interval::sin_cos_periodic`. `solid::PlanarField` /
`RevolvedField`: material is closure({f<0}); Boolean fields are one-Lipschitz, not signed
distances, and zero alone is no boundary (A minus A is the negative control). See
`docs/continuous-volumes.md`; `indexed_full_crown_union_exposes_neighboring_gear_overcut` keeps a
rejected candidate. `SpatialField` memo keys include node identity and the whole box; shared DAGs
must not expand exponentially or conflate transformed coordinates; spatial depth is bounded apart
from planar. `solid::SweptField` owns a one-Lipschitz source, a motion and a roll domain; its
`SweepEvaluator` has a capped pose cache. Evidence observers cannot define geometry or prune; do
not reintroduce a callback-based oracle. `SpatialField::read` takes prisms
(`depth:`/`from:`/`to:`/`through:`), full revolutions, bodies and placements; `SweptField::read`
binds a solid and motion. `solid::ExtrudedField` is the prism leaf (interval Gram–Schmidt frame);
`through:` spans `support_bounds` (unknown refuses). Profiles go through `PlanarField::from_loops`
(`solid/field/profile.rs`, exact, no tessellation): `ordered` joins within the larger of roundoff
and axis tolerance, `turning` checks one turn and convexity; convex loops are half-planes and
sectors, others `Node::Profile` (ray-parity sign only when the box clears every wall, else
`[-d, d]`). Partial revolutions and lofts refuse.
**The body rule's third side (0.22):** `tip bound body` keeps what lies within `tip`: a solid is its
stock, plus `union`, minus what `cut`s it, within what `bound`s it. `SolidDef::Body` carries
`bound`; `Term::Inter`; facets, fields and the CAD recipe (`"bound"`, `BRepAlgoAPI_Common`) all
evaluate it. Union first; `cut` and `bound` commute; a swept solid may only be `cut`. `bound` and
`union` (0.32) are body words, not names; `coincident` between two solids is refused. Spiral bevel:
blank `heel` bound by `tip`; modules `design.sv`, `views.sv`, `pitch/`, `blank/`, `crown/`,
`generation.sv`, `layout.sv` (`HypoidLayout`), `members.sv` (`HypoidPair`) (README walks them).
`gears.sv` is the pair; `pair.sv` adds faces for checks (`verification.sv`, `ReferenceFaces`).
`configuration.sv` states `shaft_angle`, `offset` (E = 0 is bevel); the pinion's cone solves
against the gear's (`pitch/pinion.sv`); members roll at `N_c / N` off the gear's triangle, not
`1 / sin`. `tests/hypoid_layout.rs` is the gate. The mesh export refuses a sheet a placement
carries inside the blank; CLI test readers zero the offset.
**Continuous motion solids:** `removal := solid(tool, under: generating, from: -30deg, to: 30deg)`
is the union over the whole interval, not posed meshes; `at:` and intervals exclusive.
`MaterialField::read` promotes static DAGs when a sweep appears; nested sweeps refused; caches
read the whole motion graph. The native recipe rejects sweeps; `EvaluatedSolid` meshes the field
(`from_surface`, `solid::FieldMesher`). Terminal and tests mesh in place (`FieldMeshing::Now`).
**The page never does**: `field_meshing` is `Deferred`; `Sketch::field_jobs` keys swept objects
by a digest of `solid::reads`; `app/field-preview.ts` compares keys and runs `app/mesh-worker.ts`
(own core), posting surfaces (~120 ms, worst facet first) to `supply_field`; a provisional
surface exports only via the preview choice. Mesh fineness is view state
(`FieldMesher::with_fineness`, 0.25–4).
**A swept object of the generating class also gets its exact surface**: worker instances (up to
four) step `brep::export::Builder` (admitted, blank, sheets, sector, cut, pattern, display mesh at
`DISPLAY_SAG`/`DISPLAY_ANGULAR`), shown in the footer; the result replaces the preview via
`supply_field` with `ExactFaces` (`from_exact_surface`), kept across fineness; a refused build
keeps the field surface.
**Field meshing** ([docs/field-meshing.md](docs/field-meshing.md)): resumable Delaunay refinement
(`delaunay::refine::Progressive`, exact predicates, a `refine::Domain`), sharp edges protected by
weighted points. A first pass on `MaterialField::tight_support`, `solid::crease` traces creases,
the final pass protects them. A crease is where the deciding operand changes (leaf,
`PlanarField::carrier` piece, sweep contact time; `OperandId::whole`); a point needs the field at
zero and both leaves active; `End` says why one ends. Crease reads only `crease::CreaseSource`.
Rays seed beside the centre, never on it. `MaterialField::reading`: value, gradient, `OperandId`,
crease flag. Terms compile once (`material/plan.rs`); support, bounds, enclosure are `Fold`s. Every
point query is `MaterialField::query(p, &mut Query)` (`Want::Sign | Reading`, `Source::Exact |
Cached(Resolution) | Warm {hints,local}`; shorthands `side`, `reading`); `SweptField::query` takes
box, floor table, cache, search, in order. Readings are branch and bound; crossings by safeguarded
Newton, memoised. A `construction` solid is never an object. App example `spiral_bevel`
(`gears.sv`). `solventc --stl-backend refine` skips admission, gated by field agreement
(`SOLVENT_FEATURES=field`).
**Meshing speed (2026-09-25):** sweeps read from adaptive distance fields (`solid/field/adf.rs`,
`SweptField::cached`, `Source::Cached`): an octree per sweep and `Resolution`, exact corners,
trilinear between, split while wider than a facet or its centre reads off by over a tenth of the
facet distance. Readings, never claims; creases and admission read
exactly. Sweep readings are one `RollSearch` (`swept.rs`) under a `Goal`; clones share
`SweepCaches`; keys and locks in `field/memo.rs` (immutable). Root finders are `crate::roots`. A
body's cuts are one union (`document.rs`, `Spread` lower bounds); `MaterialField::symmetries` /
`crease::creases_under` trace each crease once. Measure with instructions retired
(`/usr/bin/time -l`); Spotlight on a fresh target doubles wall-clock noise.
**Generating sweeps and the native export** ([docs/generating-sweeps.md](docs/generating-sweeps.md)):
`solventc --step/--stl` builds swept cuts only for the bevel/hypoid generating class.
`solid::admission::admit_body` checks rows T1–E4 (revolved line/arc tools, rotation with changing
contact, clear at both limits, one contact per point, no fold, no self-crossing), sampled, refusing
with row and witness; it checks a placement once when the blank reads alike at every point
checked. A rack is in the class (#61): a prism tool with its caps clear of the blank
(`ToolSurface::Extruded`, sides only), a translation seen from a rotation (contact affine in the
roll, `NormalVelocity::Affine`), sectioned along its extrusion (`brep::sweep::cutter`, a station a
distance), flat caps joined across sector sides (`brep::pattern`, `Kept::Flat`); a blank reaching
its axis is built whole. `tests/rack_cut.rs`, `examples/generation/rack_cut_gear.sv`. A fold is
the sign of the area factor times the contact condition's rate. Built in
`gcs-cli/src/cad/native/sweep_boundary.rs`: cutters sectioned (`backend/sections.cpp`), contact
times from `SweepContacts::at_point_normal_over` (`PointContactError`), sheets traced and
resampled by the core (`solid::contact_trace`, fed section samples and nothing else; rows stop
outside the blank before a time leaps: `charted`; fall by walk length, or by length in space where
that fit misses or folds: `Rows`), indexed by the declared motion, blank split (fuzzy 1e-5 mm),
cells judged by `MaterialEvaluator::probe`.
`UnifySameDomain` widens shared tolerances: unify a copy. `solid::agreement` probes 0.1 mm each
side (a one-sided disagreement withdrawn only where the centroid reads on the boundary); nothing
is written unless it agrees: outputs go through `cad::output::Staged`; native swept
construction needs `admission::Admission` (only `admit_body` makes one). Contracts in
`solid::contracts`; every refusal is one `solid::export::ExportRefusal` (`Stage`,
`SOLVENT_STAGE_TRACE`), reported at the solid's statement. Refine features:
`solid::blank_features` (`BlankTopology`). The configured pair is a 25 mm hypoid
(`pressure_shift`, `spiral_angle`, `admission.rs::the_admission_grid`); recorded tests pin
`fixtures::gear::bevel` / `hypoid6` (`rust/fixtures`, dev-only);
`tests/native_surfaces/gear_cells.rs` holds the recorded tooth-space volumes;
`tests/generating_harness.rs` (ignored) locates refusals by stage.
The pair carries **backlash** (`crown/section.sv`, `side: left`), **tip relief** (`crown/relief.sv`,
`relieved`) and **end relief** (`blank/ends.sv`, `ends_relieved`); `fixtures::gear::design` pins
all to zero, `fixtures::gear::fabricated` sets them. Files:
`build/exports/hypoid-{pinion,gear}.{step,stl}`; `gcs-cli/tests/pair_check.rs` (ignored) checks
the pair from the STLs.
**Screws, the constant-twist class (#64):** a sweep under `motion(about:, advance:)` is admitted by
rows S1–S4 (`solid::constant_twist`, docs/generating-sweeps.md): the tool's characteristic (each
ring's `A cos + B sin + C` roots under the screw's twist, fixed on the tool) walked through the blank,
a coarse pass then `rows` across the reach; S3 is signed. Its sheet is exact, S(s, t) = M(t)·σ(s), rows
along its section square to the axis (`Characteristic::on_section`: the characteristic has corners
where the profile's curvature jumps, the section does not), fitted by the shared `sheet::settle`
(`brep::sweep::helical`). A body built from swept material (`drill := solid(fluted)`, `shank union
drill`) is built by the body rule over exact operands (`brep::export::compose`; the page's `Builder`
too) and measured by it over operand meters (`accuracy::Meter`). A drawing reads a swept object's
exact B-rep (`brep::export::supply_exact`, `Sketch::supply_exact_solid`). Kernel rules it found: every
vertex cuts the edges it lies inside (`boolean`, after the faces' meetings), a traced fit is laid and
graded by length and never crowds a vertex it must pass, a cone at its apex writes radius 0.
`rust/examples/twist_drill/`, `fixtures::drill`, `tests/twist_drill.rs`.
**The Wankel, the planar class (#65):** a point's planar envelope is its path (`bore :=
envelope(apex, under: rotor_turn, …)`); `generate::chosen` skips a stationary root. A generated
curve over whole turns of its motion is closed (`Sketch::curve_closed`), a face's lone loop, split
at its middle for the recipe. The rotor is a cut: its blank less the housing's wall swept over the
period (`bound` by a sweep keeps the union's, another set). A pocketed prism under a planar motion
is admitted by rows P1–P5 (`solid::planar_class`): the pocket's inner envelope over a whole period
(`envelope::planar::InnerEnvelope`, over a formula or generated source: contact roots per sample,
kept where inside every pose, corners where an image recurs), built as the blank in common with
its prism (`brep::sweep::planar`), metered as envelope × slab (`accuracy::Planar`). `claim over
MOTION` advances the placements under it (`Sweep::motion`). A profile field's walls are boxed
(`Boxes`); `ssi` cuts an extrusion square in closed form and `boolean` gives its iso edges straight
pcurves. `rust/examples/wankel/`, `fixtures::wankel`, `tests/{bore,planar_envelope,wankel}.rs`.
**Export tolerance ([plan](docs/native-hypoid-plan.md)):** `solventc --tolerance [LENGTH]` (default
0.01 mm; bare number in document units); `solid::export::Tolerance` states every bar once. Sheets
pass within half of it of withheld contacts (`contact_trace::Withheld::Sides`), normals within
`Tolerance::turn`, refined where they miss (`marked`, `Grid::refined`, ≤4 times, 480×400). Fit:
`cad/native/sweep_boundary/fit.rs`, `surface_feet_near`. STL meshed until `mesh_sag` is within
half (OCCT's deflection is no bound), float32 rounding counted with it (`mesh::stl_rounding`,
`brep::export::written_within`: far from the origin refused, #60); the mesh contract counts clusters
(`TinyTriangles::clustered`); the field is probed at max(2 tol, 2 µm).
`--measure FILE` checks an export (`solid::accuracy`) and exits 1 over tolerance.
**One sector, sewn round ([plan](docs/native-speed-plan.md)):** a body whose swept cuts are turns
of one placement about one axis (`solid::sector::indexing`), from a blank alike under the turn, is
built as one sector (`cad/native/sweep_boundary/sector.rs`): its side runs midway across the gaps
between neighbours' contacts (`sector::Boundary`, sliced by spheres about the blank's centres or
planes square to the axis — a spiral tooth space turns most of a pitch, so no flat half-plane
clears it), is read back against gaps and field, and with its pitch turn splits the blank (remade
about the axis, `solvent_cad_revolved`). Its material is turned and sewn (`solvent_cad_pattern`, no
face intersected); a ring's pieces are moved into one period and split at the seam so rings close
on an iso line. A failed premise builds whole and says why (`SOLVENT_SECTOR=off` forces it,
`SOLVENT_SECTOR_DEBUG` narrates). Faces and volume are the whole construction's; bytes are not.
**Speed (phases 3–4):** the STL is the sector's mesh turned, seam points shared bit for bit
(`solvent_cad_sector_stl`; `SOLVENT_SECTOR_STL=off`); pattern and STEP are measured as copies of
one (`SOLVENT_SECTOR_CHECK=full`, `SOLVENT_STEP_CHECK` measure whole). Volumes, admission,
sections, splits, cells, probes and agreement run on every core (`gcs_core::par`, serial on wasm,
answers in order; `progress::side_by_side`/`beside`/`under`). A revolved cutter is sectioned once
and turned. OCCT's parallel `BRepCheck_Analyzer` is unreliable: `valid_solid` checks faces in
separate analyzers. **Phase 5:** admission proves placements alike from the solid graph when every
blank operand is a full revolution about the indexing line or a ball on it
(`admission::Equivalence::Revolved`, 1e-12 of the blank; else `Sampled`); such a blank is its
meridian section turned once (`Session::construct_meridian`, `solvent_cad_revolve_region`;
`SOLVENT_BLANK=booleans`); field agreement probes one sector, reading cuts not proved positive
(`agreement::Sector`, `MaterialField::without_cuts`; `SOLVENT_AGREEMENT=whole`); STEP text is
formatted on every core, the kernel's to the byte (`SOLVENT_STEP_TEXT_CHECK`).
**The Rust B-rep kernel** ([plan](docs/rust-kernel-plan.md)); clean-room — OCCT is LGPL, this repo
MIT: its code is never translated, only its behaviour and conventions studied. `gcs_core::brep`:
`geom` (surfaces parameterised as OCCT/STEP do, closed-form inverses, signed distances, `Traced`
intersections), `topo` (`Brep`: oriented uses with pcurves, degenerate poles; `check`, `pinches`),
`build` (prisms, revolutions, lofts of line/arc/circle/spline profiles), `props` (volume by Green's
theorem), `query`, `ssi` (closed forms, else traced), `boolean` (split, arrange, classify,
assemble), `mesh` (constrained Delaunay refined to a measured sag), `step` (AP214 with pcurves and
seam curves), `recipe`. A traced pair is seeded where edges cross faces; between analytic
surfaces `ssi::unseen` then searches one face for curves no edge reaches (cells proved clear by
the signed distance's Lipschitz and second-order bounds, else beside a known curve, else seeded),
refusing what it cannot resolve: an untraced meeting is never read as none (issue #58).
`Brep::bounds` encloses (issue #59; `through:` cutters, tool culls): `Curve::bounds` exact for
lines and conics, the poles reaching a B-spline's range, a trace by its measured sagitta; a face bulges
past its edges only off a `ruled` surface (`Surface::bounds_over`: meridians at each axis's
extreme angle). Never a sample grid: extremes between samples were missed.
**This kernel is every export's default** (phase 5): OCCT answers only `--kernel occt` /
`--stl-backend occt` / `SOLVENT_KERNEL=occt` (the oracle; refused in a build without OCCT), its
STEP checked by `step_check` against `Solid::of`. `gcs-cli tests/brep_oracle.rs` builds every
corpus recipe node both ways; `tests/brep_census.rs` holds a census and `brep_body_debug`;
`SOLVENT_BREP_DEBUG` narrates, `SOLVENT_BREP_TIME` times slow Boolean steps. OCCT's volume of a
STEP it reads (~1e-5) is no gate on our accuracy. Rung 1 is done: every corpus solid with a recipe
agrees with OCCT (volume 1e-9, faces by kind) but for named refusals of self-touching designs
(surfaces meeting under 1°, pinches).
**Phase 1 (OCCT's shape, our files):** `SOLVENT_WRITER=rust` reads OCCT's solid into `brep`
(`backend/dump.cpp` → `Session::brep_json` → `brep::json::read`; uses ordered by vertex and
parameters since OCCT's wire explorer misreads a closed edge) and writes STEP/STL by ours.
`Surface::BSpline` is a `nurbs::Net`, **rational where it carries weights** (`BSpline`/`Net`
`weights: Option<…>`, finite and positive; `None` divides nothing, so polynomial exports keep their
bytes; `nurbs::arc` is a circle exactly; STEP's `RATIONAL_B_SPLINE_*` complex entities;
`tests/rational.rs`, `brep_oracle.rs`'s OCCT round trip); `Pcurve::Curve` written exactly; `Edge::tol` is
measured and `check` honours it, so it no longer proves pcurves meet edges: a reader holds
tolerances to a bar itself. `props::fluxes` tables a polynomial B-spline face's `G` per span (a
rational one's Gauss stretch by stretch) and **closes every
loop in the face's parameters** across the kernel's gaps (unclosed, the volume moved with the
origin). The mesher is a neighbour-array CDT, refined for sag **and** for a facet turning from the
surface's outward normal (signed); slivers get a point off the longest side; points nearer than
1e-9 are welded. `Mesh::turned` counts facets facing against their surface (thicker than bar/100);
the export refuses any.
**Phase 2 (the pattern ours):** `brep::pattern` turns a one-sector body into the whole (the
kernel's union is never made): sides matched by a pitch's turn, revolution faces keep the sector's
surface with pcurves shifted along `u`, pieces of one surface across a side become one face, a ring
keeps its last junction as seam. `Net::segment` cuts sheets exactly. `Built::mesh` meshes the
sector once (`mesh::mesh_with`) and shares seam points by index. Full verification reads our STEP
back by OCCT and measures it by `props` (1e-5; OCCT's volume held only to 1e-4).
**Phases 3–5 (the gear without OCCT):** `brep::sweep` builds an admitted swept body: blank from its
meridian region (`brep::planar`, `recipe::meridian`), cutters sectioned exactly (`brep::section`,
`planar::Chart`), sheets traced and fitted (`sweep::sheet`, `nurbs::interpolate_net`), blank split
(`boolean::split`), cells judged by the field (`sweep::sector`; else `sector::whole`), sector
patterned. `brep::export::{exact,step,stl}` is shared by `solventc` and `app/export-worker.ts`
(`gcs_solid_exact`), so native and wasm files are byte-identical: the core's trigonometry, `exp`,
`ln`, `pow`, `hypot` are its own (`fmath`, `x.dsin()` via `Det`; no platform libm). With OCCT and
`--verify-step full` the CLI reads our STEP back (`cad::read_back`). A face met twice is told apart
by `cutter::OCCURRENCE`; a trace ends off a sheet's patch (`Surface::off_patch`); interpolations
past `nurbs::BANDED` points solve banded; a gross sheet falls back to chord length.
**Rung 3 (the app reads the exact solid):** a static solid's `EvaluatedSolid` is the B-rep's
(`solid::Exact`, `Sketch::exact_solid`, cached against `solid::reads`; `from_brep`), built about its
own origin (`cad::shifted`), faces named by document path (caps leading as the facet term's, so
edges are named alike), meshed no coarser than `BREP_RELATIVE_SAG` or 64 steps a turn, one
polyhedral `Prim`. The facet term answers what the exact path refuses (touching tori, empty
solids); `SOLVENT_SOLIDS=facets` throughout. Silhouettes on curved faces are traced on the surfaces
(`EvaluatedSolid::silhouettes`), never the mesh's seams, which zigzag.
Rung 2 is done: `Curve::BSpline` (`brep::nurbs`), `Surface::Extrusion`/`Revolution`, a face's
stretch of a traced or formula curve (`k from p to q`, `CurveE::trim`, `cad::FIT_MM`), lofts
(`Surface::Blend`, `Curve::Iso` rails, `step::FIT`) paired at equal parameter fractions.
`curve_surface` drops roots off the surface. The meter reads a static body by `accuracy::Exact` and
a field taking splines as chords within `CHORD_SLACK`. Edge sampling and `curve::tessellate` test
quarter points (a midpoint misses a cubic's S-bend).
Phase 0: `SOLVENT_ABI_TRACE=FILE` logs native entry points, Boolean face pairs
(`backend/probe.hpp`, `BOPDS_InterfFF`) and smallest pieces; `rust/gcs-cli/tools/abi_trace.py`.
**Removed tracks (2026-09-25/26):** the certified swept boundary (`solid/swept_boundary`), the
Manifold arrangement (`--stl-backend manifold`, `solid::sweep_candidates`), CGAL Mesh_3, the Ju et
al. experiment, and the candidate construction (contact covers, meridian charts,
`envelope::edge_contact`, `solid::tool_faces`, `rust/examples/swept_boundary/`). Git history keeps
them; do not revive them without a new plan.
`solid::MaterialField` composes static and swept operands; its evaluator keeps every sweep query's
domain, witness, enclosure and status. Budgets apply per sweep query; exhausted operands are never
omitted; memo includes the input box; indexed clones share a capped pose cache; cached visits
repeat no evidence. An interval containing zero stays unresolved.
`MaterialField::support_bounds` is a conservative box or explicit unknown.
`MaterialEvaluator::boundary` needs that support, complete cells, strict corner signs and two-sided
distance evidence for `FieldBoundary`. `BoundaryOptions::domain` takes a caller's box instead, but
it **is** an arbitrary crop: its signs prove nothing beyond it, and closure is the caller's
judgement. Never substitute a crop, residual magnitude or same-sign corners for coverage.
Ambiguous vertices, missing mesh evidence, work limits and failed topology refuse extraction (see
`docs/field-boundary-extraction.md`). Keep the extractor as a measured baseline; evaluate
`docs/implicit-meshing-methods.md` before extending uniform refinement.
`SolveOpts::acceptance_tol` controls hard-row success and the DogLeg-to-LM retry; `tol` controls
stopping; defaults keep interactive behavior. Analytic callers request accuracy instead of
switching optimizers, and still check geometric errors. `GeneratedEnvelope::evaluate` returns
trials; `at` requires the envelope equation. `EnvelopePatch` owns one source with its material
conditions; do not reintroduce parallel untrimmed and trimmed copies of a generating face.
Snapshot domains are private, returned by value; source edits require new snapshots.

**Shared generating seams:** `join := seam(flank, transition)` takes two envelopes or patches of
envelopes sharing one actual profile vertex, a source revolution and a named motion.
`seam::EnvelopeSeam` snapshots them after solving, checks coincidence and tangent-plane agreement
with explicit `SeamTolerance`, and intersects their angular domains; first-source u is fixed at the
junction. `SeamIntersectionOptions` exposes only [v,roll]; `intersect` solves both envelope
equations and one section through the DogLeg adapter (one normal-velocity tolerance for both
faces), and the final root checks both envelopes and material trims. Not a nearest-point weld or
an arbitrary surface intersection. Local singular intersections are refused. Browser
`core/seam.ts`; ABI kind 13. Contacts carry the first face's normal and generating velocity, not
the seam's curve derivative.

**Envelope/boundary seams:** `tip_edge := seam(flank_region, tip.wall)` names an envelope (or its
patch) cut by a finite analytic surface. `seam::BoundarySeam` reads one `EnvelopePatch` and one
`SurfaceProjector`, keeping the full [u,v,roll] domain; it solves envelope, boundary support and
section equations in that order, then checks finite incidence and trims. A separate type from
`EnvelopeSeam`: three unknowns, no junction tangency tolerances. Browser `boundarySeamDomain` /
`boundarySeamSample` in `core/seam.ts`, kind 13. Reading a seam asserts no regular branch or
oriented face loop.

**Shared spatial vertices:** `corner := vertex(tip_edge,toe_edge)` meets two boundary seams on one
exact named generating face; a generating junction may also meet a boundary seam on either face.
Coincident declarations keep distinct identity. `BoundaryVertex` owns one face and two boundaries
(three unknowns); `JunctionVertex` one junction and one boundary (two), checking finite incidence
on the face the boundary seam names — a coarser seam tolerance cannot transfer finer boundary
membership. `position` is xyz only, no artificial corner normal. Local solves check rank, not
global uniqueness. Ordinary formals/privacy/copy; ABI kind 14, `core/vertex.ts`.

**Finite spatial edges:** `extent := edge(seam,from: a,to: b,along: axis)` names a finite axial
slicing interval between spatial vertices. `edge::SpatialEdge::named` rechecks endpoint witnesses
and maps junction endpoint u into the correct incident-face chart. At a junction a boundary on the
other incident face shares the corner through boundary identity; never infer this from proximity
or require duplicate corners. `sample` solves via seam APIs, checks local rank/material/incidence
inside, and reuses stored endpoints checked by their full vertex constraints. Never re-solve a
known endpoint with a potentially tangent slicing equation. Fraction is linear projection along
the line, not arc length; local samples are no global branch certificate. No planar coordinates or
glyph; ABI kind 15, `core/edge.ts`.

**Checked shell topology:** `topology::ClosedShell` owns explicit edge endpoint identities and
ordered directed uses in connected faces with boundary loops; construction checks closure, two
opposed uses per edge, one circular link per vertex and face connectivity (periodic self-uses and
annular faces supported). Separate from planar `FaceE`; no coordinates or tolerance weld.
`from_triangles` uses index identity. `mesh::stl_topology` verifies the encoded float32 coordinates
(exact equals, signed zero normalized). Neither establishes incidence, non-self-intersection,
outward orientation or deviation error. See `docs/shell-topology.md` before connecting faces.

**Spatial face boundaries:** `working := face(toe,tip,heel,join,on: region)` binds an ordered
finite-edge loop to an exact named surface/envelope/patch. `FaceSupport` distinguishes inherited
planes from spatial supports; `FaceE::plane()` refuses the latter, never treating them as page
profiles. Spatial faces build/copy after edges, `on:` an ordinary dependency.
`spatial_face::SpatialFaceBoundary` validates shared vertex identity and maps junction parameters
to this face's chart; samples keep edge xyz and check this support's incidence. No holes or
repeated vertices/edges yet; solid assembly still needs an interior/embedding certificate. Browser
`spatial-face.ts`. See `docs/analytic-face-boundaries.md`; a closed boundary walk is not a valid
disk.

A geometric constraint solver, and **Solvent**, the language a drawing in it is written as.

**Start here.** Asked to *draw* something — a sketch, constraints, why a figure will not solve —
read [`docs/solvent-primer.md`](docs/solvent-primer.md) first: the language as the implementation
accepts it. `solvent-spec.md` is the normative specification but specifies constructs that do not
parse yet (`hint` as a statement, `path`), so write from the primer.  Asked to work on the *solver*
— kernels, diagnosis, decomposition, the bindings, the app — the rest of this file is the contract,
and `gcs-solver-program.md` is the staged program it is built to.

Currently: **Stage 5 done, and Stage 7a/7b — solids**, in **one** implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System` (`kernels.rs`,
  `system.rs`), our own DogLeg/LM plus dense and sparse linear algebra (`newton.rs`, `linalg.rs`,
  `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`), decomposition into solve plans
  (`cgraph.rs`, `decompose.rs`), witness analysis (`witness.rs`), presentation (`style.rs`),
  drag/solution management (`solve.rs`, `homotopy.rs`), dimension callouts (`callout.rs`), dimension
  expressions (`expr.rs`), parametric curves (`curve.rs`), **solids** (`solid.rs`, `csg.rs`,
  `mesh.rs`, `hidden.rs`), the **Solvent** language (`syntax.rs`, `flatten.rs`, `program.rs`,
  `edit.rs`, `tape.rs`), JSON export (`io.rs`, `json.rs`) and the reference sketches (`examples.rs`,
  over `rust/examples/`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a self-contained
  `wasm32-unknown-unknown` module and a native `cdylib`, where the *panic boundary* is checked
  (`gcs-ffi/tests/panic_boundary.rs`): `guard`'s `catch_unwind` only catches on native (wasm
  aborts), hence `panic = "unwind"` in release.
* **binding**: `web/src/core/` (TypeScript, WebAssembly) — *thin*: proxies over handles, buffers for
  hot-path numbers, JSON for ragged results.  It contains no algorithm and re-derives no number the
  report carries (a motion's `movingParams`).  **A `use` resolves against what the host handed over,
  then the library**: `gcs_module_set` is the browser's "beside the document" — `core/modules.ts`
  asks `gcs_program_uses`, fetches each through the app and hands it over, transitively; linking
  stays the core's (`modules::link`, in the CLI's order).  The dev server serves `rust/examples/` as
  `examples/…` and `app/remote.ts` asks it first (edits show on refresh, no wasm rebuild), else the
  compiled-in copy is read.
* **CLI** (`rust/gcs-cli/`): `solventc` parses, elaborates, solves, diagnoses and reports on a
  document, and resolves modules (`engine/parts.sv` beside the document or an ancestor, then
  `library::MODULES`). It **invents no wording**: a per-document line is `diagnose::summary`, a
  culprit is `io::describe_with` (`SourceMap` names, never `P0`; the app uses `gcs_elab_describe`),
  `--json` is `report::*_json`.
  **Where a name landed is part of the report**: `report::positions` zips `EntKind::scalar_names`
  against `Sketch::entity_params` (`hinge.x`, `base.r`); `--where NAME` filters it (a name and
  everything under it), `--json` publishes it, narrowed by the same flag. Exit codes 0/1/2 are
  `Diag::severity` and `SolveResult::success`.  The importer seam is a `Source { name, text }` list
  in, a report per source out — **the core takes text and has no filesystem** (it runs in wasm).
  `--stl` writes binary STL through `gcs_core::mesh` (`--solid` says which).  `--output` writes SVG
  through `gcs_core::svg`, as `File ▸ Export SVG` does; it **chooses a `unit`** from the page width
  (`--width`) and every constant size follows.  The *camera* is never consulted.
* **app** (`web/src/app/`): a 3D-workspace sketcher (see **The workspace** below), the only front
  end.  The *view* is the canvas:
  `view.ts` holds the state and modules take that view as first argument (`paint`, `gesture`,
  `tools`, `dimension`, `edit`, `camera`); `underlay` is the picture traced over — **view state,
  never the document** (never saved, exported, solved or undone), handled by the ordinary select
  tool, with two rules: **the drawing outranks it**, and **only its frame is clickable, never its
  interior**.  It is not a `Primitive` and never joins `selected`, but the two selections are
  exclusive, so Delete is unambiguous.  Each direction is **one place**: `pickImage` clears
  `selected`, and `selected` is a *setter* that lets the picture go, so paste, a rubber band and the
  constraint list inherit the rule.  Pick precedence is stated once, in `gesture::whatIsAt`.
  `SketchView` keeps a one-line delegator for each verb the shell calls.  The *shell*: `shell.ts`,
  `commands` (constraints bar), `dialogs`, `lists`, `dimbox` (a dimension's number), `program` over
  `editor` (knows nothing of Solvent), `ui`, `main.ts` (only wiring). `index.html` is structure and
  `app.css` the whole of the styling.

Commands:
`make` (native `build/libgcs.dylib`), `make solventc` (`build/solventc`),
`make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (both released artefacts, cargo and the web suite),
`cargo test --manifest-path rust/Cargo.toml` (**never `--release`**: the suite runs under
`[profile.test]` — optimised, no LTO, no debuginfo; `rust/Cargo.toml` says why; one file of the
core suite is filtered as `cargo test chain::`),
`cd web && npm test`, `make bench` (the native `bench` binary and `npm run bench`, read side by
side), `cd web && npm run serve`.

`npm run build` is `tsc` (module per file, what `node --test` runs against) then esbuild, which
rolls `dist/app/main.js` into one `dist/app/bundle.js`, what the page loads.  The bundle is written
*beside* the entry module: `core/wasm.ts` finds the core at `../wasm/gcs.wasm` relative to
`import.meta.url`, which under bundling is the bundle's URL.  The node-only fallback imports in
`wasm.ts` are left external.  **three.js is the project's one runtime dependency**, bundled (not
from a CDN) so the app opens from a file.

Conventions:
- **A name is defined one way, `NAME := VALUE`** ([plan](docs/definitions-plan.md), Solvent §5,
  [0.29]).  `w := 100` is a value (`StmtKind::Param`), `c := circle(…)` a declaration, `t :=
  Tooth(…)` an instance, `dims := {…}` a group (a brace after `:=` is a list, lexed across lines),
  `profile := (ab := line(a, b)) -> line -> close` a chain whose link is named in place, `k :=
  leg.toe over u in (a, b)` a curve, `p := point(x: e, y: e)` a computed point.  A dimension's
  number defines no name (named dimensions are gone, [0.41]).  `:=` is `Tok::Define`; a lone `=` is
  no token.  `:=` binds loosest, so a link is named in parentheses; with no `->` in the statement
  the name goes to the one declaration (`l := horizontal line(a, b)`: a prefix word's value is its
  operand).  `P::definition` lowers to the existing `StmtKind`s (`Param`, `Group`, `Instance`,
  `Decl`, `Chain`), so nothing below the parser knows; `syntax::words::named_link_at` tells `(l :=
  line)` from an operator's own parentheses.  `label:` never defines: it fills a slot.  A name in a
  child slot (`line(a, q := hint(…))`) is not implemented yet.
- **Every seed is written in one `hint(…)` clause, and nothing else is** (Solvent §4.3, §6.4): `p :=
  point hint((0, 0))`, `c := circle(center: o) hint(r: 25)`, `point_on_spline(p, s) hint(t:
  0.4)`.  Keys in any order, an omitted coordinate is 0 — an omitted *radius* is computed from the
  geometry (`UNSEEDED_RADIUS` where it gives none; 0 is stationary for on-circle rows); the clause
  sits in the trailer loop beside `knots`, `class`.  **The brackets after the name are what the
  thing is made of; the `hint(…)` is where the solve begins.**  §4.3's rule is lexical: *a number
  inside a `hint(…)` is a seed, and every other number is not*.  The four retired spellings (`at (0,
  0)`, `hint at (0, 0)`, a scalar in a constructor arg, and `hint at REF [bearing (…)]`) do **not**
  parse, and each errors saying where the number belongs.
  **A place is two keys of the same clause** (issue #47, item 2): `point b hint(at: orbit, bearing:
  u + f.angle)`, `point p hint(at: pin)`. `hint_body` reads `at:` as a reference (`Hint::place`),
  and the declaration's trailer loop is the one table that takes it, into `Decl::seed_at` (an
  `AtRef`); a clause with `at:` and a scalar, or `bearing:` without `at:`, is refused at the key.
  Both lower to the coordinate spelling's tapes — no second grammar or `bearing` keyword.
  A **pin** stays in the argument list — `point_on_spline(p, s, t == 0.4)` — because `hint` marks
  what a solve revises and a pin is what it does not.  **What `hint` marks is that a solve revises
  the number, not that the number is seed-class** — so a **callout placement keeps its bare `at`**:
  inert, but nothing in the solve path writes one (`callout::drag`/`callout::reset` are a person
  acting).
- **A seed may read geometry, and reads its seed** (Solvent §6.4).  `hint((k.center.x + k.r,
  pin.y))`, `hint(at: k, bearing: b)`, on the sheet and in a child slot. The flattener settles a seed
  text over scope parameters; failing that, a text naming a dotted scalar is kept (`settle_seed`,
  `reads_geometry`) with every dotted name resolved to its absolute name in `Decl::seed_names`
  (`rescope_seeds`, via `lookup`; never rewritten into the text); `program::build` records each as a
  `Deferred` and `settle_deferred` works them out after every kind is built, in statement order
  (`seed_read`, `place_of`).  A read is a `Length` where the document names a unit, else a bare
  number (`seed_eval`).  A `param` may not read geometry, and `commit_seeds` never writes an
  expression back, so P3 holds.  A sheet `hint(at: …)` bearing is `substitute`d over the scope's
  numbers, printed **with their unit** (`of_vals`: `(180deg)`, `(150mm)`).
  **A place may be a step** ([0.33]): `hint(at: a, toward: b, by: f, turn: θ)`, or `along: l` for
  `toward:` (`AtRef::{toward, along, by, turn}`, `entities::place_of`), not coordinate arithmetic. A
  place in another view is read in space and projected into the seeded point's (`seed_in`), so
  `program::build` settles seeds a second time after `solve_planes` when one crosses
  (`crosses_views`), and only then.  An unwritten arc radius settles with them (`Deferred::Radius`).
  A traced point refuses a step.  Components are closed over their arguments: only their own
  parameters and formals are available. `module_params` exports values to importing root bodies, to
  be passed explicitly. `tests/seeds.rs` and `tests/closed_scopes.rs` are the gates.
- **A part is one component carrying `in view { … }` blocks** (Solvent §6.7, `P::in_comp`): the
  block form is allowed inside a component body (the plane is a formal) and still refused inside a
  root block.  With `repeat flag { … }` over a 0/1 `Int` formal for the views an instance does not
  show in, a part's whole design is one module (`engine/block.sv`, `engine/head.sv`,
  `engine/crankshaft.sv`, `engine/conrod.sv`); view modules hold only what the assembly adds.
  Instances inside a block copy are indexed like declarations (`cyl[0].small`, `copy_of`).
- **Which way is a word, not a sign** (Solvent §9.2, §9.4; issue #48, item 4).  A distance measured
  *from a line* (`PointLineDistance`, `ParallelDistance`) is a **magnitude**: its kernel is `|g| −
  d` (`kernels::point_line_magnitude`, degree 1 — **not** the squared form, whose gradient vanishes
  at `distance(0)`), both sides are solutions, and the seed picks.  `side: left|right` pins one, and
  then the *signed* kernel runs with the word's sign. `CKind::side_words` is the one table of words
  and signs (`left`: +1 of a line, −1 along the page).  A negative magnitude is E040 **by value**,
  so `p distance(-hw) axis` is refused; where a sign is arithmetic, use a signed datum ordinate. The
  run, the rise and the directed angle keep their signs and gain `along: right|left|up|down` and
  `sense: cw|ccw`.  `io::dimension_text` draws the number the statement **makes**.  A component
  takes a side as `Ty::Side` — a word in `Scope::sides`, never a ±1 in `vals`.  `cgraph`'s PL edge
  asks `Constraint::signed_gap`: the word where pinned, the *pose* where not. `tests/refusals.rs` is
  the gate.
- **A selector says what it means, or it is refused** (Solvent §9.2). `CKind::words(slot)` is the
  vocabulary a `Str` slot takes (`at: start|end`, `at: p1|p2`) and `constraints::ALONG` the one
  table `along:` is read by (choice and message).  A key naming no slot, a word outside a slot's
  set, and a bad `along:` (e.g. `along: z`) are all E040 **at the key** (`Written::key_span`), never
  silently dropped or misread; `report::registry_json` publishes each slot's words.
  `tests/refusals.rs` is the gate.  `along:` also takes a reference (an axis, a line): see the
  ordinate below.
- **An ordinate is one kind along any direction, and `level` its zero**
  ([plan](docs/ordinate-plan.md), Solvent [0.46]).  `a distance(d, along: t) b` is `CKind::Ordinate`,
  `(q − p)·t̂ − d`, signed along `t`; `a level(t) b` is `CKind::Level`, no number, no callout (the
  direction positional, as `symmetry(l)`'s line).  Slots `(p, q, t: SpecKind::Along, [d], along:
  Str, form: Int)`: `t` an axis, a line, or a plane standing for its normal (only via the word
  `n`); the word (`constraints::ALONG`, `Toward`) keeps the spelling and the sign
  (`right`/`left`/`up`/`down`), and `infer_entity` reads a page word's axis off the view both points
  are drawn in (none in a planeless sketch: refused).  `form` (`OrdinateForm`: `PageU`, `PageV`,
  `InView`, `Space`, `CoordU`/`CoordV` — from a view's own origin, the point's one coordinate, the
  radius kernel's one column — and `FrameU`/`FrameV`/`FrameN` — a plane's own word from its origin
  for a point drawn elsewhere, over the plane's columns, so `v` is its frame's `v̂`, square to `u`,
  which an axis `P.v` written outright need not be) is set by `Sketch::add_quiet` from `constraints::ordinate_form` and picks the
  kernel (`Constraint::kernel`; the run and the rise keep their constant-Jacobian kernels,
  `K::OrdinateU`/`V`); `Constraint::reads_space` asks it where `CKind::spatial` would.
  The plane forms (`q distance(d, along: u) P`) are rewritten from `P.origin` along `P.u` in
  `program::relations::ordinate_operands`, which also refuses a plane as a direction, a word that
  also names a direction in scope, and a literal zero (E040, naming the level); `a horizontal b`
  is `std`'s word for `level(up)` (`use std (horizontal)`, §9.9), and the printer
  (`print::ordinate_text`) spells each case back.  `tests/ordinate.rs` is the gate.
- **A recorded root choice is one record of one triangle** (`decompose::branch_record`).  `ccw(a, b,
  c)` ("c left of a→b") is the same fact as `ccw(a, c, b)` with the sign turned, so a record is
  **canonical**: point indices ascending, the sorting permutation's parity folded into the sign
  (else `apply_gauge` and `Step::stated` write different keys).  Every writer goes through
  `branch_record` — the elaborator, the plan's `branches`/`apply_branches`, `io::graft`,
  `Part::branches_out` — and `io::from_json` **re-records** what it reads, so older documents
  migrate on load.  `tests/decompose.rs` and `tests/order.rs` are the gates.
- **A name declared over a built-in is said** (Solvent §3.3, §5; `program::shadowing`, W112).
  `expr::eval` knows `expr::CONSTANTS` and `FUNCTIONS` before the document and
  `flatten::substitute_with` only the document, so a value, input, formal or block index called
  `tau` reads differently in the two: the **warning**. Asked of the text (every component,
  instantiated or not, and every module body).  `expr::builtin` is the one table; `tests/names.rs`
  is the gate.
- **A call is the entities by position and the numbers by label** (Solvent §4.1,
  `flatten::check_call`).  `Cylinder(swing, side, top, piv, rod, across, dir: dir, fw: fw, o_s: o_s,
  o_t: o_t)`: an argument bound by position must fill an *entity* formal and must stand before every
  label; either mistake is **E004** at the argument.  Asked of the **text**: once per written call,
  before anything is bound.  `tests/components.rs` is the gate.
- **Modules** (Solvent §14.4, `modules.rs`, `library.rs`).  `use engine.parts` is parsed into
  `Program::uses`; `modules::link(prog, resolver)` resolves each once, transitively, via
  `syntax::parse_from(text, base, first_id)` — **every span is one integer into one virtual text**:
  the document, then each module after a one-byte gap (a splice, root body only, never meets a
  module span).  Module statements are numbered from `modules::MODULE_STMTS` (2³⁰), so a statement
  appended to the document never renumbers the library's ones the source map names.
  `Program::source_at` maps an offset to its text; `modules::localize` shows a
  module's diagnostic at its `use` (`Module::via`) with `name:line:col` in front.  A module
  contributes its components (`Component::module` says which) and its top-level params and groups,
  each reached by the module's full path (`engine.parts.Rod`, [0.30]); its own drawing is not
  drawn.  **The core has no filesystem**: the resolver is the host's — `solventc` reads
  `engine/parts.sv` beside the document, then `library::resolve`; the FFI and `examples::document`
  use `library::parse_linked`, over `library::MODULES`.  **`rust/lib/` is the standard library**
  (`std`: `std.StandardDatums`, `std.Turned`), in the Makefile's `RUST_SRC`.  `program::reparse` relinks from the
  texts in hand (`modules::relink`), so an edit never asks the host.  E070 no module, E071 defined
  twice in one file.  `tests/modules.rs` is the gate.
- **`port` is retired** (issue #47, item 1).  Everything an instance makes is reached by its dotted
  name (`five.s[0].p1`).  The **computed point** is `p := point(x: xexpr, y: yexpr)`
  (`Decl::computed`), refused on the sheet by the flattener and compiled to two tapes when traced.
  The parser keeps the word in `OPENERS` only to refuse it.  Aliasing is untouched: it is argument
  passing (`bind_instance`).
- **A class stands on a relation and on an instance, and `display: none` hides** (Solvent §13.2).
  `Relation::class` is parsed in both trailing-clause loops (a chain's and a lone relation's, before
  `at`) into `Constraint::class`, through `io::dumps`, `from_json`, `graft` and `write_relation`,
  and **not** in the binding's record.  `callout::style_of` resolves `.dimension` (`.reference` for
  a claim), and `layout` skips one not `shown()`. `Instance::class` (`Scope::in_class`) is stamped
  **over** each emitted declaration's and relation's own (`stamp_scope_plane`), the way `in` is.
  `Style::display` is `display: none | inline | geometry` (`style::Display`): `shown()` is what an
  entity asks, `dimensioned()` what `callout::layout` asks; `geometry` is drawn but never
  dimensioned (`style .phantom { …; display: geometry }`).  `svg::render` and `paint.ts` skip what
  is not shown, and every point is drawn under the implicit class `.point`
  (`EntKind::implicit_class`, `styleNamed('point')`).  The idiom for a dense drawing is `style
  .dimension { display: none }` and `class shown` on the few to draw.
- **A declaration need not name its children** (Solvent §6.1, §6.2).  `l := line` mints two points,
  `c := circle` one, `a := arc` three; a child slot may hold a `hint(…)` instead of a reference
  (`alt_a := line(A, hint((15, 5)))`).  `Decl::children` is `Vec<Vec<Kid>>` — a name *or* a
  seed; "anonymous and unseeded" is an *empty slot*: an **implicit child**, minted by
  `program::build` (`(l1 := line) -> (l2 := line)` is three points).  E103 refuses only a list with
  *more* children than slots.  A joint threads a *name*, so a seeded slot reads as unfilled there;
  where neither side names it, `thread` mints the earlier-built side's dotted boundary (`l1.p2`),
  refusing only a name-link whose kind has no boundary field.
  **The dotted path is the name.**  `program::build` mints an anonymous child *with* `l.p1`, binds
  it in `map.names`, and records it against the parent's statement, so it resolves, constrains,
  drags, picks and survives re-elaboration (`Document.entity`).  Its seed lives in the *parent's*
  statement, so `commit_seeds` splices inside `Kid::Hint`'s spans — and where the source wrote no
  list, writes the whole argument list at `hint_span` in one edit (two splices at one offset race).
  **The element's own name is optional too**: `line`, `line(p1, p2)`, `circle hint(r: 25)` and
  `arc(center: c)` are anonymous forms, and a name is written *before* the value (`l := line(p1,
  p2)`), never after the keyword. An anonymous declaration carries a `Decl::name` key the source
  cannot write — `#a` and its keyword's offset — with an **empty span where `name := ` would go**
  (`hint_span`'s idiom); in a chain `Decl::mint_close` says where the `)` of `(name := …)` goes.
  **A name is three questions, each known where the name is minted — never sniffed back out of the
  characters**: does it **resolve**, does the source **call the thing that** (shown, selected by),
  and may a statement be **written** with it.  `Decl::name` is a **`DeclName`** fusing the answer:
  `Written(Name)` (`l0`, `s1.p0`), `Copy(Name)` (`#3.0.p`, a block copy) and `Key(Name)` (`#a41`);
  accessors `key()`, `shown()`, `written()`, `span()`.  Stamped by the parser and by **the
  flattener, which knows whether a prefix is an instance's name or a block's id** (`Scope::copies`,
  `DeclName::prefixed`).  `SourceMap::bind` (vocabulary `DeclName::named`) files each name into
  `by_name` always, `names` when `shown()`, `writable` when `writable()`.  So `map.names` is "what
  the source calls the thing" (`name_of` is `names.first()`); `writable_name` is `edit::reconcile`'s
  gate, refusing a gesture on a block copy *with the cause*.  `syntax::hidden` survives only for a
  thread-filled slot's `Kid::Ref`.
  The key (what a corner welds by, what `res` resolves) must never reach the source: `write_decl`
  omits it, `commit_seeds` leaves a thread-filled slot *empty* — forcing labels on kept children
  after a gap (`decl_args`) — and diagnostics spell the kind.
  `edit::reconcile` **mints on demand**: when an appended statement must reference an anonymous
  element (a constraint, a gauge on a fixed point — `held_refs` is the one walk `gauges` shares), a
  real name is spliced in at that empty span, **every entity the statement made** renamed with it (a
  child by the path `program::child_names` gives its *position*) and `bind`ed `Named::Written`.
  `SourceMap::ents_made_by` (build order) is shared by `commit_seeds` and `reconcile`.  The guards
  are one question, "does the source call this anything": named, no root statement to name it on
  (refused with the cause), named since the map was made, or mint.  Insertions racing for one offset
  are ordered by `splice`'s stable sort, so reconcile pushes appends before flags before names;
  `tests/anonymous.rs` is the gate.
  Where an unseeded point *starts* (an implicit child, `a := point` with no `hint(…)`) is
  `program::scatter`, an implementation choice the spec must not carry — never the origin (a
  zero-length line, a `distance` at a stationary point), and minted points may not pile up or seed
  a self-crossing contour (a collapsed side satisfies every direction constraint).  `scatter` walks
  the bearing a fixed irrational step per minted point in creation order (a chain's traversal
  order: a simple polygon).
  "No clause" is the empty `hint_span` (a lifted declaration has `None`), and `commit_seeds` writes
  the solved pose in as the clause.  A component's points take the same clause; `gear.sv` /
  `gear_trace.sv` state theirs (else a flank's first step can reach the mirror branch).
- **Presentation is a separate statement from what the drawing is** (`style.rs`, Solvent §13.2).
  A declaration carries a **class** (`datum := line(o, q) class construction`) and a top-level
  `style .NAME { dash: 7 4; width: 0.5; color: #888888 }` says what a class looks like.
  **No algorithm in the core consults a class**.  `construction` is a class, not a word:
  `style .construction { dash: 7 4 }` is the one rule in `style::base()`, which a document may
  override.  **The core resolves and the front end strokes**: `paint.ts` reads `ent.style` (dash,
  width, colour) and knows what a class is nowhere; `app/edit.ts`'s toggle sets and clears the
  *name*.  A sheet's lengths are **screen pixels**, never world units.  An unmatched class is not
  a diagnostic (no rule, as in CSS).  The cascade is **two layers, not one interleaved pass**: the
  whole base sheet under the whole document's, each in written order, so a document beats what
  ships whichever class it is written on.  A base rule states only what its class **adds**
  (`.reference` is the lighter ink only; a reference dimension is `class dimension reference`).
  A value the sheet cannot read (`color:` with nothing after it) is **dropped**, as an unknown
  property is.  `Sketch::style_epoch` is bumped by the one write path (`set_class`, `set_sheet`)
  so a binding may cache against it.  JSON writes `"class"` and **reads `"construction": true`**
  and never writes it.
- A constraint may own *unknowns* of its own: a `SpecKind::Param` slot in its `spec`, allocated
  by `Sketch::add` and moved by the solver like any other parameter.  The slot holds a seed
  number on the way in (what a document stores, `graft` copies, and `constraints::seed_param`
  supplies when omitted — the `Param` counterpart of `infers_arg`) and an index into
  `Sketch::params` once added.  It is not a stated value: `describe` leaves it out, both bindings
  publish it read-only, `same_constraint` ignores it.  `Sketch::remove` retires an orphaned one to
  `fixed` (a parameter no equation mentions is no DOF); the rebuild walk reclaims the slot.
  **A contact's place may be shared** (#70 part 2): `t == s` over a declared unknown (`param s:
  Angle hint(318)`, or an unbound formal) is `syntax::Arg::Tie` seeded by the declaration
  (flattener; a `hint(t:)` beside it is refused in `assemble`), then
  `Arg::Shared`, which `Sketch::add` turns into one `Param` per name (`Sketch::shared`, a
  `SharedPlace`: the unknown and its curve) for every contact on that curve (E040 otherwise, and
  for a name a dimension also reads: `Fault::Place`). `remove` retires it with its last owner;
  `graft` and `lift` read `Sketch::owned_arg`, the JSON `"shared"`. `tests/shared_contact.rs`.
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/` — **a new file there is listed in `tests/main.rs`** (one binary,
  `autotests = false`; `every_file_is_a_module` fails an unlisted file).  A binding changes only
  when the *surface* changes.  Geometry or numerics in TypeScript belong in Rust instead.
- A new *entity* kind stops the build in the exhaustive `match e.kind` arms — `model.rs`
  (`entity_params`, `own_params`, `own_length_params`, `children`, `min_children`, `count`,
  `bounds`, `distance_between`, `point_to_drawn`, `class_of`, `set_class`, `spatial`), `io::graft`'s
  remap, `overview::drawable`, `svg::entity`, `program`'s build and `set_class`,
  `syntax::kind_initial` and the FFI's `ent`/`kind_id`.  Give it an arm in each; `primitives()`
  and `topology_key` are where it joins the document.  A kind **evaluated rather than drawn**
  (`Face`, `Solid`) answers `spatial()` and owns no parameter.
- Every new constraint type = a vectorized kernel in `kernels.rs` (added to `KERNELS`; the
  registration order **is** the kernel id) declaring its `degree` — the power of length its
  residual carries, 1 for a signed distance and 2 for a squared one — a `CKind` variant in
  `constraints.rs` declaring its `spec` (constructor args as (attr, kind) pairs), `params()`,
  `consts()` and `default_arg()`, and a row in `rust/gcs-core/tests/jacobians.rs` (FD check,
  spec round-trip).  Both bindings generate their classes from `report::registry_json`.
- A type whose spec carries a `Length` or an `Angle` is a *dimension*, and dimensions are drawn.
  `callout.rs` matches `CKind` exhaustively in two places: give a new type a `Pen` arm (its
  drafting figure) and a `frame` arm (the `Frame` its placement is written in), or list it in
  `undrawn!`.  `every_dimension_is_drawn` checks the arm produces a figure.
- A parametric curve (`curve.rs`) is *linear in its control points*, `C(t) = Σ Bᵢ(t) Pᵢ` (every
  B-spline).  A contact carries its own curve parameter as a `Param` slot and says
  `p − C(t) = 0`: two residuals, one new unknown.  A contact kernel needs only the basis values
  and two t-derivatives — a second curve family is a second basis, not a second constraint
  family.  A line against a curve is a tangency; a *circle* against one is `SplineCurvature`, the
  osculating circle ("the centre is the centre of curvature", no `side` to infer).  It divides by
  the turning rather than multiplying: multiplied, every row vanishes with `C'` and the solver
  collapses the parameterisation.  Control points are ordinary `Point`s.  Only `DEGREE + 1`
  control points are non-zero at any t, so a contact addresses one *span* (fixed-width blocks);
  the span is derived from t, not stored, and `Sketch::topology_key` carries it — a contact
  walking past a knot is a recompile.
- **A spline may be rational** (`spline(…) weights [1, w, w, 1]`, `SplineE::weights`, `None` for
  all 1): document data like the knots, so the curve stays linear in its control points over the
  rational basis `Rᵢ = Bᵢwᵢ / ΣBⱼwⱼ` (`curve::weigh`, applied after `basis` in `eval_on` and the
  kernels' `span_frame`; all 1 divides nothing, so polynomial splines keep their bits). A spline
  contact's consts are `SPAN_C` = knot window then the span's weights; `topology_key` carries
  them. Knot insertion is homogeneous; a deleted control point takes its weight (`graft`). The
  trailer's entries are `syntax::Weight`s, settled over the scope by expansion (E103 if not a
  positive plain number). The recipe writes `"weights"`, so the profile is `BSpline::rational` in
  our kernel and in OCCT's (`solvent_cad_bspline` takes a weights pointer). `nurbs.sv`,
  `tests/weighted_spline.rs`.
- A control polygon is edited three ways.  *Inserting* is `curve::insert_control` (Boehm's knot
  insertion: C(t) unchanged, every contact keeps parameter and place; `DEGREE - 1` neighbours
  move, keeping identity).  *Deleting* shortens the curve: `Sketch::min_children` is the general
  rule (an entity survives while enough children do), and `curve::knots_without` drops one
  interior knot per lost control point.  *Interpolating* is `Sketch::spline_through_held`
  (chord-length parameters, averaged knots, one collocation solve).  A place from empty space
  leaves nothing behind; a place from a Point is *held* by a `PointOnSpline` whose parameter is
  **pinned** at the fit's value (unpinned, the curve keeps m DOF sliding along itself).  A pin
  travels in the seed: `Arg::Seed { value, pinned }`, consumed by `Sketch::add` at the one seam
  turning a number into a Param.  `clamp_contacts` leaves a pinned parameter alone.
- A curve parameter is bounded (`t0 <= t <= t1`), which least squares cannot say.
  `System::solve` does — clamp, compare `curve::contact_spans` against the compiled spans,
  rebuild when one moved — so *every* caller gets it.  A clamped parameter is *pinned* for the
  retry.  A contact *seeded* off the end is clamped **before** the first solve and left free (a
  seed says only where the search begins, P3; #45.7).  `SolveOpts::rehome` turns it off for
  `PullPolish`, which owns a pair of systems and re-homes both together.  An empty span map
  costs nothing.
- A block's columns and its constants are ONE compile-time choice: which span of a spline a
  contact sits on.  `System::new` makes it once and passes it to both `params_on` and
  `consts_on`, and remembers it — so `refresh_consts` skips curve contacts outright (their knots
  are document data no solve moves) rather than re-deriving a span that may since have walked.
- `Param::scale` is the world length one unit of a parameter is worth — 1 for a coordinate or
  radius, the curve's mean speed |C'| for a curve parameter (read by `System::new`).  `System`
  solves in `z = x * col_scale`, so trust region and minimum-norm step measure world units;
  unscaled, tangencies stall as a drawing grows.  All-1 systems take the untouched path.
- "Solved" is `System::max_relative_residual <= 1e-6`: each row's residual over its own units
  (`extent^degree`).  Never one absolute threshold — kernels are of degree 1 and 2.  **The
  iteration sees the same vector**: `residuals_into` and `compute_csr` divide every row by
  `row_scale`, so the dogleg weighs an `angle` row and a `distance` row alike (issue #43).  A
  test comparing against a kernel multiplies by `row_scale` to get the raw residual.
  `System::stationary` tells apart two non-solutions: at a stationary point the residual is what
  the constraints cannot agree on (a conflict); elsewhere the solver merely stopped, the
  diagnosis solves a scratch copy, and a consistent drawing is `State::Unsolved` — no conflict
  set, no culprits, the unsatisfied rows listed.
- Its Jacobian twin: rank and null space are judged on `System::conditioned` — hard rows over
  `extent^(degree−1)`, columns in world length — against one absolute, dimensionless
  `system::RANK_TOL`.  Never a raw `J`, never relative to `σ₀` (the largest row may be in another
  figure).  `Conditioned` is the only matrix the *diagnosis* judges, taking only an absolute
  tolerance; the `Tol`-taking factorisations are `pub(crate)`.  (`decompose::deficiency` and
  `homotopy` keep a relative rank on their own synthetic matrices.)  `jacobian_dense` stays the
  raw `∂r/∂z` for solvers and finite-difference checks.
- A tangency whose contact the drawing already holds to the curve is stated *at* that point, or
  it is a double root: `PointOnCircle(p, C)` + `TangentLineCircle(L, C)` with p an endpoint of L
  is rank-deficient at *every* solution.  `TangentLineCircleAt(line, circle, at)` (the
  `tangent_arc_line` kernel) is regular, and `commands::cTangent` states it whenever the line has
  an end already on the circle.  For degenerate pairs that still arise, the diagnosis and the
  witness settle-test the surplus motions: step along a null direction, settle, and a motion that
  walks back is `shaky` — out of the DOF, "blocked at second order", never painted under- or
  over-constrained.  `numeric_rank` already includes them; nothing adds `shaky` back.  Every
  guard is inside `witness::screen`: at a solution, a tangency present, removals capped at what
  the matching cannot account for, at most `SCREEN_MAX`.  A double root's singular value is as
  small as a real freedom's, so a candidate is recognised only by trying it.
- `Horizontal`/`Vertical` level a line; a `Level` along a view's own axis levels a *pair of
  points* with the same line kernels, and `cgraph` gives it a `virtual_line` in the ground
  x-axis's direction class, so a levelled pair decomposes rather than falling to the residue.
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
- A **`claim`** (Solvent §9.7) is *judged, never solved for*: **no** `System` compiles a row for
  it, and `cgraph`, `io::Part` and the witness's jitter skip it, so it never moves geometry,
  welds drag parts, or paints Over or Conflict.  `Constraint::acts()` is the named half, asked by
  `hard_constraints`/`hard_ids`; seams needing a row index spell it out inline, as `soft` is.
  Anything learning what determines the drawing (`known_radii`, entity colouring, conflict
  candidates, `duplicated`) must go through `acts()`.  The diagnosis reads it at the *end* of
  `diagnose_with` via `System::conditioned_with`, stacking its rows onto the compiled system:
  *theorem*, *violated*, or *consuming* (holds only by the pose).  Never judge it by compiling a
  second `System` — a compile calls `locus::forget`.  A claim may not own a `Param` slot nor bind
  a free variable (`CKind::claimable`; E040 at elaboration; document readers drop the flag;
  `expr::evaluate` refuses the binding).  The flag travels through `graft`, JSON (`"claim"`,
  written only when set) and both bindings.  `peaucellier.sv` is the case, `tests/claim.rs` the
  gate.  `io::describe` prefixes `claim `; `callout.rs` draws a claimed dimension
  **parenthesised** — the whole label (`(R50)`, never `R(50)`), wrapped *before* measuring.
  **A verdict is shown where the claim is written, quietly**: a wash behind the statement
  (`app/program.ts::marks`, `.claim-proved` / `.claim-refuted` / `.claim-independent`), no banner.
  *proved* (entailed), *refuted* (this drawing is a counterexample), *independent* (true here, not
  implied) — never "inconsistent".  `CodeEditor` carries a *set* of overlapping marks; a mark
  tints and **never changes the face** (background/box-shadow/outline, not border/padding).  Run
  `npm run overlay` (headless Chrome, the real `CodeEditor`) when you touch `editor.ts`;
  `make test` cannot.
- `same_constraint` is "says exactly the same thing"; `same_relation` is the same *without* the
  numbers.  A repeated *relation* is refused by the app (`edit::applyConstraints`): equations
  without rank.  A **dimension is never deduped by the UI**: redundant or contradictory is the
  diagnosis's reading (`over` naming both).  `gcs_constraint_stating` is the core's question;
  `commands::dimension` states, `commands::editDimension` opens an existing one (double-click).
- An argument the core reads off the geometry (a tangency's side or sense) declares
  `CKind::infers_arg`; the registry publishes a null default for it so a binding leaves it
  omitted and the core fills it in.  A binding that substitutes a constant picks the branch.
- Mutating a constraint's constants (drag target, edited dimension) must be followed by
  `System::update_consts` / `refresh_consts` on any compiled system, or a recompile.
- `System` is the compile-once / evaluate-many seam; keep the object model out of the hot loop.
  The bindings own a handle — call `dispose()` when you drop one (the drags, the plan solver and
  diagnosis all do).
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees
  theorem-type dependencies — that residue is Stage 4's corpus, keep logging it
  (`Diagnosis.warnings`).  Above `NUMERIC_MAX` (300) free parameters the whole SVD is not
  taken, since it runs after every edit: the rank is read **part by part** (#88,
  `diagnose/parts.rs`, `Diagnosis::by_parts`) over `System::block_order` — over part first,
  square blocks in solve order, under part last — each factored alone (pivoted QR where it is
  only asked for a null space, `linalg::null_full_row_rank` for the under part, SVD otherwise),
  every part's null vectors carried into later parts by minimum-norm solves and checked against
  the rows a part could not make up: exact, not a bound (`tests/numeric_parts.rs` holds it to the
  whole SVD over the library; the large cases in the slow tier).  The second-order screen runs
  on its null space as on the whole's.  The V-twin (925) reads in ~0.2 s.  A consistent dependency is `over` ("remove one") only when a
  dimension takes part in it — editing that dimension is the next conflict; one among pure
  relations is a theorem that nothing can break, so its wholly-implied constraints are
  `implied`: noted, never painted as an error.
- **Every case in the library is a Solvent document**: a `.sv` file in `rust/examples/` whose
  builder elaborates the text (`examples::document`).  `with_params` gives the document's own
  named numbers (`w := 100`, `distance(a := 30)`) the caller's values — no second copy in Rust.
  A start off the solution is `jitter`, a function of the sketch.  **Seeds must track
  parameters** (`pythagoras.sv` places its square from `la`/`lb`).  `tests/examples_sv.rs` holds
  each document to what the library advertises.
- **What is not a drawing is not a case** (`fixtures.rs`).  `laman` and `henneberg_edges` make
  random graphs for property tests; they stay in the core, in their own module, off the case
  library and the app's menu.  A case belongs in Solvent when its numbers are things a person
  would state, and stays a generator when a program measured them.
- **The document is the Solvent source (`gear.sv`, `syntax.rs`, `program.rs`, `edit.rs`).**  The
  drawing is what elaborating it produces; *every* edit is a **splice** of the text somebody
  wrote and **never a reprint** (which would flatten components and drop comments).
  `edit.rs` is the whole of it: `commit_seeds` (a solve), `reconcile` (a gesture), `remove`,
  `set_dimension`, `add_point`/`add_entity`/`add_relation`, and `mint` for a name.  Three edit
  classes, and **the core says which — the front end never guesses**: `Structural`, `Numeric`
  (only numbers a solve may move, so a compiled plan survives), `None`.  Writeback is one
  lexical rule: *a seed is writable iff it is inside a `hint(…)` clause, is a literal and not an
  expression, and is reached by exactly one instance path.*  `Decl::hint_span` carries where the
  clause *is* or — empty — where it *would go*; `commit_seeds` splices each seed in place when
  all have spans, else writes the whole clause there; a decl seeded by *place* is skipped.
  `reconcile` and `retext` **apply themselves to the `Elaborated` and do not rebuild the
  drawing** (that would invalidate a half-finished tool's proxies).  `retext` re-parses (same
  ids); `adopt` extends the map onto new statements.  `reconcile` reads only the *append* (past
  the map's high-water mark); it cannot follow a **renumbering**, so deletion is `edit::remove`
  and not `io::without`.
- Deletion, copy and paste are one rebuild walk (`io::graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset.  A new thing that travels with a constraint or an entity — a flag, a
  placement — belongs in `graft`, or the three will disagree.  JSON is now the *export* format,
  derived rather than canonical; `io::dumps` is still what the Rust tests and the benchmarks
  compare against.
- **Every constraint is written as a prefix or an infix operator** (Solvent §9.2); `name(args…)`
  is retired.  `radius(25) c`, `p1 distance(80) p2`, `horizontal l`, `fix((0, 0)) p`,
  `a symmetry(l) b` — the word, its one or two operands, and everything else in the parentheses on the word: the
  number, a selector (`side: -1`, `at: start`, `along: x`), a third entity, a pin (`t == 0.4`).
  A *seed* for an owned slot stays the trailing `hint(t: 0.4)`, where every seed is.
  **A pin and a seed are one `OpArg::Slot { key: Name, arg: Arg }`** — the word is the only
  difference (as separate variants a pin lost its expression, a key naming no slot filled any
  `Param` slot, and `write_written` disagreed with `operator_text`).  The value is
  **`syntax::Arg` and not a second encoding of it**: `assemble` hands it straight on, and
  `flatten::settle_arg` is the one walk that reads a component's parameters out of an argument.
  `Written::assemble` matches the key against the spec; an unknown one is an E040 **at the key's
  own span**, so the key is a `Name`, `assemble` returns a `Result` and `settle` carries a `Span`
  beside its message, in `program.rs`'s `(Span, String)` order.  `syntax::slot_text` is the one
  place a slot is spelled, read by both printers, as `hint_of` is for the clause around it.
  **The shape is the library's**: every user-facing kind has one or two entity slots, always first
  in spec order, with `Symmetric` the single three-slot exception.  Several kinds share a word —
  **`coincident` is fifteen kinds, `distance` nine, `tangent` six** (twins in space included; a set's uses besides) — and `horizontal`/`vertical` are
  two each with the *fixity* doing the work.
  **What a word means is the kinds of its operands, and a name does not carry its kind until
  elaboration** — so the parser resolves *nothing*: it produces a `syntax::Written` and
  `program::settle` turns it into a `CKind` plus arguments in spec order, through
  `constraints::infix_op` / `prefix_op` and `Written::assemble`.  A **lone infix statement is a
  one-joint chain**.  `CKind::operator()` is the inverse and is matched exhaustively, so a new
  kind stops the build there; `syntax::operator_text` is the one printer, read by
  `write_relation` and `io::describe`.  **The surface word and the wire name are separate**: the
  registry goes on publishing snake_case `name`, which the binding and the JSON export key on,
  with `operator`/`fixity`/`operands` beside it — **the binding is untouched**.  `ccw`/`cw` keep a
  call (`Fixity::Call`, every operand in the parentheses): the predicate is about the *triangle*.
  **The gauges and the orientation predicates are entries of the same table** (issue #47):
  `CKind::Fix`, `Ccw`, `Cw`, read by the one relation parser and settled by the word alone
  (`constraints::gauge_op`, before the operands' kinds are asked — which numbers `fix` holds is
  the entity's, and `ccw(a, b, c)` has no operand outside its parentheses).  They are
  **applied, not added**: `program::apply_gauge` marks the parameters fixed or records the root
  choice, `constrain` returns no id, and no `Constraint` is one — so they are not in `ALL_KINDS`,
  the registry never publishes them, and `CKind::gauge` is the question every table that would
  reach for a kernel asks first.  A `claim` on one is refused (E040): a gauge adds no row.
  **A `fix` states what it holds** ([0.34]): `fix((0, 0)) p`, `fix(x == 5) p` (partial),
  `fix(r == 25) c`, `fix(half == 30deg) k`, `fix(dir == (0, 0, 1), origin == (0, 0, 0)) t` —
  pinned slots named by the entity's members (`EntKind::members`: `x`, `dir.x`, `origin.z`;
  `Fix`'s spec is their union; `apply_gauge` checks the kind's).  **A vector** ([0.45]) is `(a,
  b[, c])`: `OpArg::Vector` keeps it whole so `apply_gauge` refuses one shorter or longer than
  what it holds (a point in space is three, so `(0, 0)` cannot leave `z` free unsaid; a partial
  hold is by member); `Written::slots` expands it into member slots, and `hint_body` expands a
  seed's at parse.  Writers say each vector whole where all of it is given, else by member
  (`syntax::said`; `lift_gauge` writes a `Written` fix).  `o := (10mm, 20mm)` is a group of
  members `x`, `y` (`o.x`).  `ground`, `fix c.r` and a bare
  `fix p` are gone (`relations::fix_spelling` refuses the others where written).  **A held number
  is its own seed**: fixes are applied once points have their places (`entities::places`), before
  `axes_along`, motions, envelopes and `settle_deferred` (`relations::is_fix`), and
  `views::stand_axes` then stands each plane's free axes through its held origin, so no `hint`
  repeats a `fix`.  Holds and root choices are gathered (`relations::Gauges`) and applied once
  all are in (`hold_all` after the fix pass, `choose_all` after phase 5's `branch`es): alike, one
  gauge; differing, E031 at each statement and nothing applied, so no order decides (#113).
  `commit_seeds` takes a held number out of a written clause (the whole clause
  where all are held) and writes it back when the hold goes.  `tests/seeds.rs::a_fix_is_its_own_seed`.
  `edit::reconcile` diffs holds per entity and field (`gauge_key`, `held_refs` over
  `program::holds`) and appends a statement built by `program::lift_gauge` with the numbers; a root choice under a key no triple spells stays the
  `branch(KEY, ±1)` statement (`StmtKind::Branch`).
  Operand order carries meaning — `arc tangent line` is `TangentArcLine`, `line tangent circle`
  is `TangentLineCircle` — and a name that is also an element keyword cannot lead a statement
  (`spline_follower.sv`'s spline is `cam`, not `curve`).
- A **chain** (Solvent §6.6) is parser-level sugar and nothing else: `horizontal line bottom(b1,
  b2) -> tangent arc a(center: c) hint(r: r) -> tangent …` desugars in `syntax.rs` into the
  ordinary statements it stands for — a prefix word (any `CKind` whose spec is one entity slot)
  becomes that unary relation, and a joint's word the relation between its two neighbours:
  `tangent` maps per pair of kinds, and any binary `CKind` (`perpendicular`, `equal_length`, …)
  is an infix spelling of itself, type-checked against the pair.  No other module knows chains.
  **Threading is a statement, not an inference** (issue #31): the `->` marker on a joint says
  its two links share a boundary point, threaded left-to-right (`p1 → p2`; `start → end`, CCW),
  and its absence says they do not — `->` alone is the plain corner (`to` is retired into it),
  `-> tangent` a corner also tangent there, and a bare word states the relation and welds nothing.
  A joint may state *several* relations (`-> equal angle(30deg)`), the marker on either side or
  both; the run ends at the first word opening the next link, so fixity sorts them.  A doomed word
  splices out where it stands (`Chained::Member`); for a whole joint doomed at once each member
  carries the joint's written word count and one-word doom (`of`, `fall`, `out_of`) and
  `edit::doomed_splices` composes the single splice for `remove` and `reconcile` alike, and
  `remove` refuses a doom set whose splices leave text that no longer parses.
  A trailing placement attaches to the line's **one** relation and is refused on a line stating
  several; where none is written the parser records the spot (`place_span`, an empty span), so
  callout writeback splices where the parser said and re-derives nothing.
  At a threaded joint the shared point is named by exactly one side (or both, in agreement), so a
  threaded `tangent` always desugars to the regular At-form (`TangentArcLine`/
  `TangentLineCircleAt`, or `Parallel` between lines) — never the bare pair, rank-deficient at
  every solution; `at:` is only supplied by a threaded joint, and an *unthreaded* `tangent` is the
  plain pair, correct exactly when the two are separate.  At a corner with a name-link the
  declared side names the shared point (`(t := line(p3, k.start)) -> tangent k`;
  `follow_building` resolves such a child when the entity's kind builds later).  Only lines and
  arcs are threaded.  `equal` is the second polymorphic word (`syntax::equal_kind`): length
  between lines, radius between circles/arcs; `Relation::poly` carries it and
  `program::constrain` settles it once the entities resolve, **before** reading the spec.  Each
  desugared statement keeps its own id and a span into the chain's text.
  **How a statement is spelled is recorded, never sniffed back out of the characters**:
  `Stmt::chained` is a `Chained` (`No`/`Link`/`Prefix`/`Joint`/`Infix`/`Member`/`Stuck`/`Close`)
  and `edit::doom_splice` matches on it — a doomed threaded joint steps down to `->`, a doomed
  unthreaded joint becomes a statement break (inside a closed chain there is no safe break:
  `Stuck`, refused), a doomed prefix word goes where it stands, and a link has *no* splice, which
  is how deleting one is refused.  Which slots a chain threads is `EntKind::ends()` in `model.rs`,
  exhaustive, so a new kind with ends stops the build.  A line ending in a joint continues its
  chain onto the next; `-> close` seals a loop, and a `close` with no marker is an error.
  The colouring asks `opens_link` — the *same* predicate `chain_starts` asks.  Both reach it
  through `past_args`, the lookahead **stepping over the operator's own parentheses**, returning
  an **index** (like `past_ref`) since callers ask both what word is there and whether the line
  ends there.  A bare name in an argument list (followed by `,` or `)`) stays untinted.  It runs
  per keystroke: put the pointer test before the registry lookup (`chain_kind` allocates), and
  `tint_word` computes the lookahead only in the arm that reads it.
  **A block body may end mid-joint** (issue #38): a *threaded* trailing joint at the body's `}`
  threads onto the next copy's first link — every pair in a `cycle` (`cycle 4 { (s := line) ->
  perpendicular equal }` is the square), all but the last in a `repeat`.  Recorded as
  `Block::joint` (an `OpenJoint`), its word statements minted through `joint_relation` with the
  right operand `next.<leaf>`, which `flatten::lookup`'s `next` arm resolves under the block's
  `cyc` (a `repeat` body still may not say `next`).  The weld is the flattener's (`weld`/`fill`):
  `builds_first` generalized to (kind, copy, statement), since `follow_building` refuses a reach
  into an unbuilt entity's implicit child.  At most one boundary slot may name its point; a
  name-link boundary is refused until #35.  A statement in a braced body ends at the `}`
  (`end_of_stmt`).  `square.sv`, `ngon.sv` are the cases; `tests/open_joint.rs` is the gate, and
  `tests/chain.rs` holds `rect_fillets`'s chain spelling to the shipped longhand.
- A statement expanded by `flatten` **keeps the id of the statement it came from**: the line is
  what a span points at, a caret lands on and a splice edits; the `path` every `Site` carries
  tells copies apart, and `commit_seeds` needs to see the multiplicity.  `Program::stmts` walks
  into block bodies; whether the *root* may splice a statement on its own is asked against
  `root().body` (`edit::in_root`).
- **`ring` solves one copy** (#96, the paragraph near the top; it was refused from #47 until it
  could).  A diagnostic carrying its own code (E041) goes
  through `Expansion::coded`, since plain `errors` are sorted into E101/E103 by message.  An
  expression's failure is an `expr::ExprError` with a `Fault` — `Dimension` (E103, §3.3:
  `distance(45deg)` is an error, never a coercion), `ClaimFree` (E040, §9.7) or `Uncomputable`
  (W110, the last number stands).  A point-to-point distance and a radius are
  `CKind::magnitude`; a negative literal in one is E040 where written.
- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- **A solid is a term, and a verb is a noun that has not been given a name** (Solvent §6.9,
  `solid.rs`; issue #48).  A solid is its **stock, plus everything in `union` with it, minus
  everything that `cut`s it**, over primitives that are faces swept — no stateful feature tree.
  Both groups are *sets*, filled anywhere in any order (P2).  A design needing the other order
  (a pocket with a boss in it) **names the intermediate**.
  **A swept solid takes features too** (§6.9, [0.31]): the first `union`/`cut`/`bound` naming one
  makes it the body over its own sweep (`program/solids.rs`'s body pass) — the name keeps its
  index, and the sweep moves to a stock of the same name (`SourceMap::also_made`), which
  `operand_paths` gives no step, so `plate.near` stays its name.
  **Nothing three-dimensional is solved for past the sketch** (axes and planes are, with the
  sketch).  `EntKind::Face` and `EntKind::Solid` own no `Param`, and every extent is an `Extent`:
  the text written and the number the *flattener* settled.  The strata run one way: the sketch
  solves, depths are worked out, terms are ordered (`solid::resolve`), outputs are read.  No
  `SpecKind` takes a face or a solid, so a 2D constraint cannot name one.  Built after every other
  kind including `Curve` (`program::solids`).
  **The kernel is the term and nothing is built or stored**: a view, section, volume, mesh and
  clearance are questions asked of `Csg` by classification (Requicha & Voelcker), memoised against
  `solid::reads` — every scalar of every edge the term reaches, each plane's pose and basis,
  every extent, and `unit`.
  **The classifier must read the facets the candidates are cut from** (against the true circle a
  bore's wall vanishes).  **A BSP prunes what a split loop cannot** (`csg.rs`): splitting every
  facet by every plane goes as the square of the facets.  **No coordinate snap grid**; `EPS`, four
  orders coarser than the solve's noise, and `same_plane` (near-coincident planes read as one; a
  facet is never split by its own plane) keep the classifier out of the gap.
  **A face is a loop walked in order**: an arc entered by its `end` is walked backwards over the
  same stretch of circle, never forwards over the rest (*how far* is the arc's, *which way* the
  walk's).
  `tests/solid.rs` is the gate and checks against **arithmetic, not a second kernel**: a block is
  `w·h·d`, a bore takes exactly its faceted polygon, a flush bore and one drilled past are one
  solid, a boss's shared face counts once, a revolution is Pappus.  `tests/solid_lang.rs` holds
  the language, `tests/derived.rs` the pictures.
- **The sheet is a report** (Solvent §6.12, `hidden::generated`; issue #48).
  `dimensions(body) in views.right` asks for the callouts that *follow from the object*: overall
  extents in that view and the diameter of every round feature seen square on.  They go through
  `callout::layout`'s pen and lanes, and their ids start at `callout::GENERATED`, past any
  constraint: a reading of the *solved* drawing, not a statement in it.
  **The boundary is the feature.**  Which datum, which fit, what is reference or controlling is
  design, which the sheet states.  `tests/sheet.rs` asserts the three it makes and no fourth.
- **A claim about a solid is judged, and can never act** (Solvent §9.8, `clear.rs`; issue #48).
  `disc clear(2mm) cyl`, `head fits(0.15mm) trap`, `piston inside bore` compile **no row** —
  `tests/solid_claim.rs` asserts parameter count, equation count and DOF unchanged.  The words
  are in `constraints::OPERATORS` and settle to no `CKind`; `constraints::solid_word` reads the
  word before the spec, like `gauge_op`.
  **What a reader is owed is the measurement**, not a yes or no, with the *sagitta* beside it: a
  claim decided within the faceting is **undecided**, a third answer.  The implicit min/max
  reading of a term is only a lower bound for a difference, so it culls; the answer is piece
  against piece.
  **`claim over crank.theta in (0deg, 360deg) { … }`** sweeps one of the drawing's **free
  variables** (never a `param`), reporting the *worst* pose; it is **sampling** at `SWEEP_STEPS`
  and says so.  Claims are read **after phase 4** (a free variable is what `expr::evaluate`
  allocates), and the interval **in the unknown's own units**, never converted as an angle.
- **A solid leaves as glTF, and that is the format this kernel's data already is** (`gltf.rs`):
  positions, normals and a named group per face — `mesh::Mesh` — in a header and two chunks, one
  JSON from `json.rs`; no ZIP, no dependency.  STEP is a B-rep, which this kernel is not.
  **Every face of every object is a named node**, the path carried **twice**, as `name` and in
  `extras`, because a loader may sanitise one (three.js strips dots; `extras` becomes `userData`).
  **Metres, because the spec says so normatively**: a `mm` document is scaled out, its unit in
  `asset.extras`; one naming no unit is written as it stands.  Exported are the document's
  *objects* (`overview::objects`) — a bore is a hole in a part, not a part — and glTF holds a
  scene, so it need not be told which part of an assembly to be.  `solventc --gltf`,
  `File ▸ Export solid (glTF)` / `(STL)`.
- **A mesh is welded, and grouped by face** (`mesh.rs`).
  **Welding** is a T-junction fix and not a vertex merge: `weld` puts the vertices lying on an
  edge's interior into it.  A hash **cell** is sized to the object (`scale / 128`) while the
  tolerance stays `1e-9`; never conflate them.  `triangles` fans **from a corner where a piece has
  only corners, and from the centroid where it does not** (a vertex fan's zero-area sub-edges,
  dropped, re-open the T-junctions).  The test is per piece: has this polygon a vertex that is
  not a corner?
  **Grouping** is `mesh::grouped`: triangles in face-path order, a normal per vertex — the
  facet's own where flat, averaged where the face is `smooth`.  Across the ABI as **buffers plus a
  small table** (`gcs_solid_mesh`, `gcs_solid_normals`, `gcs_solid_faces_json`), all reading one
  memoised `Want::Mesh`.  `Document.solids()` names them (a solid has no proxy).  `tests/mesh.rs`
  is the gate and asserts the *before* as well as the after.
- **A mesh is cut to the object and a volume to the report** (`solid::mesh_unit`, `MESH_SAGITTA`).
  `REPORT_UNIT` makes a *volume* good to 1e-4; a mesh is cut to a sagitta that fraction of the
  *solid's own diagonal* (scale-free), asked of the primitives' boxes.  `gcs_solid_mesh_unit`
  publishes it, and **a unit at or below zero *is* that choice**, resolved at the one seam
  (`Sketch::cut_unit`, read by `solid_boundary`, `solid_edges`, `solid_mesh`) — never per entry
  point (a sagitta of zero hangs the tab).  `tests/mesh.rs` asks it of all three.
  **The glass box asks with 0, and that is why zooming is free**: `unit` (a screen pixel) is
  wrong for a scene with its own camera.  `overview::scene3d` asks the same way (`SCENE_PX` for
  drawn polylines, `0` for the object's edges), so `Box3D`'s rebuild key omits the zoom.
- **A face is one loop and a solid's faces are named by path** (§6.8).  A face is a closed loop of
  edges on the one plane every point agrees about — *read* off the memberships, never written on
  the face.  There are **no holes**: a hole is a solid that `cut`s the body.  An `in` block leaves
  a face and a solid alone, as it does a datum — they bear no points, and refusing them would
  split the design and the solid it is a section of across blocks.
  A boolean **never renames**: `block.near`, `block.far`, `block.side_l` (a prism), `bore.axis`,
  `bore.start` (a revolution), and through operands — `body.bore.wall`, `part.base.pocket.floor`.
  A name whose face a boolean ate is a *fact the report carries*, never an error.
  E080 a face that is not a loop on one plane, E081 a revolution's axis, E082 a face a body no
  longer has, E084 a section cut across its own view.
- **A gesture writes a solid** (#162 F0): `edit::add_face`, `add_solid` (`SolidSweep`, extents
  as text into `Arg::Dim`, mixtures refused by the parser's own `sweep_of`) and `add_body_word`
  append over names (FFI `gcs_elab_add_face`/`_solid`/`_body_word`, `Document.addFace`/
  `addSolid`/`addBodyWord`), each **held to the elaborator before it is handed back**
  (`append_checked`: an error inside the appended text refuses the edit in its words; one
  elsewhere was there before the gesture).  A hole loop that is not one circle is written as a
  face first: **a face is a hole as it stands** (`holes: h`, its outer loop; one holed itself, or
  declared after, is E080).  `edit::remove` dooms a body word or a sweep naming a gone solid.
  Fresh names pass every bound name (`taken_names`).  `tests/edit_solids.rs` is the gate.
- **A chain is a named traversal** (§6.6; issue #49, item 3). `profile := line -> … -> close`
  records a `NamedChain` beside the usual desugared declarations and constraints. The flattener
  scopes the binding and its links like ordinary component members; edges stay `boss.ab`, the
  grouping is `boss.profile`. Closed traversals build ordinary faces and can be swept directly.
  Open traversals retain their ordered references for `face(trail, -> close)` and are refused
  as direct sweep sections. All geometry validation stays in the existing face/solid code.
  Source edits preserve the expression; deleting its geometry is refused under the existing
  chain rule. Canonical flat printing refuses named chains atomically and retains source,
  as it does for components. `tests/solid_lang.rs` and the browser binding test cover this.
  `repeat e in CHAIN { … }` (and `cycle e in`, refused on an open chain) makes a copy per link,
  `e` an alias to that link in each copy (`#<id>.<k>.e`); the count is the chain's, so the block
  is set aside where it stands and expanded once the chain resolves (`flatten::expand_pending`),
  through instances and group formals alike. `tests/chain_edges.rs` is the gate.
- **A face closes itself** (§6.8, `program::build_face`; issue #49).  The brackets hold a *walk*,
  and an item may be a **point**, with `-> close` sealing the run back to the first item.  Where
  two neighbours do not meet, `build_face` mints the straight line between them, class
  `.closure` (hidden by the base sheet): `pist_f := face(crown, pL0, pL1, pL2, pL3, pL4, s0.p,
  -> close)`, `hole_f := face(x0.p, x1.p, x2.p, x3.p, -> close)`.  A minted run names a face
  of what is swept (`close0`, `close1`, … in mint order, skipping existing edge names).
  **The shorthand may not swallow a mistake**: an interior gap between two *edges* is still E080
  (`face(ab, cd, bc, da)` must not become a bowtie); **an edge takes its direction from a
  neighbour it actually meets**, so one between two gaps is refused (`bad := face(a, bc, d, ->
  close)`; name the corner: `bore_f := face(m0.p, b_br.p, bore_r, hx.p, -> close)`); and the wrap
  is minted only under `-> close`, so "the loop closes" stays something the source states (on a
  loop that already meets it mints nothing).  `tests/solid_lang.rs` is the gate.
- **A stack is planes and distances**: parts standing on one another are drawn in parallel
  planes, `P distance(d) Q` with an axis square to both through both origins (`against`, placed
  planes and E083 are gone, #81).  `hardware.Groove` states the O-ring rule once (10–20% squeeze,
  a groove a third wider), so `dims.sv` derives `grooveb` and `groovew`.
  **A component contributes a `cut` to a body it was handed**, which is the body rule being a
  set and not a sequence: the feature owns the void it cuts.
- **A part carries no views; a sheet asks for them** (§6.11, `hidden.rs`).  `view(body) in
  views.right` and `section(body, at: swing) in views.front` are *outputs*: no `Int` draw flag, no
  `repeat draw_side { … }`, no second copy of the geometry and no `project` to keep in step.
  Three draughtsman's rules.  **A corner is drawn and a tessellation seam is not** — a `smooth`
  seam is drawn only where it is a *silhouette*.  **What the material covers is dashed, not
  dropped** — the eye's ray is classified against the term and the piece carries `.hidden`.
  **Coincident page lines are drawn once, visible winning** — an *interval* rule, not a segment
  one (a rim seen edge-on folds onto itself, splitting at different places): every stroke is laid
  on its line, the visible stretches unioned, the hidden what is left over.
  Everything comes back in **the plane's own coordinates** (`PageFrame` over `Basis::view_coords`),
  so a derived view sits where a hand-drawn one tied by `project` would — `tests/derived.rs`'s
  strongest gate.  **The core projects and the front end strokes**:
  `hidden::layout` resolves the ink through the sheet, `svg::render` and `paint.ts` stroke what
  they are handed, and neither owns 3D arithmetic or a rule about hidden lines.
  Derived views are the `.svd` paper's; the model workspace shows a solid as its mesh.
  **Every part of the V-twin is written this way** — `vtwin/components/cylinder.sv`, `piston.sv`,
  `disc.sv`, `flywheel.sv`, `throttle.sv` and the plate in `frame.sv`: one section and its solid.
  **Where a part's turned features are is where its section has to be**: a turn about a line in
  the section puts what it makes *on* that plane (the disc's and plate's sections are mid-planes).
  A **hex pocket about a radial line** (`parts.Grub`) is neither sweep nor turn, so it stays four
  hidden lines; a feature drawn as a *centreline* (the exhaust vents) is told its width, `wch`.
- **The workspace** (`overview/workspace.rs`, `app/view.ts`) is the only model canvas: one scene
  in space, every sketch on its own plane, solids and panes drawn beneath by three.js
  (`app/box3d.ts`).  A view's page reaches the eye's picture plane by **one affine map**
  (`workspace::Projection`: `Basis::lift`, the orthographic `overview::eye`), so
  `camera.ts` composes it with the eye's similarity (`Camera::through` → `ViewCam`) and no 3D
  arithmetic exists above the ABI.  **What is under the pointer is asked where the eye sees it**
  (`workspace::pick`, `nearest_point`, `inside`, `callout::pick_seen`): two views' coordinates
  overlap but are apart in space, so nothing is picked in view coordinates.  `w2s`/`s2w` read the
  view being worked in — the painter's (`inView`), else the current plane's; a drag reads the
  dragged point's view, a callout its dimension's (`calloutView`).  A view seen edge on refuses a
  press (`ViewCam::readable`).  The plane tool picks two drawn lines and writes `plane(u:, v:)`.
  Points in space stand in no view: seen where they are (`Projection::point`; per frame
  `workspace::space_points`, `gcs_workspace_space_points`, stroked through the eye's own
  camera), drawn and picked, and dragged where the eye sees them (`PlanDrag::seen`, numeric:
  `CKind::DragSeen`, two soft rows across the picture plane, so depth is the constraints' and a
  free point keeps its own).  A view opens drawing on `std.front`
  (`drawOnFront`), and File ▸ New is `use std`.  `workspace::Views` is what does not depend on
  the eye — each view placed in space (slot 0 a 2D sketch's front plane, `Basis::page`), the view
  each point and entity stands in (a curve, owning no point, its tool's: `Sketch::curve_view`), and each view's **place** (the first view on the same plane in
  space: `std.front`, `std.up`)
  — read once an edit (`gcs_workspace_json`); the maps are per frame (`gcs_workspace_maps`, a
  buffer).  The front end compares place ids, never maps.  The chooser in the viewport's upper right (`#plane-select`) lists `std.front`,
  `std.side`, `std.top`, then the document's planes; `choosePlane` sets `v.plane` and swings the
  eye square on (`workspace::look_at`).  A standard plane the document lacks is `pendingPlane`
  until a tool's first press adds `use std` (`edit::add_use`, `ensurePlane`) — choosing writes
  nothing.  A double-click on a pane (select tool, nothing drawn under the pointer) chooses its
  plane without turning the eye (`choosePlane(name, false)`): `workspace::panes_at` lists them
  nearest the eye first, `paneAt` takes the first the chooser offers.  Callouts are drawn for the
  current plane's place only (`showsCallouts`), and the one being written or focused.  Right-drag
  orbits, middle or ⇧right-drag pans, the wheel zooms.  A flat document opens square on to the
  front, one with a solid or off-front geometry from three quarters (`homeOrbit`);
  `workspace::bounds` frames figures and solids.  The orbit is view state (never saved, exported,
  solved or undone).  `tests/workspace.rs` and `app.test.ts` are the gates.
- The **glass box scene** (`overview.rs`) folds a multiview drawing into space: each view on its
  own plane, the object reconstructed between them.  **Nothing is solved for and nothing is
  stored** — a point in view P has view coordinates `(a, b)` (what `project`'s residual reads),
  sits at `a·u_P + b·v_P` (`Basis::lift`), and a corner tied by `project` into
  two non-parallel views is four rows in three unknowns, exact *because* the projection holds.
  "Non-parallel" is `overview::RCOND`, about a degree (past it 1/σ₃ flings a corner); `validate`
  refuses only the exactly-parallel pair.  A corner is a **pair of images and never a transitive
  class** (merging `Ff project Fa` with `Ff project F2a` collapses the object), its images
  **ordered by plane**, never by the statement.  An object edge is one both views draw, deduped by
  its **3D segment to a tolerance** (`SAME_POINT`).  Where a point *stands* is `overview::view_of`
  — its membership.  A line stands with each end where that end is.  `overview::drawable` is the
  per-kind polyline walk `svg::entity` and the workspace share, refined to `curve::flatness`.
  **Every plane is a pane**, drawn in or not; `overview::pane` is the *one* rule for its reach
  (geometry and origin, grown a little, never thinner than `LEAST_SIDE`), so face and axes agree.
  **The box shows the objects, not the features they are made of** — a solid is the object exactly
  when nothing else is made of it (`overview::objects`).  `box3d.ts` draws what `scene3d` says
  stands in space — panes, axes, the object's creases (a `smooth` seam dropped: shading draws the
  round) and the axes — and every object's mesh
  (`mesh::grouped`), computing no coordinate; sketches on planes are the 2D canvas's, stroked over
  it.  Its camera is set from `v.orbit` and the eye's camera; the current plane's pane is bold, a
  material write per frame.  **`⇧⌘B` toggles the solid's surfaces** (on by default; off is a
  wireframe).  The flat shaded path (`Part::Shell`, `overview::shell`, `Item.shade`) is what
  `overview_json` offers a front end without a depth buffer.
- Slow tests are gated by `#[ignore]` (cargo). The **slow tier** — ten seconds to minutes, still
  passing on every landing (gear whole-member verifications, a native tooth space) — is
  `#[cfg_attr(not(feature = "slow"), ignore = "slow tier, …")]`, run by `make test-slow`
  (`OCCT=1` for the native one); `make test` leaves it out.
- **Measure compile time and test time separately**
  ([`docs/build-performance.md`](docs/build-performance.md)). `make test` builds both release
  artefacts and runs every integration suite and doc test. Release: incremental ThinLTO, 16
  codegen units; native test profile: opt-level 2, debug assertions, no debuginfo. The core suite
  is one binary (`gcs-core/tests/main.rs`); libraries and binaries have `test = false`, so an
  in-source unit test needs that target's harness enabled again. The TypeScript cache lives in
  `web/dist`, so removing the output removes it.
  Unpacked debuginfo in `target/debug/deps` makes macOS launches slow: keep `debug = 0`;
  `cargo clean --profile dev` clears an existing accumulation.
- Benchmark on a quiet machine (`uptime`; a JVM indexer often runs here).  Native:
  `rust/gcs-core/src/bin/bench.rs` (`cargo run --release -p gcs-core --bin bench`); wasm: `npm run
  bench`; `make bench` runs both.  Wall-clock medians only — no harness dependency.
- The front end is two layers and they are kept orthogonal.  *Geometry* is the core's: a click
  picks by `workspace::pick` (what is *drawn*, where the eye sees it, as against `point_to`, the
  idealised entity a dimension means), callouts by `callout.rs`, curves by `curve.rs`.  *Linear
  algebra* is the front end's, and the whole of it is `app/camera.ts`: the eye's similarity
  (uniform scale, translation, the y flip) and a view's affine composed with it (`ViewCam`).  A
  tolerance travels as an eye length (`PICK_PX * unit`), never pixels.  Nothing outside
  `camera.ts` multiplies by `scale` or writes a minus sign in front of a y, and nothing in `app/`
  measures a distance to an entity itself.
- Dimension callouts (`callout.rs`) are geometry: extension lines, heads, leaders, arcs, the
  label's box and the hit test are laid out in the core, the front end only strokes.  Sizes are
  screen-constant through `unit` (world length of one pixel), as is the pick tolerance.  Where a
  callout sits is a *placement*: two numbers in a frame following the geometry, automatic until
  dragged, then `Sketch.placements` document state **saved on the statement it qualifies** —
  `at (t, r)` after the dimension, `"place"` in the constraint's JSON.  **The sheet takes
  everything it shares** (issue #16, §13.1): `style .dimension` / `.reference` / `.extension` own
  ink, weight and dash (`paint.ts` asks `styleNamed`, holding no callout ink).  Never by position
  in a list, entity index, type-and-arguments selector or minted id — each fails silently or
  collides.  A placement goes with its dimension (`Sketch::remove`; in the source, the splice).
  `io::from_json` *reads* the old position-keyed `placements` table and never writes one.
  The stated number comes from `io::dimension_text`; a bare number is read through `io::reading`
  (`READING_SIG`, six digits) for the callout, `arg_text` and `describe`, since `syntax::num` is
  the *source* printer.  A literal naming its unit (`60deg`) is drawn as written with no second
  sign (`expr::names_unit`), but one past six significant digits reads at six in printed text
  (`io::read_literals`): the flattener wrote it (`flatten::fold`: `13.333333333333334mm`).
  `callout::pick` owns what outranks a callout: a point within the tolerance beats its lines, but
  not the number's box, filled solid.
- One straight dimension figure, `Pen::linear`, draws them all: it measures along a *given*
  direction, puts a head where each point falls on that line and runs an extension line out to
  each point.  `Pen::aligned` is the case where the direction is the pair's own; a run or a rise
  passes a page axis instead.
- A dimension between two points is three — `Distance`, and an `Ordinate` `along: x` or `y` —
  *stated by where the number is put*: `callout::pair_dimension` (`PairDimension`) picks the
  line (pair's own, page x, page y) nearest the direction from the pair's middle to the
  placement; bisector borders, no threshold, a tie to the length.  The front end asks
  (`gcs_dimension_pair_kind` → 0 length, 1 run, 2 rise) and swaps the constraint as the pointer moves
  (`SketchView.startDimension`, where a dimension is written: stated at once, number edited on
  the drawing).  Nothing reaches the undo stack until the number is accepted; Escape takes it out.
  *Nothing is solved while it is carried*: `afterEdit` skips the solve while `liveDim.placing`;
  `placeDimension` (planting click, or release of a later drag) is the one solve.
  The run and the rise are signed from the first point to the second (`(qx - px) - d`, a constant
  Jacobian), so a front end orders the pair to read positive.  In `cgraph` they are `unsupported`
  on purpose: no cluster element for the line they are measured from.
- A dimension's number may be an *expression* (`expr.rs`): `Arg::Expr { text, value }` in a
  `Length`/`Angle` slot, `value` in arg units (radians) for the kernels, `text` as written
  (degrees).  `w = 80` names a value, `h = w / 2` reads one; `expr::evaluate` is a Kahn walk of
  the graph (earliest first among the ready), reporting per expression its name, deps and error —
  defined twice, undefined, a cycle, a non-number.  One that cannot be computed keeps its last
  number.  Trigonometry is in degrees.
  A mixed fraction — `3 1/2` — is one `Num` from the tokenizer (`31/2` and `1/2` are divisions).
  `expr::literal` does *not* claim it; `expr::notation` marks it notation, so `arg_text` prints it
  as written where a *formula* prints text and value (`h = w * 2 = 80`).  A **callout carries the
  expression** (`io::dimension_text`), a `param` included: the flattener settles `distance(w)` to
  `100`, so the spelling is read back at the argument's span into `Constraint::written`
  (`program::relations::written`), presentation only, dropped by any write of a number — at the
  root and in a same-file component body (`design.module`).  A module's body and a block copy draw
  the number; `program::relations::repeated` marks later copies drawing the same label
  (`Constraint::repeated`), left out of the full layout, drawn when requested by id.
  `expr::set_dimension` is the one write path for text (a bare number becomes `Arg::Num`, angle
  converted — the app converts nothing); `Sketch::add` and `io::from_json` evaluate;
  `Sketch::set_constraint_num` writes a number and re-evaluates when it dropped an expression.
  Documents save `{"expr", "value"}` and accept a bare string; binding records keep the number in
  `args`, the text under `exprs`, and proxies `sync()` before handing out a value.
- **Every number has a dimension, and it is checked** (`units.rs`, Solvent §3.3).  Two bases,
  **length** and **angle**, with *rational* exponents (`sqrt` halves one).  `*` and `/` derive,
  `+` and `-` demand agreement, `^` takes a whole power on a dimensioned base, and the result is
  checked against its slot (`SpecKind::dim()`).  `Aff` carries the dimension beside the value.
  **The asymmetry between `Dim::fits` and `Dim::agree` is the design.**  A *context* (a slot, a
  function's argument) may take a bare number; two *operands* may not mix: `90 / N + ivp` is an
  error, `90deg / N + ivp` the answer.  A **name** is worth a number (`w = 80` in a Length slot
  does not make `w` a length); a literal's unit and a formal's declared `Ty` *do* travel, so the
  formal catches `x := w + phi` after `flatten::settle` substitutes a parameter away.
  A literal's unit is converted **to the document's own** by the tokenizer (`expr::parse_in`
  takes `Units`): `unit mm` names it, and without one a suffix is refused.  **Feet-and-inches is
  one literal** — `1' 6 3/16"` — so there is **no string literal** (`"` is the inch mark, a `Str`
  argument and a raw branch key are bare words).  `pi` is dimensionless and `tau`/`turn` a
  **turn**, so `tau == 2 * pi * 1rad`; radians are written `* 1rad`, not `* 180 / pi`.  Storing
  the unit costs the solve nothing, and `io::paste` converts between documents' units
  (`Sketch::rescale`, written out by kind: a `Param` cannot say it is a length).
- **A number is named one way, and an unknown is declared** (issue #77; Solvent §3.4, §5, §6.3,
  [0.41]).  `w := 60` is a value; `param w := 60` (`ParamDecl::input`, a modifier the parser reads
  in `statements.rs::input`) marks one of the document's **inputs**, the only lines
  `examples::with_params` rewrites; `param beta: Angle hint(30deg)` with no value is an
  **unknown**: `flatten::values::params` puts `free(name, ty)` in `vals` (as `bind` does for an
  unbound formal) and records a `model::Declared` (dimension, seed) in `Expansion::unknowns` →
  `Sketch::declared`, which `expr::evaluate` reads (the seed, skipping `settle`; the declared
  dimension, checked in `check_dim`).  An unbound
  `param` states its type (`Length`/`Angle`/`Scalar`), takes the one keyless `hint(E)`; a seed on
  a bound one, an `Int` unknown and a module's unbound `param` are E040; `param` is refused by the
  parser inside a component (formals are its inputs) or block.  A call seeds a formal it leaves
  unbound with `InstVal::Hint` (`Wing(f, beta: hint(15deg))`).  **A name nothing declares is
  E101** — in a dimension (`settle_text`), a pin (`settle_arg`), a dotted read nothing made
  (`resolve`, unless it is an instance's unbound formal) — and the relation is not emitted
  (`Walk::refused`), so it never reaches the expression graph as an unknown; W111 is gone.  A
  dotted name read in a body is made absolute under `Scope::instance_prefix`.  `t == s` over an
  unknown is `Arg::Tie` seeded by the declaration (a `hint(t:)` beside it is E040 in `assemble`).
  `commit_seeds` writes each declared unknown's solved value into its `hint(…)`
  (`edit::unknown_seeds`); `to_program` declares a sketch's unknowns (`lift::unknowns`);
  `edit::set_dimension` over a dimension reading a bound root `param` writes the param's line; a
  drawing's `dimension m.w` is the first top-level dimension written `w` (`render::written_as`).
  `tests/names.rs` is the gate.
- At the **sketch level** (JSON documents, the bindings) an expression reads names and defines none:
  every name it reads is a **free variable** (`expr::Free`) — the form a declared unknown takes once
  flattened.  It is an unknown of the sketch tying the dimensions that read it.  `expr::evaluate`
  (document order; no definitions, so no dependency graph) allocates one Param per free name into
  `Sketch::free_vars`, retires it to `fixed` when unread — keeping the *slot*, so the parameter
  count `topology_key` ends with does not move — and rewrites every binding (`Constraint::free`, at
  most one) from scratch each run.  It runs on `Sketch::add`, `remove`, `set_constraint_num`,
  `set_dimension`, `from_json`, `report::exprs_json`; a whole document uses `Sketch::add_quiet` and
  evaluates once (`io::graft`, `io::from_json`), since per-add evaluation is quadratic. The tie is
  **affine in one free name** — `a`, `a / 2`, `2 * a + 5`, `value = m*a + c`, all a fixed-width
  block carries; `expr::eval` works in `Aff`, so `a * a`, `sin(a)` and two free names are errors by
  arithmetic, and an erroring dimension keeps its last number. Every `Length`/`Angle` type needs a
  *free twin* kernel (one more column, (m, c) as constants) in `CKind::free_kernel`, exhaustive;
  `every_dimension_can_be_written_free` checks it.  `params_on` appends the free column (always
  last), `consts_on` returns `[m, c]` turned by `side:`, `along:` or `sense:` (a skew distance's by
  its seed's side), `kernel_id` picks the twin.  A fresh one seeds from the stated number, else by
  Newton on its row (`expr::settle`).  A free dimension is `unsupported` in `cgraph`, never jittered
  by the witness, part of `topology_key`, and joins `io::Part`'s walk.  `expr::sync_free` updates
  the numbers shown from every seam writing parameters directly — `Sketch::set_x`,
  `io::Part::write_back`, the wave's writes.
- The page is the drawing *and the source it is written as*: the program panel is a permanent
  second child of `<main>`; there is no other sidebar.  A component selected names itself in the
  status line (`describeEntity`) and brings up one floating window listing the constraints that
  reach it, and only those, open on a `subject` — the selection, else what it was last opened on,
  so focusing a constraint does not pull it away.  `openPanel` is how a clicked callout or the
  banner's culprits pick without selecting; `closePanel`, wired to `onSelect`, is a press on
  nothing.
- **The colouring is the parser's own scan** (`syntax::highlight`), not a second lexer in
  TypeScript, so a colour and the parser cannot disagree.  `Tint` names the classes and the
  stylesheet says what they look like; the front end writes one element per run, parsing nothing.
  A function of the *text*, not of an `Elaborated`, since the program is usually half-typed.
  A `.svd` is coloured by `drawing::highlight`, the drawing lexer's own scan (`gcs_drawing_highlight`,
  `core/drawing.ts`); the panel picks one by the file shown (`CodeEditor.colouring`).  Relation
  words are known by the text: those it defines or names in a `use (…)` (`relation_words`); units
  join their number's run; `expr`'s functions, constants and measures are `builtin`.
- Offsets cross the ABI in **UTF-8 bytes** and index a **UTF-16 string** on the other side
  (`gear.sv` has an em dash).  `core/program.ts::Offsets` is the conversion and `Document.adopt`
  is the **one seam** every report crosses: diagnostics, source map and coloured runs are string
  indices by the time anyone sees them.  A wholly-ASCII program builds nothing; otherwise it is a
  binary search, since offsets do not arrive in order.  Nothing sends an offset back.
- The box it is typed in is `app/editor.ts` — a `CodeEditor`, which knows nothing of Solvent and is
  handed the text and a function saying which runs are what: a `<textarea>` over a `<pre>` of the
  same text.  **The two layers must put every character in the same place**; shared CSS is not
  sufficient.  Four rules: the line height is a whole number of pixels; kerning and ligatures are
  off; a run may change the colour and **never the face**; and the copy is **translated** to
  follow the box, never scrolled (the box's scrollbars shorten its range).  `npm run overlay`
  (`web/tools/overlay.mjs`) drives headless Chrome against the *real* `CodeEditor`: metrics agree,
  colouring moves no glyph, the copy follows to both ends.  Not in `npm test`: it needs Chrome,
  and `make test` must not.
- `SketchView` holds a `Document` (`core/program.ts`), not a `Sketch`: `view.sketch` is what the
  source came to and `view.source` the document.  **A new document is one verb**,
  `SketchView.load` (File ▸ New is `newDocument`; Open and a test case too): the outgoing text is
  one undo step; `settle()` is the one list of what is in flight — a gesture, an animation, a
  carried dimension, a tool's half-collected clicks, the remembered scene — cleared before any
  swap and before the box.  `setProgram` is the *edit* of the same shape.  Undo is program text,
  comments and all.  A selection crosses a re-elaboration **by name** (`Document.nameOf` /
  `Document.entity`); a proxy dies with its `Sketch`.  `swap` is the one seam that replaces the
  drawing, and disposes the outgoing document.
  The source catches up at exactly two seams and **never per frame**: `syncSeeds` at the end of a
  drag (`gesture::endGesture`, guaranteed numeric) and `syncSource` at the end of `afterEdit`.
  The panel is wired to `onProgram`, never to `onDragFrame`.
- A drag is an operation on the dragged point's *part* of the document (`io::Part`): what is
  reached through shared points and constraints, stopping at fixed entities.  `PlanDrag` builds its
  plan, systems and numeric fallback on that part alone and writes each frame back, so a drag
  costs the figure, never the document; anything exchanged by point index (guards, flips, branch
  keys) crosses through the part's maps.
- A drag made `PlanDrag::on` the document's own `PlanSolver` (`view.plan()`, cached per topology
  and pinned for the gesture) starts with one pass over the residuals
  (`PlanSolver::ensure_solved`) and runs on the document itself; `PlanDrag::new` builds a plan
  over the part.  The plan comes back with every `move_to`/`guard_triangles`/`branches` (`None`
  for a drag of its own), and `part` is `None` exactly while the drag moves the document directly.
- Within the part, a frame costs the *region*: `decompose::Wave` moves the plan's roots as rigid
  bodies — those holding the dragged point, growing by shared elements only while the cursor is
  out of reach — with every element shared with the rest an anchor and every direction class the
  rest carries pinning a rotation.  Pull (cursor row) then polish (anchors only, min-norm from the
  pulled pose) on the tiny merge system, rotations about each body's centroid scaled by
  `TURN_COST` × its gyration radius so a free body slides rather than spins.  The wave keeps its
  poses and never re-reads what it moved, so nothing compounds; it starts solved (`PlanDrag::new`
  solves first) and hands over to the numeric `Drag` only for unsupported constraints, a region
  past `WAVE_MAX`, or a body solve that will not converge.
- In `decompose`, a direction class is a *relation*, not an *adjacency*: merge candidates, the
  worklist refresh and the core frontier come from shared elements (`neighbours`), and the class
  counts toward rank once on the table (`pair_rel`, `relation_bound`) — as adjacency, one
  `Horizontal` class made every cluster a neighbour of every other.  `relation_bound` must stay an
  upper bound on the merge rank: validate a change by forcing the factorisation on every call and
  asserting `rank <= bound` across the cases.
- **The ellipse is a library component** (issue #47, item 4): `std.Ellipse(c: point, a, b,
  tilt, u)` in `rust/lib/std.sv`, a computed point at eccentric angle `u` about `c`, its major axis
  turned `tilt`, traced as a curve — so
  `p coincident e`, `e tangent l` and `e curvature k` are the curve contacts, exact to third order;
  the entity kind, its kernels and `CKind`s are gone from every arm, the FFI, the binding and the
  app (an ellipse *tool* is a follow-up).  The parser keeps the word only to refuse it, naming the
  spelling; `io::from_json` refuses a document carrying the old `"ellipses"` table.
  `tests/ellipse.rs` holds the rim, the tangent and the osculating circle against closed forms,
  and the rim turning with its tilt.  An axis is a value the curve takes — stated or a `param` —
  since a component of one computed point cannot be drawn as an instance whose formal is left
  free.
- **The sphere, the cone and the cylinder are library components too** (see the paragraph near
  the top): FFI kind ids stay contiguous, the axis now 16.
- **A curve is a point of a component, as one of its numeric formals runs** (Solvent §6.5).
  No curve family: `path := leg.toe over theta in (0, 360)` asks a *drawn* instance, and
  `e := Involute(base, phase: a0).p over u in (u0, u1)` one written in place and never drawn.
  `syntax::CurveSpec` is the statement (`CurveTarget` `Drawn`/`Anon`, swept formal, interval);
  the flattener records every instance it binds (`flatten::InstanceInfo` — prefix, component,
  absolute actuals, numbers) and resolves a drawn target onto the instance owning the longest
  prefix, so `build_curve` never re-derives it.  `program::compile_curve` picks the body: a
  **computed** point, `p := point(x: xexpr, y: yexpr)` (`Decl::computed`), compiles to two
  `tape.rs` tapes, and a component with one is refused on the sheet; any other point is a
  **locus**, lowered by `compile_trace`.  Either way the tapes are differentiated forward in the
  swept formal *and* every coordinate they read (`∂C/∂θ`), or a point falls off when the circle
  is dragged.
  **The body is expanded by the real flattener, symbolically** (`flatten::expand_component`,
  `Sym`): every numeric formal is a *free value named after itself* (the `Aff` an unbound
  formal is on the sheet): `substitute` writes it back by name (or `(m * name + c)`), `settle`
  keeps a dimension with no number, and `Walk::keep_text` keeps a text nothing can work out
  (`sin(u)`) as text where it is read (`Sym::texts`, keyed by absolute name, looked up through the
  scope's prefixes).  A nested
  instance's *own* unbound formal is its own unknown (`#c12.i.u`), no column of the curve, never
  captured by an outer formal of the same name (`tests/curve_of.rs`).  One expansion per
  `(component, point, formal)` — `CurveDef::key` — shared by every instance.  `Walk::owner_of`:
  the innermost drawn instance owning the prefix *whose component has the swept formal*, handed
  on as `CurveSpec::of`.
  The variable table is the swept formal, then the entity formals' scalars **in `entity_params`
  order** (`EntKind::scalar_names`), then the numeric formals a drawn instance left unbound
  (`CurveDef::columns`, keyed; `CurveE::unknowns` names them in `free_vars`), then the other
  numeric formals (constants) — `params_on`'s column order, so a tape's gradient *is* a Jacobian
  row.  A whole turn of an `Angle` formal that comes back is closed (`Sketch::curve_closed`): a
  contact wraps across the seam (`clamp_contacts`).  A dimension reads geometry (`c.r`) only in a
  trace body; elsewhere E103 (`Walk::dim_reads`).  `EntKind::Curve` is the one kind whose children
  need not be points, built and grafted **last**.
  A curve's kernel belongs to its **definition**, not its type (definitions differ in width).
  `CKind::kernel()` panics for the three curve kinds, `kernel_id_in(sk)` returns
  `N_KERNELS + 3·def + slot`, and `System` owns the static kernels plus **three per definition**
  — `PointOnCurve`, `CurveTangentLine` (`inv tangent l`) and `CurveCurvature`
  (`inv curvature k`); the registry publishes `kernel: -1` for all three, which the bindings'
  tests key on.  The tapes ride in `consts`, so `KERNELS` stays `'static`.  Tangency and
  curvature need the **frame** (`kernels::CurveFrame`): `C` to `C'''` and the gradient of the
  first three orders in `[u, θ…]`.  A formula gives it exactly (`tape::eval_series_flat`,
  `tape::Series`, checked in `tests/tape.rs`).  A trace gives `C`, `C'` exactly and, asked
  (`need`, `Val::orders`), `C''`, `C'''` exactly: **Taylor orders of the implicit function**, one solve each
  with `finish`'s factorisation (`locus::higher_orders`; Wagner–Walther–Schaefer), the rows read
  over `taylor::Jet`s — each kernel's **Taylor form** (`taylor::form`; affine kernels by their
  `J`), held to its kernel by `tests/taylor.rs`.  Gradients along θ are a **forward difference**
  from the memoised centre (`locus::kernel_frame`: one warm block solve per column from the
  remembered pose, so the branch cannot change); along `u` the exact orders.  **A residual never
  builds the frame**: `curve_value` gives derivatives alone, only a Jacobian pays for the
  gradient (the `EllFrame` bargain).  A trace with a row lacking a form
  (`Locus::without_form`) gives no `C''`: `constraints::validate` refuses curvature naming the
  kernel and its slot is the `refused` kernel (rows NaN, not converged).  In the plane a
  generated profile is such a trace (normal through the instant centre; `tests/generation.rs`).  `CKind::family_kernel` (`FamilyKernel`: discriminant = slot, knows its row count)
  is read by `kernel_id_in`, `n_residuals`, the registry and `kernel_table`, so a fourth kind is
  one arm.  `Sketch::curve_polyline` is memoised against everything it reads (picks walk every
  curve per pointer move).  `tests/curve_contact.rs` holds the contacts against the
  involute's closed forms; `tests/common` is the shared finite-difference Jacobian check.
  **An unbound numeric formal is an unknown of the drawing.**  `leg := Leg(axle, pivot)` without
  `theta` binds a *free* `Aff` named `leg.theta`, one per instance; `value_aff` passes it
  (refusing a name nothing binds), a `param` over it is affine in it, a nested instance shares
  it, and `substitute` writes the name into dimensions (`expr::Free`).  The curve's anchor
  follows it: `CurveE::home` is `Home::Free(name)`, read by `Sketch::curve_home`, since the
  unknown is allocated after the curve is built.
- A **locus** (`locus.rs`) is what a traced point is: the curve is wherever the body's
  constraints put `p`.  The body is lowered once (`program::compile_trace`) through a scratch
  sketch and `Constraint::params_on` — never a second column mapping — into static-kernel rows
  over `[u, θ, values, q, w]`: `q` the body's coordinates, `w` a dimension over `u` computed by a
  tape and read by its **free twin** kernel (`(m, c)` the unit conversion), so no new derivative
  code exists.  `C(u)` is a small damped Newton solve, its derivatives the implicit function
  theorem from one factorisation.  The body encodes to flat `f64` in the contact's consts
  (`locus::eval_at` the one evaluator, `eval_flat` its cold entry); `System` picks
  `point_on_body_kernel` over `curve_kernel` per definition, bindings untouched.  A trace contact's
  constants are `[anchor, n_values, values…, has_pose, flat…, pose…]` (`kernel_eval` reads,
  `consts_on` writes, `kernel_table` sizes; `view` ignores what trails): for a **drawn**
  instance the **pose on the sheet**, read at every compile and refresh (`CurveDef::pose_of`,
  `CurveE::pose`, `Sketch::curve_pose` the one reader, `model::whole`: a pose with a hole is no
  pose), where the anchor solve starts (`locus::Anchor`).  The polyline sweep walks **outward
  from the anchor**, down to `u0` then up to `u1`.  Chirality is a *branch*, which no regular
  residual can state.  Three instruments, by strength (spec §6.5): a **signed constraint**
  (`point_line_distance`); an **orientation predicate** — `ccw(a, b, x)`, no residual, enforced
  only at the **anchor** by reflect-and-resolve, with deterministic restarts (fixed-seed
  `rng::Rng`, scaled by the entity formals' coordinates and *nothing else*) when there is no
  pose; and a **seed**.  Continuity carries the branch elsewhere — one warm-started march, and a
  body with predicates never trusts a direct solve *from the seeds* at the target.  A carried
  branch is **kept**: each contact's pose is remembered and the next evaluation *resumes*,
  trusted only as far as `locus::continues` checks it against the tangent `∂C/∂(u, θ)`
  predicted; a failure falls back to the anchor and full march.  A pose is addressed by *where
  its contact's constants live* (`refresh_consts` rewrites in place; values it may change ride
  in `outer` and miss rather than read stale), so `System::new` calls `locus::forget`.
  A seed is a *place*: `hint(at: c, bearing: u + phase)`, `hint(at: t)` (`program::at_seed`
  lowers both to tapes), and `hint((xexpr, yexpr))` only in a component that is only ever
  traced (`build` refuses a drawn one, since on the sheet a seed is a number a solve writes).
  A traced body must be square — as many rows as inner coordinates — or elaboration refuses it.
  `tests/trace.rs` holds the taut-string involute against its closed form (seeds 3× wrong), a
  gear on traced flanks, and the same parameters asked in three orders (guarding the resume);
  `tests/jansen.rs` holds the drawn form against a circle-intersection model at 24 angles.
- A curve is *geometry*, laid out in the core and only stroked by the front end:
  `curve::tessellate` refines to `FLATNESS_PX` through `unit`, and `curve::closest` is the pick
  test and a fresh contact's seed.  No binding evaluates a basis function or writes the degree:
  `report::registry_json` publishes `curve.minCtrl`, matching `Sketch::spline_with`.
- `solve::Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
  counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
  r − target, so no kernel of its own); the front end only translates coordinates.
- The ABI is a panic boundary: every entry point runs inside `guard`, so a core panic becomes
  `gcs_last_error()` and a neutral return.  That needs `panic = "unwind"` in the release profile.
  `wasm32-unknown-unknown` aborts whatever the profile says, so untrusted input (a document) is
  bounds-checked in the core as well.
- Determinism: ordered containers only (`Vec`, `BTreeMap`/`BTreeSet`), never `HashMap` iteration
  in the solve path.  Every random draw comes from the seeded `rng::Rng`.
- The trust-region loop is `newton::dogleg` over a `TrustRegion`; a new thing to minimise
  implements the trait rather than copying the loop.
- **Block-triangular solve** ([docs/block-triangular-solve-plan.md](docs/block-triangular-solve-plan.md)):
  `System::block_order` (memoised per compile) is the DM-matched square part in strongly
  connected blocks (`graph::blocks`: Tarjan, lowest row first), over/under parts beside it.
  `System::subset` evaluates through the whole system's helpers, so its numbers match to the bit;
  `BlockTr` minimises one block with `newton::dogleg`, compiling no `System`.
  `SolveOpts::blocks`: `Rescue` (default) runs the pass and a whole-system DogLeg polish after a
  failed DogLeg, before LM, only with `retry` and ≥2 blocks, kept only if it solves; `First` is
  for measurement only (it changes bits and roots). `BLOCK_XTOL` 1e-15, accepted at
  `acceptance_tol`. The spiral bevel relies on it (`hypoid_layout.rs` holds rough starts).
- **Settled** ([docs/iteration-limit-rescue-plan.md](docs/iteration-limit-rescue-plan.md)):
  `SolveResult::settled` is `success && status != 4`; a DogLeg accepted on its iteration limit
  can be 1e-3 of the extent off. With `retry` and `Rescue`, `System::block_rescue` tries the block
  pass on an unsettled DogLeg: a failure from the start, kept if it succeeds; a limit stop from
  the stop, kept if it converges (status 0), else from the start, kept if it settles, else the
  first pass that settled, else the stop bit for bit. Drags (`retry` off) never see it.
- Nothing in the project is auto-formatted: there is no `rustfmt.toml`, and `cargo fmt` would
  reformat every file.  Match the surrounding style by hand (100 columns).
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `rust/gcs-core/tests/linalg.rs` checks them against `nalgebra`, on purpose.  **The library has
  no dependencies; its tests have one reference implementation**, a `[dev-dependencies]` entry so
  nothing links into the cdylib or wasm.  Each test also states the property (`A ≈ QR`,
  `NᵀN ≈ I`, minimum-norm orthogonal to the null space): agreement is evidence, the property the
  contract.
