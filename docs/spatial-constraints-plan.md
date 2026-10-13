# Spatial constraints with solved attitudes: execution plan

**Status (2026-10-05): superseded in part.** The spatial relations
built here stand. The solved views it built (folds, `attitude: free`, `offset: free`,
`through:`, quaternion attitudes, hinges, `against` between solved views) were replaced by
planes over rays, with points in space and no paper placement: [planes-plan.md](planes-plan.md)
(#81). The sphere, cone and cylinder entities became the library's `std.Sphere`, `std.Cone` and
`std.Cylinder` (2026-10-06): what stands on one is a distance from its centre or axis, or
`std.CircleOnSphere`, `std.PointOnCone`, `std.TangentCones`; since #101 they are sets, and a
point on one is `p coincident S`, a line touching one `l tangent S`. Read the spellings below as
history.

The [spiral-bevel layout plan](spiral-bevel-layout-plan.md) needs a true 90° hypoid: the shaft
angle and the offset between two skew axes drawn in different views are stated, and the
pinion's pitch cone is solved to touch the gear's at the mean point. No document can say that
today. A view's attitude in space is a constant fixed at elaboration (`solvent-spec.md` §6.7
says an implementation "MUST NOT make the attitude an unknown"). This plan makes views into
solved workplanes and adds the spatial relations between them. Each phase ends with an
executable result and an exit gate, and every phase leaves the existing corpus byte-identical.

## Decisions

- **Scope is general 3D sketching.** Views are solved. The relations are spatial distance,
  point–line and skew line–line distance, angle, parallel, perpendicular, coincidence in space,
  a point or line on a plane, and spatial circles and spheres; cones and cylinders are P4's.
- **The app solves and shows only.** Views and the glass box display solved geometry. There are
  no new tools and no new picking.
- **Nothing 3D is an unknown *after* the sketch stratum.** That replaces "nothing
  three-dimensional is an unknown". Solids, fields and exports still read a solved snapshot.
- **A relation between entities in different views means space.** It is inferred from the
  operands' views, with no selector.
- **A solved view's place on the page is held silently.** Its picture's x, y and turn are
  presentation and are not counted as freedoms.
- **There are no free space points in v1.** Every point is drawn in some view.

## Semantics

Stated once, in spec §6.7 and primer §1.13.

- **A view is a workplane.** A point drawn in view V *is* the point
  `lift_V(p) = V.o + a·V.u + b·V.v`, where `(a, b) = plane::in_view(p)`. A point with no
  membership is on the page. A plane's own datum points (origin, toward) place the view on the
  sheet and are never lifted.
- **A relation reads its operands' views.** Operands in one view get the 2D relation, unchanged;
  it equals the spatial one because the lift is rigid. Operands in different views get the
  spatial relation over the lifts. `project` is the one exception: it ties two drawn images of
  an undrawn point and stays as it is.
- **The switch is audited first.** The P0 audit lists every non-`project` relation whose
  operands lie in different views. Each is a bug to fix, or a layout statement on datum points,
  which are read by their role and not their membership (see the P0 findings).

## Syntax

A plane is **fixed unless its brackets name an unknown**, and `hint(…)` seeds only unknowns.
The keys are `fold`, `offset`, `u`/`v` and `o`.

- `fold: E` and `offset: E` behave as today. If `E` names an undefined variable
  (`fold: beta`), that variable is a solved unknown (W111), seeded by `hint(fold: 30deg)`.
- `fold: along l`, with `l` drawn in the parent, puts `l` in the view and folds it 90° about
  `l`. The view follows `l` and has no unknowns of its own. This closes spec §17.1's open item.
  E064 when `l` is not in the parent.
- `through: M` puts the solved point `M` in the plane, which fixes its normal offset.
- `attitude: free` and `offset: free` are 3 and 1 unknowns, seeded by `hint(u:, v:)` and
  `hint(offset:)`.
- A hint on a stated quantity is E040.
- A bare `from:` still means "placed by `against`". With a solved attitude `against` is refused,
  under a new code, until P4.

A new entity, `s := sphere(center: p) hint(r: …)`, takes the prefix `radius` and the relations `on`
and `tangent`. A circle drawn in a view is already a spatial circle.

The spatial words are entries of `constraints::infix_op`, dispatched on the operand kinds and on
"different views":

| word | operands | meaning |
|---|---|---|
| `distance` | point, point | true length |
| `distance` | point, line | to the infinite line, a magnitude |
| `distance` | line, line | skew common-perpendicular distance, a magnitude |
| `distance(along: n)` | point, plane | signed along the plane's normal; `n` joins `ALONG` |
| `on` | point–line, point–circle, point–plane, line–plane, point–sphere | incidence in space |
| `coincident` | point, point | the same point in space |
| `angle` | line, line | unsigned, 0–180°; `sense:` refused |
| `parallel`, `perpendicular`, `equal` | line, line | directions; true lengths |
| `tangent` | sphere with line, circle or sphere | in space (P3) |

New diagnostics: E064 (`fold: along`, or a position stated twice) and E065 (a degenerate
spatial relation at the solve: parallel lines in a skew distance, zero-length lines in an angle,
views that came out parallel in `project`). `--where Q` reports `Q.u/v/n/o`, and the DOF ledger
names `Q.attitude` and `Q.offset` freedoms.

## Core design

**The attitude.** `PlaneE.att: Option<Att { q: [u32; 4], d: u32, ab: [f64; 2], hinge: … }>`
(`model/entities.rs`). It is minted only for a plane that is solved, or that a spatial relation
reads; otherwise it is `None`, no parameter is minted, and an existing document compiles
identically. It stays out of `entity_params` and `fields`, so traced tape widths
(`tape::MAX_VARS`) and column layouts do not move. The attitude is a unit quaternion held by an
intrinsic `quat_unit` row (degree 0, `frame_unit`'s form), with `Param::scale` the extent of the
plane's content. Position is `o = R(q)·(a, b, d)`: `a` and `b` are constants carrying today's
`o` exactly, and `d`, the normal offset, is the only position unknown. In-plane translation is
already the datum's 2D freedom.

**Hinges.** A fold from a solved parent is four rows, `q_child − q_parent ⊗ q_rel = 0`. `q_rel`
is a constant for a stated angle, or a half-angle rotor with a unit row for a solved one.
`fold: along l` adds rows tying that rotor to `l`'s direction in the parent, and a through-row
for `l.p1`. Chains of fixed links keep today's constant `program/planes.rs::plane_bases`.

**Hidden lift points.** Each view point a spatial relation reads gets one hidden 3D point X
(three parameters, `EntKind::spatial`, minted in constraint order). X is tied by an intrinsic
`lift` row (X − L, degree 1, 14 columns), or by `lift_fixed` (9 columns, the basis as constants)
when the view's `att` is `None`. Spatial kernels read only X's columns, and one helper,
`plane::lift_q`, gives the lift and its derivatives.

**Kernels.** Each gets a `KERNELS` entry, a `CKind` with `params_on` and `consts_on`, a free
twin through `CKind::free_kernel` where it states a dimension, and a row in
`tests/jacobians.rs`. All are unsupported in `cgraph`, so these documents drag numerically.

| kernel | residual | degree | columns |
|---|---|---|---|
| `quat_unit` | \|q\|² − 1 | 0 | 4 |
| `lift` / `lift_fixed` | X − L | 1 | 14 / 9 |
| `hinge` / `hinge_free` | q_c − q_p ⊗ q_rel | 0 | 8 / 10 |
| `project_free` | m·(X_A − X_B), m = n_A × n_B unnormalised; whenever either view has `att` | 1 | 14 |
| `coincident3` | X − Y | 1 | 6 |
| `distance3` | \|X − Y\|² − D² | 2 | 6 |
| `point_line3` | \|(X−A)×(B−A)\| / \|B−A\| − D | 1 | 9 |
| `line_line3` | (e₁×e₂)·(C−A) / \|e₁×e₂\| − E, signed by the seed | 1 | 12 |
| `angle3` | cos form | 0 | 12 |
| `perpendicular3` | normalised a·b | 0 | 12 |
| `parallel3` | two rows against seed-refreshed constants ⟂ a | 0 | 12 |
| `point_on_plane` / `point_on_plane_fixed` | n(q)·X − d | 1 | 8 / 3 |
| `point_on_circle3` / `point_on_circle3_fixed` | \|X − C\| − r and n·(X − C) | 1 | 11 / 7 |
| `sphere_on` | \|X − C\| − r | 1 | 7 |

**The system.** No structural change: row scaling by `extent^degree` already handles mixed
degrees. `consts_on(Project)` uses `plane::fold_line` only when both views are fixed, and
`refresh_consts` refreshes `parallel3`'s constants. Minting an `att` or a hidden X is a topology
change and recompiles.

**The page-placement gauge.** For a view with a solved attitude, the datum's page pose is held
at its seed and left out of the DOF count, unless the document states a relation on the datum
points; then those points are ordinary.

**Basis consumers.** Every reader asks `Sketch::basis(i)`, which is the stored basis today and
one built from the solved `att` later. The field is private to the model, so the compiler finds
any reader that does not. The memo keys (`solid/document.rs`'s solid reads,
`renderer::document::inputs`) key on the accessor's value. The elaboration-time parallelism
checks — `against` in `program/solids/claims.rs`, E084 in `program/solids.rs`, E061 in
`constraints.rs` — stay as they are for fixed attitudes and become post-solve (E065) for solved
ones.

**IO and bindings, surface only.** `basis.o` is written to JSON (P0). An optional
`"att": {q, qfixed, d, dfixed, ab, hinge}` follows; intrinsic rows and hidden points are
re-minted on load and never serialized. The graft remaps the `att` parameters. The FFI's
`gcs_plane_basis` returns the accessor's value, and a new `gcs_plane_frame3` returns nine doubles
including `o`. The web `Plane.basis` getter reads the solved value; the glass box already reads
bases through the core.

## P0 — Refactor only

- `Sketch::basis(i)` is the one reader of a plane's attitude. `PlaneE.basis` is
  `pub(in crate::model)`, so every reader outside the model, in the core, the CLI, the FFI and
  the tests, goes through the accessor, and the compiler enforces it with no grep lint.
- Two writers: `Sketch::set_plane_origin(i, o)`, what `against` and a derived offset do at
  elaboration, and `Sketch::set_basis(i, b)`, for a caller that turns a sketch's views rigidly
  (the tests).
- The memo keys read through the accessor.
- JSON writes a plane's `"o"` when it stands off the origin and reads it back. A view at the
  origin writes nothing new, so every existing export is unchanged.
- `tests/cross_view_audit.rs` (ignored) lists every non-`project` relation in
  `rust/examples` whose operands lie in different views.

**Exit:** the corpus reports and sheets, every export golden and the native pinion STL are
byte-identical to main, and the full suite passes.

### P0 audit findings

`cargo test cross_view_audit -- --ignored --nocapture` elaborates every `.sv` in
`rust/examples` with its modules. It finds 215 non-`project` relations whose points carry
different memberships, in 7 documents; the three unit-less parameter modules
(`engine/dims.sv`, `vtwin/components/dims.sv`, `spiral_bevel/configuration.sv`) do not
elaborate alone and hold no relations. None is a genuine relation between two views:

| document | layout | own datum | hinge | what they are |
|---|---:|---:|---:|---|
| `engine.sv` | 4 | 189 | 0 | ordinates of every part from `O`, the front view's datum origin, and one view placed from another |
| `spiral_bevel/pair.sv`, `gears.sv` | 5 + 5 | 0 | 0 | each view's datum origin made coincident with `std.origin` |
| `vtwin/assembly.sv` | 4 | 2 | 1 | the right and top views placed from `O`; `so` from the right view's origin |
| `solid_loft.sv` | 0 | 3 | 0 | an end of the loft coincident with `std.origin` |
| `vtwin/components/throttle.sv`, `frame.sv` | 0 | 0 | 1 + 1 | `section_center on fold` |

- **Layout** means every point is some plane's datum point: where views sit on the sheet.
- **Own datum** means one view's points measured from a datum point of that view: an ordinate.
- **Hinge** is the throttle's `section_center on fold`. `fold` is drawn in the front view
  between the longitudinal view's datum points, so it is the line the two views fold about. It
  holds in space as well as on the page, and it is exactly what `fold: along l` will state.

The audit corrects the plan in one place. **Datum points are not page points by membership.**
`std.origin` is a member of `std.front`, every `bracket.sv` view is drawn from an origin that is
one of its own corners, and `back_datum.origin` is a member of `pair.back`. A rule that reads
"no membership" as "on the page" would turn 212 of these statements spatial and let every view
drift off its place on the sheet. The P2 rule is therefore by **role**: a point that is some
plane's origin or toward is never lifted when it is related to other datum points (layout, on
the page), and reads in view V when it is related to V's points (an ordinate, in V, where its
lift is V's own origin by construction). With that rule the corpus has no relation whose
meaning changes, and the throttle's hinge reads in the longitudinal view.

## P1 — The model and kernels, as a Rust API

`Att`, quaternion helpers (`plane::from_quat`, `to_quat`, `lift_q`), `quat_unit`,
`lift`/`lift_fixed`, hidden X, and the spatial kernels.

**Exit:** finite-difference Jacobian rows for every kernel, the corpus identical, and
closed-form fixtures in a new `tests/spatial.rs`:

- a regular tetrahedron, its base on the page and its apex in a free view, with height
  `a·√(2/3)`;
- the distance between two skew lines against its closed form;
- an unconstrained free view reporting exactly 4 DOF.

### P1a — the attitude and the lift (done)

- `PlaneE::att: Option<Att { q, d, ab, seat }>`, minted by `Sketch::free_attitude(i)` with the
  intrinsic `QuatUnit` row; `fix_attitude(i, bool)` is the params' fixed flags and
  `restore_attitude` is what JSON and the graft use. `q`'s `Param::scale` is the farthest the
  view's content (its origin's in-plane part, its offset, its member points) stands from the
  point it turns about, never less than the datum's chord. `d` is a length (scale 1).
- `Sketch::basis(i)` reads the stored basis while `(q, d)` hold the numbers they were minted at
  (`Att::seat`, compared bitwise), and otherwise `R(q/|q|)` with `o = R·(a, b, d)`. A quaternion
  read back from a basis does not rebuild it to the bit, and the seat is what keeps freeing a
  view from moving any number a reader sees.
- Hidden points are not an entity kind: `Sketch::lifts: Vec<LiftE { point, x }>`, minted by
  `Sketch::lift_point(p)` once per view point, held by the intrinsic `Lift` (14 columns) or
  `LiftFixed` (9 columns, the stated basis as constants). Their spec is the view point and its
  plane, and the hidden point is found by the point, so a spatial relation will name drawn
  points and read the lifts' columns. Freeing a view turns its `LiftFixed` rows into `Lift`
  rows. A point on no view has no lift yet; the page lift and the role rule are P2's.
- JSON writes `"att"` on a solved plane only; the rows and the hidden points are re-minted and
  never written. The graft carries both, and a drag part maps their params.
- Deferred to P1b: the spatial relation kernels and their `tests/spatial.rs` fixtures (the
  tetrahedron, the skew lines).

### P1b — the relations in space (done)

- Eleven kinds, `CKind::spatial`: `Coincident3`, `Distance3`, `PointLine3`, `LineLine3`,
  `Angle3`, `Perpendicular3`, `Parallel3`, and `PointOnPlane` / `PointOnCircle3` each with a
  `…Fixed` twin. Their slots name the **drawn** points, lines and circles; the kernels read only
  the hidden points' columns (`Constraint::lifted_points`: a point, a line's two ends, a circle's
  centre). `Sketch::add` mints the lifts they read, so no caller mints one, and `validate`
  refuses an operand on no view, a negative magnitude, a line of no length in space and a skew
  distance between lines parallel as drawn. `Constraint::in_space` is the Rust constructor,
  through `io::seed_omitted` like every other.
- Fifteen kernels (63 in all), with free twins for the four dimensions. `line_line3` is the
  **signed** gap, stated along the side the seed stands on: an inferred `sign` slot
  (`constraints::infer_value`, ±1, written to JSON) folded into the constant, and into (m, c)
  for the free twin, so neither kernel carries it. `angle3` is `â·b̂ − cos θ`, unsigned, with no
  `sense`. `parallel3` is two rows `(â × b̂)·e_k` against two unit vectors across the first
  line, constants re-read by every `refresh_consts`.
- A kernel has one degree, so `point_on_circle3`'s radius row is the magnitude `|X − C| − r`
  (degree 1, beside the plane row) rather than the squared form the table first had.
- The two that read a plane follow P1a's `lift`/`lift_fixed` split: `CKind::attitude_twin` is
  the one table of the pairs, `Constraint::attitude_read` says which plane, `Sketch::add` picks
  the twin its plane can feed and `free_attitude` flips every statement reading the view it
  frees. Lifts are unchanged.
- No operator and no callout yet (`undrawn!`): the words and their figures are P2's.
  `tests/spatial.rs` holds the regular tetrahedron (height `a·√(2/3)` to 1e-9 relative; the
  ledger is 2 for the apex, 6 with its view freed, 3 after the three lengths, the view's gauge
  about the apex), skew lines against the hand-worked bearing and offset, parallel,
  perpendicular, a point on a stated and a solved plane, a point on a circle in a stated and a
  solved view, coincidence, the refusals, claims (a true one a theorem, a false one violated,
  nothing counted), and the JSON and copy round trips. Every kernel and twin has a
  finite-difference row.

## P2 — The language

The plane clauses, inferred cross-view semantics, the word table, `sphere`, the diagnostics,
hinges and `fold: along`, `project_free`, the page-placement gauge, IO, graft and FFI, and the
refusals in `tests/refusals.rs`.

**Exit:** a document states a gear axis and a pinion axis in two views, with
`gax angle(90deg) pax` and `gax distance(E) pax`. It solves to DOF 0, agrees with an independent
computation, and round-trips through JSON. A redundant fold row is flagged as over-constrained
without naming an intrinsic row.

### P2a — solved views in the language (done)

- **Clauses.** `fold: E` affine in a name nothing defines solves the fold, a free variable
  (W111) seeded by `hint(fold: …)`; `fold: along l`; `attitude: free`; `offset: free`;
  `through: M`. `syntax::Attitude` gains `Along` and `Free`, and `Decl::plane: PlaneSolve`
  carries the position (`Stated`, `Free`, `Through`) and the plane's `hint` keys
  (`fold`, `offset`, `u`, `v`). The printer spells every clause back; the flattener settles the
  seeds and rewrites the `along` and `through` references; deleting the line or the point a view
  stands on deletes the view.
- **The pass.** `program/views.rs` runs after the memberships and before any relation, parents
  first. A stated parent a solved child reads is given **held** unknowns (its `quat_unit` row
  then has no free column). A chain of stated folds from stated planes mints nothing.
- **Kinds and kernels** (67 kernels). `Hinge` (`q_c − q_P ⊗ fold_rotor(θ)`, four rows, degree
  0, no unit row on the child; `Att::hinged`), with the free twin `hinge_free` reading θ as the
  document's free variable — so `beta` is one unknown with every dimension that names it, not a
  second rotor. `HingeParallel` (`from: P, offset:` over a solved `P`, the same kernel with the
  identity). `HingeAlong`: the hinge over a half-angle rotor `(hc, hs)` that is the constraint's
  own `Param` slots, its unit row, and a row putting the fold's bearing on `l` as drawn in the
  parent (16 columns, six rows); a `PointOnPlane` beside it puts `l.p1` in the view.
  `ProjectSolved` (`project_free`, `(n_A × n_B)·(X_A − X_B)`, 14 columns) is `project`'s twin
  wherever either view has an `att`; `Sketch::add` and `free_attitude` pick it
  (`Constraint::attitudes_read`) and give the other view held unknowns and both images lifts.
  `plane::fold_rotor` is the one statement of a fold as a quaternion.
- **Limits, refused as E064.** A view derived from one whose offset is solved; a solved fold or
  `along` from a parent whose origin stands off its own normal; a position stated twice;
  `through:` a point of the plane itself; `along` a line of another view. `against` with a
  solved view is E066. E065 (`program::solid_diagnostics`, after the solve): views a `project`
  relates, or lines a skew distance names, that came out parallel.
- **Reports.** `report::positions` adds `Q.u/v/n/o.{x,y,z}` for a plane with an `att` only;
  `diagnose::view_freedoms` names the movable ones `Q.attitude` and `Q.offset`, and `solventc`
  prints them as `free views:` where there are any. The ledger has no other naming hook.
- **IO and writeback.** JSON writes `"hinged": true` on a hinged view's `att`; the hinge rows
  are ordinary constraints of the document. A relation in space read without its skew side
  infers it (`io::from_json`). `edit::commit_seeds` splices a solved fold, attitude or offset
  into its written seed, or writes the clause where none was. `program::to_program` lifts a
  solved view as a stated one where the solve left it.
- **Tests.** `tests/spatial_lang.rs`: the gate (gear axis in the front view, pinion axis in a
  view whose fold is solved; `Angle3` 90° and `LineLine3` 17.5 through `Constraint::in_space`)
  solves to DOF 0 from a 30° seed, the fold at 0° to 2e-11°, the angle between the lifted axes
  90° (cos < 1e-9) and their skew distance 17.5 (to 1e-9), and it round-trips through JSON byte
  for byte; `fold: along`, `through:`, free attitude and offset with their seeds and freedoms,
  `project` between a stated and a solved view (β = atan2(40, 30)), the print round trip, the
  refusals with their spans, writeback, deletion and a solved fold in a component.
- **Deferred to P2b.** The inferred cross-view word dispatch with the role rule, `sphere`, the
  page-placement gauge, the primer's worked example, the FFI's `gcs_plane_frame3`, and lifting a
  solved view's clauses in `to_program`.

### P2b — the words, the role rule, the sphere and the page gauge (done)

- **Dispatch.** `program/reading.rs::in_space`, asked by `constrain` once the arguments resolve:
  the points of each entity operand (a point, a line's ends, a circle's or arc's centre and
  ends) are read in their views by the **role rule** (`reading_views`): a point that is no
  plane's datum reads where its membership puts it; a datum point reads in a view another point
  of the relation is drawn in when it is that view's datum, is on the page when the relation is
  among datum points only (layout), and otherwise reads by membership. One view (or the page)
  keeps the 2D kind. Across views: `coincident` → `Coincident3`, `distance` → `Distance3` /
  `PointLine3` / `LineLine3` (side inferred), `angle` → `Angle3`, `perpendicular` / `parallel`
  → `*3`, `equal` between lines → `EqualLength3` (new), `on` a line → `PointOnLine3` (new: two
  rows across the line, `parallel3`'s refreshed constants, since the magnitude has no gradient
  where it vanishes), `on` a circle → `PointOnCircle3`. `side:` and `sense:` are E040 at the
  key; every other kind across views is **E062**, as is a relation across views naming a page
  point or a datum point read in a view it is not drawn in. Radii (`radius`, `equal` circles, a
  ring's width), `along: u`/`v` (a datum ordinate on the sheet, which `paired_references.sv` and
  the V-twin frame use between views) and `project` are view-free.
- **Words in space regardless of views**, from `infix_op`: `p on P` (`PointOnPlane`), `l on P`
  (`LineOnPlane` / `…Fixed`, new, two rows), `p distance(d, along: n) P` (`PointPlaneDistance` /
  `…Fixed`, new; `n` joins `ALONG`; the stated-plane form is `point_on_plane_fixed` with d folded
  into the offset, and its free twin carries the plane's normal and offset beside (m, c)). A
  view's own point or line on it is refused (E061: identically zero).
- **Sphere.** `EntKind::Sphere` (last in the enum; FFI kind id 16), `SphereE { center, radius }`,
  `s := sphere(center: p) hint(r: …)`, built after planes, in `primitives()`, JSON (`"spheres"`, only
  when there is one) and the graft. No sheet glyph (`svg`, `drawable`, `pick`); `scene3d` draws
  three great circles about the centre's lift. Words: `radius(r) s` (`SphereRadius`, the radius
  kernel), `p on s` (`SphereOn`, `sphere_on`), `s tangent l` (`SphereTangentLine`, `point_line3`'s
  free twin at (m, c) = (1, 0)), `s tangent s2` (`SphereTangentSphere`, `sphere_sphere`,
  `external` inferred). A sphere against a circle is refused with a message (P3). Test fixtures
  that named things `sphere` were renamed (`globe`, `ball`); no corpus document did.
- **Page gauge.** `reading::hold_page_placement` (after the relations) fixes the origin and toward
  of every view with an `att` when every view they place has one and no non-intrinsic statement
  names them (as an operand, an end or a centre) and they are not already held; they are recorded
  in `Sketch::page_held`, which `edit::held_refs` skips so no `ground` is written back.
  `to_program` still grounds them, since it lifts every view as stated. A free view alone reports
  DOF 4 (`q.attitude`, `q.offset`), and 7 once `o distance(40) t` names its datum.
- **Equations = rank.** A held view's `quat_unit` row whose columns are all fixed compiles as not
  hard (`System::new`), so no count, rank or conflict search sees it: the gate reads 27 params, 27
  equations, rank 27.
- **FFI.** `gcs_plane_frame3` (u, v, o), and `Plane.frame3` in the binding. `to_program` still
  lifts a solved view as a stated basis (P2a); lifting its clauses (`fold: beta` with its solved
  seed) is deferred — the lifted text is a stated drawing of the same solved pose.
- **Kernels** 76 (nine new), kinds 65. **Tests:** `spatial_lang.rs` — the hypoid axes in words
  (`gax angle(90deg) pax`, `gax distance(17.5) pax`) solve to DOF 0, agree with P2a's Rust-API
  gate point for point, round-trip through JSON and through a lifted program; each word across
  views, within one view, and in space on a plane; the role rule; the refusals; the page gauge
  (DOF 0 with no datum grounded; a free view 4; 7 once named); equations = rank; the sphere's four
  words and a solve against closed forms. `cross_view_audit.rs` asserts the corpus's 215
  cross-membership relations settle to 2D kinds and read in one view or the page, with no E062
  anywhere. `refusals.rs` and `jacobians.rs` rows for every new kernel and twin.
- **Deferred to P3:** sphere–circle tangency; a `midpoint` or `symmetry` in space; a datum point
  read in a view it is not drawn in (refused); lifting solved-view clauses.

## P3 — Circles, spheres, tangency and `through:`

**Exit:** in a hypoid pitch-cone fixture, the pinion cone touches the gear's at M with a common
pitch plane, and `fixtures::gear::bevel` designs are unchanged.

### P3 — the hypoid's pitch cones, and the deferred words (done)

- **The gate** is `tests/fixtures/hypoid_pitch_cones.sv`, in words only: the pitch plane P stated
  with M grounded on it; the gear apex O and the pinion apex Aₚ drawn in P, with `horizontal` on
  the generator O→M as the turn gauge; G and Q folded `along` the generators O→M and Aₚ→M; each
  axis drawn in its view from the apex's image (`gax.p1 on P`, `O project gax.p1`: on the fold
  line and at the apex) with a drawn length. The design is Ng = 48, Np = 24 at a 4 mm module
  (pitch radii 96 and 48, `M distance(Rg) gax`), shaft angle 90° and offset E = 20
  (`gax angle(90deg) pax`, `gax distance(E) pax`), and **the gear pitch angle Γ = 60°**
  (`gen_g angle(60deg) gax`), the one condition the pitch cones need beyond those four: the gear
  cone is then its radius and its pitch angle, and the pinion's apex (two) and pitch angle (one)
  are held by its radius, the shaft angle and the offset. A spiral angle would do instead, but
  that belongs to the tooth trace, which this fixture does not draw. The common pitch plane is by
  construction: each axis is in a view square to P along its generator, so P is each cone's
  tangent plane along it. It solves to DOF 0 (56 params, 56 equations, rank 56) from seeds near
  the answer, with ε = 10.7221°, γ = 29.5650°, |MAₚ| = 97.2822.
  `the_hypoid_pitch_cones_touch_at_the_mean_point` recomputes from the lifted geometry alone:
  each axis through its apex, each axial view square to P holding its generator and its axis,
  the shaft angle, the common perpendicular, M's distances to both axes, the gear pitch angle,
  and each cone's surface normal at M parallel to P's normal (and P's normal ⟂ each generator
  and in the plane of each axis and generator); every residual is below 5e-15, and cos ε =
  tan Γ·tan γ holds to 1e-9. The bevel designs are untouched: no example changed, and the corpus,
  the goldens and the gear suites are the gate.
- **`to_program` lifts a solved view's clauses** (`program/lift.rs::lift_view`): a `Hinge` as
  `from: P, fold: <its expression>` with `hint(fold: <the solved fold>)` where the fold is free, a
  `HingeAlong` as `fold: along l` (the `PointOnPlane` it mints is not lifted again), a
  `HingeParallel` as `from: P, offset: k`, a free attitude as `attitude: free` seeded with its
  solved `u`/`v`, and a solved offset as `offset: free` seeded where it stands — so `through: M`
  comes back as `offset: free` beside `M on Q`, the same statement. A held view (a stated one a
  solved view reads) is lifted stated as before, and the page gauge's points are not grounded:
  the lifted views hold them again. Round trips: the axes gate (the free fold, already solved on
  load), the hypoid (two folds along, the same holds, the same kinds, the same points), and a
  free attitude, a free offset, `through:` and a stand-off from a free view (same bases, same DOF).
  A stated plane stood off its parent along the normal is still lifted as `u:`/`v:` without the
  offset, which `Attitude::Basis` cannot carry (as before P3).
- **`Sketch::page_held` travels through JSON** as `"page_held"` (point indices, written only when
  nonempty, read only for a point that is fixed) and through the graft, so a loaded document reads
  the same DOF and a writeback or a lift never spells a `ground` for those points.
- **A circle on a sphere**, `k on s` (`CircleOnSphere` / `…Fixed`, the attitude twin pair): the
  sphere's centre on the circle's axis (`u·(S − C)`, `v·(S − C)`, the view's in-plane axes) and
  `√(‖S − C‖² + r²) − R`, three rows of degree 1 over the circle's view solved (12 columns) or
  stated (8, `(u, v)` as constants). This is the useful meaning for a gear blank (a toe or heel
  circle on its end sphere). `s tangent k` stays refused, now saying why: a circle and a sphere
  touch at a point or all the way round, and the word does not say which.
- **`midpoint` and `symmetry` across views** are `Midpoint3` (`X − (A + B)/2`, three rows) and
  `Symmetric3` (`Q + P − 2F`, F the foot of P on the line: the half turn about the line, which on
  the line's own plane is the page's mirror; three rows, a unit gradient in Q). Both degree 1.
- **A datum point read in a view it is not drawn in stays refused (E062).** It has a lift (an
  origin at its view's `o`), but its meaning would then turn on whether the other operands are
  datum points: beside datum points of other views the same statement is sheet layout, and the
  corpus has 215 of those. A document that means space draws a point in the view at the datum
  (`m coincident o2` inside the view) and relates that.
- **Kernels** 80 (four new), kinds 69; `jacobians.rs` rows for each and both circle twins,
  `refusals.rs` accepts `midpoint`/`symmetry` across views. **Tests:** `spatial_lang.rs` — the
  gate, the three lift round trips, the page gauge through JSON and a copy, a circle on a sphere
  in a stated and a solved view (every point of the circle at the sphere's radius, three
  equations), the midpoint and the mirror solved and checked in space.
- **Deferred to P4:** cones and cylinders as entities (a pitch cone is still two views and an
  axis, not a surface a relation can name), `against` with solved views (E066), and a stated
  plane's normal offset in a lifted program.

## P4 — Cones and cylinders, and `against` with solved views

### P4 — cones, cylinders, mates with solved views, and `o:` (done)

- **Spelling.** `gc := cone(axis: gax) hint(half: 60deg)` and `c := cylinder(axis: ax) hint(r: 10)`:
  the brackets are what each is made of — a line already drawn in some view, never minted, a
  cone's apex its start and its axis running toward its end — and the one number each owns is a
  seed (`hint(half: …)` in degrees, held in radians; `hint(r: …)`) that a relation states
  (`angle(60deg) gc`, `radius(10) c`) or a solve finds. No other freedom: the surface is where
  its axis is. `EntKind::Cone`/`Cylinder` (last in the enum, FFI kind ids 17 and 18, labels `n`
  and `y`), one struct `AxialE { axis, param, class }`, `Sketch::cones`/`cylinders`, built by
  `program::entities::build_axial` after the spheres, in `primitives()`, JSON (`"cones"`,
  `"cylinders"`, only when there is one) and the graft. No sheet glyph; `scene3d` draws each as
  two circles square to the axis and four rulings. `Sketch::seed_value` is the one place a
  half-angle is turned back into degrees, for a writeback and a lifted program.
- **Words and kernels** (85 kernels, 76 kinds). `p on k` (`ConeOn`, new `cone_on`:
  `ρ cos α − h sin α` over the point's, the apex's and the axis end's hidden points and α — the
  point's distance from the generator in its meridian half-plane, degree 1, zero only on the
  nappe the axis points into); `p on c` (`CylinderOn`, `point_line3_free` at (1, 0), the radius
  the free column); `angle(θ) k` (`ConeAngle`, new `half_angle`/`half_angle_free`, the radius
  arithmetic at degree 0); `radius(r) c` (`CylinderRadius`, `radius`/`radius_free`);
  `c tangent l` (`CylinderTangentLine`, `line_line3_free` at (±1, 0), the side an inferred `sign`
  read off the seed as a skew distance's is; parallel lines refused, and E065 when they come out
  parallel); `k1 tangent(M) k2` (`ConeTangentCone`, new `cone_cone`, 17 columns: the second cone's
  normal at M square to the first's generator and circle there, two rows of degree 0 — one
  tangent plane at M; `M on k1` and `M on k2` are stated beside it, as an on-circle is beside a
  tangency at a named end). `tangent` now reads an unlabelled entity in its parentheses, as
  `symmetry` does. The new kernels take their Jacobians by a forward-mode `Dual<N>` in
  `kernels.rs` rather than hand-derived chains. A line on a cone or a cylinder (a generator) is
  refused with the spelling that says it of the line's points; a cone against a line, the
  cylinder written second, a half-angle on a cylinder and two cones with no point are refused at
  the word (E040); an axis that is not a line, or none, is E103.
- **The hypoid, named.** `rust/examples/hypoid_pitch_cones.sv`: P stated with M grounded on it;
  the gear's axial plane G is P folded square about its vertical axis and the gear's apex on P
  (so P is the gear cone's tangent plane at M); the pinion's axial plane Q is `attitude: free,
  through: M`, its turn held by `horizontal pax`; the cones named, `angle(60deg) gc`, `M on gc`,
  `M on pc`, `gc tangent(M) pc`, and the radii, shaft angle and offset as in P3. It solves to DOF
  0 (39 params, 39 equations, rank 39), and against P3's fold construction the gear and pinion
  pitch angles, the offset angle, |MO|, |MA| and |OA| agree within 1.3e-11 (Γ = 60°,
  γ = 29.564957707°, ε = 10.722102067°, |MA| = 97.282181449). The pinion's apex comes out on P
  (1e-11) with nothing saying so, and the two cones' normals at M are parallel and P's.
- **`against` with solved views.** `program::solids::claims::place` reads each plane's **attitude
  root** (through `from:` clauses with no fold). A mate where either plane is solved needs the two
  to share a root — then they are parallel whatever the solve does, and the placed plane turns
  with its datum (a `HingeParallel` from P2a where its parent is solved). Where the datum's
  offset is held (a solved fold about the shared origin) the placed plane is stood off by the
  faces' gap exactly as a stated one is; where it is an unknown (`offset: free`, `through:`), the
  placed plane's offset is freed and held by a new `CKind::Mate` row, `d_f − d_g − gap` (kernel
  `mate`, degree 1), recorded against the `against` statement and never written as a relation.
  Views that turn apart are E066 naming both; so is a placed plane that follows a solved offset
  and has views derived from it (P2a's E064, one statement later). Stated-only mates take the
  path they always did, byte for byte.
- **`o:` and the lift.** `u: (…), v: (…), o: (x, y, z)` gives where a basis given outright stands
  (`Attitude::Basis::o`, three lengths; alone a syntax error). `to_program` writes it for a stated
  plane whose origin is off the shared origin — a stand-off, a mate, a view folded from one,
  whose origin is off its own normal — and a view whose offset is solved writes only the
  in-plane part there, its place along the normal being its `hint(offset: …)`.
- **Tests.** `tests/spatial_surfaces.rs` (new): each entity's fields and freedoms; each word's
  kind; a point on a cylinder its radius off the axis and on a cone at its half-angle (and on the
  right nappe), a free half-angle found by a point, a line touching a cylinder at its radius; the
  named hypoid against P3's; JSON, a lifted program and a half-angle written back in degrees; the
  glass box; the refusals; a mate on a `through:` view following it to where a stated offset puts
  it (with the row counted in the rank), a mate in a view with a solved fold turning with it, and
  views that turn apart refused; and a lifted stand-off and fold-from-stand-off keeping their
  origins. `refusals.rs` and `jacobians.rs` rows for every new kind, kernel and twin.
  `spatial_lang.rs`'s old E066 test now reads the stack's E083 (a view whose offset is solved is
  not a placed plane).

## Status

The feature is complete as planned. A view is a workplane whose attitude and offset may be
solved (`fold: beta`, `fold: along l`, `attitude: free`, `offset: free`, `through: M`); a
relation between entities drawn in different views is the relation in space, inferred from the
views with the role rule for datum points; spheres, cones and cylinders are surfaces a relation
can name; and a hypoid's pitch cones can be stated either by construction (P3) or by naming the
cones and their contact (P4), with the same answer. Every corpus report, drawing and export
golden is byte-identical to main at every phase.

The spiral-bevel example is now a true hypoid on it: its pinion's axial view is folded along
the pinion's generator (P3), and the shafts are stated square and `offset` apart
([layout plan](spiral-bevel-layout-plan.md#the-true-hypoid)).

What remains open:

- **A line on a cone or a cylinder** (a generator) as one relation, and a sphere tangent to a
  circle; both are refused with the spelling that says them of points.
- **Two cones touching with no point named**, and cones touching along a common generator with
  a shared apex (the bevel pitch cones): stated at a point of the generator, `tangent(M)` is
  rank-deficient there, since its generator row vanishes identically once M is on both cones.
- **Mates between views that turn apart**, which would need a parallelism row beside the offset
  row; refused (E066).
- **Decomposition.** Every spatial kind is `unsupported` in `cgraph`, so documents with views
  in space drag on the numeric path.
- **The app** solves and shows only: there are no tools for drawing cones, cylinders or solved
  views, and no picking in the glass box.
- A datum point read in a view it is not drawn in stays refused (E062, P3).
- `to_program` does not lift solids, faces or `against`, so a lifted document keeps a mate's
  result (the placed plane's pose) and not the statement.

## Documentation in every phase

- Spec: line 68, §3.2, §6.7 (the MUST NOT removed, the workplane rule and the general `project`
  residual added), §9.2/9.3, §16 (the ledger), §17.1, §19.
- Primer: §1.2, §1.4, §1.5, §1.13, §1.14, and a new worked example.
- CLAUDE.md: the plane bullet.

## Verification in every phase

- `cargo test --manifest-path rust/Cargo.toml --features gcs-cli/occt --no-fail-fast`, once.
- The corpus byte gate and the export goldens, the pinion STL byte-identical.
- `make wasm && cd web && npm test`. From P2, open a fixture in Chrome and see the solved views
  and axes in the glass box.
- From P2, `solventc fixture.sv --where Q` shows the solved `Q.u/v/n/o`, and the diagnosis names
  the free attitude freedoms.
