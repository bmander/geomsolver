# Spatial constraints with solved attitudes: execution plan

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
  a point or line on a plane, and spatial circles and spheres. Cones and cylinders come later.
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

A new entity, `sphere s(center: p) hint(r: …)`, takes the prefix `radius` and the relations `on`
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
| `point_on_plane` | n(q)·X − d | 1 | 8 |
| `point_on_circle3` | radius row plus plane row | 2 / 1 | 11 |
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

## P2 — The language

The plane clauses, inferred cross-view semantics, the word table, `sphere`, the diagnostics,
hinges and `fold: along`, `project_free`, the page-placement gauge, IO, graft and FFI, and the
refusals in `tests/refusals.rs`.

**Exit:** a document states a gear axis and a pinion axis in two views, with
`gax angle(90deg) pax` and `gax distance(E) pax`. It solves to DOF 0, agrees with an independent
computation, and round-trips through JSON. A redundant fold row is flagged as over-constrained
without naming an intrinsic row.

## P3 — Circles, spheres, tangency and `through:`

**Exit:** in a hypoid pitch-cone fixture, the pinion cone touches the gear's at M with a common
pitch plane, and `fixtures::gear::bevel` designs are unchanged.

## P4 — Cones and cylinders, and `against` with solved views

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
