# Intrinsic rows derived from the model: plan

**Status (2026-11-06): proposed**, following #817.  Nothing here is built yet.

An *intrinsic row* is a constraint the drawing adds itself (`Constraint::intrinsic`): an axis's
unit row, a plane's axes through its place, a normal square to its plane, a point's lift. The
document never states one. Each follows from what an entity *is*.

#817 showed that the project treats these rows two ways. Some are **derived**: a constructor mints
them from a field of the model, and anything that copies the model mints them again from the same
field. Others are **records**: the row is the only place the fact is kept, so anything that copies
the model loses it unless it recognises the row by its shape. An instance's square frame (#548)
was a record. `graft` mints a frame's rows again from its `Chart` and has no field to say "square",
so every part, drag and copy around a seated instance could shear it. The fix in #817 recognises
the row by its shape (`Sketch::chart_square`). It is the second such recognition, after
`graft::axes_along`, and it closes one case of a class.

This plan makes every intrinsic row derived, and makes a record impossible to write.

## What the Design Bible asks

- **Each fact is stated once, and everything else is derived** (principle 2, "Solvent shrinks").
  A row that is the only record of a fact is the fact stated in the wrong place: a reader has to
  invert the derivation to recover it. `chart_square` and `graft::axes_along` are those inversions.
- **Redundancy and regular structure are found from the rank of the equations, not from flags or
  shape-specific code.** Matching rows by kind and argument order is shape-specific code.
- **Silently wrong geometry is the worst failure** (principle 1). A dropped row is silent. The part
  solves, a drag moves what the drawing holds, and the only symptom is a count of freedoms nobody
  can trace (#817 took a bisection of a 5 500-unknown document).
- **Do it right even if it's hard.** A test net alone catches the class after the fact. The
  principled fix makes a new structural fact unable to compile without its derivation, then keeps
  the net as the independent check.
- **Frames are the unit of placement and of motion; frames compose by nesting.** Most of the
  records are facts about frames: square, standing along lines, sharing a place. Recording them on
  the frame makes the frame the one owner of its structure.

## The inventory

Every site that mints an intrinsic row today, what the fact is recorded on, and whether the two
readers that copy a model (`graft`'s rebuild, used by deletion, copy, paste, drags and a station's
`Part`; and the JSON reader, `io.rs`) mint it again.

| row(s) | minted by | fact recorded on | `graft` | JSON |
|---|---|---|---|---|
| `AxisUnit`, `AxisFoot` | `Sketch::axis`, `place_axis` | `AxisE` | re-minted | re-minted |
| `PlaneAxis` ×2 | `hold_through` (`push_plane`) | `Chart` | re-minted | re-minted |
| normal: `Perpendicular3`, `PerpendicularAbout`, `PlaneAxis` | `stand_normal` | `Chart::normal` | re-minted | re-minted |
| `Lift`, `ChartLift` | `lift_point` | `lifts`, `PointE::chart` | re-minted | re-minted |
| twin: `PointOnPlane`, the fold | `tie_twin`, `hold_twins` | `twins` | re-minted | re-minted |
| frame square: `Perpendicular3(u, v)` | `square_chart` (#548) | **the row** | recognised (#817) | **lost** |
| axes along lines: `Parallel3`, `PointOnAxis`, origin `Coincident3`, `PlaneAxis` removed | `entities::axes_along` (#535) | **the rows** | recognised | to measure |
| a datum's shared place | `plane_at` (#542) | the shared `o` params | copied as a place of its own, plus redundant `PlaneAxis` rows | same |
| arc in space: `Coincident3` ×3, `Perpendicular3` | `entities::arcs_in_space` | **the rows**, `stands_for` | **lost** (a ledger "gap") | to measure |
| circle in space: `Coincident3`, `Perpendicular3` ×4 (gauge) | `entities::circles_in_space` | **the rows**, `round_gauges`, `stands_for` | **lost** (a ledger "gap") | to measure |
| `DevelopedLength` | `program::developed` | **the row** | to measure | to measure |

"To measure" is what reading the code suggests and a gate has not shown. Stage 0 measures it.

Readers that infer structure from rows today, and so become readers of fields: `chart_square`,
`graft::axes_along`, `views.rs` (the axes-along `Parallel3` filter, the normals' rows, the twins'
check), `model/duals.rs` (the axes-along `Parallel3` filter), `hold_twins`'s `built_through`, and
`model/turns.rs`, which retains rows by entity.

## The design

### 1. Every structural fact is a field of the entity it is about

The facts that are records today become fields. Each is set where the elaborator decides it, and
copied by the entity's `remapped`, so `graft`'s and the JSON reader's exhaustive destructuring
stops the build on a new one.

- `Chart::square: bool`, set by `square_chart` (#548).
- `Chart::along: [Option<Along>; 2]`, an axis held along a drawn line (`Along { line, at }`, the
  point it stands through), and `Chart::stands_at: Option<u32>`, the shared end its origin is
  coincident with (#535).
- `Chart::place: Place`, `Own([u32; 3])` or `Of(frame)`: a datum standing at its frame's place
  (#542). This removes the copy's extra place and redundant rows, so a copy is structurally the
  drawing.
- An arc's or a circle's plane in space (#363 S6) as a field of the arc or circle, for example
  `RoundIn { plane, ties, gauge }`, replacing the side tables `stands_for` and `round_gauges`. That
  closes two of the ledger's "gaps, not choices".
- A development's held length as a property of `CurveBody::Developed`, which it already implies.

### 2. One derivation, keyed by owner

```rust
/// What an intrinsic row is derived from.  Exhaustive: a new structural fact is a new arm.
pub enum Owner { Axis(u32), Frame(u32), Normal(u32), Lift(u32), Twin(u32, u32), Round(EntRef),
                 Developed(u32) }

impl Sketch {
    /// The rows `owner` implies, as the model now says.  The only place an intrinsic row is made.
    fn rows_of(&self, owner: Owner) -> Vec<Constraint>;
    /// Replace `owner`'s rows with `rows_of(owner)`.
    pub fn derive(&mut self, owner: Owner);
    /// Every owner's rows, in a fixed order (axes, frames, normals, lifts, twins, rounds,
    /// developments), so ids and the block order are deterministic.
    pub fn derive_all(&mut self);
}
```

`Constraint::intrinsic: bool` becomes `Constraint::origin: Origin`, where `Origin` is `Stated` or
`Intrinsic(Owner)`. `is_intrinsic()` keeps the thirty-odd read sites that only ask "stated or not"
unchanged.

The constructors (`square_chart`, `stand_normal`, `push_plane`, `lift_point`, `tie_twin`,
`axes_along`, `arcs_in_space`, `circles_in_space`, `developments`) set the field and call
`derive`. Seeding (`stand_axis_through`, `seed_normal`, `turn_axis_along`) stays in the
constructors: a seed is a start, not structure, and `views::place` still owns it.

### 3. Copying the model never mints a row

`graft` and the JSON reader copy entities and their fields and then call `derive_all` once. Its
step `axes_along`, the #817 branch in `axes_and_planes` and `chart_square` are deleted, along with
the JSON reader's per-kind `stand_normal`, `spatial_chart` and `tie_twin`/`hold_twins` calls. The
ledger line `constraints: _ // intrinsic rows minted` becomes two: stated rows by `constraints`,
intrinsic rows by `derive_all`.

`turns.rs` and the elaborator's later passes that edit structure (`axes_along` removing a
plane's `PlaneAxis` rows, `retake_round_gauges`) change the field and call `derive`. They no
longer edit rows.

### 4. Rows are a cache, never read back as facts

This is the rule the code holds. No reader infers structure from an intrinsic row: the readers
listed in the inventory read fields. A row is still stored in `constraints`, so the solve, the
block order and the diagnosis read one list as today. It is a cache of `rows_of`, rebuilt by
`derive`, and `Origin::Intrinsic(owner)` says whose.

A row that is stored rather than compiled was chosen over computing intrinsic rows inside
`System::new`. It keeps the change out of the solver's hot path and out of the thirty-odd readers.
Compiling them is the further step if the cache ever disagrees with its owner, and the gates below
are what would show that.

## Gates

- **A rebuild keeps every freedom** (`rebuild.rs`, beside
  `a_rebuild_that_removes_nothing_keeps_everything`). For every document of the corpus, the
  rebuild that removes nothing and the JSON round trip have the source's dof and rank, and the same
  intrinsic rows by `(Owner, kind)` once renumbered. This is the independent check: it holds
  whoever adds the next fact. It lands first, in stage 0.
- **A part keeps its owners' rows.** For each owner in a part (`Part::around_all` from each entity
  of a corpus sample), the part's rows of that owner are `rows_of` of the copy. #817's two tests
  (`a_part_rebuilt_around_a_seated_instance_keeps_its_frame_square`,
  `a_part_seated_off_the_screws_line_leaves_the_station_held`) stay as named cases.
- **The compiler**: `Owner` is matched exhaustively in `rows_of`, and `Chart`, `ArcE`, `CircleE`
  and `CurveBody` are destructured exhaustively by `graft` and the JSON writer.
- **No coordinate moves**: every example solves to the same coordinates before and after each
  stage, checked as `examples_sv` does.

## Stages

Each stage leaves the suite green and lands on its own.

0. **Measure.** The rebuild-keeps-every-freedom gate, run over the corpus and the JSON round
   trip. The cases it fails are the "to measure" cells of the inventory, recorded here. Fixes for
   them wait for the stages below, apart from a silent wrong-geometry case that a user hits, which
   is fixed at once, as #817 was.
1. **Owners for the rows already derived.** `Origin`, `Owner`, `rows_of` and `derive` for axes,
   planes, normals, lifts and twins, the rows already recorded on fields. The constructors, `graft`
   and the JSON reader go through `derive`. No behaviour changes.
2. **Records become fields, one at a time:** square (deleting `chart_square`), along (deleting
   `graft::axes_along` and the `Parallel3` filters in `views.rs` and `duals.rs`), the shared place,
   rounds in space (closing `stands_for` and `round_gauges` in the ledger), the developed length.
   Each closes a failing case of stage 0's gate.
3. **The flag goes.** `intrinsic: bool` is removed and the contract is rewritten: the `graft`
   paragraph of [app.md](contract/app.md), [space.md](contract/space.md)'s planes and normals,
   [language.md](contract/language.md)'s square frame, and [solver.md](contract/solver.md) for the
   rule that rows are a cache.

## Not in this plan

- **Whether every frame is square.** The Bible defines a frame as "an origin with three
  right-handed axes" and says "a plane is a frame", but the language gives `P := plane` seven
  freedoms: `plane(u: a, v: b)` spans two lines at any angle, and only an instance's frame is
  square. Deciding that every chart is square is a language change, with a changelog entry and a
  meaning for a plane over two lines that are not square. After this plan it is one line of
  `rows_of`. It is the owner's decision and is left open here.
- **Naming what a station leaves free** (#817's second ask). A refusal that carries the free
  entities, not a string, so the core names them as the author wrote them. That is a separate
  change to how a solid's refusal reaches `program::refused`.
- **What a station holds.** A station still solves parts the running number cannot reach, such as
  #817's seated discs. Holding what is upstream of no row reading the parameter is an optimisation
  of `solid::family::part_of`, measured before it is built.
