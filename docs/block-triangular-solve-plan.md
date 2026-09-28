# A block-triangular solve: plan

A document stays one unordered set of statements; the solver finds the order. This plan adds
a numeric **block-triangular solve** to `System`: the compiled rows are matched to the free
columns, the square part is sorted into strongly connected blocks, and each block is solved on
its own with every column it does not own held where the blocks before it put them. Nothing in
the language changes, and no document states an order. This is what Modelica does with
equation-based models (block-lower-triangular sorting, then a Newton solve per block), and
what the geometric decomposition (`decompose`, `cgraph`) already does for the figures its
cluster vocabulary covers.

## Why

The spiral-bevel layout is 357 equations over 357 unknowns, and today it is solved as one
system (`solve::solve`, one DogLeg with an LM retry). Measured with `tests/block_structure.rs`,
it is 116 blocks in a chain at most 15 deep: 71 blocks of two rows, one of 40 rows (the
pinion's pitch cone), one of 24 (the gear's), three of 16 (the rack sections) and nothing else
above six. Solved as one system it needs closed-form seeds everywhere (27 seed formulas in the
example use trigonometry), and with the normal module left an unknown it stalled from
module 10 upward, although that unknown is settled in one upstream block and only read
downstream. Solved in order, each block would start from its predecessors' solution and the
seeds could be rough.

The geometric decomposition does not help here: every spatial kind is unsupported in `cgraph`,
and `PlanSolver`'s "numeric residue" is a whole-document `System::solve`, not a sub-solve.

## Design

**Where.** Inside `System::solve`, as a stage of `solve_compiled` (`solve.rs`): after the
DogLeg attempt fails and before the LM retry. A document that solves today solves by the same
path and produces the same bits; the block solve only runs where the whole-system solve has
already failed. Drags run with `retry` off (`PAIRED`) and never reach it. `PlanSolver`'s
fallback, solventc, the drawing compiler, the fixtures and the FFI all reach it through
`System::solve`.

**The order.** `System::block_order()`: `structure()` → `graph::dulmage_mendelsohn` → Tarjan
over the well-determined part (the code in `tests/block_structure.rs`, moved into the core as
`graph::blocks`). Blocks in topological order, each a list of rows and the columns matched to
them; deterministic (ordered containers, ties by row index). Memoised on the `System`, which is
compiled per topology. The over- and under-determined parts of the DM decomposition are not
blocks: they are left to the final polish (below).

**A block solve.** A `TrustRegion` over one block (`BlockTr`), minimised by the existing
`newton::dogleg` (CLAUDE.md: a new thing to minimise implements the trait). It evaluates only the
block's rows and differentiates only against its columns, reading every other column from the
current `z`. That needs one new seam in `System`: residuals and a CSR Jacobian for a *subset* of
constraint instances (today `residuals_into` and `compute_csr` evaluate every kernel block).
Each block's instances are precomputed as (kernel block, instance) pairs. Rows keep the full
system's `row_scale` and columns its `col_scale` — a block is never compiled as a sub-sketch,
whose different `extent` would scale it differently — and no `System::new` runs per block, so
traced curves keep their `locus` warm poses.

**Acceptance.** A block is accepted when its rows meet `acceptance_tol` (the fixtures ask
1e-12). A block that does not converge ends the block pass: the result so far is kept only if
the polish succeeds from it.

**The polish.** After the block pass, one whole-system DogLeg from the block solution, as today.
This handles the DM over/under parts (a document with degrees of freedom left, or redundant
rows), removes any leftover coupling error, and means the pose the diagnosis is handed came
from a whole-system solve: `stationary` and the conflict analysis see what they see today. If
the polish fails, the LM retry from the original `z0` still runs, and the best result is kept,
as now.

**Curves.** The rehome loop stays outside `solve_compiled`; a contact that leaves its span
during the block pass is handled by the same clamp-and-retry.

## Phases

1. **The seam.** *Done.* `graph::blocks` (Tarjan over the matched square part; blocks in a
   solve order by Kahn's walk of the condensed graph, the lowest row first among ready blocks)
   and `System::block_order` (memoised per compile: each `SolveBlock` its full residual rows,
   its matched columns and its (kernel block, instance) pairs; the over- and under-determined
   parts beside them). `System::subset` / `subset_residuals` / `subset_csr` evaluate a subset
   of instances against a subset of columns, the whole system's `row_scale` and `col_scale`,
   contiguous instances one kernel call on the same slices. `tests/block_order.rs`: a chain of
   triangles is four two-row blocks at levels 1–4; a closed linkage is one four-row block; a
   free point is the under part and a length stated twice the over part; the hypoid layout is
   116 blocks, 15 deep, sizes as measured, every block reading only earlier blocks' columns;
   and every block's rows and Jacobian entries equal the whole evaluation's **bit for bit** on
   the triangles, `gear_trace.sv` (traced contacts: a trace's consts are read at the address its
   memory is keyed by), `hypoid_pitch_cones.sv` (spatial) and the layout (scaled rows and
   columns). `tests/block_structure.rs` is a report over `block_order`.
   *Finding:* the coarse DM puts every row a redundant row reaches — everything upstream of it —
   in the over part, so one repeated statement downstream leaves nothing to order. A fine DM of
   the over part would recover the blocks; not needed yet.
2. **The block pass as a rescue stage.** *Done.* `BlockTr` (only the block's rows and columns,
   the rest read from `z`; dense minimum-norm step up to `DENSE_MAX` columns, the regularized
   normal equations above, `newton::normal_step` shared with the whole system's sparse path),
   `blocks_then_polish` and `SolveOpts::blocks: BlockMode` — `Rescue` (default: after a failed
   DogLeg, before LM, only with `retry`, only with at least two blocks, kept only if it solves),
   `First` (the pass and polish before the whole-system DogLeg, which with LM becomes the
   retry; for phase 4) and `Off`.
   *Findings.* (a) A block held to the fixtures' 1e-12 with the whole solve's relative `xtol`
   (1e-12) stops short: an `angle(180deg)` on a 1.6 mm line needs a step of a few 1e-12 mm to
   take its last 4e-12 radians. Accepting blocks at 1e-6 instead does not help, since the
   whole-system polish then stalls at ~1e-11 on the same `xtol` (from good seeds it passes over
   that point in one quadratic step). Blocks therefore run with `BLOCK_XTOL` = 1e-15 and are
   accepted at the caller's `acceptance_tol`. (b) The design's own seeds at module 25.4 already
   defeat the whole-system DogLeg (LM rescued them before); the block rescue now solves them
   first, within the same 1e-9 of the recorded numbers (8.1e-11 at worst).
   (c) *The rough-seed gate* (`hypoid_layout::the_layout_solves_from_rough_seeds_in_block_order`):
   each of the twelve designs started from the seeds the layout computes at 0.4× and 2.5× its
   size (mean module and offset scaled — right in shape, wrong in size). The whole-system solve
   (DogLeg then LM) fails from 23 of the 24 starts (max residual 6.5e-7 to 2.4e-2); the 24th
   (32×32 m0.2 from 2.5×) solves on another root of the mate section, 2.3e-2 off. The block path
   (`Rescue` and `First`) solves all 24 and lands on the recorded pair: worst 8.1e-11 (a length
   over the cone distance, or an angle), against the gate's 1e-9. Under the test profile a rough
   start solves in 30–40 ms as a rescue (the failing DogLeg included) and in about 5 ms with
   `First`, where the failing whole-system DogLeg and LM take 45–75 ms. (d) What does not work: uniform random noise on every
   unknown (5–50 %) lands blocks on other roots (the mate section's arcs), since a seed picks
   the root; and removing every formula seed (the language's scattered unit-size starts) leaves
   the gear's toe-sphere diameter block — its two ends started at one place — in the collapsed
   basin. Phase 3's crude hints must keep each block's own start non-degenerate and on its
   branch. (e) A genuine conflict is diagnosed as before (`tests/block_solve.rs`): the rescue
   fails, is discarded, and the pose is the whole-system solve's to the bit; the minimal
   conflict set is the three impossible lengths. Over- and under-determined parts go through the
   polish (`First`): a free point stays exactly where it was, a repeated length and a point on a
   circle are satisfied. The same rough start solves to the same bits twice.
   *Gates (both phases):* the corpus and the fast export goldens byte-identical to `main` (every
   example's report and sheet, every export: no document in them fails its whole-system DogLeg
   with two blocks or more and solves in blocks); the full suite with OCCT, the slow tier with
   OCCT, and the wasm build with the web suite, all green.
3. **Use it.** In the spiral-bevel example: the normal module becomes a constructed length again
   (no `cos`), and the seed formulas shrink to rough hints where the block solve makes them
   unnecessary. Re-run the layout's regression and the slow tier; interference and exports
   unchanged within mesh noise.
4. **Measure, then decide on block-first.** Time the hypoid solve and the corpus both ways. If
   solving in blocks first (then polishing) is faster or more robust across the corpus, making it
   the default is a separate decision: it would change solved coordinates at the ULP level in
   every document, so every byte gate and exact-equality test would need re-recording (the
   exact `assert_eq!`s in `drag.rs`, `decompose.rs`, `unseeded.rs`, `anonymous.rs`, `io.rs`).

## Risks

- **A block with no good start.** A block's own seed still matters; the pass removes upstream
  error, not a poor local seed. The polish and the LM retry remain as fallbacks.
- **Branch choice.** A block solved alone could land on a different root than the whole solve
  would; orientation predicates (`ccw`) and signed words still hold inside the block, and the
  recorded-number gates catch a different root.
- **Cost.** The DM matching and Tarjan are linear-ish in the nonzeros and are memoised per
  compile; the pass runs only after a failed solve in phase 2.
- **Diagnosis.** Unchanged by construction: it receives the full `System` and a pose from the
  whole-system polish.

## Files

- `rust/gcs-core/src/system.rs`: subset evaluation, `block_order`.
- `rust/gcs-core/src/graph.rs`: `blocks` (Tarjan over the matched square part).
- `rust/gcs-core/src/solve.rs`: the stage in `solve_compiled`; `BlockTr`.
- `rust/gcs-core/tests/`: the seam, the ordering, the rough-seed hypoid test.
- `rust/examples/spiral_bevel/`: phase 3.
