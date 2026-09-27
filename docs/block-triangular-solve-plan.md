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

1. **The seam.** Subset residuals/Jacobian on `System`, with a test that a subset's rows and
   entries equal the full system's rows and columns bit for bit. `graph::blocks` in the core,
   tested on small documents with known block structure (a chain of triangles, a figure with a
   cycle) and on the hypoid layout (116 blocks, depth 15). `tests/block_structure.rs` becomes a
   thin report over it.
   **Gate:** the corpus and export goldens byte-identical (nothing solves differently yet).
2. **The block pass as a rescue stage.** `BlockTr`, the pass, the polish, wired between DogLeg and
   LM.
   **Gates:** the corpus and export goldens byte-identical (only failing solves take the new
   path); the full and slow suites; a new test that the hypoid layout solves from *rough* seeds
   (every seed formula replaced by a crude hint: the apexes near the origin, the mean point on
   its axis) at all twelve recorded designs, where the whole-system solve fails, and lands on the
   recorded numbers (`tests/fixtures/hypoid_layout.tsv`, 1e-9); a failing document (a genuine
   conflict) still reports its conflict set as before.
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
