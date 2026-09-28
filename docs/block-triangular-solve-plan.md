# A block-triangular solve: plan and record

A document stays one unordered set of statements; the solver finds the order. This plan added
a numeric **block-triangular solve** to `System`: the compiled rows are matched to the free
columns, the square part is sorted into strongly connected blocks, and each block is solved on
its own with every column it does not own held where the blocks before it put them. Nothing in
the language changed, and no document states an order. All four phases are done; the default
is the rescue (below), and the follow-ups are listed at the end. This is what Modelica does with
equation-based models (block-lower-triangular sorting, then a Newton solve per block), and
what the geometric decomposition (`decompose`, `cgraph`) already does for the figures its
cluster vocabulary covers.

## Why

The spiral-bevel layout was 357 equations over 357 unknowns, solved as one system
(`solve::solve`, one DogLeg with an LM retry). Measured with `tests/block_structure.rs`,
it is 116 blocks in a chain at most 15 deep: 71 blocks of two rows, one of 40 rows (the
pinion's pitch cone), one of 24 (the gear's), three of 16 (the rack sections) and nothing else
above six. Solved as one system it needed closed-form seeds everywhere (27 seed formulas in the
example used trigonometry), and with the normal module left an unknown it stalled from
module 10 upward, although that unknown is settled in one upstream block and only read
downstream. Solved in order, each block starts from its predecessors' solution, and the seeds
can be rough.

The geometric decomposition does not help here: every spatial kind is unsupported in `cgraph`,
and `PlanSolver`'s "numeric residue" is a whole-document `System::solve`, not a sub-solve.

## Design

**Where.** Inside `System::solve`, as a stage of `solve_compiled` (`solve.rs`): after the
DogLeg attempt fails and before the LM retry. A document that solved before solves by the same
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
current `z`, through one seam in `System`: residuals and a CSR Jacobian for a *subset* of
constraint instances (`System::subset`; `residuals_into` and `compute_csr` evaluate every kernel
block, and the two share every step of it). Each block's instances are precomputed as
(kernel block, instance) pairs. Rows keep the full
system's `row_scale` and columns its `col_scale` — a block is never compiled as a sub-sketch,
whose different `extent` would scale it differently — and no `System::new` runs per block, so
traced curves keep their `locus` warm poses.

**Acceptance.** A block is accepted when its rows meet `acceptance_tol` (the fixtures ask
1e-12). A block that does not converge ends the block pass: the result so far is kept only if
the polish succeeds from it.

**The polish.** After the block pass, one whole-system DogLeg from the block solution.
This handles the DM over/under parts (a document with degrees of freedom left, or redundant
rows), removes any leftover coupling error, and means the pose the diagnosis is handed came
from a whole-system solve: `stationary` and the conflict analysis see what they saw before. If
the polish fails, the LM retry from the original `z0` still runs, and the better result is kept,
as before.

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
   the over part would recover the blocks (a follow-up).
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
3. **Use it.** *Done.* In the spiral-bevel example the normal module is a constructed length
   again, and the seed formulas are rough hints wherever the block solve makes them unnecessary.
   * *The normal module.* `design.sv` no longer states it (it was `mean_module *
     cos(spiral_angle)`). `HypoidLayout(front, design, normal_module: Length)` is instantiated
     with the formal unbound, so it is one unknown of the solve; `ToothTrace` constructs it
     (`K distance(normal_module) normal`, the claim it used to be) and the depths read it through
     the formals of `MemberLimits`, `CrownTooth`, `CrownMate`, `RackSection`, `MateSection` and
     `TipRounding`. The layout is 358 equations in 117 blocks, 15 deep: the normal module is a
     block of one row in the trace, read downstream. A module's preview that draws the crown ties
     its sections' unknown to the trace's with the same row (`trace.K distance(tooth.normal_module)
     trace.normal`), since a root instance's argument cannot name another instance's unknown.
   * *The seeds.* Trigonometric code lines in the example went from 32 (29 seed-only, the design's
     one `cos`, and two preview constants written with `hypot`) to 3: C's seed in `ToothTrace`
     (below) and the two preview constants. Lines with `sqrt` went from 20 to 8 (the cone distance
     in five seed `param`s and the design, and `CrownTooth`'s three projections onto the normal
     view, which the crown sections are seeded from). Replaced by rough hints: the crown sections'
     corners (in pitch widths, a flank leaning by its pressure angle in radians, a normal module
     taken as 2/π of the pitch width), the tooth thickness (the quarter pitch straight across from
     M, the trace circles 0.7 module either side of the cutter radius), the pinion's cone (apex
     about the offset aside, the bevel pinion's axis), the gear's triangle foot, H, the cutter
     reach, the face span, and the cone boundaries' feet and crossings (`hint(at:)` on the axis).
   * *What had to stay.* C's exact seed: the normal view N is folded along MC and the crown
     sections are seeded in it, so with C seeded roughly the whole-system solve at 44 of 480
     designs stalled in a basin where a section's narrow tip collapses (`hypoid_layout::
     design_sweep`), against 13 with it exact. The same sweep also found the failure mode that
     rough seeds make common: a whole-system DogLeg that runs out of iterations at a residual of
     1e-7 is a *success* by the interactive acceptance (1e-6), so the rescue never runs and the
     drawing is a pose up to 1e-3 of the extent off the solution. The crown previews did exactly
     this with tip seeds narrower than the tip; `every_modules_preview_converges_as_solventc_
     solves_it` now holds every module's preview to a converged residual. At `HEAD` before this
     phase (exact seeds, stated normal module) the sweep found 151 of 480 designs doing it; now 13:
     seven stalls (24×48 at module 2, shift 7.5–12.5°), four designs 0.71 R off (13×40 at module
     25.4, E 381 mm) where the block path fails too and the default rescue's polish is accepted at
     4.9e-8, and two where the default solve is right and `First` is the one that fails.
   * *Per design (the regression's twelve, held to 1e-12).* The whole-system DogLeg solves the
     bevel pair, hypoid6 and the six module 0.2 and 2 sizes (8–16 ms under the test profile); the
     block rescue solves the configured hypoid and the three module 25.4 sizes (36–37 ms, the
     failing DogLeg included; 4 ms with `First`). Before, the DogLeg solved nine and the rescue the
     three at module 25.4. Every design matches the recorded numbers to 1e-9 (worst 8.2e-11).
   * *Rough-seed gate.* From the rough seeds at 0.4× and 2.5× size the block path still reaches the
     recorded pair from 23 of 24 starts; the configured hypoid from 0.4× stops a few 1e-9 short in
     a crown section's block (trust region collapsed) and is allowed to, one start of that design
     only. The whole-system solve reaches the recorded pair from none (one lands on another root).
   * *Gates.* The full suite with OCCT (1340 passed) and the slow tier with OCCT (1347), wasm and
     the web suite (255); the corpus and fast goldens byte-identical to phase 2 outside the spiral
     bevel. Its reports agree with phase 2's to 1.7e-9 relative (a space cutter's face area), its
     field-meshed members' volumes to 1.4e-4 (pinion, 8260 → 8204 triangles) and 6.5e-5 (gear,
     17184 → 17370), and the refined exports refuse at the same feature-curve balls. Measured off
     the field meshes, the shafts are 90.000° apart and 24.992 mm off (25.001 before), and through
     one gear pitch the members overlap by 0.44–1.15 mm³ (0.44–1.10 before).
4. **Measure, then decide on block-first.** *Measured; the default is unchanged.* The corpus is
   every example that elaborates (90 documents under `rust/examples/`, the three that are only
   parameters skipped) and the regression's twelve spiral-bevel designs (`gears.sv` at each), 102
   in all, each solved as solventc solves it (`SolveOpts::default()`, a `System` compiled per
   solve). The tool is `tests/block_measure.rs` (ignored; `timing`, `robustness`, `blast_radius`).
   *Timing.* Wall-clock medians of 7 solves per document and mode, under the test profile, three
   runs (totals within 2% of each other); the machine's load average was 7–10 from a virtual
   machine while `top` read the CPU 90% idle. In milliseconds:

   | | Off | Rescue | First |
   |---|---:|---:|---:|
   | whole corpus (102) | 795–809 | 520–533 | 94–95 |
   | spiral bevel (22 modules and 12 designs) | 725–740 | 447–454 | 48 |
   | the rest (68) | 69–70 | 73–79 | 46–47 |
   | `spiral_bevel@configured` (358 rows, 117 blocks) | 75.1 (fails) | 37.3 (blocks) | 2.5 |
   | `spiral_bevel@bevel` | 14.3 | 14.9 | 1.8 |
   | `gear.sv` (363, 123) | 19.2 | 19.2 | 7.2 |
   | `gear_trace.sv` (147, 51) | 13.2 | 13.0 | 4.8 |
   | `vtwin/assembly.sv` (680, 195) | 9.4 | 8.9 | 5.2 |
   | `engine.sv` (1327, 671) | 8.2 | 8.5 | 10.4 |
   | `truss200.sv` (800, 400) | 0.47 | 0.45 | 1.86 |
   | `truss_conflict.sv` (fails every way) | 6.5 | 9.2 | 9.3 |

   `First` is far faster where the whole-system solve works hard (the spiral bevel, 6–30×; the
   gears and the V-twin assembly, 2–3×), and slower on the small and the easy: of the 49 documents
   outside the spiral bevel with two blocks or more, 35 are more than 10% slower under `First` and
   11 faster (the ordering and the per-block setup cost more than a DogLeg that converges in a few
   iterations — `truss200`'s 400 two-row blocks, 4×). `Rescue` costs nothing where the DogLeg
   succeeds and adds the pass to a document that fails anyway (`truss_conflict`, +40%).
   *Robustness.* From each document's default solution, scaled about the free points' centroid by
   0.5 and by 2 and jittered three times by up to 1e-3 of the extent (seeded), 495 starts; for the
   85 documents with no freedom left (425 starts) a solve is on the reference root when every
   point in space and every radius is within 1e-6 of the extent of the default solution:

   | | solved (all 495) | solved (425 determined) | on the reference root | halved | doubled | jittered |
   |---|---:|---:|---:|---:|---:|---:|
   | Off | 464 | 396 | 349 | 61 | 58 | 230 |
   | Rescue | 488 | 420 | 352 | 64 | 58 | 230 |
   | First | 488 | 420 | 361 | 63 | 49 | 249 |

   `Rescue` and `First` solve the same starts (24 more than `Off`, nearly all the spiral bevel's).
   `First` returns to the reference root more often from a jittered start (249 against 230: each
   block starts from its predecessors' answer, so a small perturbation stays small), and less
   often from a doubled one (49 against 58: a block solved alone from a start twice the size
   finds another root — the V-twin's bank, cylinder, frame and piston, the trusses).
   *Blast radius.* Solved from their own seeds, `First` and `Off` agree to the bit in 51 of the
   102 documents — those with fewer than two blocks, or whose DogLeg `First` never replaces —
   and differ in 51. In 39 of those the difference is rounding (at most 1e-11 of the extent),
   and in eight it is where the whole-system solve fails (the spiral bevel's layouts; there
   `First` gives the rescue's answer). In four it is **another root**: `engine.sv` (683 of 1329
   parameters, up to 0.48 of the extent), `k33.sv` (2.1), `laman.sv` (0.39) and
   `vtwin/assembly.sv` (6.4e-3) — mechanisms and multiply realizable graphs whose own seeds pick a
   root the whole-system solve finds and the block order does not.

## Recommendation

Keep `Rescue` the default. `First` would change the bits of half the corpus and the root of four
documents (a different drawing, not a re-recorded ULP), make most small documents slower, and
trade robustness to jitter for robustness to scale; its speed matters only where the
whole-system solve struggles, which is exactly where `Rescue` already runs it.

## Follow-ups

- **An iteration-limit stop is not rescued.** A whole-system DogLeg that stops on its iteration
  limit under the interactive acceptance (1e-6) is a success, and phase 3 found that to be the
  commonest failure with rough seeds (the sweep's 13 of 480; the rescue never runs). Treating
  that stop as a failure for the rescue's trigger is a narrow change worth measuring on its own;
  `every_modules_preview_converges_as_solventc_solves_it` guards the example meanwhile.
- **A document that asks for `First`** (a hint in the source, or `solventc --blocks first`),
  where its author knows the layout is a long chain, as the spiral bevel is.
- **A fine DM of the over-determined part** (phase 1's finding), so one redundant statement
  downstream no longer puts everything upstream of it outside the blocks.

## Risks, as they came out

- **A block with no good start.** A block's own seed still matters; the pass removes upstream
  error, not a poor local seed (phase 2 (d): unit-size scattered starts collapse the toe-sphere
  diameter's block). The polish and the LM retry remain as fallbacks.
- **Branch choice.** A block solved alone can land on a different root than the whole solve
  would: `First` does in four documents (phase 4). Orientation predicates (`ccw`) and signed
  words still hold inside a block, the recorded-number gates catch a different root, and
  `Rescue` never replaces a whole-system success.
- **Cost.** The DM matching and Tarjan are linear-ish in the nonzeros and are memoised per
  compile; under `Rescue` the pass runs only after a failed DogLeg, and adds its time to a
  document that fails anyway (`truss_conflict`, +40%).
- **Diagnosis.** Unchanged by construction: it receives the full `System` and a pose from the
  whole-system polish, or the failing solve's own pose when the rescue is discarded.

## Files

- `rust/gcs-core/src/system.rs`: subset evaluation, `block_order`.
- `rust/gcs-core/src/graph.rs`: `blocks` (Tarjan over the matched square part).
- `rust/gcs-core/src/solve.rs`: the stage in `solve_compiled`; `BlockTr`.
- `rust/gcs-core/src/newton.rs`: `normal_step` and `min_norm_step`, the Gauss–Newton steps
  `BlockTr` shares with the whole system.
- `rust/gcs-core/tests/`: `block_order.rs` (the order and the seam), `block_solve.rs` (the
  rescue, the polish, a conflict), `hypoid_layout.rs` (the rough-seed gate and the preview
  convergence gate); the tools `block_structure.rs`, `block_measure.rs` (phase 4's timing,
  robustness and blast radius) and `hypoid_layout::design_sweep` / `solve_paths`. The corpus
  and the twelve designs are read through `fixtures::examples` and `fixtures::gear::designs`.
- `rust/examples/spiral_bevel/`: phase 3.
