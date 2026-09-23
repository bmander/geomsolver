# Generating sweeps: the error class, and a robust construction

Written 2026-09-22 after Phase 4's 25° gear. The gear example is a showpiece, but the subtasks it
exposes are the actual goal. This document names the class of error it keeps exposing and
proposes a construction and a test harness that address the class rather than its instances.

## What has gone wrong, side by side

| Where | Symptom | How it was found | Fix applied |
| --- | --- | --- | --- |
| Manifold mesh path, hypoid 15°–30° | 19–37% of the surface wrong; volume and shell passed | A field probe, weeks later | Path parked |
| Native sheet, 30° symmetric rack | "a cutter point has no contact time" | Construction error, unhelpful | Admission refuses the design first |
| Native sheet, 25° gear | Contacts jump 12–35 mm at the ends of the profile walk; fit off by 0.5 mm; split ground for minutes | By hand: a stuck process and a stack sample | Margins trimmed at a jump heuristic |
| Native sheet, 25° gear | A folded sliver strip at the crease between the cutter's crowns, 0.005 mm² | The final field gate, after 3 minutes | None yet |
| Earlier (roadmap) | Two sheets meeting tangentially made the kernel split fail | Construction error | One tangent-continuous sheet per column |
| Earlier (roadmap) | A cell judged 0.03 mm inside its boundary | Chance | Measured distances |
| Field probe | 40% of its samples on 0.005 mm² of micro-triangles | Counting | Not yet: sampling is by count, not area |

## The class

All of these are one kind of error. **An exact analytic description is turned into a numerical
representation, and the turn is not checked where it is made.** The pipeline goes through these
representations in order:

1. the contact equation per tool point, which is exact and algebraic;
2. a grid of samples chosen by heuristics: the root nearest the neighbour's time, margins grown
   by 1.6, a jump threshold of 10× the median;
3. an interpolating B-spline, checked only at withheld midpoints;
4. kernel Booleans with a fuzzy tolerance;
5. a mesh.

Each step can silently leave the true surface:
- **Step 2:** the chosen root changes branch, or the grid crosses a fold of the tool's time chart
  or a crease fan.
- **Step 3:** the spline oscillates or folds between samples.
- **Step 4:** near-tangent faces produce slivers.

The only arbiter is the field gate at the very end. It catches the result, but it can't say which
step failed. And the heuristics that pick branches and margins were tuned on the bevel pair, so a
new design is a new experiment.

Admission, Phase 2, shows the alternative. It evaluates the same exact quantities (contact times,
branches, the area factor) and decides from them. The construction doesn't use them: it samples
by continuity and hopes.

## The robust construction: charts from the analytic data, contracts at every representation change

**1. Build the sheet from regular charts, not by continuity.** The contact equation for a tool
point is a sinusoid in the roll, with at most two labelled roots (`ContactTime::branch`, `turn`).
A chart is a region of (tool point, branch) where:
- the root exists (discriminant > 0);
- the fold measure K = J·g′ is nonzero and of one sign (the quantity admission already uses);
- the contact lies inside the blank, or inside an explicit margin.

Chart boundaries come from the analytic data: where the discriminant reaches zero (a time-chart
fold, the "jump"), where K reaches zero (a surface fold), a profile vertex (the crease fan,
parameterised by normal angle), or the blank's exit. Nothing is chosen by nearest time or a
median threshold. A root outside every chart is a refusal naming its cause. This replaces
`sheet()`'s continuity walk, the ×1.6 margin loop and the jump trim.

**2. Fit each chart with a stated error.** Refine the fit until a dense check (every grid cell's
centre, not a third of the rows) is within a stated tolerance of the true contact. Also check
that the fitted surface's own normal agrees with the contact normal, which catches oscillation
and folding between samples. Refuse, naming the chart and the worst point, otherwise.

**3. State kernel pre- and post-conditions.**
- **Before the split:** the least angle between each sheet and the blank faces it crosses, and
  between neighbouring placements. Near-tangency is where slivers come from, and it should be
  refused or remodelled before the kernel sees it.
- **After the split:** the expected cell count (1 + placements); removed cells congruent in
  volume; no face or cell below a size floor.
- **After the fuse:** the same floors.

**4. Final acceptance that measures what it claims.**
- Field probes sampled by **area**, so a speck doesn't take 40% of the probes.
- Mesh quality counted separately: the area in triangles below a size floor, and their
  clustering. A cluster is a defect in its own right, named with its location.

The principle: every representation change carries a checkable contract, checked where the
change happens, and a failure names that stage. Reaching the final gate and failing there
means some stage's contract is missing, and the harness should treat it as that.

## The harness

**Fast fixtures, one per event class, through the native construction.** Each is a single
tooth space or small blank, seconds to build, with an independent answer:

| Event class | Fixture | Independent truth |
| --- | --- | --- |
| Regular chart | Sphere tool, relative roll, through a post | Tube about the centre's path: closed-form torus volume and distance |
| Time-chart fold in the margin | A tool whose contacts end past the blank | The discriminant's zero curve, located analytically |
| Surface fold (undercut) | 17.5° symmetric rack (Phase 1) | Refused at chart construction, row E3 |
| Crease fan | Lens tool (two intersected spheres) under a roll | A circle's sweep: closed-form torus section |
| Near-tangent blank face | A sheet grazing a plane of the blank | Refused before the split, with the angle |
| Neighbouring placements | Two spaces whose sheets meet at a pointed tooth | Refused, or the sliver volume measured |

**The design space as a property test.** Sweep a grid of designs: offset, pressure split,
spiral, tooth counts and module, all as single spaces at 15–40 s each.

- **The property:** each design is either refused at a named stage, or exported with every stage
  contract passing and the field gate agreeing.
- **What counts as a harness bug:** a failure at the final gate, a stuck stage (a time budget
  per stage), or an unnamed error.
- **What it records:** a status matrix of design, stage reached, contract values and time. The
  swept-boundary work's discipline applies: one status matrix, a negative control per class, and
  inspectable output even on refusal.

**Negative controls retained:**
- the Manifold 15° space;
- the 25° gear sheet as it is today (walk-end jumps plus the crease sliver);
- the 30° symmetric rack.

Each must be refused at its own stage once that stage's contract exists.

## Order of work

1. **Harness first, red.**
   - Build the fixtures and the design-space sweep, with the stage timeline and status matrix.
   - Make today's failures reproducible as failing cases, located by stage.
   - Measure how many classes the sweep reveals before building anything.
2. **Contracts as checks only:** after sampling, after the fit, before and after the split,
   area-weighted acceptance. Every current failure should move from "final gate" to its own
   stage.
3. **Charts from analytic data**, replacing the continuity walk and the margin and jump
   heuristics. The fixtures for time-chart folds, surface folds and crease fans go green.
4. **Fitting with error control.**
5. **Kernel preconditions:** near-tangency handling, for which the design of the remedy (refuse,
   blend or remodel) is itself a decision.

Each step's exit is a set of harness rows turning green, with the negative controls still
refused. The 25° gear is an acceptance row, not the lens the work is designed through.

## Step 1 results (2026-09-22): the harness, red

**How the harness runs.** `rust/gcs-cli/tests/generating_harness.rs` runs the real `solventc` for
each case in a child process with a wall-clock budget.
- **Stage trace:** `SOLVENT_STAGE_TRACE` makes `solventc` append a line for each completed stage:
  admission, blank, clearance, reach, sheet, fit, withheld, split, classify, fuse, stl,
  agreement, written.
- **Kept output:** `SOLVENT_KEEP_REJECTED` keeps a refused mesh.
- **What each case records:** the stage reached, where it was refused and with what message, the
  time and volume, and the field agreement. Where the case has one, it also records an
  independent truth's agreement: for the sphere, the exact tube; for the torus and lens,
  their least distance sampled over the roll every 10⁻⁴ rad. It measures the mesh's tiny
  triangles too, those under 1 µm².
- **The matrix** is written to `build/harness/<test>.md`.

**Tests.** Three ignored tests: `fixtures` (seconds each), `controls` (the gear's known cases
plus the negative controls) and `design_space` (48 single tooth spaces, about half an hour).

**Probe sampling.** The field probe now samples triangles by area. A 0.005 mm² sliver that
drew 40% of the count-weighted probes no longer passes for a surface fault. It is left to the
tiny-triangle measure, which is not yet a contract.

The classes the harness reveals, located by stage:

| Class | Cases | Where it is met today |
| --- | --- | --- |
| **A.** The walk reaches the end of a contact branch: a jump, or no root at all | Pinion 20/5/35 and 25/10/25 ("no contact time"); gear 25/10/25 (jumps, trimmed); the 25/10/25 pinion **regressed** under the trim | The sheet, with no chart reason |
| **B.** Crease-fan slivers in the gear cutter | **Every gear with a 10° pressure split**, including at 0° offset: 7–10k triangles under 1 µm², about 1.5×10⁻³ mm² | Nowhere: exports pass the area-weighted gate |
| **C.** Surface disagreement on a regular-looking case | Pinion 15/0/35 and 15/0/25 (15 and 14 of 2000 probes); the torus fixture (2 of 514 by the field, 1 against the independent truth) | Only the final gate |
| **D.** Stationary points: a pole on the spin axis, in contact at every time | The sphere fixtures (crossed and parallel axes) | The construction; admission skips points whose normal is undefined |
| **E.** A Boolean tool whose operands have different axes | The lens fixture | Reach: "a section of the cutter did not close into a loop" |
| **F.** A sweep that never reaches the blank | A torus through its own hole | Reach; admission admits it vacuously |

What holds:
- The negative controls: Manifold 15° refused at agreement; 17.5° and 30° symmetric racks
  refused at admission (E3, E2).
- The bevel spaces, and most of the design space, export cleanly with no tiny triangles.
- The torus grazing a post's top is refused at the sheet, with a message.

The degenerate-contact refusal said "does not depend on the motion" for any degenerate root,
a double root included. It now says the contact equation is degenerate, with the point and
normal.

Step 2 turns the stage at which each class is met into the stage that names it:
- **A:** a chart-validity check on the sheet grid, covering branch identity and discriminant.
- **B:** a mesh-quality contract.
- **C:** a dense fit check and a check after the split, to find which stage C comes from.
- **D:** admission names degenerate points, and does not skip them.
- **E:** refused at the tool, with its cause.
- **F:** admission refuses a sweep that reaches nothing.
