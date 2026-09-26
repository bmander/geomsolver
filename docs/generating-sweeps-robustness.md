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
- the Manifold 15° space (removed with the Manifold backend, 2026-09-25);
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
  admission, blank, clearance, reach, sheet, fit, withheld, split, classify, fuse, step, stl,
  mesh, agreement, written (`cad::progress::mark`, keys `solid::export::Stage::key`). A refusal
  appends `refused:<stage>`, the stage its `ExportRefusal` was met at, which the harness reads
  rather than inferring the stage from the message.
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

## Step 2 results (2026-09-23): contracts at the stages

Each contract is a check at the stage whose output it judges, and a violation is a refusal
naming that stage.

- **Admission.**
  - **M2 now covers poles.** A face's profile meeting the tool's axis has no normal of its own,
    so the sampled checks skipped it. Its normal is the axis, found square to the small circle a
    point beside the pole turns on. A pole in contact at every time whose path enters the blank
    is refused.
  - **E0 (new):** a sweep none of whose contacts reach the blank is refused, not admitted
    vacuously.
  - **Ordering:** clearance (E1) is still reported before either.
- **Reach.** A degenerate point is skipped only when its path over the roll never enters the
  blank. Otherwise it is refused, now with a true message: the contact equation is degenerate,
  at this point, with this normal.
- **Sheet: the chart contract.** Wherever the grid lies in the blank, each column keeps one root
  of the contact equation (`ContactTime::branch`, which changes only where the two roots merge)
  and its contact time does not leap by more than a radian (the same root a turn away).
  - **Position is deliberately not tested.** A first version flagged steps over ten times a
    column's median, and so refused the 25° pinion, which exports correctly: near a time-chart
    fold a contact runs fast along the sweep for a small step along the profile.
  - **Trim replaced.** Last session's margin trim is gone. The widening margin is back, and the
    contract is judged at the blank.
  - **Class C traced.** The trim was itself the cause of class C on the 15° pinions: without it
    they export with every probe agreeing.
- **Fit.** A contact is withheld at the centre of every grid cell, a section at each column's
  mid angle and each row's mid length. At those in or within 0.5 mm of the blank, the fitted
  sheet must pass within 0.25 mm and its normal within 20° of the contact's. Feet are found by
  one projector per face (`solvent_cad_surface_feet`).
- **Split.** A deadline on the kernel's splitter, 15 s plus 5 s per sheet, through its progress
  indicator. The kernel polls it only between phases, so the message gives the budget and the
  time actually spent. Validity failures name the kernel's status (`BRepCheck_UnorientableShape`
  rather than 27).
- **Cells.** After classification:
  - no cell under 10⁻³ mm³;
  - one removed cell per placement;
  - the removed cells congruent to 10⁻⁴.
- **Mesh.** No more than 100 triangles under 1 µm². More is refused with their total area and
  bounding box.

**Harness after step 2:**

- **Controls: all green.**
  - The bevel spaces export.
  - The 25° pinion exports again: it was a regression of the trim.
  - The 25° gear is refused at the split (`BRepCheck_UnorientableShape`) instead of exporting its
    slivers.
  - The symmetric racks are refused at admission.
  - The Manifold arrangement is refused at the mesh contract (797 triangles under 1 µm²), before
    the probe.
- **Design sweep: no case now reaches the final gate and fails.** The one that timed out is now
  a named split refusal.
  - **Exported:** every pinion admission admits, except 15/0/25, which is refused at the split
    as unorientable. Also every gear with a 35° spiral and a pressure split of 5° or less.
  - **Refused:** the rest of the gears, all at the gear cutter's crease (class B):
    - with a 10° split and a 35° spiral, by the mesh contract (10 500 triangles under 1 µm²,
      in one 1.6 mm box);
    - with a 25° spiral, by the fit contract (normals 61–78° off at distances of 0.004–0.02 mm:
      the sheet collapsing towards the crease line), an unorientable split, a cell without an
      interior sample (a 1.4 mm³ sliver among them), or the split's budget.
- **Fixtures.**
  - The sphere cases are refused at admission, poles in the blank.
  - The torus is refused at the sheet: roots swap in the blank, the sheet following roots
    outside the declared roll through their fold.
  - The lens is refused at reach (class E).
  - The tangent torus is refused at the sheet.

**What step 2 leaves for step 3.** Every remaining refusal comes from one of two causes:
- **Walking the sheet by nearest time over ±180°.** This covers the torus, and branch ends
  generally.
- **Sampling the crease fan into a sheet that collapses there.** This covers the gear cutter
  with a 25° spiral or 10° split, and presumably the pinion 15/0/25 as well; step 3 should
  confirm that.

Both are what step 3's charts from analytic data replace:
- a chart per root, bounded by the declared roll and its folds;
- the crease as its own edge-sweep chart, not a fan folded into a face's grid.

## Step 3 results (2026-09-23): contact curves traced, and what the fixtures taught

**The construction.** Each station's column is now its contact curve, traced in the plane of
walk length and time, not a grid filled by the nearest root:
- The curve starts from a contact in the blank within the roll. It runs both ways around the
  closed section loop until it has been outside the blank for the margin.
- Where its root meets the other, a fold of the time chart, it turns onto that root and back.
- Rows are spaced by unfolded walk length (τ), common to every column. Rows matched by each
  column's own arc-length fraction shear the sheet: every gear failed its fit normals at about
  90° that way.
- Stations whose contacts miss the blank are traced over the same span, anchored beside their
  nearest neighbour. The rows run over the span every column reached, which must hold every
  contact in the blank, or the sheet is refused as ending inside it.

**What the fixtures found, each fixed where it arose:**
- **Steep roots.** Where the contact equation's amplitude is small against its constant, a step
  along the profile can move the root by more than a radian. The walk now follows it in halved
  steps before calling the root ended, and finds where a root does end by halving too. A double
  root found there is the fold itself, not an error.
- **The time window.** Roots were sought over ±180°, so a curve whose time crossed that bound
  looked like a curve that ended. The window is now the declared roll and a turn either side, and
  a root ending at its edge is no fold.
- **Runaway guard.** The guard counts length spent in the blank. A curve outside it may run far,
  in space, before its span ends.
- **The station band.** The reach samples coarsely and missed stations whose contacts are in a
  thin post. Each end of the band now moves outward until the station there has no contact in
  the blank anywhere on its loop.
- **Seams.** The reach's stations sat at exact multiples of the step. The first was the profile
  plane, where every revolution's seam lies, and a section plane containing a seam loses that
  face's section. They are now half a step off.
- **Sections through the axis.** A face about another axis, or a sphere's meridian, gives one
  plane-section edge that crosses the axis. It was kept or dropped whole by its middle; it is
  now split at the axis. A chain open through the axis is also extended backward from its head,
  not only forward from wherever it started.
- **Extension re-entering the blank.** The sheet is rectangular, so it runs on past the roll.
  On a tall post, the torus turned on past its limit comes back into the post. There the sheet's
  edge lies in the blank at a time outside the roll. Widening only carries the extension
  further, so this is now a named refusal: a time-trimmed sheet is needed.

**Admission, sharpened by the same fixtures:**
- **Stationary points inside a sample cell.** With two axes that meet, a tool point whose normal
  passes through the crossing point is in contact at every time. On a sphere these are isolated
  points, which the between-samples test found only when one lay on a segment between samples.
  A cell around which (a, b) winds right round, with roots at every corner, is now refused too
  (M2).
- **A coaxial tool is not required.** The gear's cutter is its outer crown bounded by the inner
  crown's indexed neighbour, a revolution about another axis. A check that the faces share one
  axis refused both gear members and was withdrawn.

**The fixtures were redrawn to be what they claim.**
- The earlier "skew" roll spun the tool about its own axis. That changes nothing of a
  revolution, so the sweep was a single rotation with an awkward time chart. The tool is now
  carried about a cradle axis 1 mm off its own, as a generator carries its cutter.
- The observer turns about a line parallel to x through (0, 0.5, 0.5).
- Each geometry was chosen by a search against its independent truth: clearance at both roll
  limits, a deep cut, and the tool turned past the limits clear of the post.

**Fixture matrix after step 3:**

| Case | Result |
| --- | --- |
| Torus through a post (skew) | Exported, 0 of 750 truth probes disagree |
| Sphere through a post (skew) | Exported, 0 of 672 disagree |
| Ring lens (two tori intersected, a sharp rim, no pole) | Exported, 0 of 696 disagree |
| Torus through a tall post | Refused at the sheet: the extension re-enters the blank |
| Torus grazing the post's top | Refused at admission, E3 |
| Torus, meeting axes | Refused at admission, M2 (stationary ring) |
| Sphere, meeting axes, post taking in its stationary points | Refused at admission, M2 (inside a sample cell) |
| Sphere, parallel axes | Refused at admission, M2 (poles) |
| Lens about two parallel axes | Refused at the reach: one face meets a section twice |
| Lens with poles (class B reproducer) | Refused at the STL check: see below |

**Class B located: a pleat in our fitted sheet, not the kernel.** The spherical lens agrees with
its truth everywhere probed. Its mesh has 928 of 1675 triangles under 1 µm², and an edge used
three times.
- **Where:** all but one of those triangles lie on the sheet face, in a strip 0.7 µm wide and
  0.15 mm long, next to the post's wall. About half face backwards.
- **What:** a pleat of the fitted surface, not a defect of the split or the mesher.
- **What moves it:** how many rows the crease's fan is given. The walk weights a fan by its
  turning (0.5 mm a radian).
  - At 0.05 or 0.15 the lens exports, with every probe agreeing.
  - At 1.5 the split leaves a cell unseparated.
- **What didn't work:** weighting each fan by its measured sweep. At the stations measured, this
  crease's fan sweeps 3–10 mm, all at times outside the roll and away from the blank. What
  matters is only the part of the fan near the blank, which no single weight describes.

So step 4 (fitting with error control) has to treat the fan as its own chart, not as rows
folded into a face's grid. The 25° gear's crease slivers are presumably the same mechanism, so
they are the construction's to fix.

**Owners.** The harness now gives every refusal an owner, by the stage where it was met:
- the **class** (admission);
- the **construction** (reach, sheet);
- the **fit** (the fit and its withheld contacts);
- the **kernel** (blank, clearance, split, classify, fuse, STL, mesh);
- the **gate** (the field probe).

The matrix ends with a tally. This is where a refusal is met, not what caused it: the lens is
refused at a kernel stage for a pleat in our sheet.

**Design sweep after step 3:**
- **Exported:** 26 of 48, against 21 after step 2. The new ones are the 15/0/25 pinion and
  every gear with a 25° spiral and a 5° split.
- **Refused:**
  - by the class: 10;
  - at kernel stages: 11. Nine are at the mesh contract (crease slivers, every gear with a 10°
    split, and 0/0/25), and two are invalid splits (20/0/25, 25/0/25);
  - at the fit: 1 (gear 15/0/25).
- None fails at the final gate or runs out of time.
- The gear slivers form one long thin strip, 13% of it facing backwards: consistent with the
  lens's pleat.
- One gear space read directly by `native_surfaces` (`generic_sheet_reproduces_the_gear_tooth_space`)
  now fails its fit contract near the crease, and is ignored with that reason.

**What came next.** The export moved to meshing the field directly ([field meshing](field-meshing.md)),
which exports every admitted fixture, including the class B lens.
