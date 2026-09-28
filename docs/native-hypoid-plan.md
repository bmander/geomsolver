# The hypoid pair for real: plan

Export both members of the configured hypoid pair (`rust/examples/spiral_bevel/gears.sv`) as
fabrication-grade files, by the native construction and not by field meshing: a **STEP** master
(exact analytic cones and spheres, fitted generated flanks and fillets) within **10 µm** of the
exact generated surface, an **STL** meshed from it at that tolerance, and a design a shop could
make: stated **backlash** and **edge relief**. The material field stays what it already is in the
native path — an independent check (admission, agreement) — and becomes one of two meters; it
builds nothing.

## Where it stands

- **Pinion:** exports natively in about 98 s. Its tooth-space sheet (73×44 samples, one chart)
  fits the withheld contacts within 16.5 µm, normals within 8.2°; field agreement probes 0.1 mm off
  each side and finds no disagreement.
- **Gear:** refused until phase 2. Its sheet (77×32) fitted the contacts within 60 µm, but its
  normals missed by 83° against the 20° bar. It now exports natively in about 3.5 minutes (see
  Phase 2's findings).
- **The bar is coarse:** `FIT_DISTANCE = 0.25` mm and `FIT_TURN = 20°` in
  `gcs-cli/src/cad/native/sweep_boundary.rs`. Passing it is not a fabrication claim for either
  member, and nothing measures the exported file against the exact surface.
- **The cutters differ:** the pinion is cut by one revolved crown tooth; the gear by the space
  between two mate teeth, `outer_crown` bounded by its indexed neighbour
  (`crown/space.sv`) — a Boolean with a crease.

## Phases

### 1. A meter

An independent measure of an exported file's accuracy, before anything else changes:

- sample points on each STEP face (OCCT surface evaluation) and each STL triangle;
- measure each point's distance to the **exact** generated surface by two routes: the analytic
  envelope evaluators `verification.sv` declares (`GeneratedEnvelope`, seams, patches), and the
  material field's exact reading (`MaterialField::reading`, exact below its cap), which should
  agree;
- report per face: max, 99th percentile and mean deviation, and where the worst lies.

**Exit:** the current pinion's deviation distribution, recorded in this document; a test that the
meter reads a known perturbation (a face offset by 20 µm) as 20 µm.

#### Phase 1 — findings (2026-09-28)

**The meter.** `gcs_core::solid::accuracy::Meter` reads a body's exact faces from the solved
sketch and measures points against them by two routes that share nothing but the snapshot:

- *analytic*: a blank face (cone, sphere: a turned line or arc) by its exact meridian projection;
  a generated face as the envelope of one cutter face at one tooth index — the roll `t` where the
  normal velocity at the point's foot on the rolled cutter vanishes (`n · v(f) = 0`), so the foot
  is a critical point of the distance to the envelope. The roll is scanned in 64 steps and each
  sign change refined by false position. A foot counts only on its finite cutter face, on the
  cutter's own Boolean boundary (the gear's bounded crown) and within the blank; a blank face's
  foot only on the blank's boundary and outside every cut. Off a convex edge, where each face's
  foot lies beyond the other, the edge point is found by alternating projections. Tooth indices
  are pruned by a table of cells the unindexed envelope passes through.
- *field*: `MaterialField::reading`, exact below its cap. Near a smooth face it is the signed
  distance; off a convex edge and deep inside a cut it is a lower bound.

`solventc DOC --solid NAME --measure FILE.step|FILE.stl [--measure-samples N]` samples a STEP
file's faces on a grid over each face's trim box (OCCT: `solvent_cad_read_step`,
`solvent_cad_face_kind`, the existing `face_point`) or an STL's triangles (chosen by area; each
gives its centroid, edge midpoints and corners), measures them on every core and prints the
core's report: overall, generated against blank, by sample kind, by the file's face class, by
exact face, the worst exported faces and locations, and where the routes differ most. Signed
distances are positive outside the material.

**Validation.** `tests/accuracy.rs` (core): exact contacts on the configured pinion's flanks (from
the cutter's own contact chart, which the meter does not read) and exact blank points, offset
0 and ±20 µm along the normal, read as the offset within 3e-14 mm (analytic) and 8e-10 mm
(field). `gcs-cli/tests/native_measure.rs`: a pulley (planes, cylinders, cones, a groove cut), a
sphere and a torus exported natively. Their STEP faces read as 0 on both routes (≤ 6e-13 mm); the
STL's corners read their float32 rounding (≤ 1.0e-6 mm), its centroids the mesher's chordal sag
(≤ 8.6 µm against the 10 µm deflection); every STL corner moved 20 µm along its normal reads
20 µm within 1.3e-5 mm on both routes.

**The pinion as exported today** (`--measure-samples 100000`; µm, max / p99 / mean of |d|):

| exact face | native STEP (37,795 face points) | native STL (74,536) | field-meshed STL (46,060) |
|---|---|---|---|
| heel sphere | 0.000 / 0.000 / 0.000 | 8.40 / 7.36 / 2.46 | 579.3 / 334.3 / 18.8 |
| tip cone | 0.000 / 0.000 / 0.000 | 9.75 / 9.34 / 1.53 | 12.15 / 11.83 / 3.70 |
| toe sphere | 0.000 / 0.000 / 0.000 | 8.51 / 7.53 / 2.55 | 173.9 / 171.0 / 8.78 |
| back cone | 0.000 / 0.000 / 0.000 | 8.69 / 8.69 / 2.70 | 83.8 / 70.4 / 14.5 |
| outer flank | 0.001 / 0.001 / 0.000 | 8.86 / 7.14 / 1.29 | 642.4 / 163.9 / 9.41 |
| outer fillet | 11.64 / 11.64 / 2.39 | 11.67 / 9.87 / 2.30 | 408.6 / 315.1 / 68.6 |
| root (crown tip) | 0.218 / 0.218 / 0.200 | 1.90 / 1.74 / 0.50 | 618.4 / 431.3 / 86.5 |
| inner fillet | 16.94 / 16.75 / 6.43 | 16.73 / 15.54 / 4.31 | 498.6 / 498.6 / 64.2 |
| inner flank | 0.191 / 0.150 / 0.003 | 9.54 / 6.65 / 1.23 | 621.0 / 169.6 / 25.6 |
| **all** | **16.94 / 14.20 / 0.76** | **16.73 / 10.20 / 1.97** | **642.4 / 285.7 / 23.7** |
| routes differ, max / p99 | 0.000 / 0.000 | 0.74 / 0.002 | 240.6 / 4.3 |

- The native STEP's cones and spheres are exact, and so are its generated flanks and root to
  0.2 µm. The whole error is in the **fillets**: 11.6 µm (outer) and 16.9 µm (inner), the sheet
  lying inside the exact material (signed mean −2.3 and −5.3 µm: over-cut), normals off by up to
  9.1° and 15.3° (flanks 0.23°). One 73-row chart across the whole profile under-samples the
  small fillet arcs; phase 3's refinement belongs in the section direction there. This is the
  withheld-contact figure (16.5 µm) measured independently on the written file.
- The native STL adds OCCT's chordal sag to the sheet's error: ≤ 9.7 µm on every face at the
  0.01 mm deflection, the inner fillet's 16.7 µm again the worst (its corners alone read 16.6 µm,
  the sheet's own error). Its centroids' normals are within 5.5° (p99), 24° at worst.
- The field-meshed STL is about forty times coarser: 642 µm max, 286 µm p99, 24 µm mean.
- The routes agree on every STEP sample and to 2 nm (p99) on the native STL; the 0.7 µm most
  lies beside an edge. On the field mesh they part only where a sample is deep inside a cut
  (0.2–0.6 mm), where the field reads the deepest single cutter pose, a lower bound.
- Cost: 68 s for the STEP (28 s of OCCT face sampling), 85 s for the native STL, 55 s for the
  field mesh, on 12 cores.

**The gear** is refused natively; its field-meshed STL (72,436 samples) reads 360.7 / 111.4 / 10.9
µm overall (generated 360.7 / 129.9 / 15.9, blank 323.8 / 27.0 / 5.3), the routes differing by
9.6 µm at most (p99 1.7). By exact face: heel 323.8 / 28.3 / 7.45, tip 3.29 / 3.23 / 1.27, toe
43.1 / 28.6 / 6.96, back 33.6 / 13.1 / 2.89; the outer crown's flank 94.6 / 94.6 / 16.9, fillet
274.0 / 202.7 / 32.1 and root line (`close1`) 290.0 / 290.0 / 53.7; the bounding neighbour's
flank 129.9 / 122.1 / 5.82, fillet 145.4 / 131.5 / 33.3 and root line 360.7 / 290.0 / 37.1.

**Not claimed.** A generated face is not trimmed by the other cuts (another index or stretch of
the roll), so a sample where two cuts meet may read short; the roll scan and the index cells are
sampled; the field route is a lower bound off convex edges and deep in a cut.

### 2. Why the gear is refused

Locate the 83° miss with the existing `where_a_sheet_misses_its_contacts` tool. Hypothesis: one
fitted sheet spans the crease where the neighbour bounds the outer crown, so two flank families
meet at a corner no single smooth chart can follow — a near-perpendicular normal error at a good
distance. If confirmed, the construction makes one sheet per cutter face, each trimmed by the
Boolean, instead of one per cutter. Confirm the cause before choosing the fix.

**Exit:** the gear exports natively at today's bar, and the meter reads it.

#### Phase 2 — findings (2026-09-28)

**Not the crease.** The gear's cutter has six faces: two cones (the outer crown's inner flank
and the neighbour's outer flank), two tori (their rounds), and two planes. Both crowns' tips lie
in the one plane square to the crown axis, and the indexing turns that plane into itself, so the
two tips are one face and the Boolean leaves no crease between them. Every station's walk
through the blank is flank, round, tip, round, flank, with no corner fan. The Boolean's creases
are all far from the blank. The instrument is `where_the_configured_gear_sheet_turns`
(`native_surfaces/gear_cells.rs`, ignored). It maps each node and withheld contact back to its
cutter face through the motion's inverse at its contact time. It also compares each column's
chord-length row parameters with the average the interpolation uses, fits the grid under each of
OCCT's three parametrizations, and scans the fitted face for folds a quarter of a cell apart.

**Cause 1: one margin column leapt, and moved every row's parameter.** The 83° withheld
misses were in the blank (0.9–4 mm deep), at rows 29–33, on the outer crown's round, which
generates the gear's root fillet. Their gaps were small (0.004–0.06 mm). The cause lay outside
the blank:
- The band's last station (column 31, wholly outside the blank) has a last row (76) whose
  contact time jumps from 0.279 rad to 3.235 rad. That contact sits on the corner where the
  neighbour's flank meets a far plane, 46.4 mm from row 75.
- The chart contract rightly ignores steps outside the blank. But OCCT's chord-length
  interpolation averages each row's parameter over all the columns, so this one column moved
  every row. A column's own parameters sat up to 61 rows from the average.
- The same grid fitted with even parameters misses by 22 µm and 6.1°. With that one row
  dropped, the chord-length fit misses by 17.7 µm and 6.6°.

**Cause 2: a pleat at the root fillet, between the withheld contacts.** With cause 1 removed, the
fit passes today's bar. The export then failed at the STL shell check:
- OCCT's mesh had 2,802,382 triangles. Of these, 2,202,118 were under 1 µm², 0.31 mm² in all,
  and 18 were degenerate after float32 rounding.
- The meter read one cluster of 18,378 of them (near (2, 46, −27) mm). It lies within 4.9 µm of
  the outer crown's generated flank, with normals up to 175° off: a Z-fold.
- The fold scan puts it at rows 25–27, where the outer crown's flank meets its round. In
  walk-length rows, the flank's contacts crowd at 0.07 mm a row and the round's stretch to
  0.55 mm, 7.5× between adjacent rows. That onset falls on row 28 in column 8, row 27 in column 16
  and row 25 in column 24.
- The averaged parameters cannot follow every column, and the fitted face turns 179.8° between
  quarter-cell neighbours in the blank (144 such pairs in or near it). The withheld contacts, one
  a cell, straddle it.
- The pinion's walk-length sheet turns at most 30.1° this way.

**The fix** (`solid::contact_trace`, `cad/native/sweep_boundary.rs`):
- `contact_trace::charted` keeps a sheet's rows outward from those holding a contact in the
  blank only until a column's time leaps a radian or more between rows. A row with a contact in
  the blank is never trimmed. For the gear this drops row 76 (77×32 becomes 76×32). The pinion's
  rows are untouched.
- `contact_trace::Rows` says where a sheet's rows fall: by unfolded walk length (`Walk`, as
  before) or at even lengths in space down each column's own curve (`Length`).
- The fit contract gains a fold check. At points a quarter of a cell apart
  (`solvent_cad_surface_grid`, no trim test), the face's normal may not turn more than 90° in the
  blank. The withheld refusal now also says where its worst normal is.
- The construction tries walk-length rows first, then length rows if that fit misses or folds:
  - The gear's walk-length sheet passes the withheld bar (17.7 µm, 6.6°) but folds (179.8°).
    Its length sheet, 86×32, fits within 1.9 µm and 1.17°, and turns at most 4.5° in the blank.
  - Length rows are not general. On the pinion they fail: the pinion's margin columns run far
    in space, so its 225 length rows fall sparsely in the blank against their neighbours
    (29.6 µm, 90°, folds of 180°).
  - The pinion therefore keeps walk-length rows, and its native STL is byte-identical.

**The gear exported.** `solventc gears.sv --solid pair.gear.body --step … --stl …` takes 215 s
on 12 cores (1.90 × 10¹² instructions). The walk-length sheet is refused for its fold at 8.6 s.
The length sheet is split 48 ways into 49 cells in 96 s. Classification keeps 1 cell and removes
48. The fused solid is 22,343.554 mm³ with 99 faces. The STL has 190,192 triangles, none under
1 µm². Field agreement probes 1000 triangles 0.1 mm off each side: none unresolved, none
disagreeing. STL alone takes 175 s.

The meter (`--measure-samples 100000`; µm, max / p99 / mean of |d|):

| exact face | native STEP (47,718 face points) | native STL (87,598) |
|---|---|---|
| heel sphere | 0.000 / 0.000 / 0.000 | 10.48 / 7.59 / 2.77 |
| tip cone | 0.000 / 0.000 / 0.000 | 3.02 / 2.80 / 0.61 |
| toe sphere | 0.000 / 0.000 / 0.000 | 9.05 / 7.40 / 2.68 |
| back cone | 0.000 / 0.000 / 0.000 | 4.24 / 4.24 / 1.14 |
| outer crown's flank | 0.482 / 0.219 / 0.011 | 9.48 / 8.32 / 1.62 |
| outer crown's round (fillet) | 1.181 / 0.922 / 0.177 | 10.05 / 8.89 / 1.57 |
| outer crown's tip (root, `close1`) | 1.106 / 1.106 / 0.352 | 8.05 / 5.48 / 0.63 |
| neighbour's flank | 0.661 / 0.228 / 0.011 | 9.23 / 5.51 / 1.04 |
| neighbour's round (fillet) | 1.061 / 1.061 / 0.228 | 10.26 / 9.09 / 1.73 |
| neighbour's tip (root, `close1`) | 1.106 / 1.106 / 0.199 | 8.12 / 8.06 / 1.77 |
| **all** | **1.181 / 0.800 / 0.043** | **10.48 / 7.56 / 1.75** |
| routes differ, max / p99 | 0.000 / 0.000 | 0.915 / 0.007 |

- The STEP's generated faces are within 1.2 µm of the exact envelope, normals within 2.3°.
  The worst is on the outer crown's round, as the pinion's is on its fillets. Length rows sample
  the fillets as densely as the flanks. The pinion's walk-length rows give its fillets 16.9 µm.
- The STL adds OCCT's chordal sag, at most 10.5 µm, on the heel sphere. Its centroids' normals
  are within 8.2° (3.4° p99). The large normal figures are at vertices and edge midpoints on a
  face's edge, where the nearest exact face is the other one.
- The field-meshed gear STL of phase 1 read 360.7 / 111.4 / 10.9 µm.
- Measuring took 105 s for the STEP and 100 s for the STL.

**Tests.**
- `contact_trace::a_sheet_keeps_its_rows_only_while_the_margin_is_one_chart` (core, instant)
  checks the trimming rule on grids of times.
- `a_sheet_of_contact_curves_is_one_chart` now builds the torus fixture's sheet both ways. It
  checks that neither leaps in its margin and that length rows are even in space.
- The slow tier's `the_configured_gear_exports_natively` (`gcs-cli/tests/cli.rs`) exports the gear
  to STEP and STL through `solventc` and checks the shell and the field gate.
- `generic_sheet_reproduces_the_gear_tooth_space` joins the slow tier; it was ignored before.
  - The bevel gear's single space had the same cause 1. At the phase-1 commit its 73×39 sheet
    missed by 73 µm and 61.6°.
  - Trimmed to 71×39, the walk-length sheet fits within 3.5 µm and 1.24° and needs no length
    rows.
  - Its space is 117.135260 mm³ against the recorded 117.137321 (1.8 × 10⁻⁵), so the recorded
    volume stands. 264 side checks agree with the field.

### 3. Precision to a stated tolerance

- An export tolerance (`solventc --tolerance 0.01mm`, default 10 µm for this pair) replaces the
  fixed 0.25 mm and 20° bars; the normal bar follows from it and the sampling.
- The sheet's sampling and fit refine where withheld contacts exceed the tolerance (adaptive in
  both the section and the station directions), rather than a fixed grid; higher-degree fits if
  refinement alone does not reach it.
- The field-agreement probe's offset tightens with the tolerance.
- Cones, spheres and the blank's faces are exact already; only generated flanks and fillets are
  fitted.

**Exit:** both members' STEP within 10 µm by both meters, with the time it costs.

### 4. Fabrication features in the model

- **Backlash:** a design parameter (normal backlash, e.g. 0.05 mm for this module) that thins
  each member's teeth by half of it. It belongs in the crown: the tooth and its mate stay built on
  shared flank lines, offset by the backlash along the normal, so complementarity by construction
  becomes complementarity-with-a-stated-gap.
- **Edge relief:** a chamfer or radius on the tooth tips' edges (tip cone against the flanks, and
  the toe and heel ends), stated in the design; exact analytic faces where they can be (a
  chamfer cone, a torus), fitted otherwise.
- Admission (generating class) re-checked for both members with the features in.

**Exit:** the configured pair with backlash and relief, both members admitted and exported at
10 µm.

### 5. Files

- **STEP** is the master: `build/exports/hypoid-pinion.step`, `hypoid-gear.step`.
- **STL** from OCCT's mesher at a chord deviation below the tolerance, validated by the existing
  shell checks and the meter.
- Nothing is written unless both pass (the existing staged output).

### 6. The pair, checked from the files

- Shaft angle and offset measured from the exported solids (90°, 25 mm).
- An interference sweep through one pitch with both native meshes: no overlap, with the
  clearance between flanks equal to the stated backlash within the tolerance.
- Optionally, the contact pattern at the mean point.

## Risks

- **Cost:** a 10 µm fit may need several times today's sampling; the pinion's 98 s could grow
  several-fold. Progress output throughout; the slow tier holds the whole-member exports.
- **The gear's cause may not be the crease.** Phase 2 confirms before building on it.
- **Relief faces** meet generated flanks along curves that are themselves generated; where an
  exact face is impractical they are fitted, and the meter holds them to the tolerance.
- **Admission with backlash:** thinner teeth move the contact limits; the grid in
  `docs/spiral-bevel-layout-plan.md` is re-run.

## Files

- `rust/gcs-cli/src/cad/native/sweep_boundary.rs`, `backend/*.cpp`: per-face sheets, adaptive fit,
  tolerance.
- `rust/gcs-cli/src/main.rs`: `--tolerance`.
- A meter: core (envelope and field distances) plus a CLI or test harness reading STEP/STL.
- `rust/examples/spiral_bevel/`: `design.sv`/`configuration.sv` (backlash, relief), `crown/`
  (the offset mate), `blank/` (relief faces).
- Tests: the meter, the gear export, the pair checks (slow tier for whole-member exports).
