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
- **Gear:** refused. Its sheet (77×32) fits the contacts within 60 µm, but its normals miss by 83°
  against the 20° bar.
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
