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
