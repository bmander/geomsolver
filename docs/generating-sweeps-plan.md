# Generating sweeps: execution plan

This plan delivers the [supported class](generating-sweeps.md) through to a hypoid pair
export. Each phase ends with an executable result and an exit gate. Unit tests use small
closed-form fixtures that run in seconds. The gear cases are acceptance runs, not the lens
the components are designed through. Measure at the point a decision is made before
changing the construction.

## Phase 1 — Measure where the hypoid stands

Before any new construction, find out which offsets are in the class.

- For `offset_angle` at 0°, 6°, 15°, 30° and 45°, and for each member, evaluate E1–E4 by
  sampling on the actual `gears.sv` source:
  - the clearance at both roll limits;
  - the number of contact times per source point that reaches the blank;
  - the minimum generated area factor;
  - self-crossings of one placement's sheet inside the blank.
- Run the existing native OCCT path (`generic_sheet_reproduces_the_pinion_tooth_space`, with
  the offset rewritten through `read_gears_with`) at the same offsets. Apply the field probe
  to its faces as well as to the mesh path's triangles. The recorded failure is the mesh
  path's. Whether the native path fails the same way has not been measured.
- Record a table of offset, member, each predicate's worst value and witness, native
  field-probe disagreement, and time.

**Exit:** the table is in this document, with a statement of which offsets are inside the
class. **Decision gate with the user:** if the offsets the design needs fail E2–E4, choose
between restricting supported offsets and widening the class (a Phase 5b below). Do not
start construction changes before this gate.

### Phase 1 results (2026-09-22)

Measured by `where_the_pair_stands_against_the_generating_class` in
`rust/gcs-cli/tests/generating_class.rs` (ignored; about 30 s per offset). It samples each
revolved tool face over 300 × 1200 source points across the band of revolution that reaches the
blank, and finds every contact time from the analytic sinusoid (`SweepContacts::at_source_over`).
It counts contacts strictly inside the blank's field.

- **E3** is the generated surface's oriented area factor. It is computed by central differences
  on the same contact branch, relative to the source surface's own.
- **E4** counts pairs of in-blank sheet points within half the sample spacing whose source
  points are more than five spacings apart. "Crossing" means their normals differ; "parallel"
  means they nearly coincide.
- Failing points (J < 0 and E4 pairs) are judged against the removal's own material field:
  "buried" means strictly inside the swept material.

The native column is `the_native_space_at_an_offset_against_its_field` in
`native_surfaces/gear_cells.rs`. It checks the field on both sides of every eleventh interior
sheet node inside the blank.

Axis offset ≈ 53.67 mm × sin(offset). The pinion has 24 teeth, module 2 mm, 35° spiral and
20° pressure. Tool patches: 1 outer flank, 2 outer round, 3 tip, 4 inner round, 5 inner flank.

| Offset | Axis offset | E1 | E2 | E3: least J, folds | E4 far-source pairs | Failing points buried | Native path |
| ---: | ---: | --- | ---: | --- | --- | --- | --- |
| 0° | 0 mm | clear | 0 | 0.47, none | 0 | — | 120.705 mm³, 260 agree / 0 |
| 6° | 5.6 mm | clear | 0 | 0.65, none | 0 | — | 130.427 mm³, 258 / 0; withheld error 0.024 mm |
| 15° | 13.9 mm | clear | 0 | 0.33, none | 0 | — | 153.552 mm³, 258 / 0 |
| 16° | 14.8 mm | clear | 0 | flank 0.17, none | 0 crossing, 70 parallel | — | not run |
| 17.5° | 16.1 mm | clear | 0 | inner flank folds (30k samples J < 0) | 295k crossing, inner round × inner flank | 191 / 201 | not run |
| 20° | 18.4 mm | clear | 0 | inner flank folds (56k) | 32k crossing | 198 / 201 | not run |
| 25° | 22.7 mm | clear | 273 | inner round and inner flank fold | 31k crossing | 180 / 201 | not run |
| 30° | 26.8 mm | clear | 868 | inner round folds | 51k crossing | 189 / 201 | refused: "a cutter point has no contact time" |
| 45° | 38.0 mm | clear | 1695 | inner round folds | 23k crossing | 170 / 201 | not run |
| Gear, 0° | — | clear | 0 | 0.86, none | 0 | — | recorded 117.137 mm³ (existing test) |

**Reading.** Up to 15° the pinion is inside the class: one contact per source point, no fold, no
self-crossing. The native OCCT construction agrees with the field at 0°, 6° and 15°. At 15° the
traced-sheet mesh path had 19% of its triangles wrong, so that failure was the construction,
not the geometry.

Between 16° and 17.5° the side the crown's inner flank generates leaves the class. First,
the inner flank's envelope folds, and the crown tooth's inner tip round's envelope crosses it. From 25° the round's own
envelope also folds, and some source points touch twice. Nearly every failing point is buried
inside the removed material. The few on the boundary lie near the crossing curves.

This is **undercut** on that side of the pinion. The true surface is the flank and fillet
envelopes, trimmed where they cross. It is the case the scope names under "Undercut is
excluded deliberately", plus a self-fold that 5a alone would not cover. The native
construction does not see it and refuses with an unhelpful reason.

These are sampled observations of the reference design, not bounds. A design with other
pressure angles, tip radius or tooth counts moves the threshold. That is part of why
admission belongs in the core rather than in a supported-offset list.

**Decision needed:** support offsets up to about 15° with this design and refuse the rest
through admission (Phases 2–4 unchanged), or take on 5a and 5b now.

### Designing undercut out (2026-09-22)

The user chose to change the reference design, not the class. Elsewhere a hypoid is designed
free of singularities on its working surfaces: Litvin's synthesis checks for them, and a
hypoid's two sides get unequal pressure angles. The configuration now carries two knobs
besides `offset_angle`:

- `pressure_shift` is added to the crown tooth's inner flank pressure angle and taken from
  its outer flank's. `RoundedRackSection` reads `outer_pressure` and `inner_pressure`.
- `spiral_angle` is the crown's spiral angle; the pinion's is this plus the offset.

Both default to the bevel pair's values (0° and 35°). At those defaults the bevel pinion space
is 120.704660 mm³, as before, and the core envelope and example tests pass unchanged.

Measured with `SOLVENT_CLASS_SHIFTS`, `SOLVENT_CLASS_SPIRAL` and `SOLVENT_CLASS_TIP`
(`generating_class.rs`). Mean pressure is 20°; the spiral column is crown / pinion.

| Offset | Spiral | Shift in the class (least J) | Shift outside it |
| ---: | --- | --- | --- |
| 20° | 35° / 55° | 5° (0.44), 7.5° (0.76), 10° (0.92) | −5°, 0°, 2.5° |
| 22.5° | 35° / 57.5° | 7.5° (0.38), 10° (0.80) | 5° |
| 25° | 35° / 60° | none | 5°, 7.5°, 10° (at 10°: 326 fold samples, no crossings) |
| 25° | 25° / 50° | 10° (0.59) | 0°, 5° |
| 30° | 35° / 65° | none | −5° to 10° |
| 30° | 20° / 50° | 12.5° (0.23, with 1191 near-tangent pairs), 15° (0.75) | 10° |
| 30° | 15° / 45° | 12.5° (0.30) | 10° |

- The gear stays in the class at shifts of 5°, 7.5° and 10° (least J 0.76, 0.66, 0.45), and at
  spiral 25° with shift 10° (0.35).
- Halving the tip round (0.15 normal modules) at 25° and 30° with spiral 35° changes nothing.
- The native construction at 25° / spiral 25° / shift 10° builds one space of 188.265 mm³, with
  272 side checks agreeing and none disagreeing. That confirms the prediction from the class.
  Its withheld contact error, 0.070 mm, is ten times the bevel pair's. That is sheet-fitting
  accuracy, left for the accuracy budget.

**Reading.** Unequal pressure angles move the in-class limit from about 16° to 22.5°. Also
holding the pinion's spiral near 50° by lowering the crown's reaches 25° with a 30°/10° split,
and 30° with a 32.5°/7.5° split. Splits beyond about 10° are lopsided for a real tooth, which
would be weak on its low-pressure side. The approximate generation (one crown slid by the
offset, not per-member machine settings from local synthesis) is the likely limit past 25°.
Replacing it is the path to larger offsets, and the motion class already admits it.

These are sampled observations and not bounds. Tooth strength and contact are not checked.

## Phase 2 — Admission predicates in the core

Add one core entry point that takes a solved sweep graph and returns either `Admitted`,
carrying its evidence, or `Refused { condition, witness }`. The name is decided at
implementation time; the entry point lives beside `SweepContacts` in `solid/`.

- T1, T2 and M1 are structural reads of the solid and motion DAGs. M2 and E1 reuse the
  existing refusals. E2–E4 reuse `SweepContacts::at_source_over` and the ring roots.
- Every check is sampled and reports its sample spacing. The result type distinguishes
  "sampled, none found" from "certified", so a later interval check can strengthen the
  answer without changing its callers.
- Fixtures, each with a closed form:
  - sphere and cylinder under a rotation about a parallel axis (M2 refused);
  - cone under a rotation about an intersecting axis (admitted, one branch);
  - torus under a relative rotation about skew axes (admitted, known contact);
  - a profile with a concave corner (T2 refused);
  - a union tool (T1 refused);
  - a roll that leaves the tool in the blank (E1 refused);
  - a small-radius round driven into a fold (E3 refused).
- The general-sweep fixtures (tumbling cylinder, thin plate, slid and tilted cylinders,
  dumbbell) each produce the expected refusal.

**Exit:** every fixture's test passes in under a few seconds. `solventc` reports the
condition row and witness for a refused document.

## Phase 3 — Admission gates export

- `solventc --step/--stl` runs admission before any construction. A refused sweep writes
  no output and preserves old files, as today.
- The native path is the construction for admitted sweeps. Remove from it the per-case
  checks that admission now owns, so there is one statement of each rule.
- Add the field probe to the native export's acceptance, over a bounded sample of faces
  with its coverage reported. Export fails on any disagreement.

**Exit:** the common-apex pair still exports and reproduces the recorded tooth-space
volumes (120.7088 / 117.1373 mm³ within the recorded tolerance) with zero probe
disagreement. The mesh path's hypoid tests remain diagnostics.

## Phase 4 — The hypoid inside the class

Using Phase 1's supported offsets, export both hypoid members through the ordinary path.

- One tooth space per offset first (a `repeat 1` rewrite), then the indexed members.
- Gate on field agreement, encoded shell checks and a STEP round trip. Compare volumes
  against the Ju et al. reference run on the same tool and motion as an independent check.
- Offsets outside the class must refuse with E2–E4 witnesses. That refusal is a passing
  test.

**Exit:** both hypoid members export at every supported offset with zero field
disagreement. Every unsupported offset refuses with its reason.

## Phase 5 — Widening only where the design needs it

Each item needs a user decision and its own gate. None is started by default.

- **5a Undercut:** a declared intersection between the tip-round envelope and the flank
  envelope of one sweep, built on `BoundarySeam`/`EnvelopeSeam`. It admits E4 crossings
  only between those declared pairs.
- **5b Crossing branches at larger offsets:** if Phase 1 shows the needed offsets cross, the
  same mechanism generalizes to declared branch pairs.
- **5c Certified admission:** interval enclosures of E2–E4 over whole regions, extending
  the circular-crown verifier.

## Phase 6 — Pair acceptance and the WASM path

- The pair acceptance work in the [roadmap](spiral-bevel-roadmap.md) resumes: engagement
  across a tooth period, the 0.0254 mm end-to-end budget, and multiple configurations.
- The kernel-free path for the one-core WASM target is decided here. It reuses the core's
  admission and contact machinery, and it must pass the same field gate the native path
  does. The Manifold mesh arrangement is one candidate if it can pass that gate.

## What does not change

Solvent syntax is unchanged. `solid removal(tool, under:, from:, to:)` is still the
declaration, and the class is a property checked of it. Material evaluation
(`MaterialField`, `SweptField`) still answers membership for every sweep, admitted or not.
Only boundary construction and export are restricted.
