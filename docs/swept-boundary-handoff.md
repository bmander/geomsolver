# Handoff: the certified swept boundary, milestone 5 (2026-09-12)

**Latest experiment:** [Phase 2](swept-boundary-phase-two.md) adds shared parametric domains,
common edge samples and an explicit cylinder source-chart adapter. The cylinder and three
perturbations pass full interval acceptance at tolerance 0.04 with calibrated value/probe
settings. The ordinary sweep path is unchanged; Phase 3 must restore its tracer-chart connection.

**Latest baseline:** [Phase 1a](swept-boundary-phase-one-a.md) repairs source ownership and
records the corrected candidate and replay inputs for Phase 2. Earlier counts below are historical.

**Historical handoff.** The [Phase 1 record](swept-boundary-phase-one.md) now provides the
independent cylinder oracle, frozen alias/zip counterexamples and source-provenance barrier.
Read it and the revised architecture plan before choosing a repair region. The [Phase 0a record](swept-boundary-phase-zero-a.md) covers validator
calibration and inspectable partial audits. The [Phase 0 record](swept-boundary-phase-zero.md) contains the
current evidence contract and status matrix. The seven-loop numbers below describe the
pre-Phase-0 construction, and its certificate was not a sound acceptance gate.

Read this before touching `solid/swept_boundary`. `docs/swept-boundary.md` is the full running
record; this is the short version plus the traps.

## Where it stands

Every case in the table closes and certifies except one. **The tumbling cylinder refuses with
`UnpairedRim`, 7 open loops.**

| | |
|---|---|
| triangles | 6713 |
| volume (divergence theorem, open mesh) | 9.3057 |
| independent reference (field-contoured) | 9.4786 |
| that reference's measured bias | +0.6% on the turned box control |
| open loops | 7 (sizes 4, 7, 5, 7, 4, 3, 3) |

So the construction is roughly 1–2% short of the material, not the ~30% it was before the stitch
fixes. Do not read the loop count as a progress bar — but do note the volume gap is now small.

Suite green at the last full run (1209 passed, 0 failed, 75 ignored). Three `#[ignore]`d instruments
have been added since and were each run individually. Nothing committed since `5e95019`.

## The diagnosis, established 2026-09-12

**The seven loops are real holes in a curved surface, and the only fill the code has is a flat fan
from the loop's centroid, which chords down through the solid.** The rims are in the right place;
the patch that would cover them is the wrong shape.

Measured two ways, with both failure modes as controls. The probe ladder is a two-sided strict-sign
bracket at `0.04, 0.02, 0.01, 0.005, 0.0025, 0.00125, 0.000625, 0.0003125, 0.00015625` — four steps
below `certify`'s own floor of `least_probe` = 0.01. Real surface, however thin, brackets
`(Material, Exterior)` once the probe is shorter than the material is thick; surface buried in the
solid never brackets at any distance.

**Controls** (`creases::whether_the_open_loops_are_thin_surface_or_buried_surface`), loop 0's own fan
moved bodily to two places whose answer is known:

```
CONTROL buried at (3,0,0)        0 of 4 bracket; never: Material/Material x4
CONTROL in open air at (10,0,0)  0 of 4 bracket; never: Exterior/Exterior x4
```

Neither ever brackets, and each reads the sign its position demands. The ladder discriminates.

**Fan centres** — mixed, and starkly so. What brackets, brackets at the *full* 0.04; what does not
reads `Material/Material` all the way down to 1.6e-4, matching the buried control exactly:

```
loop 0  2 of 4      loop 3  2 of 7      loop 6  0 of 3
loop 1  4 of 7      loop 4  3 of 4
loop 2  3 of 5      loop 5  3 of 3
```

**Rim midpoints, apex-free** (`creases::whether_the_open_loops_own_rims_lie_on_the_boundary`) — this
is the decisive one, because every fan triangle has the apex as a corner, so a buried apex would bury
pieces wherever the rim lay. Asked at each loop edge midpoint, which lies exactly on the rim, along
its owner triangle's outward normal:

```
loop 0  4 of 4      loop 3  6 of 7      loop 6  3 of 3
loop 1  5 of 7      loop 4  2 of 4
loop 2  5 of 5      loop 5  3 of 3      → 28 on the boundary, 5 not
```

Loop 6 is the cleanest demonstration: **every rim midpoint brackets at full probe, every fan centre
is buried.** The rim is on the surface; the flat patch is not.

## Three traps. Each has already cost real work

**1. The field is not a distance function.** It returns conservative brackets and may decline to
answer. Five separate times a reading was taken as a plain number — an enclosure midpoint as a
distance, a sign as a position, a raw value as a depth — and each produced a confident wrong
conclusion that had to be retracted, sometimes after work was built on it. `0.1340 = 1 − √3/2` and
`0.2929 = 1 − √2/2` turned up as "depths" and were exact field values. **Establish what a number
*is* before reasoning from it. Prefer two-sided sign brackets; they are all the one-Lipschitz
contract licenses.**

**2. `certify`'s `thin` verdict is not a licence to fill.** `Certificate::is_complete()` is
`failures.is_empty()`, and `thin` is not a failure — so a fill can pass the gate while being wrong.
The arithmetic:

```rust
let reversed = matches!(last,InsideNotMaterial(Sign::Exterior))
    || (matches!(last,OutsideNotExterior(Material)) && inside == Exterior);
if reversed { failures.push(Reversed) } else { thin.push(last) }
```

A triangle buried deep in the solid probes `(Material, Material)` → `OutsideNotExterior(Material)`;
the reversed test then needs the inside to read `Exterior`, which it does not. **So `thin` cannot
distinguish thin real surface from a triangle laid inside the material** — exactly the case
`loop_span`'s `Inner` exists to refuse. This was nearly acted on this session.

**3. Measure at the decision point, not on the finished mesh.** `loop_span` runs on the
*intermediate* loops of each `rim_zip` round; almost every instrument hooks `Stage::Zipped`, which is
the mesh *after* all rounds. The last `loop_span` change was predicted from finished-mesh
diagnostics: predicted 13 → 10 loops, delivered 13 → 13, shed nine triangles, moved the export, and
was reverted. **The diagnosis above carries this caveat too** — see "Next step".

## Do not retry. All refuted by measurement, with numbers in `docs/swept-boundary.md`

| approach | what happened |
|---|---|
| bowtie generator | refuted by *where* the gap is |
| trim defect at welded/zipped | no boundary deeper than its own chord, all 8 stages |
| generation gap wanting a funnel | refuted by *how much area* is present |
| component swallowing in caps | refuted |
| divider cut | no-op; every branch already cut |
| cluster weld | 0 of 13 clusters collapse; 2-vertex clusters fail too, so not an arity problem |
| weld before zip | 17 → 14 loops but the dumbbell reached `certify` non-manifold with 20 failures |
| `wind` by the live `walked` set | changed nothing: identical triangles, volume, loops, reversals |
| dropping the inward bands | 7 → 119 loops |
| sewing wide seams side to side | invented surface: volume 6.6509 → 6.6865 |
| fanning a quad across its other diagonal | closed the 30° case outright and was *still* wrong — two laid facets `Reversed` |
| judging the fan by its own normal | predicted 13 → 10, delivered 13 → 13, reverted |
| projected-apex fan | 2 of 7 loops have no boundary to project their centroid onto at all |
| field contouring inside a loop's box | extractor refuses: `AmbiguousPoint`, enclosures straddling zero |

Two findings worth keeping, independent of the loops:

- **Winding consistency is not outwardness.** 10053 shared edges walked oppositely, 0 folds, yet 169
  triangles genuinely face inward. Consistency is topological; a patch folded back into the material
  stays consistently wound while facing inward.
- **`lay` never asks the field.** `takes` checks manifoldness, area and duplicate corners only, which
  is how the zip lays 142 bands into the solid (a band is `Reversed` 12.3% of the time against 2.5%
  mesh-wide). This is upstream of the loops and would fail the certificate even with every loop
  closed. Dropping those bands is *not* the fix — it takes the case 7 → 119 loops.

## Uncommitted in the tree

| file | what |
|---|---|
| `src/.../trim.rs` | **behavioural.** A ± retry in `loop_span` when `project` says `Inner`, and skipping zero-area fan pieces. Took the case 17 → 7 loops, volume 9.2929 → 9.3057. Other four cases byte-identical. |
| `src/.../stitch.rs` | comments only — records two refuted attempts so they are not retried |
| `tests/sweep_mesh/creases.rs` | ~2200 lines of `#[ignore]`d instruments |
| `tests/sweep_mesh/reference.rs` | the field-contour reference check |
| `docs/swept-boundary.md` | this session's write-up |

Any commit of the `trim.rs` work should name the moved seams: **tumbling cylinder, sliding dumbbell,
whole-turn box.**

## Next step

1. **Re-run the two ladder instruments inside `rim_zip`'s rounds** rather than on the finished mesh.
   Same code, different hook. This is trap 3 applied to the diagnosis above — cheap, and it either
   confirms the design target or changes it before anything is built.
2. **Then design the surface-following fill.** Note that the two obvious implementations are already
   dead (projected apex, field contouring), so this is design work, not applying a known fix.

Separately and upstream: **`lay` never asks the field.** That is a prerequisite for a certifiable
tumbling cylinder regardless of the loops.

## Working rules that bind here

- Never `cargo test --release`. The core suite is `cargo test -p gcs-core --test core`.
- Verify any library change by byte-identical STL export against a baseline
  (`export_milestone_4_cases`, `SOLVENT_EXPORT`).
- Keep a negative control — a fixture shown to fail *without* the fix. One unit test was written and
  removed for passing either way.
- A case that closes on a refused certificate is a failure, never a warning.
- A certified boundary may not quietly shed surface; an open loop is the honest refusal.
- Render before reasoning. `creases::draw_the_refused_loops` writes an SVG; the STL goes through
  three.js in headless Chrome. Nothing else on this machine renders an STL. The render is what
  revealed the missing equatorial band when statistics had not.
