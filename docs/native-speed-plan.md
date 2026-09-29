# The native export, a hundred times faster: plan

The native export of the configured hypoid pair (`solventc rust/examples/spiral_bevel/gears.sv
--solid pair.{pinion,gear}.body --tolerance --step … --stl …`) takes 613 s for the gear and about
330–480 s for the pinion. The field meshing went from 430 s to 4.5 s by exploiting the gear's
symmetry and by never doing the same work twice; this plan does the same for the native export.
**Goal: 100× on both members (the gear to about 6 s), with the files as good as they are now** —
every face within the tolerance by the meter, field agreement, the pair check — and every
export without `--tolerance` still correct (its bytes may change; its solid may not).

## Where the time goes (gear, 10 µm, 12 cores)

| stage | time | share |
|---|---|---|
| sheets: sections, contacts, fits (removal and relief; one refused fit redone) | ~16 s | 3% |
| split the blank by 96 sheets into 193 cells | 278 s | 45% |
| classify 193 cells (190 s of it interior sampling) | 199 s | 32% |
| unite the material | 17 s | 3% |
| write the STEP (48.7 MB) | 37 s | 6% |
| mesh, twice for sag | 42 s | 7% |
| field agreement | ~20 s | 3% |

## The idea: one sector, patterned

Every tooth space is the same cut turned by one pitch. Construct **one sector** — the blank
between two index half-planes through the middle of neighbouring teeth, cut by one tooth
space's sheets (the removal and the relief, and the neighbours' where they reach in) — and
pattern it N times about the member's axis. Splitting and classifying become a few cells
instead of 193; the STEP carries each sheet once, trimmed to its sector, and patterns it; the
STL meshes one sector and replicates its triangles. The indexing is exact rotation, so the copies
are exact.

The admission and the field already know this symmetry (`MaterialField::symmetries`, the
admission's "1 of 48 placements checked, the rest reading the blank alike"); the construction
has not used it.

## Phases

1. **Profile precisely.** Instructions retired and wall time per stage for both members at the
   default bars and at 10 µm (`/usr/bin/time -l`, the stage log), on a quiet machine. A tool that
   times each stage. **Exit:** the table above for both members, recorded here.
2. **The sector.** Choose sector boundaries where no cut reaches (mid-tooth half-planes; prove it
   from the admission's contact data or refuse the sector form and fall back), split and classify
   the sector, unite it, pattern it (a glue fuse of shared faces, or a compound and sewing) into one
   solid. **Gate:** the patterned solid's volume and face count equal today's within the
   tolerance; the meter passes; the field agreement passes; the result is refused (and the old
   path taken) whenever the sector's premise fails.
3. **Files from the sector.** STEP: each sheet trimmed to its sector before writing, the pattern
   written as copies of small faces (the 48.7 MB comes from every tooth's face carrying its whole
   sheet). STL: one sector meshed, sag-checked, and replicated; the sector's cut faces dropped.
   **Gate:** meter ≤ tolerance on both files; shell checks; the pair check.
4. **Everything else.** Sheets in parallel (the removal and the relief, and both row placements
   rather than one after the other); field agreement probing one sector; the classifier's sampling
   per cell kept cheap. Stop when both members are 100× faster or the remaining time is a named
   floor (OCCT's STEP writer, for instance) — then say so.

## Verification

Every phase: the full and slow suites, the web suite, the corpus and non-native goldens
byte-identical; the native exports compared by volume, faces, the meter, field agreement and the
pair check (bytes will change). Timings with instructions retired, on a quiet machine.
