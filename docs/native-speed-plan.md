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

## Phase 1 — findings (2026-09-29)

Each export run alone or two side by side on the 12-core machine (`uptime` noted in each log;
12–17 while the pairs ran), `make solventc OCCT=1`,
`SOLVENT_STAGE_TRACE=… /usr/bin/time -l build/solventc rust/examples/spiral_bevel/gears.sv --solid
pair.MEMBER.body [--tolerance] --step … --stl … --no-diagnose`. A stage's time is the difference of
the trace's marks; "before" is the elaboration, the solve and the admission (the wall clock less the
trace's last mark). Instructions retired are the steady figure: wall clock moves with the machine's
load, and the tolerance rows were run twice (the first pair beside a compile read 532 s and 828 s).

| seconds | pinion | pinion, 10 µm | gear | gear, 10 µm |
|---|---|---|---|---|
| before: elaborate, solve, admit | 14.1 | 18.3 | 14.0 | 18.2 |
| static blank | 1.2 | 1.4 | 1.6 | 1.9 |
| sheets: clearance, reach, trace, fit (both sweeps) | 15.4 | 10.2 | 27.0 | 20.3 |
| split the blank by every sheet | 184.4 (48 sheets, 97 cells) | 208.1 | 328.7 (96, 193) | 371.8 |
| classify the cells | 93.6 (92.1 interior samples) | 104.9 | 213.7 (203.2) | 267.8 (256.2) |
| unite, unify, validate, measure | 8.3 | 9.7 | 20.0 | 25.8 |
| write the STEP | 19.9 | 22.0 | 39.4 | 52.5 |
| mesh and write the STL | 7.7 | 111.6 (meshed twice: 22.2 + 86.9) | 7.8 | 25.0 |
| field agreement | 4.8 | 6.1 | 10.4 | 17.6 |
| **total** | **349.2** | **492.4** | **662.6** | **801.1** |
| instructions retired | 2.52 × 10¹² | 2.90 × 10¹² | 4.82 × 10¹² | 4.77 × 10¹² |

The split and the classification are 80% of every export; the split grows faster than the number
of sheets (the gear has twice the pinion's and takes 1.8 times as long), since neighbouring sheets'
margins, outside the blank, still cross in the kernel. The solids: pinion 14,642.074947 mm³ (10 µm:
14,643.427605), gear 22,261.890333 mm³ (22,261.892874), 147 and 291 faces.

## Phase 2 — findings (2026-09-29)

**The side.** A flat half-plane through the axis cannot bound the sector: a spiral tooth space
turns about the axis across its face by most of a pitch (the pinion's by 20°, its pitch 15°), and
a meridian plane read with the planes square to the axis leaves no gap (−2.7 mm). The side is a
surface of the axis's sectors instead: sliced by spheres about the blank's end-sphere centre (on
the axis; the planes square to it are the other candidate, and the wider least gap wins), in each
of 16 slices of the cuts' span the angle midway across the gap the contacts leave between one
placement and its neighbour, read over the slice and half of each neighbour with the side's own
turn taken out (read without, the spiral ate the gap), fitted by a cubic in the slice (an
interpolated wavering angle swung 2.7° between rows). The contacts are every traced and withheld
contact of each sweep's first placement in the blank (`gcs_core::solid::sector`): the pinion's
side is 0.449 mm clear of them at least, the gear's 0.530 mm, against a bar of 0.05 mm plus twice
the sheets' fit error. The side is then built (22×9 nodes) and read back — 1,363 and 1,459 of its
points in the blank, 0.380 and 0.504 mm clear — and the material field reads 151 and 145 of those
at least half the margin deep in the blank as material by a ball certificate of that radius. A cut's contacts all fall in
one sector, so the sector holds one placement of each sweep. The blank reads alike under a pitch's
turn at 256 points, and the sides split it into the sector and the rest, the sector exactly the
blank's share of volume (800.285375 and 595.382969 mm³). It is sampled, as the admission is, and
says so; the whole-body construction is taken, with the reason, wherever a premise fails.

**The pattern.** The sector is split by its two sheets into 5 cells, 1 material, and that cell is
turned round and **sewn**: its two side faces left out, every other face of every copy handed to
`BRepBuilderAPI_Sewing` at the split's fuzzy value (1e-5 mm), no face intersected — 0.2–0.4 s. The
glue fuse (`BOPAlgo_GlueShift`) took 24–26 s and its result, once its faces were carried onto one
surface, was invalid; cutting the turned removed cells from the blank (glued) took 153 s. Two
things make the copies merge into the whole construction's faces:
- a face of revolution about the axis is put on the source's surface with its pcurves shifted by
  the turn (a cone or sphere turned about its own axis is its parameters shifted), and the blank is
  first made again by turning its meridian section about the axis (`solvent_cad_revolved`), since
  its end spheres were revolved about the generator and a sphere turned about another axis does not
  share their parameters (the first sewn pinion had 48 sphere faces for 2);
- a ring (a face the copies close round the axis: the back cone, the end spheres) is moved into one
  period of its surface and split where it crosses the seam, so the merged ring closes on an iso
  line. Closed on the junction between two copies, its parameters ran past the period and the
  mesher left chords 0.1 mm off the back cone, which the field agreement caught. The kernel finds no
  crossing on a face past the period's end, so the piece is split on its surface turned half a turn.
The blank's parameters start opposite the sector, and the sector's faces are given the period of
its middle, so a face's pieces in neighbouring copies continue one another's (the gear's lands
straddled the start and stayed 96 faces for 48).

**Against today's construction** (`SOLVENT_SECTOR=off` builds that; volumes in mm³):

| | faces | volume | today's | relative |
|---|---|---|---|---|
| pinion | 147 = 147 | 14,642.064832 | 14,642.074947 | 6.9 × 10⁻⁷ |
| pinion, 10 µm | 147 = 147 | 14,643.426027 | 14,643.427605 | 1.1 × 10⁻⁷ |
| gear | 291 = 291 | 22,261.890137 | 22,261.890333 | 8.8 × 10⁻⁹ |
| gear, 10 µm | 291 = 291 | 22,261.892102 | 22,261.892874 | 3.5 × 10⁻⁸ |

Every solid is one valid closed shell with the tolerances it had (6.0e-4 / 3.9e-4 mm pinion at the
gross bars, 1.4–1.6e-5 mm the gear); the STLs pass the shell checks; the field agreement finds no
disagreement at either bar. The meter at 10 µm (20,000 samples; µm, max / p99 / mean): pinion STEP
1.37 / 1.09 / 0.02, STL 3.75 / 1.56 / 0.29; gear STEP 0.69 / 0.22 / 0.01, STL 5.00 / 3.51 / 0.89 —
every exact face within the tolerance (today's STLs, read the same way: 3.75 and 5.21 µm). The pair
check from the new STLs: no overlap, the members at least 21.3 µm apart over the pitch; flanks
24.6–25.7 µm facing the pinion's turn and 21.3–23.4 µm against it; the normal backlash 41.7 and
47.8 µm with one pair closed — each within 0.2 µm of today's files. The shafts now read
**90.00000° and 25.00000 mm** from the files (today's 89.99970° and 25.00011 mm): the copies are
exact turns of one sector, so each member's inertia is exactly symmetric.

**After** (the same machine and runs as phase 1; the sector's side, its split and the sheets' split
together under "split"):

| seconds | pinion | pinion, 10 µm | gear | gear, 10 µm |
|---|---|---|---|---|
| before: elaborate, solve, admit | 14.1 | 13.0 | 14.1 | 13.1 |
| static blank | 1.1 | 1.2 | 1.5 | 1.6 |
| sheets | 13.8 | 7.9 | 24.8 | 15.4 |
| the side, the blank again, split the sector | 5.4 | 5.1 | 6.3 | 6.5 |
| classify 5 cells | 2.8 | 2.5 | 3.9 | 3.5 |
| unite: sew, unify, validate, measure | 7.6 | 8.1 | 21.6 | 20.4 |
| write the STEP | 17.4 | 17.0 | 39.5 | 41.0 |
| mesh and write the STL | 6.1 | 84.4 (16.0 + 66.6) | 6.0 | 15.9 |
| field agreement | 4.5 | 4.3 | 10.3 | 13.2 |
| **total** | **72.8** | **143.5** | **128.0** | **130.6** |
| instructions retired | 0.55 × 10¹² | 0.96 × 10¹² | 1.04 × 10¹² | 1.00 × 10¹² |
| **speedup** (wall; instructions) | **4.8×; 4.5×** | **3.4×; 3.0×** | **5.2×; 4.6×** | **6.1×; 4.8×** |

The split and the classification went from 80% to 7% of the time. What is left is phase 3's and
4's: the STEP writer (each tooth's face still carries its whole sheet), the 10 µm pinion's two
meshings, the unification's validation and 1e-9 volume (18 of the gear's 21.6 s), the sheets, the
elaboration and admission before any of it, and the field agreement.

**Tests.** Core `tests/sector.rs`: the indexing read off shuffled placements and refused where it
is not one (two placements, a slide, a turn off the pitch, another line, unequal counts); the side
midway across a spiral band's gaps, clear of it by the gap, each cut in one sector; no gap where the
cuts are wider than the pitch; a frame's coordinates; a blank's section span, refused where it
reaches the axis. CLI `tests/native_sector.rs` (11 s): a ring cut at six places by a unit sphere
rolled about a parallel cradle (`fixtures::tools::indexed_ring`, `motions::cradle_roll`) built as
one sector is the ring built whole — 15 faces each, 7.685469 against 7.685470 mm³, a closed mesh —
and cut twice it is built whole.

**Gates.** The whole suite with the native kernel passes with no warnings (1,331 core tests); the
slow tier passes in 15.0 minutes (21.8 before: its two 10 µm member exports are the sector's now);
the web tests pass (255). Every corpus report and sheet and every non-native golden is
byte-identical to phase 6's (`crown/space.sv`'s 25 s report timed out once under the corpus's
parallel load and was identical run alone). The native swept exports' bytes change; they are
compared above by faces, volume, the meter, the field agreement and the pair check.

## Verification

Every phase: the full and slow suites, the web suite, the corpus and non-native goldens
byte-identical; the native exports compared by volume, faces, the meter, field agreement and the
pair check (bytes will change). Timings with instructions retired, on a quiet machine.
