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

## Phases 3 and 4 — findings (2026-09-29)

Profiled with `sample` (per-thread call graphs) and the stage trace, each change kept only where
it left faces and volume, the meter and the pair check as they were. In the order of what they
saved:

- **The volume.** 43% of the gear's run was `BRepGProp` at 1e-9, most of it the sheets' faces
  (0.4 s each, the adaptive rule crossing every knot span). Each face is now integrated on its own
  core and summed in the kernel's order (bit for bit its number, `SOLVENT_VOLUME_CHECK` compares);
  a union is checked once; the pattern's volume is its copies' count times one copy's faces' flux
  about a point of the axis (`SOLVENT_SECTOR_CHECK=full` measures it whole as well, within the slack
  its tolerances leave); and the STEP reading is measured with each sheet that is, as data, an
  earlier sheet turned by whole pitches (poles, knots, pcurves and ranges) taking that sheet's flux
  (`SOLVENT_STEP_CHECK` measures every face). The reimport's volume was computed twice; once now.
- **The admission** (13–18 s before any construction) spent three quarters of itself reading every
  later placement's blank at the first placement's points, one placement after another, and the
  rest checking each tool face in turn. Both run on every core now (`gcs_core::par`, serial on
  wasm), and the result is what the ordered pass says; a native export admits its body beside the
  blank and the sheets (`Admitted::Beside`), a refusal keeping nothing built.
- **The sheets.** A cutter of revolution about its own axis (the pinion's) is sectioned once and
  every station is that section turned; any other (the gear's, bounded by a neighbour turned about
  the crown's axis) keeps its sections by angle, cuts each once (the count call kept what it cut),
  and sections the reach's 96 stations on every core. The gross fit reads each withheld contact's
  foot from the fold grid, as the tolerance fit did, trusting a local foot within 5 µm (the global
  search was 37% of the sheets). The sweeps' sheets are built side by side and beside the blank,
  each awaiting it only for its clearance, what each says said in order (`progress::side_by_side`).
- **The STL** is meshed as one sector (on every core, its sag read on every face but the sides) and
  its triangles turned into each copy; each point where a face meets the second side is paired one
  to one with the turn of a first-side point and written as that point turned into the neighbour,
  so neighbours share their seam points bit for bit (at 10 µm they coincide; at the gross bar the
  mesher places a side's points up to 5.6 µm apart along the seam, and the pairing moves them along
  it). The 10 µm pinion's two meshings went from 83 s to 3.5 s. `SOLVENT_SECTOR_STL=off` meshes the
  solid whole.
- **The STEP** writes each sheet face on its B-spline cut to the face's parameter box (the gear's
  surfaces' poles 323k to 142k): 26.4 to 12.6 MB (pinion), 44.6 to 27.4 MB (gear). It is written
  and read back on a thread beside the mesh and the field agreement (`progress::beside`).
- **The rest:** the sector's split runs the kernel's Boolean intersections in parallel (a split so
  run is given each core's processor budget); each cell's interior samples, the side's probes and
  the field agreement's triangles on every core, an evaluator a thread; a closed solid checked a face
  at a time on every core with its closure and orientation asked directly (`valid_solid`;
  `SOLVENT_FULL_CHECK` runs the whole analyzer — its own parallel mode called a valid pinion
  invalid, not every time); the native session guarded for threads (`backend/occt.hpp`).

**Against phase 2** (volumes in mm³; faces 147 and 291 throughout, one valid closed shell each):

| | volume | phase 2 | relative |
|---|---|---|---|
| pinion | 14,642.072074 | 14,642.064832 | 4.9 × 10⁻⁷ |
| pinion, 10 µm | 14,643.425981 | 14,643.426027 | 3.1 × 10⁻⁹ |
| gear | 22,261.889875 | 22,261.890137 | 1.2 × 10⁻⁸ |
| gear, 10 µm | 22,261.892118 | 22,261.892102 | 7.2 × 10⁻¹⁰ |

(The pinion at its gross bars carries 6 × 10⁻⁴ mm tolerances: its volume depends on the point it is
measured about by 2 × 10⁻⁷ of itself, which the flux about the axis and about the vertices' mean
show.) The field agreement finds no disagreement at either bar; the STLs pass the shell checks. The
meter at 10 µm (µm, max / p99 / mean): pinion STEP 1.37 / 1.09 / 0.02, STL 3.19 / 1.71 / 0.28; gear
STEP 0.69 / 0.22 / 0.01, STL 5.00 / 3.35 / 0.81 — every exact face within the tolerance. The pair
check: no overlap; flanks 24.6–25.8 µm facing the pinion's turn and 21.3–23.4 µm against it; normal
backlash 41.7 and 47.8 µm with one pair closed; 90.00000° and 25.00000 mm — each within 0.1 µm of
phase 2's.

**After** (the same machine, load average 4–9 from other work throughout; the first phase is the
admission beside the blank and the sheets, and "files" the STEP beside the STL and the field
agreement; the baseline's stages summed alike):

| seconds | pinion | pinion, 10 µm | gear | gear, 10 µm |
|---|---|---|---|---|
| elaborate, solve | 0.4 | 0.4 | 0.4 | 0.4 |
| admission ∥ blank ∥ sheets (baseline: 30.1, 29.3, 42.0, 39.8) | 3.2 | 3.3 | 5.7 | 5.5 |
| the side, the blank again, split the sector (baseline: split 184–372) | 2.6 | 2.2 | 3.1 | 3.0 |
| classify (baseline 94–268) | 1.4 | 0.9 | 1.5 | 1.3 |
| unite: sew, unify, check, measure (baseline 8.3–25.8) | 1.2 | 1.0 | 1.8 | 1.6 |
| STEP ∥ (STL, field agreement) (baseline: 32.4, 139.7, 57.6, 95.1) | 3.0 | 5.9 | 7.1 | 7.2 |
| **total** | **11.7** | **13.7** | **19.5** | **19.0** |
| baseline (phase 1) | 349.2 | 492.4 | 662.6 | 801.1 |
| **speedup** (wall) | **30×** | **36×** | **34×** | **42×** |
| instructions retired (× 10¹²; baseline 2.52, 2.90, 4.82, 4.77) | 0.28 | 0.31 | 0.40 | 0.41 |
| user time, s (all threads) | 60 | 60 | 88 | 88 |

**Short of 100×: the floor.** The totals are 3.4 to 2.4 times the targets (3.5, 4.9, 6.6, 8.0 s).
Instructions fell 9–12 times; the rest of the wall-clock gain is parallelism, and the run now keeps
four to five of this machine's six cores busy (60–88 s of user time in 12–20 s) while other work
holds the load at 4–9, so what remains is work, not waiting. Measured, it is:
- **OCCT's STEP writer and reader**: the gear's 27 MB written in 1.4 s and read back and transferred
  in 2.9 s, the pinion's in 0.6 and 1.1 s — serial, and the round trip is the check that the file
  reads as the solid. A file of one sector and its pattern (mapped items) would cut it by the
  count, but an indexed solid whose rings are merged across copies is not a set of mapped copies.
- **OCCT's Boolean intersections**: splitting the sector by its sheets and sides, 2–3 s even in
  parallel, a third of it `GeomLib_CheckCurveOnSurface`'s particle-swarm search for each new edge's
  tolerance on the B-spline sheets and sides; and the blank's four Booleans of revolutions, 1–1.5 s
  (hidden beside the sheets). A blank made by 2D Booleans of its meridian profiles and one
  revolution would remove the latter.
- **The admission's placement comparison**, 13–16 s of CPU for the gear (every read point of 47
  placements of two sweeps against the blank): it is exact per point, and only a structural argument
  (the blank is a solid of revolution about the indexing axis) would replace it — a change to what
  the admission's evidence says, left for a decision.
- **The field agreement**, about 10 s of CPU for the gear: each of its ~2,000 probes encloses every
  one of the 96 swept operands, far ones included.
- **Tracing the sheets** in the core (1.5–3.5 s a sweep, the gear's removal traced twice, its walk
  placement refused before its length one), and the cells' interior samples (~1 s, OCCT's classifier).

**Tests and gates.** CLI `tests/native_sector.rs`: the indexed ring meshed as one sector turned is
one closed shell enclosing the whole solid's mesh volume to 10⁻³; core `tests/par.rs`: `par` answers
in order and keeps a state a thread. The whole suite with the native kernel passes with no warnings
(1,371 tests); the slow tier passes (1,384); the web tests pass (255). Every corpus report and sheet
and every non-native golden is byte-identical to phase 2's (`crown/space.sv` timed out once under
the corpus's parallel load and is identical run alone); the one native swept golden,
`swept_torus`'s default STL, changes bytes by design (its cutter is now sectioned once and turned),
its volume (0.447407 mm³) and faces (3) the same.

## Phase 5 — findings (2026-09-29)

Profiled with `sample` and the stage trace; each change kept only where faces, volume, the meter, the
field agreement and the pair check held. In the order of the pipeline:

- **The admission, by structure.** Where every operand of the blank is a full revolution about one
  line or a ball centred on a point of it (the end spheres, revolved about the generator, are balls),
  read off the solid graph (`admission::structurally_alike`), and every later placement of every sweep
  is the first turned about that line (both checked on the solved axes to 10⁻¹² of the blank's size;
  the configured members are off by 1.9 × 10⁻¹³ and 1.7 × 10⁻¹³ against 4 × 10⁻¹⁰ and 6 × 10⁻¹⁰), the
  blank is the same under every turn and each later placement's checks are the first's: the sampled
  comparison (1.7 s on every core for the gear) is not run, and the evidence says which
  (`Equivalence::Revolved`, else `Sampled` as before; the CLI line says "the rest turns of it about
  the axis the blank is a revolution about"). The checks of the first placement read each tool face's
  rows on every core, what samples say of their neighbours asked in order afterwards: the configured
  members' evidence is the same to the digit (samples, contacts, spacing, least area factor), the
  admission 2.9 s to 1.2–1.5 s (gear). Tests: the indexed ring admitted by its revolution; the same
  ring bounded by a prism (no revolution of the graph) falls back to sampling and finds the same
  evidence; the bevel pinion and both configured members take the structural path.
- **The blank, by its meridian section.** Where every operand is a full revolution about one line
  (or a ball centred on it), each operand's section is taken in one half-plane of the line, the
  recipe's Booleans are taken of the sections and the region is turned once
  (`Session::construct_meridian`, `solvent_cad_revolve_region`): 30–40 ms where the Booleans in
  space took 1–3.7 s beside the sheets; any other recipe is built by its Booleans, with the reason
  (`SOLVENT_BLANK=booleans` always). The sector's blank is that region turned again from the
  half-plane its parameters start on. That blank meshes the pinion's two sides at the gross bar with
  a seam point 11 µm from its partner's turn (the Boolean blank's sections: 5.6 µm), past the 10 µm
  the pairing allowed: points move along the seam, on the same two surfaces, so the gross bar pairs
  within 20 µm; and where a seam does not pair the sector is meshed again a tenth finer (never needed
  by the members). Sectioning the meridian blank instead (`solvent_cad_revolved`) paired the gross
  seam but left the 10 µm pinion's sides with 116 and 117 seam points: the sides' discretization
  follows the blank's edges, not only its shape. Test: the ring's blank turned from its section is
  its Booleans' (volume to 10⁻⁹, faces), and a prism-bounded ring is refused the section, by name.
- **The sheets.** A layout's columns are sectioned, traced, resampled and measured on every core,
  and a sheet's stations and contacts read side by side before the ordered pass assembles it (the
  sheet, and the STEP, the same to the byte); the removal's first sheet 2.3 to 1.1 s. At a tolerance
  the two parametrizations are fitted side by side. The gear's cutters (a crown bounded by its turned
  neighbour) are still a 3D Boolean each, 1.0–1.4 s at the start of each sheet's thread, which now
  bounds this phase; Booleans run their intersections in parallel with oriented boxes, which halved
  the clearance (0.3–0.8 to 0.1–0.17 s) and left the cutters' Commons as they were. A prism,
  revolution or placement is stored with the check and volume it was just given, not checked again.
- **The split and the cells.** The partition's cells are checked and measured with the work shared:
  every face checked once on every core, each cell's shell closed and oriented, every face's flux
  integrated once and each cell's volume summed from its faces' fluxes (0.99 to 0.42 s, the gear).
  A single kept cell is its own union with its partition's volume (0.47 s to 0.01 s). A cell whose
  faces' candidates miss ten times running is a sliver, sampled along the rays between its faces at
  once (interior samples 1.7 to 1.0–1.1 s). The split itself (1.8–2 s) is OCCT's.
- **The union.** A copy's faces' flux is measured while the pattern's union is unified and checked.
  (Deriving it as the sector's volume less its sides' flux moved the pinion's gross-bar volume by
  1.1 × 10⁻⁵ — the sides' trims leave 6 × 10⁻⁴ mm gaps — and was dropped.)
- **The STEP.** Written into memory and read back from the same bytes; the model's entities
  formatted on every core, a writer of its own a run of them, the runs joined in order: the kernel's
  text to the byte (`SOLVENT_STEP_TEXT_CHECK` formats it both ways and requires it; a CLI test does),
  2.4–2.8 s to 0.4–0.6 s for the gear. Reading back without the kernel's repairs is not a check of the
  file: the reading's sphere rings (and, at the pinion's gross bars, its sheet faces) come back
  `UnorientableShape` until `ShapeFix` orients their wires, and repairing only those faces did not
  mend the pinion's; the repairing read-back stays.
- **The field agreement, a sector at a time.** Once the field reads alike under the turn at sampled
  points, each probe is turned by whole pitches into one sector and read by the cuts not proved
  positive over the box of a 96-cell grid it is in (`agreement::Sector`, `MaterialField::without_cuts`:
  what is left out may only have added removal, so the field left is the same wherever an operand
  left out is positive and nowhere higher — no verdict changes; a probe outside the boxes reads the
  whole field where it stands). The gear's probes read 7–8 of its 98 cuts, the pinion's 11–12 of
  50, the proofs 60–100 ms; the agreement 1.8–2.5 s to 1.1 s (gear). Test: the indexed ring read a
  sector at a time decides every point's side and band as the whole field does (1,728 points, one
  cut of six kept in any box).
- **The STL.** Its shell check (1.2–1.5 s for the 10 µm pinion's 952k triangles) runs beside the
  field agreement's reading of the same bytes, and numbers vertices and edges by sorting packed keys.

**Against phase 4** (volumes in mm³; faces 147 and 291, one valid closed shell each):

| | volume | phase 4 | relative |
|---|---|---|---|
| pinion | 14,642.071530 | 14,642.072074 | 3.7 × 10⁻⁸ |
| pinion, 10 µm | 14,643.425980 | 14,643.425981 | 6.8 × 10⁻¹¹ |
| gear | 22,261.889867 | 22,261.889875 | 3.6 × 10⁻¹⁰ |
| gear, 10 µm | 22,261.892118 | 22,261.892118 | 0 |

The field agreement finds no disagreement at either bar; the STLs pass the shell checks. The meter at
10 µm (µm, max / p99 / mean): pinion STEP 1.37 / 1.09 / 0.02, STL 3.19 / 1.68 / 0.28; gear STEP
0.69 / 0.22 / 0.01, STL 6.16 / 3.36 / 0.81 (phase 4: 5.00 / 3.35 / 0.81) — every exact face within the
tolerance. The pair check: no overlap; flanks 24.6–25.7 µm facing the pinion's turn and 21.2–23.4 µm
against it; normal backlash 41.6 and 47.8 µm with one pair closed; 90.00000° and 25.00000 mm — each
within 0.1 µm of phase 4's.

**Timings.** A run's load average counts its own threads (up to 20–40 runnable), so each export was
run alone once the machine's one-minute load had fallen under 5 (the rest of this machine's work
holds it at 3–5), phase 4's binary the same way just before (its recorded 11.7 / 13.7 / 19.5 /
19.0 s were at load 4–9); two back-to-back runs of each, loaded (15–28 at their starts), beside:

| seconds (quiet) | pinion | pinion, 10 µm | gear | gear, 10 µm |
|---|---|---|---|---|
| elaborate, solve | 0.41 | 0.56 | 0.43 | 0.45 |
| admission ∥ blank ∥ sheets (phase 4: 3.97, 3.41, 5.88, 6.09) | 1.71 | 2.19 | 2.60 | 2.91 |
| the side, the blank again, split the sector (2.86, 3.03, 2.92, 2.73) | 2.09 | 2.14 | 2.73 | 2.70 |
| classify (1.15, 1.09, 1.69, 1.37) | 0.90 | 1.05 | 1.30 | 1.12 |
| unite (1.10, 1.09, 2.06, 2.05) | 0.67 | 0.73 | 1.24 | 1.26 |
| STEP ∥ (STL, field agreement) (3.36, 6.11, 6.84, 6.98) | 2.82 | 5.79 | 5.33 | 6.15 |
| **total** | **8.60** | **12.46** | **13.64** | **14.59** |
| phase 4, the same way | 12.88 | 15.15 | 19.89 | 19.67 |
| loaded, best of two | 9.45 | 12.93 | 15.28 | 15.28 |
| baseline (phase 1) | 349.2 | 492.4 | 662.6 | 801.1 |
| **speedup** (quiet; loaded) | **41×; 37×** | **40×; 38×** | **49×; 43×** | **55×; 52×** |
| instructions retired (× 10¹²; phase 4 0.28, 0.31, 0.40, 0.41) | 0.19 | 0.24 | 0.28 | 0.31 |
| user time, s (all threads; phase 4 62, 66, 90, 91) | 41 | 54 | 61 | 70 |

**Short of 100×: the floor.** The totals are 2.5, 2.5, 2.1 and 1.8 times the targets (3.5, 4.9, 6.6,
8.0 s). What is left is OCCT's, serial or nearly, and each piece was measured:
- **The STEP round trip** (the gear's files stage, 5.3–6.2 s): the transfer to STEP entities 0.4–0.6 s,
  the text 0.4–0.6 s (from 2.4–2.8), parsing it back 1.2–1.5 s, transferring it back with the
  reader's repairs 1.6–1.8 s (`ShapeFix` 1.0–1.1 s of it, and needed: without it the reading's sphere
  rings and, at the pinion's gross bars, its sheet faces are `UnorientableShape`), then its check,
  orientation and patterned volume 0.9 s. Only a writer and reader of our own (the text is formatted
  in parallel already; parsing and transferring back are serial in OCCT) or a lighter verification of
  the file would move it.
- **The 10 µm pinion's mesh** (its files stage, 5.6–5.8 s): meshed at 5 µm it sags 30 µm (OCCT's
  deflection is a control; the fillet's crowded parameters), so it is meshed again at 1.25 µm (2.2 s of
  `BRepMesh` for one sector, 0.5 s of sag), 952k triangles written and checked. Meshing only the faces
  that sag, or speculatively at both deflections side by side, would take about 1.5 s off.
- **The cutters' Booleans**: the gear's two cutters (a crown bounded by its turned neighbour) are a
  3D Common each, 1.0–1.4 s at the start of each sheet's thread, and bound the first stage (the
  admission, 1.2–1.5 s, now runs beside them). Sectioning the two revolutions apart and intersecting
  the sections in the plane would remove it.
- **The sector's split** (1.8–2.0 s of `BRepAlgoAPI_Splitter`, a third of it
  `GeomLib_CheckCurveOnSurface`'s particle swarm on the B-spline sheets and sides, which no option
  of the kernel turns off), the cells' interior samples (0.7–1.0 s of OCCT's classifier and distances)
  and the union's sewing, unification and check (1.2 s).

**Tests and gates.** Core `tests/admission.rs`: the indexed ring admitted by its revolution, the same
ring bounded by a prism by sampling with the same evidence; the configured members and the bevel
pinion by their revolution; the indexed ring's field read a sector at a time decides as the whole
field. CLI `tests/native_sector.rs`: the ring's blank from its meridian section is its Booleans'; a
STEP formatted side by side is the kernel's text. The whole suite with the native kernel passes with
no warnings (1,375 tests); the slow tier passes (1,388); the web tests pass (255, on a wasm build).
Every corpus report and sheet is byte-identical to phase 2's (`crown/space.sv` timed out under the
corpus's parallel load and is identical run alone), and every non-native golden is, but for the
words of the STL stage's last line (it now says how long its shells took) and the admission's
(which evidence stood for the later placements); the native swept golden, `swept_torus`'s default
STL, changes bytes (its blank is its meridian section turned), its volume (0.447407 mm³), faces (3)
and triangles (580) the same.

## Verification

Every phase: the full and slow suites, the web suite, the corpus and non-native goldens
byte-identical; the native exports compared by volume, faces, the meter, field agreement and the
pair check (bytes will change). Timings with instructions retired, on a quiet machine.
