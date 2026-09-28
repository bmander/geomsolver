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
- **Phase 3:** `solventc --tolerance 0.01mm` exports both members within 10 µm of the exact
  surface, STEP and STL, by the meter (see Phase 3's findings); without it the gross bars below
  still stand and every export is as it was.
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

#### Phase 3 — findings (2026-09-28)

**The tolerance.** `solventc … --step F --stl G --tolerance [LENGTH]` holds a native export to a
stated tolerance (0.01 mm when no length follows; a bare number is in the document's unit; `um`,
`mm`, `cm`, `m`, `in`, `thou` name their own; it is a physical length, so a document in inches is
held to the same 10 µm). Without it every export keeps the gross bars and is byte-identical to
phase 2: the corpus's goldens and recorded volumes are of that export, and a finer fit costs time
nobody asked for. Asking for a tolerance is what exporting for fabrication means. Every bar is
stated once, in `solid::export::Tolerance`:

- **Distance:** a fitted sheet within `tol/2` of every withheld contact (the fit share); the STL's
  chordal sag has the other half, so STEP ≤ `tol/2` and STL ≤ sheet + sag ≤ `tol`.
- **Normal:** `atan(4 (tol − gap) / h)`, capped at 20°, `h` the shortest side of the cells beside
  the contact. Withheld contacts lie on a grid of half cells, so no point of the sheet is further
  than a quarter cell from one along either direction; a sheet turned by `θ` there departs by about
  `gap + tan θ · h/4` within that reach, and `θ` may not carry it past the whole tolerance. A pleat
  fails it (the gear's 83° within 60 µm); a fit rounding a narrow strip where the exact normal turns
  fast (a convex corner's fan, where the gap is 0 and the normal a few degrees off) does not. A
  first derivation, `atan(π · fit / h)` from the interpolant's half-sine error profile, was too
  tight: the pinion's fan rows sit exactly on the fit with normals 4–10° off, and chasing them split
  column after column (131 columns, 9 minutes) without closing the gap.
- **The fold check** (90° between quarter-cell points in the blank) stays, and a fold still passes
  to the next row placement rather than being refined.
- **STL:** meshed at `tol/2` deflection with the 0.2 rad angle kept, then read for the sag every
  triangle actually has (`mesh_sag`: each triangle's centroid and edge midpoints against its face's
  surface, by local search from their linear surface parameters) and meshed again finer until it
  is within `tol/2`, at most four times. OCCT's deflection is a control, not a bound: at 5 µm the
  pinion's mesh left 0.9 mm edges across its fillet 50 µm off the face, and the gear's 15 µm; the
  tip cone's trimmed-edge chords sagged 7 µm, and a 0.05 rad angle mended the first but not the
  second. The meter, sampling 14,000 triangles by area, had read those meshes as 14 µm (pinion) and
  5.5 µm (gear): it can miss a few long edges among 300,000 triangles, which the sag check cannot.
- **Mesh contract:** a triangle is microscopic under `(deflection/10)²` and a crumple is more than 100
  of them in one millimetre cube. The count rule (100 in all) refused a 5 µm mesh's 2,741 slivers,
  strung along every tooth's trimmed edges, at most 125 to a cube; phase 2's fold had 18,378 in one
  place.
- **Field agreement:** probes at `max(2 tol, 2 µm)`, confirm band half that, value tolerance a
  quarter: a mesh within the tolerance leaves a probe twice it off at least the tolerance clear on
  its own side. The floor is four times the largest vertex and edge tolerance the kernel has left on
  a united member. At 10 µm: 0.02 mm, every probe resolved, none disagreeing, on both members.

**Refinement.** A sheet held to a tolerance withholds contacts at the centre *and* the middle of
each side of every cell (`contact_trace::Withheld::Sides`; sites in half rows and half columns), so
a miss says which way a cell is under-sampled: a side down a column marks those rows, a side across a
row those columns, a centre whose sides are clear both (`contact_trace::marked`). Distance misses
refine first; normal misses only once every distance holds (a normal's bar shrinks as its gap nears
the tolerance, so a distance miss is a normal miss too and would mark the other direction). The grid
splits each marked interval at its withheld coordinate, which becomes a node, and grades so no
interval is more than twice its neighbour (`Grid::refined`). The traced stations and every contact
read are kept (`contact_trace::Layout`), so a refinement costs the new stations and contacts only:
milliseconds. At most four refinements and 480×400 nodes; past either the placement is refused
with where and by how much, and the next placement is tried.

**Two parametrizations.** Each grid is interpolated with chord-length and with centripetal
parameters, and the fit that follows its withheld contacts better is the sheet. Chord length stays
true under local refinement (halving a step halves its share) but creases where a sheet's contacts
crowd on one face and stretch round the next: refined, the pinion's chord-length fit held its
contacts within 4.7 µm yet turned 31° between points a sixteenth of a cell apart at its fillet, and
the meter read 54° there; with every row halved it folded. Centripetal parameters temper that (the
pinion's first fit came 7.6 µm from its contacts against chord length's 16.5, with no crease past 5°)
but give a halved step √2 of its share, so a locally refined grid no longer matches its spacing: the
swept torus's centripetal fit went from 0.11 to 3.1 µm on three rows added, where chord length went
to 0.10. Neither holds everywhere, and each costs a fit and a local foot search (milliseconds).

**Feet.** A withheld contact's foot is now a local search from the nearest point of the fold
check's grid (`solvent_cad_surface_feet_near`), taken only within the bar; otherwise the global
projection. A local extremum is never nearer than the nearest foot, so a gap is never read too
small. It took the feet from 13 s to 0.1 s a round, and it is more right: OCCT's global projection
put one of the gear's length-row contacts 22.7 µm and 89.6° off a sheet the local search reads
within 1.9 µm, and refining for it folded the sheet.

**The configured pair at 10 µm** (`--measure-samples 100000`; µm, max / p99 / mean of |d|; normals
max / p99 over face points and STL centroids, the samples whose normal is their own face's):

| pinion | STEP (37,459 face points) | STL (99,995 samples) |
|---|---|---|
| heel sphere | 0 / 0 / 0 | 1.05 / 0.94 / 0.38 |
| tip cone | 0 / 0 / 0 | 3.37 / 2.11 / 0.45 |
| toe sphere | 0 / 0 / 0 | 1.11 / 0.94 / 0.38 |
| back cone | 0 / 0 / 0 | 1.12 / 1.12 / 0.40 |
| outer flank | 0.128 / 0.116 / 0.008 | 1.12 / 0.67 / 0.13 |
| outer fillet | 1.15 / 1.15 / 0.33 | 3.66 / 1.94 / 0.47 |
| root (crown tip) | 0.056 / 0.056 / 0.052 | 1.08 / 0.88 / 0.28 |
| inner fillet | 3.37 / 3.17 / 0.96 | 3.94 / 3.22 / 0.98 |
| inner flank | 0.434 / 0.333 / 0.012 | 1.24 / 1.03 / 0.21 |
| **all** | **3.37 / 1.95 / 0.10**, normals 1.80° / 1.70° | **3.94 / 1.79 / 0.35**, centroid normals 4.48° / 1.58° |

| gear | STEP (47,718 face points) | STL (99,995 samples) |
|---|---|---|
| heel sphere | 0 / 0 / 0 | 1.49 / 1.02 / 0.40 |
| tip cone | 0 / 0 / 0 | 2.21 / 2.13 / 0.39 |
| toe sphere | 0 / 0 / 0 | 1.29 / 1.02 / 0.40 |
| back cone | 0 / 0 / 0 | 0.59 / 0.58 / 0.20 |
| outer crown's flank | 0.517 / 0.231 / 0.010 | 1.32 / 1.06 / 0.22 |
| outer crown's round (fillet) | 1.13 / 0.95 / 0.16 | 2.22 / 1.39 / 0.32 |
| outer crown's tip (root) | 1.54 / 1.54 / 0.39 | 1.68 / 1.43 / 0.41 |
| neighbour's flank | 0.707 / 0.433 / 0.013 | 1.27 / 1.02 / 0.18 |
| neighbour's round (fillet) | 1.11 / 1.11 / 0.23 | 2.45 / 1.72 / 0.40 |
| neighbour's tip (root) | 1.54 / 1.54 / 0.12 | 1.42 / 1.19 / 0.24 |
| **all** | **1.54 / 0.81 / 0.04**, normals 1.72° / 1.00° | **2.45 / 1.21 / 0.29**, centroid normals 3.48° / 1.35° |

- Every exact face is within 10 µm in both files of both members, and the routes agree (at most
  1.3 µm apart, p99 0.001). The meter's verdict line says so, and `--measure … --tolerance` exits 1
  when a face does not.
- The pinion's fillets went from 11.6 / 16.9 µm (phase 1) to 1.15 / 3.37 µm, their normals from
  9.1° / 15.3° to 1.8° / 1.7°. Its sheet: the 73×44 first grid, centripetal, missed 10 withheld
  contacts (7.55 µm); one row interval split made it 74×44, within 3.76 µm and every normal bar.
- The gear's length-row sheet (86×32) was within 1.91 µm at once under both parametrizations; its
  walk-length sheet still folds.
- Every STL triangle sags at most 3.36 µm (pinion, after a second meshing at 1.25 µm) and 2.69 µm
  (gear, at 1.35 µm) from the written solid.
- The large STL normal figures are vertices and edge midpoints on a face's edge, where the nearest
  exact face is the other one, as in phase 2.

**Cost** (12 cores; the sheet is single-threaded):

| | export (STEP + STL) | STEP | STL | measuring STEP / STL (100,000 samples) |
|---|---|---|---|---|
| pinion | 182 s (phase 2: 93 s STL only) | 12.5 MB | 46.0 MB, 920,950 triangles | 69 s / 122 s |
| gear | 294 s (phase 2: 215 s) | 26.9 MB | 61.4 MB, 1,227,890 triangles | 100 s / 97 s |

The sheets cost seconds; the time added is the STL's second meshing (68–70 s, and 15 s for the
first) and the fits' split and fuse, as before. The meshes are three and six times phase 2's: the
deflection the sag check settles on (1.25–1.35 µm) is what OCCT's mesher needs for its few long
edges to come within 5 µm, and it is paid everywhere. A mesher refining where it sags would not.

**Tests.** Core (instant): `contact_trace::a_sheet_is_refined_where_its_withheld_contacts_miss`
(the marking rule and the graded split) and
`a_refined_sheet_reads_its_new_nodes_where_the_coarse_one_withheld_them` (the torus's layout: side
sites where they say, a refined grid's new nodes exactly the contacts the coarse one withheld,
every coarse node kept); `export_contracts::a_tolerance_mesh_contract_refuses_a_cluster_not_a_count`.
CLI: `a_swept_export_is_refined_into_its_tolerance` (the swept torus at 0.1 µm: both first fits
miss, it is refined until it holds, the meter reads both files within 0.1 µm, and without a
tolerance the sheet is unrefined), `a_tolerance_is_a_positive_length_for_a_native_export`. Slow
tier: `the_configured_gear_exports_natively` and `the_configured_pinion_exports_natively_within_its_tolerance`
export each member at 10 µm and measure both files within it.

**Not claimed.** The withheld contacts and the sag check sample: a sheet's error between withheld
contacts is bounded only to first order by the normal bar, and the STL's by its triangles' centroids
and edge midpoints. The meter samples too, and by area. The tolerance is the nominal design's, not
the solve's or the shop's.

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
