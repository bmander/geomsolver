# The tube road's trims read by membership: a sketch for review

The tube road (`swept::tube`, #880 T1f; `docs/contract/generating.md`) builds a torus's sweep as
the points within the tube of the surface its core circle sweeps. Its sheets are maps over
`(θ, t)` and its depth (`swept::depth`, `Depth::reading`) is exact; but the solid's *topology* is
assembled by classifying the depth's seams and the events they end at, one rule an event, and
every event the rules do not foresee is a refusal. The swept-solid design's own answer to
"which subset of the sheets is the boundary" is **membership, not local rules**
(`swept-solid-design.md` §1). This note sketches the tube road brought back to that answer: its
trims are where membership changes, its vertices where what lies across a trim changes, and its
faces patches of fixed charts — so no event needs a rule of its own. Nothing below is built yet.

## 1. What the fuzzing found

brep-bench's crossed members fuzzed (`swept_tumbles`): 52 jittered tumbles, 12 built, 40
refused, none wrong. Each refusal class, shrunk to a minimal tumble, is a gate in
`gcs-core/tests/swept_tumbles.rs`. Where each is raised:

| refusal | raised in | stage |
|---|---|---|
| a trim not joined to its corner; a trim ends beside a corner, its partner no end pose | `faces::solid` → `cornered`, `join_holds` | an end event's rule |
| a double curve not joined to its swallowtail point | `faces::joined` (the normal form) | an end event's rule |
| the depth about a swallowtail point within the tolerance on no disc | `tube::disc` | an end event's rule |
| a switch whose crossing is not on its partner; a switch's rim crossing not settled; a rim crossing not settled | `faces::switched`, `crossing` | an end event's rule |
| the trims leave no angle free for the windows' cuts | `faces::windows` | the windows |
| a curve fits only to 1.6e-4 | `faces::fitted` | a window's trims fitted whole |
| three or more faces meet along an edge | `sew` | the assembly |
| N instants touch a meridian whole | `Tube::pinched` | one collar only |
| a ring's contact's sheet fits short; the boundary open; a torus's piece facing behind; a sheet's pieces end nowhere | `rings`, `patches`, `trim`, `boolean` | **the ring road**, before the tube road is tried |

cadrs (MIT/Apache, the one peer that sweeps these) builds 10 of the 15 exactly and check-clean in
0.4–1.1 s, by the method below (`cadrs_kernel/src/native/sweep_tube.rs`, `sweep_moving.rs`):
trims from membership on a grid over small patches (`cover_curves`), each crossing knowing the
instant that covers it and so the face across it (`trace`), stretches split where that face
changes (`chain_runs`), patches fitted over what they keep (`fit_tensor`), faces assembled
tolerantly (`assemble`). Our depth, sheets, charts, collar and bands are the same objects as
its; the difference is only how the trims and faces are made. (Its tolerant assembly also passes
edges its own check finds 1.5e-4 off their faces in two cases: what we keep is our exactness.)

## 2. Membership on a chart

Every chart (`Chart::Sheet(±1)`, `End(k)`, `Collar`, `Band(k, ±1)`) already reads, at a place `q`:
whether its sheet faces out there (`Tube::side`, `J`'s sign), and if it does whether another
instant covers it (`Depth::reading` → `Reading::Far(Cover)`: the depth and the covering instant
`τ` and point). That is the **membership field** `m(q) ∈ {kept, covered(τ, x), inward}` — `inward`
past a fold, which bounds nothing (`swept-solid-design.md` §1: the folded-over sheet lies inside).
Nothing new is computed; what changes is that `m` is the only thing the topology is read from.

## 3. Patches, not windows

Each chart is cut into **fixed patches** in its own parameters: a sheet into `θ` quarters by `t`
chunks of at most a quarter turn of the motion; an end pose's torus into its leading and trailing
halves by quarters (as now, `End(k)`); the collar and each band one patch each (their seams are
already their edges). A patch's edges are parameter lines of its chart, **shared** with the patch
beside it: what crosses a shared edge is read once, on the edge, and given to both. So:

- no angle need be free for a cut (`windows` goes): a patch edge may cross a trim anywhere;
- a net is fitted over a patch, over what it keeps (`fit_graded`'s `kept`, as now), so no
  window's net is graded to 944 × 849 about a near pinch, and a misfit splits one patch, not the
  build;
- each patch is one face, faces of one chart joined across their shared edges after (§6).

## 4. Trims: where membership changes

Over each patch, the **certified search** the depth already runs (`zero_set::trace`: cells proved
uniform by the depth's bound and dropped, the rest halved to a floor, the zero set marched)
gives the kept/covered boundary as chains, each point carrying `(q, τ, x)`. What changes is that
**a chain ends only where it meets the patch's edge, or closes**. It is then cut into
**stretches** at the places where what lies across it changes, each found by bisection along the
chain on a test of `(q, τ)` alone:

| where a stretch ends | test along the chain | replaces |
|---|---|---|
| the face across changes chart (a sheet → an end pose → the collar → a band) | `class(cover)` changes | `switched`, `rim_vertex`, `seam_vertex`'s classing |
| the patch's edge (a shared parameter line, a rim at `t = a, b`, a pinch's or band's seam) | the edge's own 1D bisection of `m`, read once for both sides | `crossing`, the rim and seam crossings |
| the face across is this one where the double curve turns back (a swallowtail point) | `τ − own(q)` changes sign | `swallowtails`, `disc`, `joined` |
| three faces meet (a triple point) | the cover's class changes on the *other* face too | `met` |

A swallowtail point is then the turning point of one stretch of a sheet with itself, held twice
(out along one preimage and back along the other), not an event with a disc and a normal form;
an end pose's corner is where the rim's own bisection meets the chain, not a straight join
`cornered` must prove holds. Each stretch is **one edge**: its points settled onto the double
curve of its two faces (`settle`, Newton on `X₁(u₁) = X₂(u₂)`, as `refined` does now), its curve
fitted per stretch (`fitted`, short stretches), its pcurve on each face from `q` and from the
partner place (`Tube::partner(x, cover)`, which exists). A stretch read from both sides (the
partner's march finds it too) is read once: the lower `(chart, patch)`'s reading kept, the other's
matched by its ends and dropped (cadrs's `drop_twins`).

## 5. Pinches and bands

They stay charts, their seams patch edges. Two changes follow from the patches: a band's tip
crossing (the open case in `generating.md`: the band's arc of the seam with no sheet face beside
it) is a shared-edge crossing read once for the band and the sheet; and **more than one pinch**
is more than one collar chart, each with its own seam (`Tube::pinched` refuses two or three
today) — nothing in §4 counts them.

## 6. Faces and the assembly

A patch's kept region, bounded by its stretches and its edges' kept stretches, is walked as now
(`walk`), its net fitted over its box with `kept` (`fit_graded`), each loop's pcurves the stretches'.
The assembly is exact: two faces share an edge where they share a stretch or a patch edge's
stretch — both read once — so `sew` joins edges that are the same object, and three faces at an
edge is a bug in §4, not a tolerance. Faces of one chart side by side across a patch edge are
then merged where their nets join smoothly (one net refitted over the union where it holds the
tolerance, else left as two faces): the face count is the chart's, not the patches'.

## 7. The road taken

`closed::ring` tries the ring road and takes the tube road only where the ring road says `Curved`
(a meridian touched whole). The tube road as it stands already builds three of the four slow
gates directly (a tube 0.147: 8.5463430810, cadrs 8.546343; a tube 0.27: 21.4295434731, cadrs
21.429543; a tube 0.22 seen from its own axis: 18.0825371883 against cadrs 18.081851 — 7e-4 apart,
to be settled before either is pinned). So: **the tube road wherever the ring road refuses a
whole torus**, whatever it refuses for.

## 8. Steps, each with its gate

1. **The road taken** (§7) — *done*: `closed::ring` falls back to the tube road on any refusal of
   a whole torus, and the tube road's solid is held to the sweep's boundary (`faces::held`), a build
   held off it built again with each chart's depth completed on a grid (`Tube::completed`). The 7e-4
   was ours: the tube road kept a folded sheet 0.15 inside that solid, a pocket beside its fold
   missed by the depth's search (`docs/contract/generating.md`). The three gates build, each volume
   cadrs's to 1e-6; the fourth (a torus's piece facing behind) the tube road refuses too.
2. **The membership field** (§2) as one type over every chart, and **patches with shared edges**
   (§3), the edges' crossings read once — *done* (`tube::membership`): on the half turn and both
   crossed members the patches' edge crossings are the present trims' crossings of the same lines,
   within four tolerances in space (the trims' chords stand within one of the zero set; membership's
   crossings are settled to it), a grazing crossing matched by its standing on the zero set. A line
   read past a fold's edge had to be halved toward it: a double curve beside the fold lay between a
   quarter feature's readings.
3. **Chains to stretches** (§4) over patches, beside the present road (both run; the stretches
   compared to the present trims on the crossed members and the 12 fuzzed builds: the same
   curves, ends to the tolerance). Then the end-event rules go: `switched`, `cornered`,
   `join_holds`, `joined`, `disc`, `windows`.
4. **Faces per patch and the merge** (§6). Gate: the crossed members' volumes as pinned
   (16.6109998153, 13.7898471844) to 1e-9; every `swept_tube` gate; then the `swept_tumbles`
   gates flip one by one, each to a build checked against the exact sweep at random points and
   against cadrs's volume.
5. **Pinches past one** (§5). Gate: `three_instants_touch_a_meridian_whole`.
6. **The fuzz** made a slow-tier test: a fixed set of seeds, each built or refused by name, none
   wrong — the count built only rising.

## 9. What it costs and risks

- **Grazing**: a stretch shorter than the search's floor cell is found only where the depth's
  bound proves its cell mixed; the floor stays where it is now, so nothing the present road finds
  is lost, and the same cells are read.
- **Time**: the membership reads are the present tracing's (3.4–3.7 s a sheet on the crossed
  reverse); the fits are smaller (no graded window about a near pinch); the extra cost is the
  patches' edges and the merge. cadrs's whole sweep is 0.4–1.9 s by this method.
- **Face count**: patches merged per chart (§6); where a merge does not hold the tolerance the
  solid has more faces than now, not a wrong one.
- **The twins**: a stretch read from both sides must match to be dropped; a mismatch is a refusal
  that names both readings, the one place §4 can still refuse, and a gate when it happens.
