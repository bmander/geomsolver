# Faces trimmed on nets singular outside them (#1013)

The three crossed Booleans of brep-bench's tumbling tori (`swept/tumbling-torus-crossed-union`,
`-difference`, `-intersection`) refuse. Their operands build and are valid. This file is steps 1–3
of [#1013](https://github.com/bmander/geomsolver/issues/1013): what the kernel cannot do, each thing
pinned by the smallest case that shows it, and those cases added to brep-bench. Step 4, building
them: T1, T2, T3 and T4 are built (see *Step 4* below). The crossed tori themselves are next.

## The capabilities missing

Each operand face of the tube road lies on a fitted net that runs on past the face: folded back
past the face's cuspidal edge, with a second layer near the face, a cusp, or an isolated singular
point of its parametrization just outside. The Boolean reads a B-spline face through its whole net.
It finds a point's foot on the nearest layer, finds a line's crossings over the whole net, and
marches a meeting over the whole net. Only after that does it place what it found against the
face's trim. Each of the four capabilities below fails at a different one of those readings. Each
was found and shown alone on a block under a Bézier patch with no sweep in it. Each case fails on
`main` (6836e76). Each builds to its closed form, by every Boolean asked of it, when the top face is put
on the patch cut down to its own part (`Net::segment`: the same surface, the same parameters,
nothing past the face). All four belong to the Boolean, not the sweep. The sweep can avoid them
by giving its faces nets that do not run on (the experiment branch's `Net::segment` of each face to
its pcurves' box, 8ad646d, or `claude/880-t1f-tube-faces`). But brep-bench's cases are about faces
whose surfaces do run on, and the same nets reach the Boolean from STEP files.

| # | capability | minimal case (gate, `tests/patch_layers.rs`) | fails on `main` with | how |
|---|---|---|---|---|
| T1 | **A meeting placed on a face by its own layer**: a stretch of a meeting on the net's other layer is not on the face, even where its foot on the nearest layer is | `nurbs/folded-patch-notched-at-its-rim`: patch `(s², v, 1/2 + 3s⁴/10 + v/10)`, covering itself exactly; the face `u ≥ 13/20`; a box whose side `y = 1/2` crosses the face's rim | "a bspline's pieces do not close" | the side meets the patch in one exact iso-curve over the whole net (`u` in [0, 1]). Its stretch on the folded-back layer (`u ≤ 7/20`) is given a pcurve by its foot, which lies on the face's layer. So the edge from the rim to `x = 1` is laid a second time on the face (edge 33, `u` from 0.65 to 1, over the true edge 32). The failure needs the side to cross the rim: a box that starts at `x = 0.2` builds. |
| T2 | **An edge's crossings with a face found on the face's own layer**: every crossing of a line with the net, each placed by its own parameters | `nurbs/patch-over-a-near-layer-pierced`: the same patch plus `s³/1000`, its folded layer `x^{3/2}/500` (1.8e-4 to 5.7e-4) beneath the face; a square hole through it | "a shell of the arrangement is kept on one side of a plane (`near`) and dropped on another" | each upright edge of the hole crosses the face once and the other layer just beneath it, and the crossings are read wrong: at `x = 0.4, y = 0.3` the two as one touch, at `x = 0.7, y = 0.3` the other layer's alone (the face's lost), at `x = 0.4, y = 0.6` the other layer's twice. *Corrected*: the case first stood at `s³/100`, where, given exactly, every crossing is read right and the case failed only through T1 (the hole's walls `y = 3/10, 3/5` are iso lines); the lost crossing seen there was the probe's poles, interpolated to 1e-16. With T1 built, `s³/100` builds. |
| T3 | **A meeting ended at the face, or at a fold of its net**: no march over the net past the trim into its cusp | `nurbs/cuspidal-patch-sliced-through-its-cusp`: patch `(s², v, 1/2 + s³ + v/10)`, face `u ≥ 11/20` (a hundredth from the cusp); cut by `z = 11y/10` through the cusp | "a shell of the arrangement is kept on one side of a bspline and dropped on another" (or "a bspline's pieces do not close", cut otherwise) | a march runs on past the rim to the cusp, outside the face, and comes back along itself. Cut by a slab under the plane, it is laid as a closed curve of 69 points, from the rim to the cusp and back. Cut by `build::kept`, as the gate and the bench cut it, the march from the face's far edge goes to the cusp and returns to its own start (224 points). Another march, through the cusp onto the other layer, ends the true stretch's curve without its end on the rim, so the stretch dangles. This is the issue's "a march creeping across a fold of its sheet". With the face trimmed at `u = 13/20`, or with a steeper plane, it builds. It also depends on where the cutting block's faces lie, because they seed the trace: cut by a block standing on the plane at another place, it builds. |
| T4 | **A meeting ended short of a singular point of the net outside the face** | `nurbs/pinched-patch-sliced-beside-its-pinch`: patch `(s³, t³, 1/2 + st)`, singular at `u = v = 1/2` alone; face `u, v ≥ 11/20`; cut by `−x + y + z/10 = 13/250`, 0.002 from the singular point | "a shell of the arrangement is kept on one side of a bspline and dropped on another" (or "a bspline's pieces do not close", cut otherwise) | the march runs from the face's far edge, past the rim, through the singular point where the meeting turns sharply, and on into the opposite quadrant (170 points). Its fit takes five to ten rounds and leaves gaps, and from the rim point a second march goes to the singular point and back as a closed curve (79 points), so the face's stretch is laid off the meeting or twice. Another placement of the plane, `y ≈ 2x + 0.0005`, turns back at the point instead and lays the face's stretch twice (239 points, "closed"). This is the issue's "a march cycling through an isolated singular point". It is erratic: of 32 planes within 0.002 of the point, 10 fail; the net cut to the face builds all 32. |

None of the three rules on `claude/880-t1f-trimmed-meetings` fixes any of the four cases. All four
still refuse there, the same way. The seed filter acts on seeds, and T1's meeting is a closed form
with none. The fold and self-meeting rules end marches, but T3's march is ended in the wrong place,
and T4's fails in its fit.

### Struck or not yet shown

- **"A march stalled where the projection lands back on the point it stood on"** and **"a march
  creeping at the least step"** are what T3 and T4 look like from inside the march. They are not
  separate capabilities: a march that stays inside the face's trim never meets them.
- **"Where a march ends on a face's boundary"**: no minimal case needed it. In every case above the
  end on the rim was found once the stretch past the rim was not marched.
- **The radial order round an edge**: struck as the cause of the crossed union's leak on the
  experiment branch. Each of the 21 edges that join a kept side and a dropped side of the leaking
  shell is explained by a piece that spans both sides of the other operand, with no misordered
  edge needed. One piece of `A1` (`Sheet(1.0)`), face 8 of the complex, is in 18 of them. It uses meeting edges 236, 238 and 239 twice: round
  each, "8, a face of B, 8, a face of B". So those meetings are slits inside one piece that spans
  both sides of B, and the face was never parted along them. The debug line "every meeting edge
  used four times" counted face 8 twice. See *The crossed union* below.
- **Time** is not pinned by a case of its own. The minimal cases show it: on the whole net each
  Boolean takes 0.3–0.5 s against 0.02 s on the net cut to the face (T1, T2), and a box that does
  not reach the rim of the exactly doubled patch builds in 1.4 s against 0.03 s. The tori's
  144 s of face pairs are the same reading over nets of 940 × 850 poles. A bench case for the
  bench's own reads of such nets (the reverse torus's 407 s) would time the harness's STEP reader,
  not a kernel, so it is left to the harness. Note: `harness/mass` itself cannot read OCCT's
  STEP files of T1 and T3 ("a face whose loops bound no region of its surface", "a loop that does
  not join up on its surface"). The bench's reader has T1/T3's capability gap too.

## The crossed union

On `claude/880-t1f-trimmed-meetings` plus a classification report (each edge's uses, each face
with both sides in the leaking shell, each join of a kept and a dropped side with its edge's
radial order, and the shortest walk over the side graph from the outer cell), the union refuses
in 162 s.

- The leaking shell holds both sides of 41 faces. Twenty-one edges join a kept and a dropped side.
  Eighteen of them involve face 8, a piece of `A1`, read `kept|dropped` (outside B). Its
  neighbours across A's own edges 12 and 14 are inside B, and so are the B pieces at its meeting
  edges. The other three are edges of B's own seam between `B5` and `B8`, where a piece of `B8`
  (face 110) is read inside A beside a piece of `B5` read outside it.
- Face 8 runs on through meeting edges 236 (from `P = (4.2085, 0, 2.0702)` to
  `(3.9776, 0.7733, 1.9988)`), 238 and 239 (ending at `(2.4669, 0.9743, 1.7931)`). Those meetings
  were traced and laid, but the cut of `A1` does not close round them. At both far ends the
  meeting stops on the boundary of B's face `B8`, and `A1 × B8` is traced from no seeds.
- `B5`–`B8` are four faces of `Sheet(-1.0)`, classed as one surface (they share one meeting with
  `A1`). On this branch each carries its own segment of the sheet's net (8ad646d), so a march
  for one face ends at that face's net's edge ("a step left the surfaces": the `B7` march stops at
  `(2.4668, 0.9743, 1.7931)`, on the `B7`/`B8` seam). `B8`'s own seeds there lie on a known curve
  and are dropped, so the stretch on `B8` is never traced. That is a fifth gap, **T5: a meeting
  shared by the faces of one surface class is traced over each face's own net**. It arises
  only where faces of one class carry different nets. On `main` those faces carry one net, and the
  leak there is the subject of the next paragraph.
- `P` is where four branches of the meeting cross (A1 and B5 both regular there, area factors 0.70
  and 0.85, at `v = 1/2`, `y = 0`). Probably the sheets touch there, since both sweeps pass
  through the same pose at `t = 0`. The four branches are traced and end at `P`.

On `main` (6836e76) with the same report the union refuses in 2383 s under full debug, with the
same message, "a shell of the arrangement is kept on one side of a bspline (`Sheet(1.0)`)". It is
the issue's refusal, and it is not the experiment branch's leak:

- The complex has 125 faces and 3 shells. The leaking shell holds both sides of 86 faces, and 54
  edges join a kept and a dropped side.
- **Two edges are used three times**, 247 and 248. They are one stretch of the meeting of `A1`
  with B's `Sheet(-1.0)`, from `(2.8997, 0.9035, 1.7515)` to `Q = (3.1627, 0.9089, 1.7440)`, laid
  twice: their middles are 2.1e-6 apart. `A1`'s piece (face 8) takes one copy, and `B8`'s piece
  (face 102) takes both. An edge used an odd number of times joins two cells, which is the leak.
  This is **T1**, a stretch of a meeting laid a second time; it is the issue's "a meeting traced a
  second time on the net's other layer, the two copies crossing a face's rim 3.5e-4 apart".
- At `Q` the sheet of B is nearly singular: its net's area factor is 0.14 there and falls to 6.7e-3
  a thousandth away (`tube_profile pair` `FEET`). On the experiment branch the march "met itself" at
  `(3.1571, 0.9127, 1.7456)` and "ended at a sheet's fold" at `(3.2113, 0.8999, 1.7404)`, both
  beside `Q`. So T4 (a march near a singular point of the net) is where the second copy comes from.

So the crossed Booleans need T1 and T4 on `main`. T3 is the fold rule's subject (the issue's
creeping march) and T2 is met wherever an edge crosses a face near its cuspidal edge. T5 is needed
only if the sweep's faces are given nets of their own.

## Step 4

**T1 is built** (`boolean::iso_parameter`, the rule in `contract/brep.md`). A closed-form
meeting that is an iso line of a face's net, with the net's own parameter, is cut to the face's
parameter box before its spans are read. The folded-back stretch is then never on the curve to be
misplaced. The gate passes. With the box widened to cross the fold, with two walls crossing the rim,
and with the face trimmed at `u ≥ 11/20`, all three Booleans build to their closed forms as well.
A first attempt placed each span by its parameters continued along it from an end on the face's
boundary. That fixed the gate alone: on an exactly doubled patch both layers also pass through the
face's far edge, and a reading continued from there stays on the face's layer. It was dropped.

**Fuzzed** (`examples/patch_fuzz.rs`). The fuzzer draws a random exactly doubled patch
`(s², v, h + a s⁴ + b v)`, a random trim `[u0, 1] × [v0, v1]`, and a random box whose walls fall
anywhere across the face, its rim and the fold. It runs all three Booleans, each against the closed
form and `check`. On seeds 1, 7 and 11 (the kernel's `rng::Rng`), `main` fails 15 of 120, 57 of
600 and 54 of 600 Booleans; with T1, none of the 1,320.

**Its time.** The slowest whole-patch Boolean took 2.2 s, against 0.03 s on the net cut down. Of
that, 99% went to a line's crossings with the face isolated over the whole net (the solid's rays,
then the arrangement's edges against faces), where the fold, `S_u` vanishing, leaves the interval
search nothing to contract. Both now search the face's own stretch of the net
(`Located::read_surface`, the net cut down to the face's parameter box). The slowest is now 0.02 s
and the mean 0.02 s, the same as on the net cut down (on `main`, the same seeds' slowest is 3.1 s).

**T2 is built.** Three readings of a sheet's face still went through its whole net, and each had
to be moved onto the face's own stretch (an ablation showed that every one of the three was
needed):

- The march (`Arrangement::traced_on`) now runs on the surface class's net cut to the union of its
  faces' grown boxes. Before, the wall `x = 2/5` was traced along the layer beneath the face.
- An edge's root is settled with `on_surface` on `read_surface`. Before, a root found on the face
  was polished onto the folded layer 5e-4 below.
- `kept`'s on-patch test reads the cut net. Before, the middle of the face's own span inverted onto
  the folded layer and read off the patch, so no edge was laid on the face at all.

Cutting the known meetings an edge is read against was tried and proved unnecessary, so it was
dropped. On the T2 family (layer gaps from 1e-4 to 3e-2, three hole placements) all 18 builds
reach their closed forms. `patch_fuzz` draws half its patches with the layer: on three seeds
`main` fails 669 of 1,800 Booleans, T1 alone 420, and with T2 none, the slowest in 0.12 s.

**T3 and T4 are built.** The cases that still failed had a face trimmed within a sixteenth of the
cusp or the pinch. Three things were wrong there:

- **The grown box reached past the singularity.** `Located::grown_parameters`
  (`query::regular_growth`) now grows each side only while the net stays as regular as at the
  face's own edge. The test is that `S_u × S_v` keeps half its least along the edge, and that the
  normal turns less than 60° from the edge's. The strip is sampled across the other parameter's
  grown range, so a singular point beside a face's corner is seen. The normal test is what finds
  the pinch: there `S_u × S_v` shrinks as the cube of the distance, and no grid sample comes near
  enough to see it.
- **The plane's cut was not cut to the face.** It is the net's row carried, so its parameter is
  the net's `u` while `v` runs along it, and `iso_parameter` only knew constant-`v` iso lines. Now
  any curve whose parameter is one of the net's own is cut to the face's box, the other parameter
  read at each sample by Gauss–Newton along it. Uncut, the curve ran through the cusp, and
  inverting the face's rim point onto it stalled at the cusp.
- **A spline's foot stopped at the curve's end.** Beside the fold the curve barely moves with its
  parameter and bends hard (`x = s²`), so Newton's curvature `|C'|² + (C − p)·C''` went negative
  and `BSpline::refined` gave up at the end. It now takes Gauss–Newton's step there. This one
  showed up when the box stopped short of the fold: before, the curve's end lay past the fold.

The two `…_trimmed_nearer_…` gates pass, with every patch gate.

**Fuzzed** (`patch_fuzz … cusp`, `… pinch`). Each case draws a random patch, a trim as near as
`0.505` to the cusp or the pinch (half the pinch's within 0.03 each way), and a plane through a
point within 0.003 of the singularity (half the pinch's within 5e-4, and nearly upright). The
plane is kept on each side, and each side is held to the same block on the net cut down, the two
sides to `|A|`. On seeds 1, 7 and 11:

| | cusp cuts failing | pinch cuts failing | slowest |
|---|---|---|---|
| `main` | 213 of 900 | 555 of 900 | 5.2 s |
| T2's commit | 41 of 900 | 7 of 900 | 11.4 s |
| now | 0 of 900 | 0 of 900 | 0.06 s |

The fold fuzzer (T1, T2) on the same seeds: none of 1,800, the slowest 0.09 s.

**The deeper change** (from the review of T1). Both rules read a face through the part of its net
it uses, each at the one place it is needed. The general form: carry each meeting's parameters on
the sheets it was built from out of `ssi` (the plane cut knows them exactly), and place spans by
them, not by recognising an iso line afterwards. That also covers a plane tilted across the patch,
whose cut is no iso line. Then give each surface class of the arrangement one read surface, its
net cut to the union of its faces' boxes, used by every reading. That is the union and not each
face's box, or a march stops at one face's edge and the next face of its class is never traced
(T5). A class whose faces lie either side of a fold still needs the carried parameters. This is
the road for traced meetings too (T4: "a march that carries its feet").

What T1 does not reach: a **traced** meeting on a folded net (the crossed tori's, laid twice
2.1e-6 apart on `main`) carries no parameters of its own, so it needs T4's reading of where a
march ends, or a march that carries its feet, before T1's rule can apply to it.

### The crossed Booleans

With T1–T4 in, the crossed union still refused, in 1008 s. Each fix below was found by a probe
(`tube_profile … pair`: `PAIR_FACES` traces one face pair alone, `PLACE_ONLY`, `DIST_AT`,
`PCURVES`, `PLACES`; under `SOLVENT_BREP_DEBUG` a refusal lists each join of a kept and a dropped
side and reads every face it names against the other operand on a grid). None of them is a
reading of a face on the wrong layer, the subject of T1–T4. Each is a march that went on where
it could not, or a point found twice:

| | what was wrong | rule |
|---|---|---|
| 1 | a march beside the sheets' touch landed each 1e-4 step 1e-8 on, 100,000 steps each asking for a touch: 111 s | a march whose steps land back where they stood has ended |
| 2 | A5 × B9 ran round a swallowtail of A's sheet beside B's cap torus, a loop 2.5e-4 across, lapped until its step budget ran out: 780 s of the 1008 | a march run back onto its own path has gone round a loop |
| 3 | the sheets' touch at the shared pose settled twice, 8.4e-6 apart: four branches ended two at each, A1's cut did not close | one touch, one point |
| 4 | the second touch's fourth branch stopped 4.8e-3 short of it: A6's and B1's cuts did not close | a curve ended short of a touch the trace leaves branches from is carried on to it (asked as the march stopped, it broke `round_onto_round`'s boss, whose touch is the plane's edge's) |
| 5 | beside B's sheet's near-singular point the meeting with A1 crossed the fold, turned back along itself and was fitted 7e-4 off both sheets: check refused | a march across a sheet's fold has ended |
| 6 | B's rim stated a gap of 1.1e-5 read at 33 places, its pcurve 1.5e-5 off between two: the union's check refused | an edge's gap measured as a reader measures it |

A first try at (2), ending a march with cramped steps once it stood clear of a face, read the cause
wrong: the loop's steps were regular. It was replaced, and a second, walking pcurves that do not
hold, was dropped when (5) showed the edge itself off the sheet.

With them the three build, check and mesh, and agree, at the members' own tolerance:

| | faces | volume | its mesh's | time |
|---|---|---|---|---|
| union | 81 | 25.7842683051 | 25.7695282 | 47.7 s |
| difference | 71 | 11.9944211178 | 11.9877081 | 46.7 s |
| intersection | 56 | 4.6165786980 | 4.6144377 | 46.6 s |

`|A ∪ B| + |A ∩ B| − |A| − |B|` is 3.4e-9 and `|A − B| − |A| + |A ∩ B|` 5.0e-10. At the kernel's
own tolerance (`combined`'s floor 0, a billionth of the size, 8e-9) all three are refused: the
arrangement resolves the sheets five hundred times finer than they are fitted (r/50000), and a
dozen pieces are left spanning both sides. brep-bench's runner now asks them at the sweep's
tolerance (`Tumble::tolerance`), as `ring-pushed-in-at-a-lean` asks its cut at its sweep's.

**Their time.** Each Boolean took 80 s at first: the arrangement's edges against faces 8 s, face
pairs 57 s (sheets' grid seeds 29.5 s, traces 21 s). Grid seeds now settle each sign change by false
position and drop a jump at once (10.6 s); `touch_near`, asked a hundred times beside the folds and
mostly running all thirty-two steps to nothing, now takes sixteen. Each Boolean is now about 47 s
(edges against faces 6–7 s, face pairs 27 s: grid seeds 9.5 s, traces 13.5 s); with its members
(16 s and 62 s) a case runs some two minutes, past the bench's 60 s. The reverse member's own
62 s is the larger share left, and it is #942's, not this issue's.

**On the bench.** As brep-bench's runner builds and writes them (members, Boolean at the members'
tolerance, STEP), the five crossed cases run in 13 s (forward), 35 s (reverse), 87 s (union), 87 s
(difference) and 84 s (intersection), against CI's 120 s; the reverse member is now 35 s (its graded
nets fitted on every core) and the STEP writer cuts each sheet face's net to the face (the union's
file 126 MB, where it was 2.4 GB). The harness's measurer (`harness/mass`) could not read any file
of the tube road: following a loop's feet one from another across a sheet's fold it crossed to the
other layer. It now reads a loop's path on a spline surface from the file's own pcurves, and
measures the five to the kernel's volumes. The reverse member's sheets are fitted to some
900 × 850 poles about its pinch and a near-pinch inside a kept face at (θ, t) (7.816, 0.461), and
measuring a face across them takes 34–70 s four at once; the harness's measuring budget is now
CI's runner limit, 120 s. With those, all five are graded `built` by the bench's own harness. A net
refined only about the pinch and the near-pinch (cuts across a sheet's faces through their trims)
is #942's to make.

**The experiment branch's three rules** (per-face nets, and its fold and met-itself reports) are
dropped: none fixed a minimal case, and the per-face nets are what made T5. (5) and (2) above are
the rules its fold and met-itself reports pointed at, here as rules of the march.

## The cases

Each capability is a case in brep-bench's `nurbs/` with closed-form answers (mpmath; the volume of
T1 is exact), implemented for cad-kernel (refused or wrong there, as `main` is) and for OCCT:

| case | volume | area | OCCT |
|---|---|---|---|
| `nurbs/folded-patch-notched-at-its-rim` | 0.48658855 | 4.1267385566944163 | 0.48658855000000023, area 9.6e-8 high |
| `nurbs/patch-over-a-near-layer-pierced` (`s³/1000`) | 0.54336907553665484 | 5.070253321350833 | agrees to 4e-12 (harness/mass) |
| `nurbs/cuspidal-patch-sliced-through-its-cusp` | 0.51031272172170669 | 4.2949491490074952 | 2.5e-8 low, area 4e-8 low |
| `nurbs/pinched-patch-sliced-beside-its-pinch` | 0.5244906492156999 | 4.591931211856204 | agrees to 2e-10 |

The gates are in `rust/gcs-core/tests/patch_layers.rs`. Each case is there twice. The control has
the net cut to the face and passes today. The gate has the whole net, is `#[ignore]`d with the
reason it fails today, and checks every reading against the closed forms and inclusion–exclusion:
all three Booleans for T1 and T2. For T3 and T4 the gate keeps each side of the plane by
`build::kept` (the sides summing to |A|), as the bench's implementation does, because those two
failures turn on the cutting block (see T3). `cargo test -p gcs-core --test core patch_layers --
--include-ignored` shows the four controls pass and the four gates fail, each with the message
above. `rust/cad-kernel/examples/patch_probe.rs` is the probe the cases were found with: any of the
four patches, trimmed anywhere, cut by a box or a plane, on the whole net or its part.
