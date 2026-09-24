# Meshing the material field by Delaunay refinement

Started 2026-09-23. The generating-sweep export built a sheet from traced contacts, fitted
it as a spline, split the blank with a kernel Boolean, classified the cells and meshed the
result ([robustness plan](generating-sweeps-robustness.md)). Every representation change in
that chain has been a source of failures: branch ends, folds, crease pleats, near-tangent
splits. The material field (`solid::MaterialField`) has been the reliable part all along; it
is what the final gate trusts. This experiment meshes the field directly.

## The method

Delaunay refinement for piecewise-smooth complexes: Cheng, Dey and Ramos
([DCG 2010](https://link.springer.com/article/10.1007/s00454-008-9109-3)), with Dey and
Levine's protecting balls ([DelPSC](https://doi.org/10.3390/a2041327)). Sharp curves are
protected by weighted points (balls) sized to the local feature; the surface is then refined
by the restricted Delaunay triangulation, whose facets are dual to Voronoi edges that cross
the surface. The domain need only answer which side of the boundary a point lies on. CGAL's
Mesh_3 is the maintained implementation, and the experiment uses it.

## The experiment

`solventc --stl-backend cgal`, behind the `cgal` feature of `gcs-cli`:

- `backend/delpsc.cpp`: Mesh_3 over a labelling of the domain (1 for material, 0 outside),
  1D features as polylines, the boundary facets written as a binary STL. Built against
  Homebrew's CGAL 6.2.1 (`brew install cgal`). **Mesh_3 is GPL-3.0**, so this never ships.
- `src/cad/delpsc.rs`: the field oracle, the features, and the checks. The result goes
  through the STL shell check and the field-agreement gate like every other backend.
- `SweptField::side` / `MaterialField::side` (core): a number with the field's sign, in plain
  floating point. A sweep's side is a Lipschitz branch-and-bound over the roll, using the
  motion's inverse-point speed bound, with the first 2¹⁰ dyadic poses cached. It stops as soon
  as the side is decided (at once far from the boundary, logarithmically near it) or the bound
  is within 10⁻¹⁰ of zero. `admission::a_material_side_agrees_with_its_enclosure` holds it to
  the certified enclosure.
- The harness takes `SOLVENT_HARNESS_BACKEND=cgal`, writing `build/harness/<test>-cgal.md`.

### Features, from the field alone

Blank minus sweeps has two kinds of sharp curve:
- the blank's own sharp edges where material survives;
- the curves where the swept boundary meets a blank face.

The swept boundary is smooth in the generating class (a crease sweeps into a tangent-continuous
fan), and E4 excludes self-crossing. A point on a blank face is material exactly when the field
read just inside the face (by 10⁻⁴ of the size) is. From that:
- **Edges:** along each sharp edge of the static blank (the kernel's boundary rows, read along
  the inward bisector), the surviving stretches are features. Their ends are bisected as corners.
- **Faces:** over each blank face's parameter grid, the zero contour of that reading, by
  marching squares with bisected crossings.
- **Joins:** a contour ends a cell short of a trim or a seam. There it is joined to the corner
  it runs to, or across the seam to itself.

### What it took, in the order found

1. **The domain function is a labelling.** Handed a signed value, Mesh_3 reads it as a subdomain
   index; truncated, the labels made even a plain cylinder refine forever.
2. **Features are required.** Without them Mesh_3 chases the facet-distance criterion into every
   sharp edge and never terminates.
3. **Feature hygiene.**
   - Grid points on a face's trim are read on the neighbouring face, and traced a spurious copy of
     the rim; they are excluded.
   - Excluding them leaves a gap at seams, which is closed.
   - No two consecutive feature points may coincide.
   - `edge_min_size` bounds the balls.
4. **The certified query is the wrong oracle.** Interval refinement until the enclosure clears a
   point's distance costs about 3 ms per query near the surface. The floating-point side costs
   about 7–16 µs on fixtures and about 180 µs on the gear.

### Results

Every mesh is a closed shell, with no field disagreement.

| Case | Native construction | Mesh_3 |
| --- | --- | --- |
| Torus through a post | exports | 6 s, 434 triangles, 0 of 844 truth probes disagree |
| Sphere through a post | exports | 13 s, 0 of 536 disagree |
| Ring lens (sharp rim) | exports | 8 s, 0 of 1012 disagree |
| Spherical lens (class B) | refused: crease slivers | 23 s, 0 of 1028 disagree |
| Torus through a tall post | refused: extension re-enters the blank | 5 s |
| Lens about two axes | refused: a face meets a section twice | 18 s |
| Bevel gear space 0/0/35 | exports in 17 s | 10 min, 25 124 triangles, volume 25 789.66 against the kernel's exact 25 789.54 mm³, 0 of 2993 triangles disagree |

The gear's time is almost all field queries (3.4 million at about 180 µs); Mesh_3's own work is
seconds. Its STL was refused by the shell check for one float32 needle triangle.

## Plan: our own implementation, fast enough to edit with

**Goal:** the same method in the core, in Rust with no dependencies, on one WASM core, fast enough
to follow an edit.

**Working targets:**
- a fixture's preview mesh in about 100 ms;
- a gear member's preview in about a second;
- an export at full accuracy in seconds.

Mesh_3 stays in the harness as the reference while the port is built, and the harness judges
both on the same numbers. The order follows where the time goes, which is the oracle and not the
triangulation.

- **F1: the oracle, measured inside the working pipeline.**
  - Count queries by phase (features, protecting balls, refinement).
  - A sizing field: fine only near the cut and the features, coarse on the untouched blank.
  - A warm-started side that carries the minimizing roll time between the neighbouring queries
    bisection asks.
  - For the generating class: the tool's path in its own frame is trigonometric in the roll.
  - Exit: a gear member under a minute through Mesh_3, with its query count and time per query
    recorded.
- **F2: features in the core, without OCCT.**
  - The static blank's faces and sharp edges from the core's own analytic description
    (`solid::tool_faces`: revolved, planar and extruded faces and their charted edges).
  - Contours refined adaptively on each face, not a fixed grid.
  - Exit: the same feature curves as today's OCCT-based ones, on every fixture and a gear member.
- **F3: robust predicates and the regular Delaunay triangulation.**
  - Adaptive exact `orient3d` and power tests (Shewchuk's construction).
  - Incremental insertion of weighted points with point location.
  - Unit tests against brute force and degenerate inputs (cospherical, coplanar, duplicate
    points).
  - Exit: random and structured point sets triangulate identically to a brute-force check.
- **F4: refinement with protected features.**
  - Protecting balls on the feature curves.
  - Restricted Delaunay facets by dual-edge bisection through the oracle.
  - Refinement by size, angle and distance, keeping the surface manifold.
  - Exit: every fixture and a gear member match Mesh_3's closure and field agreement, and the
    shell check passes (no needles).
- **F5: WASM and editing.**
  - The mesher behind the ABI.
  - Coarse-to-fine refinement for previews.
  - After an edit, re-mesh only where the field changed.
  - Exit: the targets above, measured in the browser.

## F1 results (2026-09-23): the oracle

**Measurement.** `generating_class::side_query_cost` (ignored; `SOLVENT_AGREE_STL` and the design
variables) times `MaterialField::side`. The points are each triangle's centroid of an exported
member and the points 1 µm and 0.1 mm either side of it, the mix a mesher's bisection asks. It
checks every side against the certified enclosure wherever that decides:
`SIDE_EVALUATIONS` counts sweep evaluations. On the bevel gear space:

| Step | µs a query | Evaluations a query |
| --- | ---: | ---: |
| First-order branch-and-bound over the roll | 420 | 229 |
| Golden section per short stretch (tried, reverted) | 880–1490 | 444–768 |
| Basins: split to 1/64 of the roll, then golden section per contiguous run | 62 | 42 |
| Position-only poses (`Family::pose_at`), 28 golden steps | 18 | 32 |

In every row but the reverted one, 0 of 10 292 decided points disagree.

**What the measurement showed.** The speed bound is about 156 mm/rad. Near a rolling contact the
path runs along the tool, so the source rises slowly away from its minimum. That shallow basin
kept 20–30 stretches alive at every level from 8 to 17. The fix treats each contiguous run as one
basin, which is a reading, not a bound: two dips inside one run, between readings, could be
missed. The certified enclosure check stays the test of it.

**The whole gear space through Mesh_3:** from 10 min to **55 s**. That is 3.35 million queries at
12 µs (41 s), and 3.9 s of feature tracing, almost all of it OCCT's face classification
(the field takes 0.2 s). F1's exit is met.

**Left:**
- the query count, from meshing the whole blank at 5 µm facet distance;
- the float32 needle triangle, for F4;
- OCCT in the features, which F2 removes.

## F3 results (2026-09-23): predicates and the regular triangulation

**`delaunay`** (core, no dependencies):
- **`expansion`:** Shewchuk's exact expansions (two-sum, two-product by splitting, grow, scale,
  with zero elimination).
- **`predicates`:** `orient` and the weighted `power` test.
  - Each evaluates in floating point first, against an error bound proportional to its
    permanent: Shewchuk's orientation bound, and his insphere bound doubled for the weight
    difference in each lifted coordinate.
  - A value inside the bound is re-evaluated exactly.
  - `EXACT_CALLS` counts the fallbacks.
- **`regular`:** the regular triangulation, built by incremental Bowyer–Watson inside an
  enclosing tetrahedron whose insphere is sixteen times the points' radius.
  - A point is located by a remembering stochastic walk.
  - A point in conflict with no tetrahedron is hidden, and a vertex engulfed by a conflict region
    becomes hidden.
  - Where the region's boundary would make a flat tetrahedron with the point, a tie, the region
    grows past that face.
  - `check` is the definition by brute force: orientation, neighbour reciprocity, no unhidden
    vertex in conflict with any tetrahedron, every unhidden vertex used, and every hidden point
    conflicting with nothing.
- **`spatial_order`:** biased randomized insertion rounds, each in Hilbert order.

**Tests** (`tests/delaunay.rs`):
- The expansions reproduce values doubles lose.
- The filtered predicates agree with the exact ones:
  - on 100 000 nearly coplanar and 100 000 nearly cospherical, weighted cases;
  - on the one-ulp perturbations;
  - on exact zeros for integer coplanar and cospherical input.
- Triangulations pass `check` for:
  - 1 500 random points;
  - a 7³ lattice (degenerate everywhere);
  - 400 cospherical points, 400 coplanar points, and an integer lattice on a sphere of radius 5;
  - duplicates (the lighter hidden) and random weights (23 of 1 200 hidden);
  - a weighted lattice.
- Half the cases run in spatial order and half in input order.

**Speed**, release, 50 000 random points, no exact fallbacks:
- 39 µs a point in random order;
- **14 µs** a point in spatial order: about 7 walk steps and 20 conflicting tetrahedra per
  insertion.

What remains is the conflict search's power tests and creating the new tetrahedra, both with
cache misses. CGAL is several times faster, and closing that gap is F5's. For now it is small
against the oracle's queries.

### F3 refinement (2026-09-23): a foundation for F4

**The API a refinement reads:**
- `insert_near(p, w, hint)` walks from a tetrahedron the caller knows is near.
- `created` and `removed` say what the last insertion changed, so a refinement's queues can
  follow it.
- `conflicts` gives the conflict region a point would take, without inserting it; a test holds it
  equal to what the insertion then removes.
- `incident(v)` gives a live tetrahedron with `v` as a vertex, kept current as tetrahedra are made.
- `mirror(t, i)` gives the facet seen from its other side.
- `orthosphere(t)` gives a tetrahedron's orthocentre and orthoradius², in floating point for
  constructing the dual Voronoi edges; each vertex's power against it is its own weight.

**Speed.**
- **The orientation check is skipped on strict faces.** A face whose far tetrahedron is strictly
  outside the conflict region needs no orientation test before the new tetrahedron is made on it.
  Both orthospheres are orthogonal to the face's three weighted vertices, so their radical plane
  is the face's plane, and the point lies strictly on the near side. Only ties are checked; the
  stamps record in, strictly out, or tied.
- **The power test shares six 2×2 minors** across its four cofactors (Shewchuk's insphere layout),
  with the permanent in the same shape for its bound.
- **Expansions are stored inline** up to 24 components, and summed by Shewchuk's linear
  FAST-EXPANSION-SUM. The exact power test on cospherical input went from 12 µs to 7 µs; COMPRESS
  was tried and did not help, the values being short already.
- **Liveness is folded into the tetrahedron** (a freed one has `NONE` as its first vertex), and
  faces are paired through a generation-stamped open-addressed table.

**Measured by `bench delaunay [case]`**, which takes `DELAUNAY_N`, under user CPU time: the machine
carried a load of 12–56 from system services throughout, so wall times were unusable.
- The degenerate lattice runs **twice as fast**.
- Uniform insertion is 5–10% faster.
- It is **size-independent** (about 12 µs a point under that load, from 5 000 to 400 000 points),
  so it is not memory-bound: the 17 orientation and 44 power tests per insertion are most of it.

Each tetrahedron is tested about 1.6 times in its life, so caching orthospheres would buy little.
A local static filter would trade the permanent for a looser bound and more fallbacks.

## F4 results (2026-09-23): refinement in the core

`delaunay::refine::mesh` (`--stl-backend refine`) meshes the material field with protected
features on our own regular triangulation. Measured through `tests/generating_harness.rs`
(`SOLVENT_HARNESS_BACKEND=refine`, test profile, not release):

- **Every admitted fixture exports** with no disagreement against its field or its independent
  truth, in 1–3 s: the skew sphere, the crease-slivers lens, the ring lens, both tori and the lens
  about two axes (the last two are refusals on the native path).
- **Four of five gear tooth spaces export** with no disagreement at 2000 probes: bevel pinion
  (20 s, 10 090 triangles), bevel gear (24 s, 12 406), hypoid 25 gear (22 s, 14 180) and the
  15° pinion (31 s). The bevel gear space's volume is 2.4 mm³ (0.009%) above the kernel's
  exact 25 789.54 mm³ at 10 626 triangles; Mesh_3's was 0.12 mm³ at 25 124.
- **The hypoid 25 pinion space refuses**: four facets share an edge at a feature curve near
  (60.27, 1.75, −0.08), and shrinking the ball there to 4 µm does not clear it. The facets
  standing off the surface did not converge either (51, 15, 15, 24, 29, 32 over rebuilds). The
  likely cause, not yet confirmed, is an **unprotected crease**: the features are the blank's
  sharp edges and the cut's traces on the blank's faces, not creases inside the cut surface.

**What the implementation needed** (each measured before it was fixed):
- **Protection is a sizing function, not a spacing per curve.** A curve's stations are a fixed
  uniform grid halved where a local target asks (`Sizing`: base spacing and `(s, h)` constraints
  graded by 0.25 per unit length), balanced 2:1 with no gap longer than both neighbours, and a
  ball's radius is 0.7 × its shorter gap: consecutive balls overlap and next-but-one do not.
  Halving a whole curve re-refined everything and moved every ball.
- **Near a corner the exemption is a distance**, within `edge_size` along the curves, not a count
  of samples: a finer rebuild otherwise lost the exemption. Two curves passing close without a
  shared corner are sized to their separation.
- **A blocked repair shrinks the balls and rebuilds** (the triangulation removes no vertex): the
  curves are refined at the blocking balls, and every kept point is re-inserted without judging,
  with the field's answers remembered by the point's bits. `orthosphere` works from its corners
  in a fixed order so a rebuilt tetrahedron has the same orthocentre bits. A rebuild went from
  about 130 000 new queries to about 3 000.
- **A surface centre is bisected along the facet's exact dual line**, the points of equal power
  to its three weighted vertices, between the orthocentres' projections. A nearly flat
  tetrahedron's orthocentre is far off and inaccurate; bisected toward it, the crossing conflicted
  with neither tetrahedron, the facet survived its own refinement and was refined again at the
  same place, leaving two vertices 4e-15 apart. A point conflicting with neither is now refused.
- **Facets a ball holds may stand off the surface** where no refinement point can reach them.
  Those more than ten times `facet_distance` off (by the field's side at both ends of that stretch
  of the facet's normal) shrink their balls as a blocked repair does.

**Heuristic still, and to be derived or replaced:** the `edge_size/8` floor near corners, the
`edge_size/1024` floor for shrunk balls, the ten-times stand-off threshold and the rebuild cap.
The feature curves (OCCT edges and 64-cell contours, joined and snapped) produced most of the
failures fixed here: a straight chord across a contour gap, zero-area loops, curves passing
within a millimetre of each other.

**Revised plan (2026-09-23).** The target is a fast rough preview that refines live, then a
longer render for export; exact whole-gear meshing at interactive rates is not required.
Refinement already takes the worst facet first, so the preview is the same loop shown at
intervals under looser criteria. Next:
1. **A complete, analytic feature graph in the core** (F2, extended to creases inside the cut:
   a sharp tool rim's sweep, the seams between generating faces), with a stage contract that
   checks it — every sharp crease present, every curve end a corner, no two curves nearly touching.
   Then the hypoid pinion again.
2. **A whole member through `refine`**, to measure the render.
3. **Preview speed:** one tooth-space sector meshed and repeated by the indexing motion, and
   cheaper field queries.

## In the app (2026-09-23)

A solid with a continuous sweep among its operands now evaluates by field refinement in the core
(`EvaluatedSolid::from_field`), so the glass box, views and the legacy mesh export draw it:
`?example=swept_torus.sv` shows the torus-through-a-post fixture in the browser. It uses
preview criteria (facet size 1/40 and surface distance 1/2000 of the support's diagonal) and
protects no features, so sharp edges are rounded to the facet size, and every facet is marked
smooth, so the shading rounds the creases too.

**Off the main thread, refining as it goes.** `delaunay::refine::Progressive` runs the refinement
in steps (`step(budget)`, `snapshot()`), and `mesh` is a loop over it. `solid::FieldMesher` holds
one solid's field and run. The page's sketch is told never to mesh a field (`defer_fields`: a
swept solid with no supplied surface is refused), and `app/field-preview.ts` hands a worker
(`app/mesh-worker.ts`, its own copy of the core) the document's text, modules and parameter
values; the worker refines each swept object 40 facets at a time, letting messages in between
steps so an edit ends the job, and posts the surface every 120 ms and at the end. Each arrives
through `supply_field`, keyed by the drawing's `solid::reads`, and the page redraws. A
provisional surface exports nothing. Measured in Chrome on `swept_torus.sv`: the first surface
(48 triangles) 0.69 s after navigation, then one every ~130 ms, the final closed 1912 triangles
at 2.77 s; the page stays responsive throughout (`performance.mark('field-surface')` per
surface). Next: features from the core, so edges come out sharp.

## Swept examples, and a regression the F4 record missed (2026-09-24)

`swept_spring.sv` (a ball screwed along a helix, two turns) and `swept_tumble.sv` (a torus
tumbled about a tilted axis) are parametric swept bodies for the app. Measured in Node's wasm: the
spring's first surface at 0.34 s and its 12 854 triangles at 12.4 s (volume 1917.1 against the
exact 1943.8 at preview facets), the tumble's at 0.19 s and 2.8 s.

**Coils need a reduction for the motion's angle.** Two turns are 12.6 rad, past interval `sin_cos`'s
[-8,8], and the support bounds refused. `Interval::sin_cos_periodic` reduces by a whole number of
turns with 2π enclosed between the doubles either side of TAU (and a period's width reads
[-1,1]); the motion bounds use it. Five tests that pinned the refusal at 9 rad as a contract now
check the enclosure, and three that used it only to build a failing operand use an overflowing
angle.

**Seeding by a lattice when the rays find no surface.** A coil's wire passes between rays from
the bounding ball's centre. After the rays the facets are judged; if none is restricted, a lattice
of `LATTICE` (28) a side seeds every neighbour pair on opposite sides, one seed a cell twice the
facet size, clear of the protecting balls by a facet. Not always: scattered over a cut, those seeds
land by creases no ball protects and broke three fixtures. `a_thin_ring_the_rays_miss_is_found_by_the_lattice`
holds it.

**Correction.** The F4 record above says every admitted fixture exports. That was measured before
the gear-space changes (the stand-off shrink among them) and not re-run: at `bbca0a4` the skew
sphere, both lenses and the lens about two axes refused, not manifold after twelve rebuilds. The
stand-off check shrank balls for 85 facets standing 0.05 mm off a surface whose facets are about
that size, and the shrinking opened it. The stand-off shrink is now best effort: the last closed
surface is kept, and a rebuild that opens it returns that surface instead. Every fixture exports
again in 1–4 s with no disagreement, and the gear controls are as before (the hypoid 25 pinion now
meshes, one probe short of the agreement gate).

**Still refused: sharp creases with no features.** A ball-end groove cut into a block
(`swept_groove.sv`, not committed) is not a manifold and has no ball to shrink: in the app the core
protects no feature curves, and the block's edges and the groove's crease with the top face need
them. The ball swept alone meshes (volume 2094.9 against 2110.7). Features from the core are next.

## Readings: value, gradient and active operand (2026-09-24)

`MaterialField::reading(p)` gives a `Reading`: the field's value, its gradient, the leaf deciding it
(numbered depth-first, a sweep's source leaves among them), the roll time a sweep decides it at,
and `ambiguous` where another operand or another contact time reads within a tie — a crease. A
static leaf's gradient is by central differences; a sweep's value is its minimum over the roll
(`SweptField::minimum_relative`, `side`'s bounded search carried on until no stretch can read
lower by more than the accuracy, or a thousandth of the value where that is coarser) and its
gradient the tool's own at that roll time, turned into the world by the pose (the envelope
theorem). Fixed poses turn gradients by `MotionBounds::gradient_mid`. Checked against the swept
sphere's closed form: value to 1e-8 asked exactly, gradient to 1e-5, the contact time, and the
point opposite the gap read ambiguous.

The refinement uses it where a caller supplies one (`Progressive::with_reading`): a crossing is
found by Newton on the value along the segment, the bracket kept by sign, a bisection step where
Newton would leave the bracket or not halve the value, ends outside the bounding ball cut back to
it. Source evaluations (deterministic) and time, bisection against Newton:

| | source evaluations | time |
| --- | ---: | ---: |
| torus through a post (app) | 2.86 M → 2.36 M | 1.46 → 1.19 s |
| tumble (app) | 6.05 M → 4.91 M | 2.46 → 1.93 s |
| spring (app) | 33.8 M → 20.1 M | 9.66 → 6.52 s |
| bevel gear space (CLI) | — | 15 s → 49 s in the field |

So the app's field mesher reads and the CLI bisects by default (`SOLVENT_REFINE_NEWTON=1` reads).
On the gear a reading costs about 0.45 ms against 12 µs for an average sign query: a sign far from
the boundary is settled by the first bound in two tool evaluations, while a reading finds the whole
minimum, and Newton reads both ends of the segment, usually far away. Making readings pay there
is the next step: along one crossing the points are close, so the contact time moves little, and a
local search from the last one needs a few evaluations where the global search needs many — with
the global bound still consulted before a local minimum is trusted near zero. The same readings
are what feature curves can be traced from: a crease is where the deciding leaf changes, or where
a sweep's contact time is ambiguous.

## Warm readings (2026-09-24)

A reading within one crossing now continues each sweep's contact from the reading before it
(`MaterialField::reading_warm`, hints by sweep leaf; `SweptField::minimum_hinted`). A warm-started
*global* minimum did not pay: its search is the proof that no other contact time dips lower, and
next to the contact the first-order bound (the tool's speed times the stretch) cannot rule
neighbouring stretches out without splitting them finely, however good the starting reading —
counted on gear points near the boundary, a sign query costs 6.7 tool evaluations, a cold
reading 55 and a warm-started global one 66. So a warm reading is a *local continuation*
(`ReadingOptions::local`): the basin-wide window about the last contact time is searched by golden
section, 13 evaluations, and the whole roll only when its minimum sits at the window's edge. What
the continuation cannot see — a deeper contact elsewhere — the crossing checks by `side`, the
query bisection trusts, a tolerance outside each end of the bracket Newton closed (inside it a
value's sign and `side`'s may both be right that close to the boundary); a bracket it disowns is
bisected from the start. Newton reads only the end it starts from: the other's side is known.

Tool evaluations (deterministic; wall time on this machine was not):

| | bisection | Newton, cold | Newton, local |
| --- | ---: | ---: | ---: |
| torus through a post | 2.86 M | 2.27 M | 1.78 M |
| tumble | 6.05 M | 4.07 M | 3.02 M |
| spring | 33.8 M | 17.8 M | 14.8 M |
| gear space (preview criteria) | 6.87 M | 11.3 M | 7.86 M |

The app's mesher reads locally within a crossing. Carrying contact times from one crossing to the
next brought the gear to parity (6.72 M) and cost the spring a third more (crossings along a coil
are far apart, so the hint is stale and the check falls back), so it was dropped. The CLI still
bisects by default: a gear's sign queries are cheap, and its first reading of a crossing is a
whole cold search.
