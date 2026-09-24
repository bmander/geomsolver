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
smooth, so the shading rounds the creases too. The wasm meshes on the main thread and the page
is unresponsive for several seconds while it does. Next for the preview: mesh off the main thread
and show the refinement as it goes, then features from the core.
