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
