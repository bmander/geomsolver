# Fields and field meshing

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `solid/field/`,
`delaunay/`, `crease.rs`, or the mesh worker. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

**Continuous motion solids:** `removal := sweep(tool over t in (-30deg, 30deg), under: generating)`
is the union over the whole interval, not posed meshes; `at:` and intervals exclusive.
`MaterialField::read` promotes static DAGs when a sweep appears; nested sweeps refused; caches
read the whole motion graph. The native recipe rejects sweeps; `EvaluatedSolid` meshes the field
(`from_surface`, `solid::FieldMesher`). Terminal and tests mesh in place (`FieldMeshing::Now`).
**The page never does**: `field_meshing` is `Deferred`; `Sketch::field_jobs` keys swept objects
by a digest of `solid::reads`; `app/field-preview.ts` compares keys and runs `app/mesh-worker.ts`
(own core), posting surfaces (~120 ms, worst facet first) to `supply_field`; a provisional
surface exports only via the preview choice. Mesh fineness is view state
(`FieldMesher::with_fineness`, 0.25–4).
**A swept object of the generating class also gets its exact surface**: worker instances (up to
four) step `brep::export::Builder` (admitted, blank, sheets, sector, cut, pattern, display mesh at
`DISPLAY_SAG`/`DISPLAY_ANGULAR`), shown in the footer; the result replaces the preview via
`supply_field` with `ExactFaces` (`from_exact_surface`), kept across fineness; a refused build
keeps the field surface.

**Field meshing** ([docs/field-meshing.md](../field-meshing.md)): resumable Delaunay refinement
(`delaunay::refine::Progressive`, exact predicates, a `refine::Domain`), sharp edges protected by
weighted points. A first pass on `MaterialField::tight_support`, `solid::crease` traces creases,
the final pass protects them. A crease is where the deciding operand changes (leaf,
`PlanarField::carrier` piece, sweep contact time; `OperandId::whole`); a point needs the field at
zero and both leaves active; `End` says why one ends. Crease reads only `crease::CreaseSource`.
Rays seed beside the centre, never on it. `MaterialField::reading`: value, gradient, `OperandId`,
crease flag. Terms compile once (`material/plan.rs`); support, bounds, enclosure are `Fold`s. Every
point query is `MaterialField::query(p, &mut Query)` (`Want::Sign | Reading`, `Source::Exact |
Cached(Resolution) | Warm {hints,local}`; shorthands `side`, `reading`); `SweptField::query` takes
box, floor table, cache, search, in order. Readings are branch and bound; crossings by safeguarded
Newton, memoised. A `construction` solid is never an object. App example `spiral_bevel`
(`hypoid.sv`). `solventc --stl-backend refine` skips admission, gated by field agreement
(`SOLVENT_FEATURES=field`).

**An operand built exactly is read by its own boundary** (#419; `document.rs`'s `built`,
`field/built.rs`): a body some of whose terms read its faces (the stratum's late ones: a round),
with neither a sweep nor a region only the field builds among what it reads (`solid::has_sweep`),
that `Sketch::exact_solid` builds with every face a plane, cylinder, cone, sphere or torus, is one
leaf of another solid's field (`BuiltField`, `SpatialField`'s `Built`), none of its terms read:
the signed distance from its B-rep (`BodyFaces::of` the whole body, `signed_distance`) — a face's
foot on the face, signed by its side, else an edge's nearest point, outside a convex edge and
inside a concave one (a ball in the material cannot touch a convex edge), by `Located::solid_place`
where the edge is no corner or the point is at its end; edges no nearer than their faces' feet are
not asked — one-Lipschitz, so a box's enclosure is its centre's value within the half-diagonal and
the boundary's tolerance; its support the faces' boxes (`BodyFaces::reach`); its pieces its faces,
each carried by its surface's side, so a crease is found as a region's terms' are. Never the solid
read itself, nor a sweep's source the walk meets (a sweep reads it box by box over the roll); a
placed copy is transformed as any leaf; anything else, or a body the kernel refuses, is composed
of its terms as before. Remembered in `Caches::built` against `solid::reads`. Composed, a rounded
operand reads a sheet of nothing where the round's cut lies on the face it rounds, which a spilled
chamfer's flat crosses, ending one feature curve on another (`tests/composed_rounds.rs`).

**Meshing speed (2026-09-25):** sweeps read from adaptive distance fields (`solid/field/adf.rs`,
`SweptField::cached`, `Source::Cached`): an octree per sweep and `Resolution`, exact corners,
trilinear between, split while wider than a facet or its centre reads off by over a tenth of the
facet distance. Readings, never claims; creases and admission read
exactly. Sweep readings are one `RollSearch` (`swept.rs`) under a `Goal`; clones share
`SweepCaches`; keys and locks in `field/memo.rs` (immutable). Root finders are `crate::roots`. A
body's cuts are one union (`document.rs`, `Spread` lower bounds); `MaterialField::symmetries` /
`crease::creases_under` trace each crease once. Measure with instructions retired
(`/usr/bin/time -l`); Spotlight on a fresh target doubles wall-clock noise.

**Removed tracks (2026-09-25/26):** the certified swept boundary (`solid/swept_boundary`), the
Manifold arrangement (`--stl-backend manifold`, `solid::sweep_candidates`), CGAL Mesh_3, the Ju et
al. experiment, and the candidate construction (contact covers, meridian charts,
`envelope::edge_contact`, `solid::tool_faces`, `rust/examples/swept_boundary/`). Git history keeps
them; do not revive them without a new plan.
`solid::MaterialField` composes static and swept operands; its evaluator keeps every sweep query's
domain, witness, enclosure and status. Budgets apply per sweep query; exhausted operands are never
omitted; memo includes the input box; indexed clones share a capped pose cache; cached visits
repeat no evidence. An interval containing zero stays unresolved.
`MaterialField::support_bounds` is a conservative box or explicit unknown.
`MaterialEvaluator::boundary` needs that support, complete cells, strict corner signs and two-sided
distance evidence for `FieldBoundary`. `BoundaryOptions::domain` takes a caller's box instead, but
it **is** an arbitrary crop: its signs prove nothing beyond it, and closure is the caller's
judgement. Never substitute a crop, residual magnitude or same-sign corners for coverage.
Ambiguous vertices, missing mesh evidence, work limits and failed topology refuse extraction (see
`docs/field-boundary-extraction.md`). Keep the extractor as a measured baseline; evaluate
`docs/implicit-meshing-methods.md` before extending uniform refinement.
