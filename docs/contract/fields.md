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

**A drafted prism of any loop is read by its sections** (#607, #588; `solid/field/sections.rs`): a
convex loop of lines and whole circles is its profile's field moved, exactly (`draftable`); any
other loop of lines and arcs is read at a point as the signed distance to its section at the
point's depth — every side moved `|w| tan draft` into the material along its own normal, each corner
where its moved sides meet (`build::drafted`'s section), a line–line corner moving affinely —
divided by `√(1 + (tan·mitre)²)`, the most the field can change per unit length, so it stays
one-Lipschitz with the drafted sides its zero; its interval bound is its value at a box's middle
widened by the box's half-diagonal.  A section that loses a side within the prism's depths is
refused when read.  `planar_subtraction::the_field_reads_a_drafted_plus`.

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

**A volume, for analysis** (#510 P6): `refine::volume` and `Progressive::with_cells` fill the
surface's inside after it is done (`Stage::Filling`); the rules are
[analysis.md](analysis.md)'s. The surface stage is unchanged by them, except that a filling
refinement takes its last build's surface, never an earlier one.

**A distance's readings clear a crossing's bracket** (#510; `Readings::Distance`,
`Domain::slack`): a domain whose readings are `Agreeing` and its value a distance from the
boundary (one-Lipschitz, reading over the distance by no more than its `slack` there) says so,
and Newton's crossing closes the bracket at every reading by the reading's size less the slack —
no point nearer it than that is across the boundary — and where Newton's step would leave the
bracket reads next at its near end, as far as the boundary is known not to be: on a body's
boundary two or three readings a crossing where the bracket alone took a dozen, an orthocentre's
nearest wall often another than the one its dual edge crosses. Any other domain is read as
before (`a_distances_readings_clear_a_crossings_bracket`).

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
`MaterialField::support_bounds` is a conservative box or explicit unknown. **A prism's is its
profile's own** ([0.194]): `ExtrudedField::support_bounds` takes the disk about its plane's origin
in common with the profile's box carried by its frame (`PlanarField::box_bound`: a convex loop's
`hull`, a loop's edges' boxes, recorded where `profile::convex` builds it beside its bounding
disk and where `from_loop` builds any other loop's), wherever every section is within the profile (no draft, or one leaning in) — a plug drawn
in a placed instance's plane 140mm from its origin was sized to that disk, and the mesher's facets
with it, and found no boundary (`tests/grown.rs::a_prism_far_from_its_planes_origin_has_its_own_box`).
`MaterialEvaluator::boundary` needs that support, complete cells, strict corner signs and two-sided
distance evidence for `FieldBoundary`. `BoundaryOptions::domain` takes a caller's box instead, but
it **is** an arbitrary crop: its signs prove nothing beyond it, and closure is the caller's
judgement. Never substitute a crop, residual magnitude or same-sign corners for coverage.
Ambiguous vertices, missing mesh evidence, work limits and failed topology refuse extraction (see
`docs/field-boundary-extraction.md`). Keep the extractor as a measured baseline; evaluate
`docs/implicit-meshing-methods.md` before extending uniform refinement.
