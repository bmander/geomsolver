# Fields and field meshing

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `solid/field/`,
`delaunay/`, `crease.rs`, or the mesh worker. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

**Continuous motion solids:** `removal := solid({ p | some t in (-30deg, 30deg): p in tool
at(under: generating) t })` ([0.199], [0.205]; through 0.198 `sweep(tool over t in (…), under:
generating)`)
is the union over the whole interval, not posed meshes; `at:` and intervals exclusive.
`MaterialField::read` promotes static DAGs when a sweep appears; nested sweeps refused; caches
read the whole motion graph. The native recipe rejects sweeps; `EvaluatedSolid` meshes the field
(`from_surface`, `solid::FieldMesher`). Terminal and tests mesh in place (`FieldMeshing::Now`); a
host meshing fields elsewhere supplies them (`Deferred`, `Sketch::field_jobs`, `supply_field`).
**The page never does** (#671): an object is its exact B-rep, built as the STEP export builds it,
or it is unbuilt with the kernel's reason.  The page's sketch builds every object elsewhere
(`FieldMeshing::Supplied`, `Sketch::supply_objects`): `Sketch::object_jobs` names one job a binding
(`Sketch::job_of`: the placements of one build share it, and the sketch places each from it by its
evaluation, a closed instance's swept copy too — `Sketch::placing`), keyed by `solid::field_key`.
A build supplied says whether it is current: what the solid reads is what the build read within a
solve's own tolerance (`solid::same_reads`, `NEAR`), so an object's last build, offered to the next
drawing, is filed under its new key where a solve settling the drawing again moved nothing it
reads, and asked of no worker.  `app/object-builds.ts` hands the jobs, swept first, to a pool of
`min(hardwareConcurrency − 1, 4)` workers (`app/build-work.ts` in `app/mesh-worker.ts`, a core each,
keeping its document between jobs and reading again only for a structural edit or other modules),
each stepping `brep::export::Builder` to the end; the build crosses as one buffer
(`Display::bytes`) and is supplied (`Sketch::supply_object`, `from_exact_surface`).  **An object a
host builds is never validated by the sketch that awaits it** (`Sketch::awaits_supply`): the host
validated it and says a refusal in its own words, and validating a sweep finds its roll
(`solid::roll::interval`), a march along its motion that held the page's thread for seconds an
object.  So `EvaluatedSolid::evaluate` takes its supply before `validate_at`, with the operands
walked unvalidated (`solid::operands_reached`, over `evaluation_operands`), and `solid::has_sweep`
answers false for it without validating it: it has no field here, its boundary is its build,
whatever approximation is asked.  Gate:
`roll_through.rs`'s `a_rolled_object_a_host_builds_is_drawn_from_its_build`.  Until then the
object is refused, on its way — `overview::unbuilt` leaves it out — and a build refused is unbuilt in
the export's words (`refuse_object`), never a field meshed in its place; an earlier drawing's build
is drawn stale (`EvaluatedSolid::stale`) till the new one arrives; a swept solid that is no object
is meshed by nothing.  `object-builds.test.ts` is the gate: no `FieldMesher` made, no object cut
on the page's thread.  Field fineness (`FieldMesher::with_fineness`, 0.25–4) is the core's alone.
**One builder serves every object** (#670): `brep::export::Builder` builds a static body as the
export builds it (`export::exact`, a stage, then its mesh) and a swept one by the generating
class's stages (admitted, blank, sheets, sector, cut, pattern), meshed at
`DISPLAY_SAG`/`DISPLAY_ANGULAR`; a body whose sweeps are built within their envelopes counts no
sheet, plan or cut.  A body built from swept material is built as `export::compose` builds it, to
any depth: each operand that holds sweeps by stages of its own to its B-rep (moved where a
placement stands it), then combined by the body rule and meshed — a compound of two shaped gears,
a gear made of a shaped blank (`a_body_of_swept_bodies_is_built_by_stages_as_the_export_composes_it`).  Its `Display` names each face as `solid::exact::named` does, carries what the
solid read, and the B-rep's edges read off the mesh — a polyline through its vertices a pair of
faces' stretch, the two faces, smooth where their own normals meet within `SMOOTH_EDGE` (a round's
tangent edge), sharp otherwise; a seam bounds nothing and is none (`solid::ExactEdge`).  A refusal
is the export's `ExportRefusal`, its stage and its words.  `tests/object_builder.rs`: every object
the export builds the builder builds — a static one with its face count and volume, a swept one,
admitted at the display's samples, to its volume within the export's gross bar (`GROSS` deep over
its surface) — and refuses what it refuses in its words: static documents here, the whole library
in the slow tier.

**Field meshing** ([docs/field-meshing.md](../field-meshing.md)): resumable Delaunay refinement
(`delaunay::refine::Progressive`, exact predicates, a `refine::Domain`), sharp edges protected by
weighted points. A first pass on `MaterialField::tight_support`, `solid::crease` traces creases,
the final pass protects them. A crease is where the deciding operand changes (leaf,
`PlanarField::carrier` piece, sweep contact time; `OperandId::whole`); a point needs the field at
zero and both leaves active; `End` says why one ends. A crease runs on across a seam where a
followed leaf's next piece is tangent to the last: the piece the field names past it, or, where
two leaves' faces lie on one another and the field names the other's (a boss's base on its
plate's face, facing down), the followed leaf's own — else the seam is a corner coming one way and
a hand-off going the other, and the crease ran twice over the round and ended microns from its
start (`crease::trace`, #780 R1;
`composed_rounds::a_chamfer_spilled_onto_a_rounded_plate_has_one_closed_crease_over_the_seam`).
Crease reads only `crease::CreaseSource`.
Rays seed beside the centre, never on it. `MaterialField::reading`: value, gradient, `OperandId`,
crease flag. Terms compile once (`material/plan.rs`); support, bounds, enclosure are `Fold`s. Every
point query is `MaterialField::query(p, &mut Query)` (`Want::Sign | Reading`, `Source::Exact |
Cached(Resolution) | Warm {hints,local}`; shorthands `side`, `reading`); `SweptField::query` takes
box, floor table, cache, search, in order. Readings are branch and bound; crossings by safeguarded
Newton, memoised. A surface centre within the refinement's margin of a vertex is not inserted
(`Refiner::crowded`, #780 R1): a restricted facet's surface ball is empty, so such a centre is
off its facet's dual edge — four corners nearly on one circle and in its plane, two pairs of a
symmetric part's protecting balls, have an orthocentre rounding puts anywhere on the circle's
axis, its side with it — and inserted it was a vertex again, 10⁻¹⁵ off: the library rod at its
reading's size held three corners of an element there, and its drive could not be refined
(`hulls::the_librarys_rod_is_sized_by_its_sag`, the slow tier). A `construction` solid is never an object. `spiral_bevel` (`hypoid.sv`) is built
exactly, both members, as every object the page draws is. `solventc --stl-backend refine` skips admission, gated by field agreement; it needs no OCCT
(#904 A15): the features it protects are read off the static blank as the core's own kernel
builds it (`blank_features::KernelBlank`, over `brep::query::Located`: the edges two faces meet
along, each face by its trimmed parameters), by OCCT only where the build has it and it is asked
for by name, or traced from the field alone (`SOLVENT_FEATURES=field`)
(`kernel_blank::a_blanks_features_are_read_by_the_core_kernel`,
`cli::a_refined_mesh_is_written_without_occt`).

**A convex meridian's arc ending on the axis about a centre off it is bounded by its tangent there**
(`solid/field/profile.rs`'s `convex`, #721 G5): the convex construction bounds an arc's disk only
within its angular sector, and the axis is no wall, so at an ogive's tip nothing bounded the wedge
between the arc's radius there and the axis — the field read material behind the tip, which a sweep
carried through a rod's core. The arc's tangent at such an end, a supporting line of the convex
loop, bounds it: it moves no boundary, and the distance within stays exact (the tangent lies
outside the loop). Gate: `swept_apex::the_drawings_field_reads_behind_an_ogives_point_as_outside`.

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
**A profile's side reads no distance it need not** (`Profile::side`, the ray's parity and then only
the edges within the level): an extruded field's side is its profile's at the point's section, a
revolved field's its meridian profile's at the point's radius and height (`RevolvedField::side`) —
the swept admission's blank membership read a turned blank's whole distance at every sample —
and each says what comparing the value says
(`swept_speed::a_fields_side_of_a_level_is_what_its_value_says`, a pierced sphere and the hypoid
pinion's blank among them). **A conic wall's distance over a box is bounded by its pieces' control
triangles, nearest first** (`profile::Wall::conic`): the arc within each piece's triangle, the
triangle within the piece's bulge of the arc, so a piece's gap is off by its bulge at most, which
falls with the square of its size; the pieces halved once into a tree (`Conic`), past it as reached,
read till the nearest left bulges no more than an eighth of the box or the rounding's slack. Read by
discs about the triangles, whose gaps fall only as their size, a point's box halved a band of pieces
to the deepest — the hypoid pinion's field agreement read 1 400 pieces a call, 320 million in all,
where it reads one or two (`swept_speed::a_turned_conics_field_holds_its_values_over_any_box`: every
box's enclosure holds its sampled values, and a point's is within 10⁻⁸ mm).
`MaterialEvaluator::boundary` needs that support, complete cells, strict corner signs and two-sided
distance evidence for `FieldBoundary`. `BoundaryOptions::domain` takes a caller's box instead, but
it **is** an arbitrary crop: its signs prove nothing beyond it, and closure is the caller's
judgement. Never substitute a crop, residual magnitude or same-sign corners for coverage.
Ambiguous vertices, missing mesh evidence, work limits and failed topology refuse extraction (see
`docs/field-boundary-extraction.md`). Keep the extractor as a measured baseline; evaluate
`docs/implicit-meshing-methods.md` before extending uniform refinement.
