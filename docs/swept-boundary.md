# The certified swept boundary

`solid/swept_boundary` builds a closed triangle mesh of the material a tool sweeps under
a motion, alone (no blank), with every triangle judged against the sweep's own
one-Lipschitz field. It replaces the arrangement in `gcs-cli/src/cad/mesh_sweep.rs` as the
source of truth for a swept solid: that arrangement kept passing its own gates (a recorded
volume, a closed STL shell) while a field-agreement probe found 19% to 37% of its triangles
wrong on the hypoid pinion at 15° and 30°. The plan that governs this module is the one
approved on 2026-09-10: the field is the truth, the traced sheets are seeds, the scope is
every Boolean body of prisms and full revolutions under every named motion, and a sweep is
reliable when closed-form cases and zero-disagreement field probes say so.

## The contract the field gives, and the two facts used

`SweptField` is `min over t of tool(inverse(motion(t)) x)` and is one-Lipschitz
(`solid/field/swept.rs`). Its material is `closure({f < 0})`. Two facts carry the module:

- **A strict sign change brackets the boundary.** Strictly material at `p⁻` and strictly
  exterior at `p⁺` puts a boundary point on the segment between them. No gradient is needed.
- **A converged enclosure containing zero is a distance certificate.** Being one-Lipschitz,
  the field's zero set lies within `max(|lo|, |hi|)` of a point whose converged enclosure is
  `[lo, hi] ∋ 0`. That is the `Near` sign, and it is a bracket end, not a failure.

Nothing else is inferred: never a distance from a value's magnitude, never a sign from an
enclosure that stopped on its budget or on the roll's resolution limit (`Unresolved`).

The cost of a query is set by the roll refinement. At an envelope point `n·v = 0`, so the
field is quadratically flat in the roll and proving a sign at distance δ from the boundary
costs refinement in proportion to 1/δ. The evaluator's travel bound is `speed × |dt|` with
`Family::inverse_point_speed_bound_over`, which for a family that is one rotation or screw
is exactly `|ratio| × distance(box, axis) + |advance|/τ`: a point on the axis does not move
and resolves at once, where the earlier bound from the point's radius about the world origin
left such a point refining the whole roll and exhausting its budget.

## Milestone 1: the judge and the labels

`FieldJudge` (`judge.rs`) wraps `MaterialEvaluator::bounds_outside` with two budgets: near
queries (band `[0, 0]`, value tolerance half the vertex tolerance) and far queries (a band
half the probe distance wide). It counts and times every query.

`project(p, m, ε, reach)` asks the signs at `p ± ε·m`: a bracket keeps the vertex (two
queries, the common case); material on both sides widens outward by doubling to `reach`,
exterior on both sides widens inward, and a sign change found on the way is bisected to a
`2ε` bracket; the same sign as far as `reach` labels the vertex `Inner` (a branch of the
envelope the sweep covers at another time) or `Positive` (off the swept material).

`directions` (`project.rs`) gives every vertex of every seed sheet the normalised sum of the
normals of the triangles incident on its position across all sheets, each triangle's normal
signed by the sheet's stored normals. On a smooth sheet that is the sheet's normal; at a tool
edge or corner it is the bisector. Judging along one face's own normal at an edge vertex
runs tangent to the other face and reads exactly zero forever, which is how the first run
produced 118 unresolved vertices on the tumbling cylinder.

Evidence on 2026-09-10 (`tests/sweep_mesh/labels.rs`, spacing 0.5 mm, sagitta 0.02 mm,
ε = 5 µm, reach 20 µm):

| case | vertices | kept | inner | near query | run |
|---|---|---|---|---|---|
| sphere turned ±60° (torus) | 306 | 306 | 0 | 0.4 ms | 0.26 s |
| cylinder tumbling ±30° | 693 | 495 (+2 moved) | 196 | 0.3 ms | 0.55 s |
| triangular prism turned ±50° | 480 | 410 | 70 | 0.3 ms | 0.30 s |

Every kept torus vertex is within 1e-15 of the analytic torus. Every transition from kept to
inner has another transition (on another sheet, or on the same sheet away from it) within
0.4 mm, median 0.13 mm: the crossing of two envelope branches shows on both. That answers
the plan's two risks for these cases: near-boundary queries are affordable, and inner turns
have traced partners.

The pinion cutter (ignored tests `the_pinion_cutters_query_cost` and
`the_pinion_cutters_sheets_are_judged`, which sample contact points through
`SweepContacts::pieces_at` rather than tracing the whole sweep): one near query at a contact
point converges in 33 ms after 3320 roll evaluations of about 10 µs each (an interval pose
21 µs, a tool-field box evaluation 3 µs), with a speed bound of 140 mm/rad. The enclosure's
width falls only in proportion to the evaluations spent (0.025 mm at 1024, 0.0025 mm at
3320), which is what a Lipschitz bound gives at a flat minimum; a second-order roll oracle
is the lever that would make it quadratic. Over 200 contact points at six parameters: 140
kept, 4 moved, 47 inner, 9 unresolved (queries exhausting 4000 evaluations at the far rim,
75 mm out), 12 ms per near query on average, the slowest 170 ms. At that price the whole
cutter's 58k seed vertices are about half an hour single-threaded, against the plan's
minute; the oracle or shards must land before milestone 7.

Two costs found on the way and worth knowing: tracing the cutter's sweep with no reach
window (the sweep alone, as this module wants it) takes minutes where the cropped trace
takes seconds, so the whole-sweep trace needs its own attention; and a point on the motion's
axis reads a constant field along the roll and, under a speed bound taken from its radius
about the world origin, refined the whole roll to its budget every time.

What is not yet judged: an end column's vertices on a tool edge are judged along the sheet's
side alone until the caps exist (milestone 3), so they are the only vertices the tests allow
to be positive or unresolved.

## Milestone 2: the triangle certificate and the closure audit

`kept_triangles` (`trim.rs`) takes every triangle all of whose vertices were kept or moved,
at the judged positions, wound so its normal agrees with its vertices' judged directions.
`certify` (`certify.rs`) probes each triangle's centroid `d` inside and outside along its
normal, `d = 2·sagitta`, and needs material inside and exterior outside. Material or exterior
thinner than `d` halves the distance down to `2ε`; a triangle that still cannot be certified
there is recorded as `thin` (its vertices were bracketed and one side still reads the
boundary or the material beyond a gap), which is not a failure but a stated limit, while
exterior inside and material outside at the least distance is `Reversed`, a failure.
`boundary_loops` chains the edges used once into loops: the closure audit's input.

Evidence (`tests/sweep_mesh/certificate.rs`, same tolerances as milestone 1, `d = 40 µm`):

| case | kept triangles | certified | thin | failed | boundary loops |
|---|---|---|---|---|---|
| sphere turned ±60° (torus tube) | 578 | 578 | 0 | 0 | 2 (the open ends) |
| box translated 10 mm along x | 2000 | 2000 | 0 | 0 | 4 (one per side band) |
| cylinder tumbling ±30° | 681 | 649 | 32 | 0 | 65 (inner turns unstitched) |

The cylinder's thin triangles lie by its tumble axis, where consecutive positions of the
wall leave a scissor-thin exterior under 10 µm wide; its 65 loops are the inner turns the
labels cut out, which milestone 4 trims and seams. A far query (band half the probe wide)
costs 0.5 ms on these cases.

## Milestone 3: caps, stitch, closed shells

The first attempt kept cap facets by the field's depth at their vertices and centroids,
and it cannot work: the sweep is only quadratically deep just past a contact curve (a
cylinder wall turned about a spindle at radius 3 is 0.0072 deep one facet past the contact
generator, 0.029 two facets past), so any facet a chord past the curve reads boundary to
any tolerance the judge can afford, the cap's rim comes out ragged across the sheet's
exact end column, and the zip between them folds. What the field cannot cut, the tracer's
column can: it is the contact curve itself, on the tool, at that instant.

`caps` (`caps.rs`) therefore tessellates the tool through the core's welded indexed mesh
(`static_solid_at_unit`, `indexed`), poses it at each end of the roll, and **cuts it along
every sheet's end column** with `CutMesh`: each column point is put into the mesh as a
vertex (on the facet, edge or vertex it lies on, by the nearest point of the nearest
facet, since the perpendicular projections of surface points onto a convex facet mesh
leave gaps at the edges; a vertex within a quarter sagitta moves onto the point), and
consecutive points are joined by walking facet by facet along the chord's projection in
the plane of the two ends' normals, every facet edge the chord crosses split at the
crossing. Only facets the walk enters are looked at, so a face round a corner, folded into
the projection, cannot offer a crossing of its own. The chain of cut edges parts the mesh
into components; each is judged by its most decisively signed facet's normal velocity
(`|n·v|/|v| ≥ 0.25`; a tessellation angle of 0.4 rad at this sagitta puts facet normals
0.2 rad off): receding components are kept at the start, advancing at the end, and a
component signed neither way (a face parallel to a translation, a face square to a turn's
axis, a sphere about its own centre) is kept at both, for the coverage drop or the planar
union to take one copy of. `caps` reports its components (`CapComponent`: facets, extreme,
kept), so a test can say what a cap must be made of. The rim is then the column chord for
chord, save the crossing vertices the cut put on the chords, which the stitch's tolerant
split puts into the sheet's edges too.

`planar_union` (`planar.rs`): a face grazing the sweep is generated many times over (a
turned cylinder's top by each half of its rim, by the whole face and by both caps), each
source triangulating one planar region differently, and no rule dropping whole triangles
can leave such a region covered once with a boundary its neighbours can join. So every
plane's triangles (grouped by unsigned plane within `1e-7`, facing their area-weighted
majority) are unioned exactly: each triangle is clipped by the fragments already placed
(separating-axis test first, then successive half-plane cuts, so only what actually
overlaps is fragmented) and only what they do not cover is kept, every fragment a convex
polygon fanned into triangles. Input vertices keep their identities; new corners are shared
where they coincide.

`without_overlaps` (`trim.rs`) drops a later sheet's triangle that earlier sheets **cover**
(`covered_by`): the earlier triangles facing its way within about 25° whose planes pass
within the tolerance of its centroid are projected onto its plane and clipped from it, and
only a triangle with nothing left is covered. Nearness was the first rule and it eats two
things a cut cap has: its facets beside the seam, where the sheet is tangent to them, and a
planar sliver beside a perpendicular wall. Tiles of one tessellation share their edges, so
their footprints leave no cracks; the triangle's own corners lie on the surface beyond a
coarser tessellation's chords and land a whisker outside their footprints, so every
footprint is grown by an eighth of the tolerance. Curved partial overlaps between two
tracer sheets with different tessellations are not handled; the cases so far share theirs.

`centroid_kept` drops a triangle whose centroid the field reads deeper than two sagittas
(`FieldJudge::deep_sign`: material only below `−depth`, exterior only above, otherwise near
whatever the enclosure's width; `sign_beyond` keeps its strict semantics for the
certificate's `sides`). `directions` weights each incident triangle's normal by its angle
at the vertex, so a fan of slivers on one face (the cut's fans at a prism's apex) does not
outvote the face beside it: unweighted, an apex read `(0.56, 0.2, 0.8)` and its inward
probe fell outside the lower slanted face.

The stitch (`stitch.rs`): `weld` identifies coincident vertices (`1e-7`);
`split_at_vertices` resolves T-junctions to a tolerance of one and a half sagittas, since
two samplings of one curve hold each other's vertices within a sagitta of their chords (a
cap's rim on the tool's edge against the sheet's column, a planar fragment's corner on its
neighbour's edge, a cut's crossing on the sheet's chord): a boundary vertex within the
tolerance of a boundary edge's interior splits the triangle on it at the vertex, which
stays where it is. Four guards keep a split from folding: the vertex must lie within a
quarter of the triangle's height over the edge, within a quarter of its distance along the
edge from either end (a vertex a whisker from the edge's start but as far from its line
folds the first triangle), the two triangles made must face the way the one did, and
neither new edge may already be walked that way by another triangle. `rim_zip` then pairs
each remaining boundary loop with the nearest loop within four column spacings and zips
them (`zip_loops`, as in milestone 2's description), and zips across every loop left that
turns back on itself at exactly two corners (a slit); the coincident-loop peel is gone.
`boundary_loops` walks each loop round the vertex fan, so loops sharing a vertex stay two.

Evidence (`tests/sweep_mesh/closed.rs`, `s = 0.02`, `ε = s/4`, `d = 2s`, spacing 0.5): each
case a closed shell (`ClosedShell::from_triangles`), every triangle certified, none thin,
the volume against a closed form the construction never sees. An inscribed mesh falls short
of a round solid (its columns are inscribed polygons, about 1.3 sagittas per unit of the
smallest radius), so a round case is allowed three sagittas per unit radius below its
closed form and never above it; a polyhedral case must match to 1e-9.

| case | triangles | volume | closed form | run |
|---|---|---|---|---|
| box 2×3×2 translated 10 along x | 1260 | 72.000000 | 72 | 9.8 s |
| triangular prism translated 10 along x | 1120 | 51.600000 | 51.6 | 5.7 s |
| cylinder plunged 6 along its axis | 950 | 24.509 | 25.133 (−2.5%) | 3.1 s |
| cylinder r 1 at 3 turned ±60° about the spindle | 1492 | 31.342 | 31.416 (−0.2%) | 5.5 s |
| sphere at 3 turned ±60°: torus segment, spherical ends | 4993 | 23.384 | 23.928 (−2.3%) | 3.1 s |
| sphere translated 10 along z: capsule (both ways) | 5174 | 34.818 | 35.605 (−2.2%) | 2.6 s |
| sphere turned ±60° about its own centre (s = 0.05) | 3968 | 4.172 | 4.189 (−0.4%) | 19 s |

The translated cases' time is their near queries at 0.9 ms: a face parallel to its motion
is the flat-minimum case for the roll refinement. The sphere about its own centre is the
worst of it, a field that never depends on the roll, so every query refines the whole
interval (7 million roll evaluations for 24 thousand queries); it runs at a coarser
sagitta to stay in seconds. The pieces have their own gates: `tests/sweep_mesh/pieces.rs`
(the union of doubled and overlapping squares, a belt cut round a cube parting it in two,
an open cut parting nothing, a point off the mesh refused by name, a T-junction on a
shared line and one a whisker off it) and `overlaps.rs` (coverage of a triangle by tiles:
inside one, across two sharing an edge, half outside, merely touching, facing the other
way, off the surface, and a wall sliver against a coarser wall).

## Refusals

Every refusal names its element. So far: `ReversedNormal { point, direction }` (material
outward and exterior inward), `Field(reason)` (the evaluator's own error). Unresolved
vertices are labelled and counted, with their offset and enclosure, and will refuse the
certificate in milestone 2.

## Next

Milestone 4: trim, creases and ribbons on the cases where sheets cross (tumbling cylinder,
turning prism, box at 30°, lens, dumbbell). Then the remaining ★ closed-form cases (box about
an axis through its face centre 360°, ring prism, bored bead, cylinder screw, diagonal box),
the `sweep_cases!` table harness with `forms.rs`, and the tracer fixes. See the plan.
