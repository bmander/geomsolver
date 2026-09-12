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

## Milestone 4: sheets that cross, and what closing them took

Where one sheet turns inner across another, the kept parts of the two must meet along
their crease, and nothing here trusts a traced crease: every crease vertex is found by the
field. `clip_sheets` (`crease.rs`) clips each sheet at its label transitions: a mixed edge
(one end on, one off; on means `Kept`, never `Moved`, since a vertex the judge moved by
0.03 is on the other branch's side of the crease and put the rim there) is bisected on the
field to the vertex tolerance, the clipped triangles are wound outward, and the rim edges
are chained into `Rim`s, dropping rims that run along a sheet's own edges and snapping
crease points within four tolerances of the on vertex onto it (so no sliver thinner than
that is left for the stitch). A cap's corners are judged differently from a sheet's: a
cap vertex on a tool edge is on the boundary by one face or the other and its label says
nothing about which, so a cap triangle's corners are judged eight tolerances in from the
corner along the triangle's plane toward its centroid, where the face itself speaks, and
a mixed cap edge is bisected twice with pulls of five and three tolerances (never past
the triangle's height) and the crossing extrapolated to the edge. Caps are refined to the
column spacing first (`CutMesh::refine`): a planar face is two facets however large, and a
crease crossing it would be invisible to the labels at its corners. Two rims that lie
along one crease are merged by `merge_creases`: rim vertices of different sheets within
one and a half sagittas are aliased, and the rim edges alone are T-split to four
tolerances, since the rims coincide only in part and pairing them whole failed.

What the crease cases then taught, each written where the rule is:

- **Which way a triangle faces is carried across the sheet, not read off its corners.**
  A zipped strip's quads come out cut either way, and at a fan point the tracer stores
  one end of the fan, so a sliver on two such points read the wrong way by its own three
  stored normals (the tumbling cylinder's rim sweeps, 15 folded slivers a sheet).
  `orientation` (`project.rs`) carries the winding across shared edges, stopping at a
  fold where the neighbour would face away (a generator through a fixed point of the
  motion sweeps a bowtie whose halves face opposite ways), and each run takes the side its
  stored normals favour in area-weighted sum; `directions` and `clip_sheets` both read it.
- **Coplanar overhangs are trimmed by their feet.** A ribbon chord dipping past the r = 0.5
  wall left planar slivers overhanging it. `planar_union` is two passes, the union then a
  trim: a foot is an edge of a non-group triangle standing on the plane, and of the
  fragments beyond its line within the edge's strip, the side with at most a quarter of
  the other's area is trimmed (a convexity rule from the far vertex read the wall from the
  wrong side, and a folded ribbon's own feet faced the wrong way until the union had
  flattened it).
- **Curved coverage is clipped along the covering sheet's own outline, never grown.**
  Two corner tubes of the turning prism sweep one surface of revolution over overlapping
  arcs; `uncovered` cuts a later triangle, in its own plane, along the outline edges of the
  earlier sheet's tangent tiles, drops the pieces whose centroid a tile's footprint holds,
  and puts the pieces' new corners onto the outline edges themselves so the seam is the
  earlier sheet's edge exactly, for the split to share. A tile is tangent when its plane
  passes within two sagittas of the triangle *where the two overlap* (the middle of the
  tile's footprint clipped to the triangle): a coarse chord of a cone spanning 17° was 0.043
  off the tile's plane at its own centroid and 0.018 at the overlap, and judged by the
  centroid it stayed whole and overlapped. A piece the snap turns over or stands up (its
  normal more than 60° from the triangle's) was a sliver thinner than the snap and goes;
  the triangle's own corners are never merged, even an eighth of a tolerance apart (a
  sliver of the sheet is still the sheet: merging them ate the turned cylinder's rim
  slivers and left saw-tooth loops of 84 vertices round each cap).
- **The tolerant T-junction split may not lay a triangle over one already there.** In a
  plane both sheets cover, a vertex of one a whisker past the other's edge has a sliver of
  its own reaching back to that edge; splitting the edge at the vertex, within the
  tolerance, covered the sliver twice and the fill afterwards used an edge three times.
  `split_where` refuses a split whose new triangles overlap, in area, a triangle on the
  vertex in their plane (`lies_over`).
- **The cap cut walks in the column's own normals and snaps to the nearer end.** The
  mesh's normal at a column point on a crease of the tool leans into the other face, and
  the walk snapping a crossing to the farther of two near ends stepped onto a vertex
  beside the true one with nothing ahead of it (the dumbbell's bar generator, cut from its
  crease).
- **Slivers are certified where the boundary passes within the least probe distance of
  their centroid** (`Certificate::slivers`): a sliver has no altitude to probe along, and
  lies within that distance of the segment its corners span.
- **Vertices merge into the more connected of the pair, at an eighth of a sagitta.** A
  merge at half a sagitta folded slivers and broke the turned cylinder's seams; merging
  into the vertex with more triangles kept the box's exact corners (72.000000).
- **The rim zip fills three- and four-vertex loops, pairs loops within the column spacing,
  zips slits (loops turning back at exactly two corners) and lays each band triangle by
  its own loop edge**, not once per band.

The reference for a turned section is a ring quadrature, since the closed form of a
turned rectangle (or lens) is one arc per circle only where the region is convex toward
the pivot, which a lens is not: `turned_area` samples each circle about the pivot,
grows what it meets by the turn, and integrates.

| case | triangles | volume | reference | run |
|---|---|---|---|---|
| triangular prism section turned ±50° about (2.5, 0) | 1436 | 11.111 | 11.168 (−0.5%) | 2.3 s |
| lens of two unit spheres 0.8 apart turned ±60° about the spindle | 2786 | 12.837 | 13.099 (−2.0%) | 5.6 s |

Both close as `ClosedShell`s with every triangle certified. The eight milestone-3 cases
stay green through it all, several of them the reason for a rule above.

Three of the milestone's five cases are written and run in seconds but do not close, each
`#[ignore]`d with its evidence in `creases.rs`, and the plan's contingency (A2) applies:
the tracer fixes of milestone 5 lead.

- **Tumbling cylinder** (radius 1, height 2, tumbled ±30° about the line through its
  centre). The stationary ring sweeps a sphere of radius 1 that lies inside every pose of
  the cylinder, so the whole sheet is inner except along the two fixed points of the axis
  where the depth is second order: within a quarter sagitta over a patch a fifth wide,
  which the labels keep and clip raggedly. The generators through the fixed points sweep
  planar bowties (their halves face opposite ways, and the zip's quads cross at the
  point). The rims' sweeps fold where a rim's tangent runs along its velocity, the fold
  double-covering one surface of revolution. Nine loops and a few dozen refused triangles
  remain after 4 s.
- **Box turned 30° about the axis through its end face's centre.** The end faces' edges
  sweep in their own plane, and a segment turning in its plane folds at its envelope arc
  (consecutive columns cross a tenth of a unit from the tangency); the planar union is the
  right region but its slivers along the arc are thinner than the split's tolerance, and
  the split and the zip lay fills over them. One refused triangle and one two-vertex loop
  remain; the volume is within a percent of the quadrature.
- **Dumbbell translated 4 along x.** The Boolean tool's mesh is forty-five thousand facets,
  the bar's wall shredded by the balls' facet planes, so the caps take most of a minute and
  their slivers leave four loops at the crease junctions. This is project 2's problem, the
  Boolean meshes, and is noted for it.

## Milestone 5: grazing faces, and what the refusals turned out to be (2026-09-12)

**5a, grazing planar faces, is done.** A face the motion carries within its own plane
(`n·v ≡ 0` over the whole face) is given nothing by the tracer: every edge bounding it enters the
fan at every parameter, is emitted whole, and is zipped into a ruled band that folds. So such a
face is swept exactly instead, as a 2D region: `Family::in_plane` reads a motion's own steps for a
turn about an axis square to the plane or a slide along it (anything else is none, and an
invariant plane grazes exactly, needing no sampling), `grazing_faces` finds them,
`SweepContacts::leave_faces` stops the tracer emitting their edges, and `swept_region`
(`grazing.rs`) builds the region in rows — radius and angle about the pivot for a turn, across and
along for a slide — with critical rows where the row's tagged interval list changes, bisected to
the coordinates' own precision. `caps` drops facets lying in such a plane **and cuts its
components at those faces' boundaries**: without that cut a slid box has no end columns and the
whole tool reads as one component. The turned box closes and certifies: 1304 triangles, 1300
certified, 4 thin, 0 failed, volume 15.3167 against 15.3237. Its last defect was in the region
builder — two cells of one strip both meshing the stretch they shared at a row where a gap between
them closes — fixed by `Sweep::share`, which halves such an overlap so a strip's cells partition
the rows they share.

Landing first with it, a guard: `rim_zip` leaves a loop that visits a vertex twice, or a walk of
two vertices, unpaired rather than laying a band over triangles already there, and
`ConstructError::UnpairedRim` refuses instead of shipping an open shell.

**5b was redirected twice by measurement, and the second redirection is the result.** The plan's
own account — that a cap is cut only where a strip survived, so a band the strip never reached is
bounded by the cap's own tessellation — was implemented and refuted: at the end poses **0 of 42
contact-curve points were uncovered, farthest distance 0.000000**, the surviving columns being the
end-pose curve point for point. Two further changes (snapping clip points at source in
`clip_sheets`; snapping to pinned vertices in `CutMesh`) measured inert or regressive. All were
reverted.

What settled it was instrumenting rather than reasoning. `hygiene` measures every stage's mesh
against what it should never hand on — an edge used more than twice, an edge two triangles walk
the same way, a degenerate triangle, a triangle repeating another's corners — and names the first
few at fault with their midpoints; an edge used *once* is no fault, since a sheet is open until the
shell closes, and a near pair of vertices is reported only between the weld's coincidence and the
snap, exact duplicates before the weld being by design. `window` gives what stands in one ball of
any stage, keyed by **position**, which is the only handle that survives the pipeline: the same
point of the surface is a different index at `Clipped` than at `Split`. `Origin` records how each
cut-mesh vertex came to be — the tool's own mesh, `refine`'s midpoint, a column point inserted, or
a chord's crossing — and `Cap` carries it out.

They gave the finding. **Every unpaired loop in every failing case fails one of the zip's two
gates, and none fails for want of a partner.** A loop that repeats a vertex is pinched, and
`simple` is the first thing the fill, the pairing and the slit pass each ask; the slit pass takes
only a loop turning back at exactly two corners. The 5° cylinder slid along x leaves four pinched
loops (14/13, 12/11, 11/10, 12/11 distinct), the thin plate two pinched and two simple turning at
1 and 4 corners, the tilted cylinders at 30° and 85° one simple loop turning at 4.

So the question was never why the zip refuses but why the boundary touches itself, and the answer
is in the caps: a cap is cut along the sheet's end column, and where that chain crosses the tool's
own rim the kept component is left touching itself at a column point the cut inserted. A walk that
returns to a vertex is two walks, and `unpinch` cuts it into them on vertex identity and no
tolerance — the pieces walking every edge of the original and inventing none. With it the sliding
dumbbell gets past the zip for the first time (a certificate of 28132 of 35396, 4 failed, though
its shell is still open at one edge used three times), the tumbling cylinder falls from 71 unpaired
loops to 8, the slid cylinder from four loops to one, and the thin plate from six to four. The
turning prism, the turned lens and the turned box certify closed exactly as before, byte for byte.

**Recorded and off the path:** `planar_union`'s foot-trimming pass takes one region from 2
fragments to 2434 over eleven feet and leaves 744 of 5935 triangles repeating another's corners,
every fragment a foot touches coming back with its largest piece 100.0000% of what went in.
Dropping them with `dedupe` changes no outcome in any case, so it costs work, not correctness.

**Still refusing**, each with its evidence: the tilted cylinders at 30° and 85° (one simple
5-vertex loop turning at four corners — too long for the fan fill, alone within the pairing's
reach, and not a slit), the thin plate under a 1.05 roll, and the tumbling cylinder. 5c — the
tumbling cylinder's rim folds and its fixed-point bowties — is not begun.

## Cost (2026-09-11)

The target is WASM on one core, so the work is total cycles and complexity classes, never
threads. Every change was checked by exporting the five milestone-4 cases' STLs
(`export_milestone_4_cases` with `SOLVENT_EXPORT`) and comparing them byte for byte with a
baseline: identical, except for the one change that moves traced points (below). Measured on
the turned lens, the Boolean tool among the milestone-4 cases:

| stage | before | after |
|---|---|---|
| seeds (the tracer) | 261 ms | 92 ms |
| caps (tool tessellation) | 6.3 s | 68 ms |
| labels | 378 ms | 180 ms |
| clip | 982 ms | 85 ms |
| kept and certificate | 1.35 s | 0.29 s |
| planar union | 122 ms | 25 ms |
| whole case | 9.3 s | 0.83 s |

The dumbbell, 35 000 triangles, went from 52 s to 10 s. What changed, by what it fixed:

- **The BSP** (`csg.rs`). A convex solid's tree is a chain as deep as the solid has facets
  (every other facet lies behind each facet's plane), so building and clipping were quadratic,
  and the recursive walks overflowed the stack on a finely cut sphere. The walks are loops over
  an explicit stack, a polygon is moved rather than cloned and classified once per level, and a
  large batch goes down a chain with bounding spheres over it (`Balls`, `build_spine`,
  `clip_spine`): a node's plane asks only the polygons whose spheres reach it, the rest being
  strictly on the side the batch goes on. At a branch the batch goes on along the side most of
  it takes and the rest leaves as a batch of its own, so no polygon is gathered into new spheres
  more than a logarithm of times; `clip_to` clips a whole tree's polygons as one tagged batch.
  About n log n for a convex operand, the same splits in the same order.
- **Roll refinement** (`swept.rs`, `minimum.rs`). A roll cell and its midpoint sample are the
  source at one pose, so the midpoint's value serves both, which halves the source evaluations.
  `deep_sign` stops once its enclosure lies strictly inside the band (`Status::Contained`), where
  the refiner's nested enclosures can only stay; a sweep reached through fixed poses alone may
  stop so, never an operand of a Boolean.
- **Cap corners** (`crease.rs`, `caps.rs`). Each triangle of the tool mesh carries its face
  through `CutMesh`, and `caps` names the cap vertices whose facets lie on more than one face.
  Only there, or where a vertex is not kept, is a corner judged in from the corner; inside a face
  the vertex's own label speaks. The queries go with the caps' edges, not their area: 12 252 near
  queries on the lens became 2 094.
- **Kept and certificate** (`trim.rs`, `judge.rs`). A triangle whose certificate probes read
  material inside and exterior outside cannot have its centroid deeper than the probe distance
  (the field is one-Lipschitz), which is all the deep sign could say, so the kept stage asks the
  probes first, of the triangles the certificate probes, and the judge remembers every band
  query: the certificate re-asks nothing of a triangle the stitch left alone.
- **Planar union** (`planar.rs`). Groups are indexed by normal, and a group's feet are gathered
  from a flat cell table within its own diagonal of its box: a foot farther away cannot cross it
  with area on both sides, and the region only shrinks. Quadratic to about linear.
- **Caps and the vertex merge** (`caps.rs`, `stitch.rs`). A cap's vertex normals are summed in
  one pass, and the merge's counts, flip checks and relabels read incidence built once a round.
- **The tracer** (`tool_faces.rs`, `sweep_candidates.rs`). Crease points and the events where a
  normal velocity changes sign are found by regula falsi with the Illinois fix, a handful of steps
  where bisection took 50 and 32. This moves traced points by at most 2e-12, and the stitch
  amplifies that into different valid meshes: the turning prism still closes and certifies with
  its volume unchanged, while the two open cases' failures moved (the tumbling cylinder from 28
  to 40, the turned box from 1 to 4). The knife edges that did it are below (Structure).

What is left on the lens is two thirds field queries: linear in the elements, a logarithm of
roll cells each, about 1.6 µs a roll evaluation in outward-rounded interval arithmetic. A
gradient-based roll bound would make the cells per query logarithmic in the tolerance where the
Lipschitz bound's grow as its inverse square root, but at today's tolerance it would about halve
the evaluations at twice the cost each. The dumbbell's planar union (2.5 s) is its clipping.

## Structure (2026-09-11, issue #57)

**The pipeline is the library's.** `construct(sk, swept, options, progress, observe)` traces the
seeds, and `construct_from` runs the rest from seeds a caller hands it (a test's own, moved or
thinned). Every tolerance is a method of `SweptBoundaryOptions`, derived in one place from the
sagitta, the spacing and the vertex tolerance: `reach`, `least_probe`, `judge_tolerance`, `snap`,
`crease_merge`, `coverage`, `coincidence`, `shortest_edge`, `junction`, `axis_tolerance`. The
kept stage takes the certificate's own probe and least distances, where it had hard-coded half a
sagitta to match them. Each finished stage is shown to an observer (`Stage`, with the judge's
counts so far), and that is the whole of how the harness prints and times: its `Diagnostics`
reads `SOLVENT_SHEETS`, `SOLVENT_COLUMNS` and `SOLVENT_DUMP` once, and nothing in the core reads
the environment. A cap is typed (`Cap`: its end, its patch, its components, its tool-edge
vertices) and a seed is a traced sheet or a cap (`Seed`), so `label_seeds` and `clip_sheets` ask a
seed what it is where they took parallel flags. The vector helpers, the triangle measures and
Ericson's closest point are written once in `space.rs`, the Illinois root in `roots.rs`; the
interval refiner's stopping rule is `minimum::Stop` and the judge's query kind `Ask`, where
booleans were.

**Adjacency** (`adjacency.rs`). `Edges` sorts a mesh's directed edges once and answers by binary
search (the boundary by one more sort), `Incident` builds the fans in one counting pass, and
`Live` keeps edges, fans and boundary up to date as the T-junction split changes triangles, so a
round costs its splits and not a rebuild of three trees. `CutMesh` keeps each vertex's facets as
it cuts and files the facets on a grid for `locate`. On the dumbbell the caps' cutting (the
tool's tessellation aside) went from 163 ms to about 70 and the split from 489 ms (eight rounds)
to about 60. Every candidate is still read in the order the scans read it, so no output changed.

**One spatial index** (`space::Grid`: ids filed under cubic cells, visited in x, y, z order and a
cell's ids in the order filed; `PackedGrid`, the same packed once for reading, dense where the
occupied box is small). It replaces the grids of the weld, the collapse, `directions`,
`clip_overlaps`, the planar union's normal index and feet table, the tracer's buckets and
`mesh::weld`, each keeping its own cell size.

**Knife edges.** A decision that turns on rounding makes different valid meshes from inputs no
tolerance can tell apart, which was the 2e-12 move of the regula falsi above (the prism changed
by 372 triangles). Two tests hold it: `seeds_moved_below_every_tolerance_leave_the_mesh_as_it_was`
(the prism and the lens, in the suite, 3.5 s) and
`seeds_moved_below_every_tolerance_leave_every_milestone_4_case_as_it_was` (all five cases,
`#[ignore]`d, a minute): every seed point moved by about 1e-12, every stage's labels, keep flags
and triangles are the same and every vertex is within the weld's 1e-7. Moving the seeds and
diffing the stages found the knife edges one behind another, and every one was a threshold
standing where exact geometry sits:

- **The judge's sign.** A sign query stopped as soon as its enclosure left zero, so a point on an
  exactly computed face (a plane, or an end face square to a turn's axis, which is a boundary of
  the swept solid) took its sign from the side rounding put it on; the turned box's rim labels
  flipped. A sign query now refines until its enclosure clears the tolerance or converges, and
  one lying within the tolerance of zero is near, as one containing zero already was: the
  decision moved from zero, where exact geometry sits, to the tolerance, where nothing in
  particular does.
- **Flatness.** Exactly zero area was the test of a flat triangle, and three points at a fixed
  point of the motion are flat exactly when traced and flat to 1e-12 when moved (the tumbling
  cylinder's clip). `space::degenerate` (least height within 1e-10 of the coordinates' size)
  gives every construction decision its normal (`stable_normal`), and the certificate takes a
  flat triangle as the sliver it is.
- **Ties.** Equal rungs in the zip's dynamic programme and its closed start, the split's vertex
  nearest an edge's middle, the zip's nearest loop and its forward or backward arc, and the
  vertex merge's order by length: a plane's grid and a symmetric pair of curves make them equal,
  and a strict comparison let rounding choose. A choice within 1e-9 of the coordinates' size is
  a tie, taken the same way every time; the merge goes by the vertices' indices.
- **Sharing and snapping.** The cap cut walked onto a face its projection sees edge-on (its far
  edges project onto the chord's own line; the turned box's cap refused); such a facet offers
  no crossing. A column point within the snap of two mesh vertices took the first corner of
  whichever facet it landed nearest; it takes the nearest corner. The planar union shared new
  corners by rounding them to a grid, splitting two coincident corners either side of a cell's
  edge; it shares them within `eps`. And `uncovered` cut at a billionth of the triangle's size,
  below the tracer's noise where two sheets' edges coincide; it cuts at a billionth of the
  coordinates.

These change the meshes, and for the better; the certificates now read:

| case | triangles | certified | thin | failed (before) |
|---|---|---|---|---|
| turning prism | 1436 | 1421 | 15 | 0 (0) |
| lens | 2786 | 2757 | 29 | 0 (0) |
| turned box | 3356 | 3352 | 3 | 1 (4) |
| tumbling cylinder | 3335 | 2568 | 739 | 28 (40) |
| dumbbell | 35 410 | 28 138 | 7264 | 8 (22) |

## Refusals

Every refusal names its element. So far: `ReversedNormal { point, direction }` (material
outward and exterior inward), `Field(reason)` (the evaluator's own error). Unresolved
vertices are labelled and counted, with their offset and enclosure, and will refuse the
certificate in milestone 2.

## Next

Milestone 5, the tracer: folds where a contact curve's tangent runs along its velocity
(the tumbling cylinder's rims, the turned box's planar edges), strips through a fixed
point, dual stationing and strip closedness; then the three ignored cases above, the
remaining ★ closed-form cases (box about an axis through its face centre 360°, ring prism,
bored bead, cylinder screw, diagonal box), the `sweep_cases!` table harness with
`forms.rs`, and gap fill (milestone 6). See the plan.
