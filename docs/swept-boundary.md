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

### The stitch is finished, and the residue is not the stitch's (2026-09-12, later)

Three changes made the mesh manifold in every case, and they are one rule: **no triangle may walk
an edge some triangle already walks.**

- `split_where` gathers a round's splits and applies them in a batch, but checked each against a
  `live` incidence the batch had not yet updated. Two boundary edges out of one vertex, split at
  the same vertex, each made `a→v`; being distinct triangles, `dedupe` kept both, and the edge was
  walked twice the same way. Each candidate now claims the four directed edges its split will make
  (`a→v`, `v→b`, `v→c`, `c→v`) and one whose edges another has claimed waits for the next round —
  deferring costs nothing, the loop already running to 64. `split_at_vertices` now hands on a
  clean mesh in every case.
- `rim_zip`'s `lay` keeps every directed edge the mesh walks and leaves out a band triangle that
  would repeat one. **Every edge used more than twice is gone from every case** — the 360° box's
  eleven used four times included, a 5a defect this was not aimed at.
- The passes each read the one walk of the boundary the round began with, so a loop another pass
  made — or left where a band was declined — was never looked at again: a three-vertex hole the
  first pass fills outright survived the round because it did not exist when that pass ran.
  `rim_zip` now re-walks the boundary and runs `zip_round` until a round lays nothing, and reports
  what is still open **off the finished mesh** rather than off the opening walk. The box turned a
  whole turn closes and certifies (2034 triangles, 1804 certified, 230 thin, 0 failed), and the
  dumbbell falls to two three-vertex loops.

That last change corrected the record as well: the tilted cylinders, the slid cylinder and the
360° box had been *reported* as certifying while their shells were open. The stale walk named
nothing unpaired, so `UnpairedRim` never fired, though the shell check had said `count: 1` all
along. They refuse honestly now.

**A loop the field says the boundary spans is filled.** `loop_span` fans a loop from its centroid
and projects each fan centre along the outward normal of the triangle owning the edge it stands
on: `Kept`/`Moved` throughout is `Spanned`, an `Inner` or `Positive` is `Open { at }`, and a
budget failure is `Unresolved`, never rounded either way. `rim_zip` asks it last and only of what
nothing else could pair, so every case that closed without it closes the same way. It
discriminates: on the tumbling cylinder the two 7-vertex loops read `Spanned` and are filled while
the 82- and 64-vertex seams read `Open`; on the plate an 11-vertex loop is filled and the
20-vertex seam refused. The fill is fanned from a **new vertex at the centroid** — the very
triangulation the field judged, and the only one whose chords cannot already be edges of the mesh.
A slit's ends are also read more widely: two reflex corners are its ends wherever the sampling
turns sharply at both, but a slit with a rounded end has fewer and is still two arcs lying
alongside each other, so the ends fall back to the loop's own diameter. What admits a slit is
still the `apart` test, so this cannot take in a loop whose arcs do not lie together. It closed
the plate's 1.88 × 0.054 slit.

**`planar_union`'s foot trim was fixed, and it changed no refusal.** `cut` places a point within
`eps` of a line on **both** sides, so a fragment lying along a foot came back whole in each half
and, kept twice, was emitted twice: the 30° case's trim returned 744 of 5935 triangles repeating
another's corners. A foot a fragment lies along does not divide it — detectable because the halves'
areas then sum to more than the whole — so the fragment is kept once. Repeats fell to **0**, the
trim's output from 5935 to 5168 triangles, and `unioned hygiene` from 40 edges used more than
twice, 48 walked the same way and 744 repeats, to clean. No refusal moved: `weld`, `dedupe` and
`drop_doubled_slivers` were already clearing them downstream. It is a cost and hygiene win, and it
fixes the cause the earlier note recorded as a symptom.

**Five mechanisms died by measurement, recorded so they are not tried again.**

- **Sewing a wide seam side to side** (the latest, and the one that looked most like the answer).
  Those seams fold back on themselves — 60 of 82 and 44 of 64 of their vertices have a counterpart
  a spacing away, four or more steps along the walk — so zipping along that correspondence reads as
  the slit treatment at a larger scale, and joining two sides should add **no area**. It added area
  instead: the tumbling cylinder's volume rose 6.6509 to **6.6865**, half a percent of surface
  invented exactly where the field says `Open` and no boundary runs, while its loops went 33 to 39
  and its seams shrank only from 42/44 to 40/36. No failed triangle appeared — but only because
  that case refuses at `UnpairedRim` before the certificate ever runs, which is an absence of
  evidence, not a pass. The sides fold and yet do not lie close enough to sew: the `Open` verdict
  stands, and the missing surface is the tracer's to generate rather than the stitch's to invent.
  (Pairing the arcs by consecutive index instead shattered them outright: 2 seams became 73 loops.)

- A **multi-slit pass**, cutting a folding loop at every reflex corner and zipping consecutive
  arcs within `within`: the tumbling cylinder went from 2 seams to **73 loops**. That 60 of 82
  vertices have a counterpart within a spacing does not make arc *k* the partner of arc *k+1*.
- A **boundary weld** merging a sub-tolerance loop's two sides: it reintroduced non-manifold edges
  (11 used more than twice on the tumbling cylinder, where there had been none) to close one or
  two loops per case. Merging two boundary vertices that share a neighbour leaves two triangles
  walking one edge the same way — the very rule above.
- That a decline is **a flip taken backwards**: instrumented, **not one declined band is free
  wound the other way** — 86 on the tumbling cylinder, 22 on the box, 13 on the slid cylinder. Both
  windings collide with edges the mesh already walks, so the obstruction is real and surface
  already stands there.
- **A quad's other diagonal.** A four-vertex loop is fanned across one of its two diagonals, and
  where that one is already an interior edge two triangles walk, the other is often free; trying
  each start vertex closed the 30° tilted cylinder outright — every edge used exactly twice, no
  boundary edge left, a first for that case. It was still wrong: the certificate read two of the
  laid facets `Reversed`, the field giving exterior a probe distance *inside* them and material a
  probe *outside* (+0.0342/−0.0368 and +0.0308/−0.0400), and the shell gained a
  `NonManifoldVertex`. A case that closes on a refused certificate is a failure, not a warning, so
  the free diagonal reaches across open air and the loop is a false boundary after all. What was
  kept from the attempt is only what is not a guess: the winding rule said once (`wind`) so the
  test of whether the mesh can take a triangle and the laying of it cannot disagree, a fan laid
  only where the mesh can take **all** of it rather than part, and a loop no longer marked paired
  when nothing was laid — which had hidden the hole from the report though never from the shell.
  Against the state before it that trade is mixed, and is recorded as such: the tumbling cylinder
  falls from 58 loops to 47, the 30° case's three loops grow from `[4, 3, 4]` to `[5, 3, 5]`, the
  slid cylinder goes from ten to eleven, and the dumbbell from `[3, 3]` to `[4, 4]` with its volume
  moving 9.5297 to 9.5277.

So the zip preserves the manifold, runs to a fixpoint, and refuses what it cannot close. Of what
remains, some loops are **slivers** no triangle can span with area (25 of the tumbling cylinder's
declined bands, 6 of the plate's, 4 of the slid cylinder's; the slid case's loops measure 0.0000
to 0.004 across and up to 0.48 long), and the rest are **false boundaries**. That last was read
off the mesh rather than inferred: every declined band is blocked on an edge that is never a loop
edge but an interior one already used twice, and its two users include a triangle on the band's own
three vertices wound the other way — `[193, 3272, 3266]` against the mesh's `[193, 3266, 3272]` on
the 30° case, `[195, 4174, 4175]` against `[4174, 195, 4175]` on the tumbling cylinder. The zip is
being asked to lay a facet the mesh already has, so there is no hole to fill and `lay` is right to
refuse (`dedupe` keys on sorted corners and would drop the duplicate too). The loop reads as
boundary because its edges never paired with those triangles: the existing facet's third corner is
not the loop's (`200` where the loop has `3263`), which is two samplings of one surface meeting at
different vertices. Both kinds are coverage, upstream of the stitch — the pairing belongs in
`split_at_vertices` and in what makes a cap's rim and a sheet's column share vertices, and no
band laid here could be right.

And the split cannot reach them, which was measured rather than assumed. Counting which of its
guards refuses, on the call that runs once the surviving loops are there: the 30° case makes no
split, with 14 edges having no boundary vertex near enough at all, 10 candidates farther from the
edge's line than the tolerance, 6 and 3 turned away by the quarter-height and quarter-run guards, 3
by an edge already walked — and **`lies_over` fires 0 times**, the fold guard 0 (the tumbling
cylinder: 830, 2719, 2031, 442, 81, and `lies_over` twice). So the split is not declining because
surface is already there, which is the one thing `lies_over` exists to say; it declines because
nothing is near enough to split at. The bulk calls work — 141 splits on the 30° case, 570 on the
tumbling cylinder — so the mechanism is sound and the residue simply has no candidate. That rules
out tuning the tolerances: `boundary_vertices` holds only vertices on the current boundary loops,
so where the facet already present has an **interior** third corner it can never be a candidate at
any tolerance. The pairing has to come from where the facets are laid, or from the tracer.

**And it is the tracer.** `merge_creases` is what would pair two sheets that cross, and it is
exonerated by its own numbers: of the 30° case's **620 rim pairs from different sheets, 2 come
within the snap** of 0.02, 2 within 0.03, 4 within 0.08 — and the stage reports exactly `2 rim
vertices welded`. It welds every pair it can reach; the rims never meet. The labels then say why,
and say it exactly: among that case's traced sheets **only sheet 2 turns inner** (80 of its 120
points, 114 of 190 triangles kept), while sheets 0, 1 and 3 are kept **entire** — 304 of 304, 304
of 304, 646 of 646, not one point inner. Its unpaired loops are bordered by sheets {0, 1}, {1, 4}
and {0, 1, 2}: so where they run between sheets 0 and 1 nothing is clipped, no rim exists, and
`merge_creases` is never asked. Those two sheets have near-mirrored boxes (y out to 3.915 against
−3.915) — two halves of one contact band. It was natural to read that as each sampling their shared
seam independently, and **that reading was measured and refused**: sheets 0 and 1 come within
**4e-9** of one another, with **40 of the second's points already coincident with the first's
inside the weld's coincidence of 1e-7**, and every other pair of that case's sheets the same
(nearest 0 to 2e-9, 40 coincident points each). The tracer emits the seam shared, and `weld`
merges those points into single vertices. So adjacent strips do *not* need teaching to share
their seams; they already do.

That leaves a different shape of defect, and it is recorded here as a candidate rather than a
conclusion, three readings on this front having been overturned in turn: that if both sheets cover
the seam's neighbourhood and their points weld, it is triangulated from **both** — an overlap
rather than a gap, which would fit the declined bands duplicating facets already there and would
point at `clip_overlaps` and `covered_by`. **It was measured and refused too.** Counting, about
each unpaired loop, the pairs of triangles from different sheets whose centroids fall within 0.01
with their normals agreeing: the 30° cylinder has **none at all** on any of its three loops, and
the tumbling cylinder 2, 3 and 4 pairs in neighbourhoods of about 40 triangles and 12 and 18 in
neighbourhoods of 465 and 613 — some 3%, with two further loops at zero. A loop that was the
residue of a doubled patch would have its neighbourhood dominated by doubling, not dusted with it,
and the case where the symptom is clearest has none. That is coincidence between sheets, not a
second covering.

So the chain is zip → split → `merge_creases` → `clip_sheets` → the tracer's seams → the coverage
clip, and **every link named has now been measured and excluded**. What the loops have in common is
not in any of those stages: **they run through vertices where many boundary edges meet**, and that
is measured, not surmised. An ordinary boundary vertex carries two. Counting the boundary edges at
each vertex of each unpaired loop: the 30° case's loops 0 and 2 both pass through **v3272 and
v3308, each carrying four**, while its loop 1 carries two throughout and is a separate, genuine
thing; and the tumbling cylinder has **v271 at six and v195 at eight**, with its loops sharing
them — loop 4 (the 82-vertex seam) and loop 6 share v195 and v4174, loops 0 and 3 share v343 and
v4098, loops 1 and 2 share v270.

A vertex carrying eight boundary edges has four pairs to make, and it was natural to suspect the
walk of choosing among them arbitrarily. **It does not.** `walk_loops`'s `next_after` starts at the
triangle owning the edge it arrived on, rotates round the vertex's fan, and leaves by the first
boundary edge it meets — so the edge it leaves by bounds the *same sector* it arrived in. The
pairing is sector-correct, and the loops are therefore **not artefacts**.

Which settles what they are. A vertex carrying eight boundary edges has **four separate surface
sectors** meeting at one welded point, each with its own boundary corner, and the loops are the
gaps between those sectors. This is where several sheets meet at once — which `chain` keeps
separate on purpose, threading them would carry a sheet through itself — and where the weld fuses
their coincident ends to a single vertex. It accounts for every earlier measurement together: a
band laid there duplicates a facet already present (the walk crosses a fan belonging to another
sheet), no surface is doubled, `unpinch` passes the walks (each visits the junction once, so each
is "simple"), the split finds no candidate, and the certificate refused the quad-diagonal fill as
`Reversed` because the surface between sectors genuinely is not there. It also explains why the
long seams and the small loops are entangled: they pass through the same vertices.

Four sectors meeting at a point is what the plan predicted of 5c — "generators through the motion's
fixed points, which sweep bowties", and rim sweeps that fold "where a rim's tangent runs along its
velocity". So the wide seams' missing surface is the tracer's to produce, and the chain ends in a
positive identification rather than an exhausted list.

**But not all of it is 5c's, and one contradiction is left standing.** Asked of every loop it is
offered, the field calls the 30° case's three-vertex loops **`Spanned`** — boundary does run across
them — and the fill refuses them anyway. It is not laying them by halves: gating the fan
all-or-nothing, as the small-hole fill is, changed **nothing in any case, byte for byte**, and the
thin plate offers the same three-vertex `Spanned` loop on four successive rounds and is refused
each time, a stable standoff rather than a ragged fill. The fan is fanned from a **new** vertex at
the loop's centroid, so no chord of it can already be an edge of the mesh, which left a degenerate
triangle or a loop edge already walked — and **the measurement settles it, in a way worth stating
exactly.**

Every band declined there has an altitude of **exactly zero** while its sides are substantial, and
those sides sum exactly: `[3263,3264,3266]` has 0.038639 + 0.114788 = 0.153428, `[3266,3264,3272]`
0.033998 + 0.119430 = 0.153428, `[3312,1895,1801]` 0.061903 + 0.029533 = 0.091436, and
`[1285,2467,2477]` 0.002942 + 0.092252 = 0.095194. `a + b = c` to the last digit is exact
collinearity, not thinness. So these loops are **zero-width slits**: the boundary walk runs out
along a straight line and back along collinear points.

That reconciles every observation at once. The field says `Spanned` because the span lies *on* real
surface, so it is right. No triangle can fill it, because there is no area to fill. `unpinch`
passes it because no vertex repeats — the return runs through different, collinear points. And the
slit pass's own zipped band is collinear too, so `lay` declines every triangle of it. Nothing here
is a tolerance failing; the geometry is exactly degenerate.

**So such a loop is collapsed, not filled** — but not by welding its arcs together, and it is worth
being exact about why, since the obvious reading is wrong. Reading loop 0's own distances back out
and taking v3263 as the origin along the line, its five vertices sit at **0, 0.0046 (v3272), 0.0386
(v3264), 0.1159 (v3267) and 0.1534 (v3266)** — five *distinct* positions, cross-checked by the sums
(0.004642 + 0.034 = 0.038642, the v3263–v3264 side). The two arcs share **no coincident vertex**,
so a pairwise weld has nothing to pair, and the boundary weld tried earlier could only ever have
merged unrelated pairs within a tolerance — which is what broke the manifold.

What a zero-area collinear loop is, instead, is a **T-junction along a line**: surface meets from
both sides and the two sides sample the shared line at different points. That is
`split_at_vertices`'s own job — and counting which of its guards refuses says exactly where, with
one guess of mine corrected on the way. It is **not** `height/4`: for a collinear candidate `off`
is exactly 0, and `0 <= 0` passes that test even when the owning triangle is itself degenerate. It
is the **span test**, `f <= 0 || f >= 1`, which fires before `off` is ever measured. On the call
that runs once the surviving loops are there, candidates skipped for lying on the edge's line but
past one of its ends are the largest refusal of all: **29** on the 30° cylinder, against 14 with no
candidate, 10 beyond the tolerance and 6 beyond a quarter of the height; and **10122** on the
tumbling cylinder against 830, 2719 and 2031.

That much is right but it is only half, and counting the boundary vertices themselves shows the
other half — correcting the paragraph above, which reduced two populations to one. Of the 30°
case's **11** boundary vertices, **6 lie inside some boundary edge's span** and 4 only on a line
past an end; the 85° case 1 and 1, the slid cylinder 31 and 7, the thin plate 22 and 27, the
tumbling cylinder 191 and 99. (The 360° box reports **0** boundary vertices, which is a clean
control: it really is closed.)

The 4, 7, 27 and 99 **orphans** are as described: only an interior edge contains them, and no
widening reaches that, so taking them would mean letting the split cut interior edges — a change
of its character, not of a threshold.

**And they were taken, guarded.** `weld_boundary_ends` merges a boundary vertex into another within
the junction tolerance, nearest pair first: being past an edge's end puts an orphan within that
tolerance of the **endpoint**, so the two are samplings of one junction and merging them is the
resolution the split cannot reach. What makes it safe where welding by proximity alone was not is
that every merge is *tried and kept only if no directed edge is walked more than once* — the rule
the whole stitch keeps, which the mesh already satisfies, so a merge that would break the manifold
is refused and nothing changes. Measured, with prism, lens and turned box byte-identical throughout
and no over-used, same-way or degenerate edge anywhere: the slid cylinder falls from eleven loops
to **`[5]`**, the thin plate from ten to **`[20, 6, 3, 3]`**, and the tumbling cylinder from 47
loops to **34**, its long seams shrinking from 82 and 64 vertices to **46 and 44**. Boundary
vertices fall 38 → 6, 50 → 30 and 316 → 214.

A merge can also leave a triangle whose corners are distinct but collinear. Dropping those removed
real surface — the tumbling cylinder's volume fell to 6.6304 — so the merge is **refused whole**
instead: leaving a loop open is an honest refusal, closing it by discarding material is not, and a
certified boundary may not quietly shed surface. Refusing cost almost nothing and gained more than
predicted: the volume comes back to **6.6457**, above even the 6.6390 the dropping version started
from, so those drops had been shedding material on merges that fired elsewhere too. The reach
barely moves — the slid cylinder stays at `[5]`, the thin plate improves to **`[20, 5, 3, 3]`**,
and the tumbling cylinder goes from 34 loops to 39 with the same seams of 46 and 44. So the
collapse-instead-of-discard that this note previously called for is **not needed**; refusal is both
simpler and truer.

But the 6 contained vertices of the 30° case are refused in a round that made **no split at all**,
and the guard counts for that round name the culprit exactly: `off` beyond the tolerance 10, beyond
**a quarter of the height 6**, beyond a quarter of the run 3, already walked 3, fold 0, `lies_over`
0. Six refused by `height/4` against six vertices inside a span. So `height/4` *is* implicated
after all — the claim above that it cannot be holds only for *exactly* collinear candidates, where
`off` is 0 and `0 <= 0` passes. For these six `off` is small but nonzero while the owning triangle
is thin, so `off > height/4` fires — and that guard is **right**, since a split on an owner that
thin would fold. Both populations therefore want degenerate geometry removed rather than a
tolerance widened, and they want different removals.

Measuring those refusals says so in numbers, and names the next target. On the 30° case the
candidates the height gate turns away sit **0.0064 to 0.0190** off the line of an edge **0.385**
long — 1.7% to 5% of it, nowhere near rounding — while the owning triangle is a **needle only
0.00625 tall**. So `off` equals or exceeds the owner's entire height, two to three times over in
most of them: such a split really would fold it, and **no floor on `off` is defensible**. What
wants removing is the needle itself. At 0.0063 across it sits *just above* `shortest_edge`
(0.0025), so `collapse_short_edges` never takes it, and `drop_doubled_slivers` only takes a sliver
doubled over another source's edge — a lone needle has nothing in the pipeline to remove it. That
is the successor to `weld_boundary_ends`, and like it the answer is a guarded removal rather than a
widened threshold.

**It is built, as `collapse_needles`, and kept.** A triangle owning a boundary edge whose height
over that edge is under the certificate's own least probe distance — already its definition of a
sliver, one with no normal worth probing along, so no new tolerance is invented — has its apex
merged into the nearer end of the edge. Every collapse is tried and kept only if no directed edge
is then walked more than once and no triangle it moved is left flat: the guard
`weld_boundary_ends` keeps, so it can neither break the manifold nor shed surface, and one that
would is refused.

It **missed the target it was built for.** The 30° case is unchanged at `[5, 3, 5]`, with the same
11 boundary vertices and the same 6 of them inside a span, so the needles were not what held those
six. The gains came elsewhere: the thin plate falls from four loops to **`[22, 3]`**, the tumbling
cylinder from 39 loops to **33**, and the sliding dumbbell from 35402 triangles to 35338 with its
volume unmoved.

A control clears it of shedding material. With the collapse switched off the 360° box comes to a
volume of **20.15572**, against **20.15401** with it — 0.008% — so the 44 needles it removes there
carry essentially nothing, and that case's 1.3% shortfall against the 20.4204 closed form is the
inscribed-mesh deficit the `Inscribed` tier allows rather than anything the collapse caused. The
cost that *is* real: those 44 triangles go, and 11 more of that case's move from certified into
**thin** (230 to 241) — a weaker certificate statement on a case that still closes with none
failed. Prism, lens and turned box stay byte-identical throughout. The wide seams the field calls
`Open` remain 5c's.

Two things inside the tracer are excluded already, and both were nearly mistaken for the cause.
**`trimmed` is a no-op for these fixtures.** It drops a curve's hidden portions on the sampling
grid without computing the crossing, which looks like the very defect — two runs whose facing ends
are different samples a spacing apart, unable to chain — but `hidden` is `|source.value(p)| >
scale*1e-9`, true only where the static field is far from zero, which is to say inside a Boolean
operand. The tilted cylinder is `solid tool(face(bottom, wall, top, axis), about: axis)` and the
thin plate a single prism: **no Boolean, so nothing is ever hidden** and every piece passes
through untouched. And **the sheets are not two halves of one curve**: sheets 0 and 1 are the
cylinder wall's two distinct contact lines (180 points each, boxes mirrored in y), two separate
`Characteristic`s, meeting the rim fans at tool vertices where `chain` keeps pieces apart
deliberately — threading them would carry a sheet through itself. So the seam between them is
real and the sheets are right. Nor do their ends merely lie within `tolerance` of each other: they
are coincident to nanometres and weld into one vertex, as the measurement above shows. Neither the
hidden trim, nor the sheet split, nor the sharing of seams is the defect — which is worth stating
plainly, because each looked like it in turn. The wide seams are a third thing: the plate's 20-vertex
loop spans the tool's whole length and **0 of its 20 vertices have a counterpart within a spacing
four or more steps along the walk**, which corroborates the field's `Open` verdict independently —
surface that was never laid, where no band may be invented.

**The case table is built.** `tests/sweep_mesh/cases.rs` holds `Case { name, source, sagitta,
reference }` and `sweep_cases!`, which makes a `#[test]` of each row; `forms.rs` holds the closed
forms and calls nothing of the construction. `Reference` says once what the suite already
practised: `Exact` (straight-edged, 1e-9 relative) and `Inscribed { volume, least_radius }` (an
inscribed mesh falls short by at most three sagittas per unit of the least radius and never
exceeds the form). Four rows — the box slid along x, the turning prism, the turned box, the turned
lens. A case that does not close yet is not a row: it keeps its own `#[ignore]`d test and its
evidence.

**Still refusing**: the tumbling cylinder (`[82, 64]` and 45 smaller), the thin plate under a 1.05
roll (`[20]` and nine smaller), the 5° cylinder slid along x (eleven, all but one under 0.005
across), and the tilted cylinders at 30° (`[5, 3, 5]`) and 85° (`[3]`). The dumbbell is open at
two four-vertex loops. 5c — the tumbling cylinder's rim folds and its fixed-point bowties — is not
begun.

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

Milestone 5c: folds where a contact curve's tangent runs along its velocity (the tumbling
cylinder's rims), and the bowtie sectors its generators sweep through the motion's fixed
points. Then the coverage the stitch is now known to be waiting on — the sliver loops,
whose two sides must be identified rather than spanned, and the loops bounded by material
already folded — the remaining ★ closed-form cases (ring prism, bored bead, cylinder
screw, diagonal box) as rows of the table, and gap fill (milestone 6). See the plan.

**Gap fill is closed off for these seams, and that was measured rather than assumed.**
`BoundaryOptions::domain` was added so a caller may contour the material within one box instead
of the whole field — the change is inert when unset, and documented in
`docs/field-boundary-extraction.md`, since a given box *is* an arbitrary crop and the caller owns
that judgement. Asked for the boundary inside each unpaired loop's own box (grown by a sagitta so
a patch would overlap the mesh), the extractor **refuses**: three of four loops come back
`AmbiguousPoint`, with enclosures straddling zero (−0.00047 to 0.00050 at
`[1.3766, −2.5221, 1.2654]`), which is the extractor keeping the contract that an interval
containing zero is never a sign. It cannot resolve those neighbourhoods, so there is no licensed
surface to be had there. The fourth loop reached `CellBudget` after **40 seconds** at a
deliberately coarse tolerance of a tenth, so a tolerance fine enough to stitch would cost orders
of magnitude more, on dozens of loops.

So both routes through existing machinery are now closed by measurement: the stitch cannot invent
the surface — the certificate refuses it, as the quad diagonal and the side-to-side sewing each
showed — and the field cannot contour it, because it declines to call the signs. What remains for
5c is generation in the tracer: the sectors a generator sweeps through the motion's fixed points.
The diagnostic that establishes this is
`grazing::what_the_field_yields_in_an_unpaired_loops_own_box`, kept `#[ignore]`d and
non-mutating.

### 5c as a design, to agree or amend (2026-09-12)

Written out because the measurements above constrain it tightly, and because two of the
constraints are not obvious and would otherwise be met the hard way. Nothing of this is built.

**Its premise was then measured and largely fails — read this before building any of it.** The
design below assumes the crowded junctions sit at fixed points of the motion. Asking how far the
motion actually carries them (`creases::whether_the_crowded_junctions_are_fixed_points_of_the_motion`,
`#[ignore]`d and non-mutating) says otherwise: of the degree-six vertices on the tumbling
cylinder's refused loops only **one** is anywhere near still — v56 at `[2.0001, −0.0162, 0.0021]`,
moved at most **0.0085** — while v387 moves **0.5176**, v1984 **0.5994**, v3965 **0.5359** and
v3996 **0.5197**, and the degree-four junctions move as far again. So step 1 below, which finds the
fixed set and splits generators there, would reach at most one of the thirty-three loops.

What the numbers point at instead: 0.517638 recurs *exactly* across several of those vertices. The
displacement is measured from each vertex's own pose over ±30°, so its maximum is 2r·sin 15°, and
r = 0.517638 / (2 sin 15°) = **1.000** — those junctions stand at distance exactly one from the
tumble's axis, not still. (An earlier reading here said "radius ≈0.5176", taking the chord as
2r·sin 30°; that was wrong by the half-angle.)

Radius one is not an accident, and closed form says why. The tool is the profile x ∈ [3,4],
y ∈ [−1,1] revolved about the in-plane line x = 3 — a cylinder of radius 1 whose own axis runs
along **y** — and it is tumbled about the world **x**-axis. Its cap rims are (3 + cos φ, ±1, sin φ),
whose distance from the tumble axis is √(1 + sin²φ): minimised at exactly 1 where φ = 0, π, which
is the four points **(2, ±1, 0)** and **(4, ±1, 0)**. At (4, 1, 0) the rim's tangent is (0, 0, 1)
and its velocity under rotation about x̂ is (0, −z, y) = (0, 0, 1) — **parallel**. That is the fold
condition itself, and every one of the thirteen loops still refused sits there (x ≈ 2 or 4 with
y² + z² ≈ 1). So 5c's step 1 has a measured replacement: **find where a tool edge's tangent runs
along its velocity — equivalently where distance from the motion's axis is stationary along the
edge — and split the edge there**, rather than looking for fixed points of the motion. That is the plan's **other** 5c
phenomenon — a rim's sweep folding where its tangent runs along its velocity — and not a bowtie
through a fixed point. Steps 2 to 4 below (split the generator exactly, emit a patch per side, and
above all share the pinch vertex by identity) and the `directions` trap still hold whatever creates
the pinch; step 1's way of *finding* those places does not.

### Two defects in the stitch, measured and fixed (2026-09-12)

Before any of 5c is built, two faults in `rim_zip` were found by asking which gate refuses each
loop, and they account for most of what the tumbling cylinder was refusing. Neither was in the
tracer, and neither was either of the two causes the note in `zip_round`'s field pass used to name
as "the only causes left".

**A pass committed a loop before laying its band.** The small-hole fill tests its whole fan through
`takes` before committing, but the pairing and the slit pass set `paired[i] = paired[j] = true` and
*then* called `lay`, which declines a triangle silently. A band declined whole therefore consumed
its loops: they never reached the field pass, in that round or in any later one, the pairing being
decided the same way each time. Measured on the tumbling cylinder, **29 of its 33 loops** were
claimed by a pass that laid **nothing**, and only the two wide seams ever reached the field pass at
all. Both passes now commit only on a band that laid something.

**`lay` and `dedupe` disagreed.** `lay` admits a triangle none of whose directed edges is walked,
which a facet wound the other way round three vertices the mesh already carries satisfies — and
`dedupe`, which runs after every round, then drops it again. The loop it had closed came back, so
the band was laid and undone every round and only the count of rounds ended it. `lay` now refuses a
corner set the mesh already has, and the cap on rounds is no longer what ends the loop (it was
being reached with work outstanding). Where this fires the loop is refused honestly: three boundary
edges used once each with a triangle already on those three vertices means that triangle is their
only user, so the loop bounds a **lone facet with no neighbour**, and laying it again the other way
makes a zero-volume flap rather than closing a hole.

Together: the tumbling cylinder goes **33 → 13** refused loops (3112 → 3255 triangles, volume
6.6509 → 6.6563), and the turning prism, turned lens, turned box and sliding dumbbell exports are
**byte-identical** through both changes. The gate is
`creases::no_loop_the_field_calls_a_hole_is_left_unfilled`, and it is a property rather than a
count: no loop the field itself calls `Spanned` may be left open. Ten of the 33 were before; none
of the 13 is now.

**What the 13 are.** Asked with the construction's own tolerances, the field calls **12 of them
`Open`** and refuses the thirteenth with `ReversedNormal`. **Do not read `Open` as "the surface is
genuinely missing" — that is not established, and an earlier draft of this section said it.**
`loop_span` projects each fan centre along the *owner triangle's* normal over a reach of one
sagitta, so `Open` says only that no sign change was found **along that direction within 0.02**.
Surface being absent gives that; so does a probe direction that runs near-tangent to a boundary
that is present. These loops sit at radius exactly 1 from the tumble axis — the fold locus derived
above — which is precisely where an owner's normal can lie along the surface rather than across it.
Which of the two it is decides whether the remaining work is the tracer's at all, and it is
settled by probing those points along other directions, not by argument.

**Probed, the spans are inside the solid.** Every fan centre of all 13 loops reads **strictly
`material`** — 51, 35, 7, 4, 3, 6, 5, 7, 4, 6, 6, 4 and 5 of them, with not one `exterior`, `near`
or `unresolved` — and where another direction does find a boundary it is **0.010 to 0.080 away**
(one to four sagittas), eight loops of the thirteen; the other five find none within four sagittas.
So `loop_span`'s `Open` is right, and for the reason its name gives: material both ways, because
the span is *within* the material. Filling such a loop would lay surface inside the solid, which
the certificate would refuse. **These loops are therefore not gaps in the boundary, and "missing
surface for the tracer to generate" is the wrong diagnosis of them** (an earlier draft of this
section said exactly that).

One alternative is not yet excluded, and it matters: `loop_span` fans from the loop's **centroid**,
and for a curved loop the centroid dips inside a convex surface wherever the loop itself sits —
loop 0 spans 2.81 with an area of 7.8, so its fan may reach well into the solid regardless. If the
loops' own vertices read `near` while their fan centres read `material`, the verdict is an artefact
of the probe's construction and `loop_span` is what needs fixing; if the vertices read `material`
too, the mesh's boundary has run inside the solid and the defect is upstream, in the trim. The
diagnostic `creases::where_the_field_puts_each_open_loops_span` asks both.

**Asked, the answer is *both*, and it partitions the loops — which changes what 5c is.**

- **Loops 0, 1 and 2 lie on the boundary; their fans do not.** Of their own vertices, 41 of 51,
  30 of 35 and 6 of 7 read **`near`** — the boundary passes through them — while every one of their
  fan centres reads `material`. So the loop is a genuine hole and it is the **centroid fan** that
  cuts through the solid: a chord across a loop spanning 2.81 does exactly that. Since `loop_span`
  judges *precisely the triangulation the fill would lay*, its `Open` here is **right about that
  fan and wrong as a statement about the loop**. The fill is correctly refused — those triangles
  would sit inside material and the certificate would reject them — and what is actually needed is
  a triangulation that **follows the surface** rather than chording through it.
- **Loops 3, 4, 5, 6, 7, 10, 11 and 12 are inside the solid for real**: their own vertices read
  mostly or wholly `material` (loop 4, 3 of 3; loop 10, 6 of 6; loops 11 and 12, 4 of 4 and 5 of 5
  with every edge midpoint `material` too) and every edge midpoint is `material`. Those
  boundaries genuinely terminate within the material, which is a defect **upstream in the trim**,
  not a hole to fill and not the tracer's.
- **Loops 8 and 9 are mixed** (1 of 4 and 2 of 6 vertices `material`), so both effects meet there.

**A caveat that may undo the second bullet, and it was measured after the fact.** `Sign::Material`
means only "deeper inside than the judge's near band", which here is a quarter of a sagitta
(0.0025) — while a chord across a curved boundary legitimately lies up to a **whole sagitta**
(0.02) inside it. So a vertex reading `material` does not by itself put the mesh's boundary inside
the solid; it may be an ordinary chord of a curved rim. Asking the same question stage by stage
(`creases::which_stage_first_leaves_a_boundary_edge_inside_the_material`) shows the condition is
present from `clipped`, the earliest meshed stage — 17 of 48 boundary-edge midpoints `material`,
so it is not introduced by the union, the coverage clip, the welds, the split or the zip — but
every depth in that table is **below the sagitta**: 0.0150, 0.0142, 0.0161, 0.0117, 0.0079, 0.0071,
0.0100. That is exactly what chords would give. **So "the boundary has run inside the solid" is not
established, and the depths, not the signs, are what decide it**; both diagnostics now report depth
and count only what lies past a sagitta. Treat the second bullet above as unproven until that
count is non-zero.

**Counted, it is zero everywhere, so the second bullet is withdrawn.** Across all eight stages and
all thirteen loops — vertices and edge midpoints alike — **nothing** lies past a sagitta: the
deepest readings are 0.0023 to 0.0105 against a sagitta of 0.02, most of them under half of it. So
no boundary edge anywhere runs deeper into the solid than its own chord accounts for, and there is
no trim defect to find. What survives is one cause for all thirteen loops, not three:

> **Every one of them lies on the swept boundary within chord error, and every one is refused
> because `loop_span` judges the centroid fan — which chords through the material.** The fan centre
> reads `material` in 100% of cases (51 of 51, 35 of 35, 7 of 7, 4 of 4, 3 of 3, …) while the
> loops' own vertices read `near` or shallowly material. `Open` is right about the fan and wrong
> about the loop.

So milestone 5's remainder is a single piece of work, and it is inside `swept_boundary`: **a fill
whose triangulation follows the surface instead of chording across it**, judged and certified as
any other. Not the tracer's generation, not the trim, not the fixed-point bowtie, not the rim fan —
all four of those were candidates this session and all four were refuted by measurement. The open
design question is only how far the fan must be subdivided: whether moving the apex onto the
boundary suffices, or a loop spanning 2.81 needs its fan refined until every triangle's centroid
reads near. `creases::where_the_field_puts_each_open_loops_span` asks exactly that.

**Asked, a projected apex does not suffice — and the answer sends this back to the probe
direction, which an earlier paragraph here dismissed in error.** For 9 of the 13 loops the centroid
finds no boundary within 8 sagittas along the loop's plane normal; for the 4 that do project, the
fan centres still read `material` — 0 of 7, 1 of 5, 0 of 6, 0 of 6 near — but only **0.0019 to
0.0052** deep. Set that beside the depth table: **nothing anywhere** (loop vertices, edge midpoints
or fan centres) lies deeper than **0.0105**, against a sagitta of 0.02, a certificate probe of 0.04
and a least distance of 0.01. Everything in question is within half a sagitta of the boundary, and
`loop_span` still calls twelve of the thirteen `Open`.

So `Open` is not reporting distance. It reports *no strict sign change along the direction it was
given within reach*, and that direction is the **owner triangle's** normal — which at these rims
runs along the boundary rather than across it, and so cannot bracket however near the surface is.
This is the probe-direction reading that the paragraph above declared refuted, and that refutation
was **wrong**: the alternative directions did find boundaries at 0.010–0.080, which is one to four
sagittas, exactly where a boundary lies relative to a chord — not "far", as it was read.

The candidate follows from the certificate's own rule — **judge the triangle you will lay, along
its own normal**. `loop_span` probes each fan centre along the *owner's* normal; `certify` probes
each triangle along *its own*. That inconsistency is the thing to test, by asking every fan
triangle's own normal whether it brackets.

**Asked, it splits the loops by size, and both halves are now settled.** Along each fan triangle's
own normal:

- **Loops 0 and 1 bracket 0 of 51 and 0 of 35.** No probe direction rescues a chord across a loop
  spanning 2.81: those fan triangles genuinely lie in the material. Since these two carry 14.20 of
  the 14.26 refused area, the **surface-following triangulation** stands as the dominant remaining
  work, confirmed independently of the direction question.
- **Loops 2, 5 and 10 bracket 7 of 7, 5 of 6 and 5 of 6**, where the owner's normal bracketed
  *none*. So the direction is a real defect for small loops, and `loop_span` now judges each fan
  triangle along its own normal, oriented outward by the owner's and falling back to it where the
  fan triangle is degenerate.
- Loops 3, 6, 8 and 9 bracket partially (2/4, 3/5, 2/4, 4/6) and still refuse, since a loop is
  closeable only if the whole of it spans; 11 and 12 bracket none; loop 4's fan is degenerate,
  which is consistent with its being the zero-area slit.

The prediction that change must meet: loops 2, 5 and 10 close (13 → 10) while the partial ones
still refuse, and the other cases stay byte-identical, none of them leaving a loop for the field
pass at all.

**It failed that prediction and was reverted.** With `loop_span` judging each fan triangle along its
own normal the tumbling cylinder kept **all 13** loops — the list merely reshaped — lost **nine
triangles** (3255 → 3246) and moved its volume 6.6563 → 6.6522, its export moving while the other
four cases stayed byte-identical and the suite stayed green. A change that moves a seam, sheds
surface and closes nothing is not justified, so it is out; the rationale and the numbers are kept as
a comment in `loop_span`.

**Why the measurement mispredicted is the lesson worth keeping.** Those per-loop diagnostics read
the **finished** mesh, while `loop_span` runs on the **intermediate** loops of each `rim_zip` round.
A per-loop verdict on the final mesh therefore cannot forecast a change that acts during the rounds
— the same error as the pass-consumption diagnostic earlier in this session, which described
pre-fix behaviour on a post-fix mesh. To predict such a change, instrument the rounds, not the
result.

### What the two wide loops are, measured (2026-09-12)

They carry 14.20 of the 14.26 refused area, so they are the remainder that matters. Asked how far
the other side of each walk lies (`creases::how_the_wide_seams_fold_back_on_themselves`, the
distance from every vertex to the nearest stretch of its own loop four or more steps along):

| loop | vertices | within 0.005 | within a sagitta | within 0.04 | within 0.5 | least | median | most |
|---|---|---|---|---|---|---|---|---|
| 0 | 51 | 0 | 2 | 2 | 30 | 0.0095 | 0.4180 | 1.3976 |
| 1 | 35 | 0 | 0 | 0 | 14 | 0.1884 | 0.7654 | 1.4142 |

**So they are not two samplings of one seam, and welding is not the treatment**: nothing is within
the snap, and for loop 1 nothing is even within the junction tolerance the weld already runs at.
The sides stand half a unit apart at the median. Sewing them invents area, which is exactly what
the earlier side-to-side attempt measured (+0.036 of volume).

Put with the rest, the picture is finally coherent. Their rims lie **on** the boundary (41 of 51
and 30 of 35 vertices `near`); their two sides are 0.19 to 1.4 apart; and a chord across them reads
`material`, though shallowly (nothing past 0.0105). That is the signature of a region where the
true boundary **dips inward around** the gap instead of spanning across it: a chord passes through
solid while the real surface goes the long way. The surface between those sides is therefore
genuinely absent, and generating it is the tracer's — which restores that conclusion for **these
two loops specifically**, on measurement rather than on the `Open` verdict mis-read earlier.

The other eleven loops are small (0.0017–0.019 of area each) and remain what the tables above say:
rims on the boundary whose centroid fans chord through material, with the direction fix refuted.

**One alarm was raised against that and checked away before it cost a run.** The harness reports
those two loops with sides of least 1.7247, middle **2.000000**, longest **2.732051** — the
cylinder's whole length and 1+√3, the full swept extent, since a cap rim reaches y = cos 30° + ½ =
1.366 at the roll's end — and both are bordered by sheet `4294967295`, `u32::MAX`, the zip's own
marker. Read as edge lengths those would be boundary edges 2.0 and 2.73 long in a mesh spaced 0.5,
which would make the loops artefacts of enormous zip bands rather than absent surface. They are
**not** edge lengths: `sides` is the loop's bounding-box extents, sorted (`closed.rs`, `[hi−lo]`
over the three axes), and loop 0's box `[2.0,−1.366,−0.859]..[4.0,1.366,0.866]` gives exactly
2.000, 2.732 and 1.725. So the mesh walks no such edge, nothing implies a giant zip band, and the
measurement above stands. `creases::the_longest_edges_the_mesh_walks` remains as the positive check
that no triangle spans far past the spacing.

The lesson is the cheap one: **ask what a reported number measures before reasoning from it.**
Checking the printer took one grep; the framing it would have licensed was the fifth of this
session, and the four before it each cost a measurement to refute.

**What that check did turn up: coarse fans about the motion's true fixed points.** Of 3255
triangles, 137 have an edge past the spacing and **none** past twice it — so no giant bands. But
the three longest (0.9859, 0.9810, 0.9797, all about twice the spacing) belong to **sheet 6**, all
lie in the plane **x = 2.0**, and all fan out from `[2.0, −0.014, −0.008]` — that is (2, 0, 0),
which lies on the cylinder's wall ((2−3)² + 0² = 1) *and* on the tumble axis, so the motion holds
it **still**. Their far corners reach `[2.0, −0.5, −0.866]`, radius 1 from that axis. Sheet 6 is
therefore a coarse fan across a disc of radius 1 about a fixed point — and sheets 6, 7, 9 and 10
are precisely the ones bordering *both* wide loops.

That fits every other measurement: near a fixed point the sweep is **second-order deep** (the
plan's own third 5c item), so an under-refined fan there has chords lying just inside the surface —
exactly the ≤0.0105 depths measured — and rims no fill can close.

It also forces a nuance on the refutation at the top of this section, and the nuance matters: the
fixed-point premise is wrong **as an explanation of the crowded junctions** (they stand at radius 1
and the motion moves them 0.52–0.60), but the motion's real fixed points — (2,0,0) and (4,0,0),
where the tool's wall crosses the tumble axis — are not irrelevant. They are the centres of these
coarse fans, and the radius-1 junctions are that disc's **rim**. So 5c's step 1 should find fixed
points in order to **refine** the sheets there, not in order to split generators into bowties.
`creases::how_coarse_each_sheet_is_and_how_near_the_fixed_points` is the next measurement.

**Measured, the lead holds for exactly one sheet and fails as a rule — and the failure is the
better clue.** Nearness to a fixed point does not predict coarseness at all:

| sheet | triangles | longest edge | mean edge | nearest fixed point |
|---|---|---|---|---|
| 0 | 8 | 0.4297 | 0.1049 | 0.0274 |
| 1 | 254 | 0.5240 | 0.2212 | 0.9937 |
| 2 | 228 | 0.5086 | 0.1986 | 0.9939 |
| 3 | 279 | 0.5240 | 0.2077 | 0.9937 |
| 4 | 231 | 0.5086 | 0.1999 | 0.9939 |
| **6** | 132 | **0.9859** | 0.2045 | **0.0038** |
| 7 | 120 | 0.5085 | 0.2555 | 0.0187 |
| 9 | 1162 | 0.4865 | 0.0584 | 0.0164 |
| 10 | 346 | 0.5003 | 0.1378 | 0.0164 |
| zip | 495 | 0.6670 | 0.1158 | 0.0038 |

Sheets 0, 6, 7, 9, 10 and the zip all sit on a fixed point, yet **only sheet 6 is coarse**: its
longest edge is twice the spacing while every other sheet is at or under it. So "sheets are
under-refined near a fixed point" is refuted as a general rule and narrowed to one sheet.

**The asymmetry is what to chase, because the fixture is symmetric.** Sheets 6 and 7 are mirror
silhouette patches — boxes `[1.998,−0.5,−1.0]..[2.02,0.5,1.0]` and `[3.98,−0.5,−1.0]..[4.0,0.5,1.0]`
— at the two fixed points (2,0,0) and (4,0,0). Same geometry, same motion, and sheet 7 meshes
cleanly at 0.5085 while sheet 6 fans off its fixed point at 0.9859. In a symmetric fixture a
one-sided defect has a cause, and comparing those two sheets column by column in the tracer is the
next measurement: a strip whose columns are sampled unevenly zips into long rungs, and that is what
`zip_pieces` would produce from a column the simplification thinned differently at one end.

**Measured, the tracer is exonerated, and the same reading names a better suspect.** Sheets 6 and 7
are structurally *identical*: both 54 points in **9 columns of 6**, largest jump between
neighbouring columns **0**, the same times (−0.5236 to 0.5236), mirror boxes. No column is thinned
differently, so sheet 6's long edges are made downstream of the tracer's columns, not within them —
the uneven-column hypothesis is refuted.

What the same line shows instead: **sheet 6's box is degenerate in x**, `[2.0,−0.5,−1.0] ..
[2.0,0.5,1.0]` — all 54 of its points lie at x = 2.0 exactly, so the patch is **flat**. Its long
edges join two of its own points straight across it (`[2.0,−0.5,−0.866]` to `[2.0,−0.014,−0.008]`
is 0.986). A coplanar patch on an exact plane is what **`planar_union`** retriangulates, x = 2.0 is
where that sheet, the caps' facets and any grazing region all coincide, and both wide loops' boxes
begin at exactly x = 2.0. So the suspect is the planar union's retriangulation.

The test is cheap and now built into the stage walk: **the longest edge per stage, with the sheet
that laid it.** If the longest edge is within the spacing at `clipped` and jumps at `unioned`, the
planar union made it; if it is already long at `clipped`, it came in with the seeds and the union is
innocent too.

**Measured, it is neither: the jump is in the zip, and on a triangle that already existed.** The
longest edge is **0.5189 (sheet 1)** at `clipped`, `merged`, `kept`, `unioned`, `uncovered` and
`welded`, 0.5086 at `split`, and **0.9859 by sheet 6** only at **`zipped`**. So the planar union is
exonerated and so are the seeds.

The attribution turns on one detail: that triangle still carries **sheet 6**, not `u32::MAX`.
`zip_round`'s `lay` stamps every triangle it pushes with `u32::MAX`, so a newly laid band would say
"the zip". A triangle still labelled with its original sheet whose edge has grown means an
**existing** triangle was *rewritten* — and the only passes that rewrite corners while keeping
sheet ids are `weld_boundary_ends` and `collapse_needles`, which merge one vertex into another.
Both were added in this session, and both guard against a non-manifold result and against
flattening a triangle they move — **neither guards against stretching one.** A weld moves a vertex
at most the junction tolerance (0.04), so a single weld cannot turn 0.52 into 0.99; `collapse_needles`
moves an apex to the nearer end of the boundary edge it owns, which is bounded by that edge and not
by `thin`, and both passes run up to eight rounds, so the stretch can compound. Which of the two
owns it is settled by disabling each in turn and re-reading this line — not by argument.

**Disabled in turn, `collapse_needles` owns it outright**: 0.9859 with both passes on, **0.5330
with the needle collapse off**, and 0.9964 with the weld off instead. So the needle collapse is
stretching an edge to twice the spacing, and it has no guard against that — it refuses a merge that
would break the manifold or flatten a triangle it moves, and says nothing about lengthening one.
The fix is that third guard, bounded by `rim_zip`'s own `within` (the spacing), so no new tolerance
is invented: **a collapse may not stretch an edge past the mesh's sampling scale.** Deleting the
pass is not the fix — it was measured to take this case 39 → 33 loops and the thin plate to
`[22, 3]`, so it earns its place and needs the bound, not removal.

**With the guard in, the tumbling cylinder goes 13 → 7 loops** (`[42, 44, 7, 5, 4, 4, 6]`), 3255 →
3288 triangles, volume 6.6563 → 6.6540, and the `zipped` longest edge falls **0.9859 → 0.6670** —
what remains is the zip's own rung rather than a rewritten sheet triangle. The turning prism,
turned lens and turned box stay **byte-identical**; the **sliding dumbbell moves** (35338 → 35370
triangles, its loops still `[4, 4]`, volume unchanged at 9.5277), so it is a moved seam to name.
The suite stays green at 1209.

Cumulatively, three stitch fixes this session take this case **33 → 21 → 13 → 7** loops: the
pairing and slit passes committing a loop before laying its band (pre-existing), `lay` disagreeing
with `dedupe` (pre-existing), and this stretch guard (a defect in one of the session's own new
passes). What still refuses is the two wide seams, back at 42 and 44 vertices, and five small
loops.

**A third moved seam, and it is coherent: the whole-turn box.** It still closes and certifies —
1990 → 2012 triangles, 1749 → 1782 certified, 241 → 230 thin, **0 failed, 0 loops left** — and its
volume moves 20.15401 → **20.15572**, which is *exactly* the "collapse off" control recorded for
that case earlier in this project. That is what a guard making the collapse less aggressive should
give, so the number is explained rather than merely observed. Any commit of this work names three
moved seams: the tumbling cylinder, the sliding dumbbell and the whole-turn box.

**The seven survivors, measured after the guard** (the thirteen-loop tables earlier in this section
describe the state *before* it): none encloses zero area; the field calls **six `Open`** and
refuses the seventh with `ReversedNormal`. The two wide seams are 42 and 44 vertices with areas
6.924 and 6.710 — **13.63 of the 13.67** total refused area — and the five small loops run
0.0028 to 0.018 each. So the shape of the remainder is unchanged by the guard: two large seams
carrying almost all of it, which is where milestone 5's remaining work sits.

### The zip's winding: a 52% figure that is probably an artefact of the wrong probe rule (2026-09-12)

**Read the correction at the end of this section before using any number in it.** The table below
was produced by asking `sides(centroid, n, probe)` once at the **full** probe of 2·sagitta = 0.04
and calling `(Exterior, Material)` reversed. `certify` does not do that: it **halves the probe down
to the least distance for thin material** and reports those triangles as `thin`, not as failures.
Where material is thinner than twice the probe, a probe inside exits the far side and one outside
enters other material, so a perfectly outward triangle reads `(Exterior, Material)`.

What exposed it was a contradiction, not a hunch: of the 243 triangles this called reversed, **235
share an edge, walked oppositely, with an outward triangle, and 0 are folds** — and two triangles
sharing an edge in opposite directions are consistently oriented, so if one is outward the other
must be. 235 impossibilities mean the test was wrong, not the mesh. The zip topping the table is
then exactly what thinness predicts, since it lays bands in the narrowest seam regions; the
whole-turn box already reports 230 `thin` of 2012.

**Re-measured under `certify`'s rule, the retraction is right in magnitude but was too total.**
Halving the probe from 0.04 down to the least distance 0.01 before calling a triangle reversed
gives **2920 outward (unchanged), 101 reversed, 267 undecided** — against 2920 / 278 / 90 at the
full probe, so exactly the 177 that stopped reading reversed became **undecided at the shortest
probe**, which is the signature of thin material. For the zip alone: 243 → **81 reversed**, with
209 of its 471 undecided, i.e. 17% reversed rather than 52%.

**But 101 triangles stay reversed at the shortest probe, and three of them matter.** On sheet 6
they sit at `[2.0, 0.0156, 0.0907]` and `[2.0, −0.0294, 0.0901]` with normals `[0.999, 0.052,
−0.018]` and `[1.0, 0.009, 0.002]` — pointing **+x at the plane x = 2.0**, where the solid lies at
x > 2 and outward is therefore −x. That is the `ReversedNormal` loop's own neighbourhood, and it
survives every probe length, so a genuine winding defect does exist on that flat patch. It is far
smaller than the retracted claim and it is real.

**Recomputed under the halving rule, the contradiction persists — and that is what finally shows
the whole finding to be mine, not the mesh's.** Of the 81 reversed zip triangles, **79 share an
edge, walked oppositely, with an outward one, and 0 are folds**. I had called that impossible on
the grounds that consistency plus an outward neighbour forces outwardness. That inference is
**false**: consistency forces the two triangles' *orientation* to agree, and says nothing about
what the **field reports at each triangle's own centroid** — a separate measurement, taken at a
different point along a different normal. In thin or sharply folded material those two readings can
disagree while the winding is perfectly consistent. So 79-of-81 is not an impossibility; it is the
signature of thin material, and it matches the 209 of 471 undecided exactly.

**The last seemingly-real case dissolves the same way.** The three sheet-6 triangles with +x
normals sit at **x = 2.0 exactly** — the silhouette plane, where the swept solid is only
*second-order* deep (the third of the plan's 5c phenomena). At a tangency the material is
quadratically thin, so no two-sided probe there can give a clean answer whatever the winding is,
and the `ReversedNormal` loop lies in that same neighbourhood. Loop 2 is therefore the seventh loop
of a kind, not a defect.

**Conclusion: there is no winding defect.** The 101 "reversed" triangles are what probing
quadratically-thin material returns, which is exactly the case `certify` reports as `thin` rather
than as a failure — and the closing cases, which do run the certificate, report 0 failed. The whole
claim in this section was an artefact twice over: first of the wrong probe rule, then of a
topological fact used to constrain a field measurement. **Both records of it are retracted in
full.** The remaining work is unchanged: the 7 loops above.

### What surface the remainder actually is, in closed form (2026-09-12)

The caps are planar faces whose plane runs **parallel** to the tumble axis at distance 1, and the
envelope of a plane rotating about a parallel axis is a **cylinder of that radius**. So the swept
boundary must contain a patch of **y² + z² = 1** about the x-axis. The contact condition says the
same thing from the other side: for a cap normal `n` and a point `p`, the velocity about x̂ is
`ω(0, −z, y)`, so `n·v = 0` selects a **diameter line** on the cap, and sweeping that line traces
precisely that cylinder.

**The tracer already produces it.** Sheets 5 and 8 have boxes `[2.0,−0.5,0.866]..[4.0,0.5,1.0]` and
`[2.0,−0.5,−1.0]..[4.0,0.5,−0.866]` — exactly `(x, −sin t, cos t)` over t ∈ [−30°, 30°], the 60°
arc the declared roll sweeps. That also explains two numbers measured earlier from the other
direction: the crowded junctions stand at radius **exactly 1** from the tumble axis (their
displacement 0.517638 = 2·sin 15°), and every small refused loop sits at radius ≈1 with
|z| ≈ 0.95–1.0 — which is the **edge of that arc**.

So the remainder is most likely **not absent surface at all**: it is the junction between an
envelope patch that *is* traced and its neighbours — the wall sheets (1–4) and the silhouette
patches (6, 7) — where a crease should be merged. That is machinery the pipeline already has in
`merge_creases`, and it is a very different piece of work from generating a surface. It is also
testable: `creases::whether_the_refused_loops_lie_on_the_cap_planes_envelope` measures how far each
refused loop's vertices stand from `y² + z² = 1` and where they sit along the arc. Vertices on that
cylinder, clustered at the arc's ends, confirm it; vertices far from it refute it.

**Measured, it explains the small loops and not the two seams** — the split called in advance from
their z-spans:

| loop | vertices | within a sagitta of y²+z²=1 | median off | at \|z\| ≥ cos 30° |
|---|---|---|---|---|
| 0 | 42 | 8 | **0.4142** | 8 |
| 1 | 44 | 6 | **0.5000** | 6 |
| 2 | 7 | 0 | 0.8834 | 0 |
| 3 | 5 | 4 | 0.0138 | 5 |
| 4 | 4 | 4 | 0.0085 | 4 |
| 5 | 4 | 2 | 0.0363 | 4 |
| 6 | 6 | 4 | 0.0196 | 6 |

Loops 3, 4 and 6 lie **on** the envelope cylinder, wholly inside the arc's band, and loop 5 lies in
the band if not quite on the surface. So those four are the junction between the traced envelope
patch (sheets 5 and 8) and its neighbours — a **crease** to merge, not a surface to generate. Loop
2 stands 0.78 or more off it, consistent with its being the silhouette tangency at x = 2 rather
than an envelope loop at all.

The two wide seams are a different thing, and their medians are exact rather than noisy:
**0.4142 = √2 − 1** and **0.5000**. Radius √2 is the cap rim's farthest point from the tumble axis
— for a rim point `(3+cos φ, sin φ, 1)` the distance is `√(sin²φ + 1)`, maximal √2 at φ = ±90° — so
these loops run from radius 1 out to ≈2, spanning the whole swept extent. They carry 13.63 of the
13.67 refused area and remain unexplained.

**Two further readings from the same case, and the second is the better lead.**

First, the crease machinery is **not** idle here: this case reports `45 rims clipped; 136 rim
vertices welded, 27 rim edges split`, against the sliding dumbbell's 0/0/0. So "the junction is
never attempted" does not explain the four envelope loops — `merge_creases` is being offered those
rims and is welding 136 vertices.

Second, and unprompted: the project's own `hygiene` check reports **"15 edges walked twice the same
way"** at `clipped` — the *earliest meshed stage* — and still at `merged`, alongside
`v1916 and v3397 are 0.000210757 apart, inside the snap 0.005`. An edge walked twice in the *same*
direction is a **fold**, and it is exactly what `hygiene` exists to name: the first stage that hands
on a defect. It has been sitting in this case's output all along while this section measured around
it.

**Across the stages it reports, though, the fold count mostly heals itself:** 15 at `clipped`,
`merged` and `kept`, falling to **4** at `unioned` and staying 4 at `without overlaps`. So the
planar union rebuilds eleven of the fifteen, which is what retriangulating coplanar fragments
should do, and only four persist. That is a far smaller defect than "15 folds from the first meshed
stage" suggests, and the record should not claim more.

**And there is a reporting gap worth closing rather than a defect worth chasing:** the harness
prints `hygiene` for only those five stages — never for `welded`, `split` or `zipped` — so whether
those four folds reach the boundary the zip has to close has never been measured. The stage walk
now counts folds and over-used edges itself at all eight stages, which needs no field queries at
all.

**Counted at all eight, the folds are gone before the zip ever runs:** 15 at `clipped`, `merged`
and `kept`, 4 at `unioned` and `uncovered`, then **0 at `welded`, `split` and `zipped`** — and **0
over-used at every stage**. The `welded` stage clears the last four. So folds are not implicated in
the seven refused loops, and this lead closes.

That leaves the stitch measurably clean on every fault that can be named of it: no folds, no
over-used edges, no reversed winding, no boundary lying deeper inside than its own chord explains,
and no edge longer than the zip's own rungs. What remains is the seven loops — four on the envelope
cylinder's arc edge, one the silhouette tangency, and the two wide seams holding 13.63 of the 13.67
refused area.

**A note on one measurement that looked like an answer and was not.** Asking the least gap between
two bordering sheets near each loop returned **0.0000 for every pair of every loop** — which is an
artefact of the instrument, not a fact about the mesh. It gathered each vertex under *every* sheet
owning a triangle at it, so a vertex **shared** between two sheets fell into both lists; `weld` has
already merged the coincident rims, so a shared vertex always exists near a loop and the minimum
distance was zero by construction. The question it actually asked was "do these two sheets share a
vertex here", which answers itself. Counting a vertex only for its **sole** owner makes the number
the real distance between the two patches' own material; until that is re-run, nothing about
whether these patches abut has been established.

**Re-run with that fix, the patches abut — so the crease explanation is refuted too.** The gaps come
back at **0.0056 to 0.1022**, which is one local edge length (sheet mean edges run 0.06–0.26, the
caps finer), and nothing approaches the ≫0.4 that would mark a coverage hole. The reading rule was
fixed before the numbers were seen, precisely because excluding shared vertices means two
*perfectly* abutting patches must still show a gap of about one edge: the nearest points of an
abutting pair are the shared vertices that were removed. So these are abutting, already welded
patches, and the four envelope loops are not an unjoined crease.

**What that leaves, and the one instrument still missing.** Every structural fault nameable of this
mesh has now been measured and excluded: no folds, no over-used edges, no reversed winding, no
boundary deeper inside than its own chord, no over-long edges, no gaps between abutting patches.
Every one of those asked *what is wrong with the mesh that exists*. None asked **where the boundary
has no mesh at all** — which is the question milestone 5 actually turns on, and which the field can
answer directly: walk a grid, bisect every adjacent pair whose signs differ onto the boundary, and
measure each boundary point's distance to the nearest mesh vertex. A covered point lies within half
an edge, about 0.1 here; a cluster standing far off *is* the missing surface, located rather than
inferred. `creases::where_the_boundary_has_no_mesh` does that.

**Run, it gives a weak negative and one located candidate — and the weakness matters more than the
negative.** Of the boundary points it found: **0 beyond 0.3, 0 beyond 0.5**, median 0.0850, most
0.2460 — about two grid cells, i.e. the instrument's own resolution. By the rule fixed beforehand
that reads as "nothing gross is missing". **But it found only 80 boundary points from roughly
22,800 adjacent pairs**, because a pair is usable only when *both* ends give a strict sign, and any
grid point inside the judge's band reads `Near`. So the negative rests on a thin, possibly
unrepresentative sample, and it must not be promoted into "the mesh covers the whole boundary".
The instrument now reports how many points fell in the band and how many pairs were discarded, so
the thinness is visible in the output rather than inferred from it.

**The candidate it did locate is not random.** The five farthest points are
`[2.36, 0.926, ±0.077]` and `[3.64, −0.926, ±0.077]` — symmetric, every one at radius ≈**0.929**
from the tumble axis, so **on the cap-plane envelope cylinder** — but at **|z| ≈ 0.08**, in the
*middle* of its arc, whereas sheets 5 and 8 cover that cylinder only where |z| ≥ 0.866. That is the
first directly located candidate for absent surface in this whole investigation, and it falls in
exactly the family the closed-form derivation predicted while lying outside the part the tracer
covers. It is at the resolution limit of a 20³ grid, so the next step is a finer grid confined to
that neighbourhood.

**The reported counts then gave a sharper reason than "thin", and it changes the plan.** Discards
were **4408 of ~22,800 pairs (19%)** and band points **936 of 8000 (12%)** — so 81% of pairs were
usable, and the sample was not mostly thrown away. Yet 80 crossings is far below the ~1400 a
surface of this area should cross a 20³ grid (cells of 0.12): `deep_sign` returns a strict sign
only **0.04 clear** of the surface, while a crossing pair usually has an end within 0.06 of it. So
the surviving pairs are those where the surface passes near a cell's **middle** — the sample is
**biased**, not merely thin, and a finer grid would sharpen the bias rather than remove it.

**So the grid is dropped in favour of sampling the candidate surface itself**, which is known in
closed form: `y² + z² = 1` about the tumble axis, walked right round θ at several x. No cells, no
band bias, a few hundred points. An arc of θ where the field says boundary while the mesh stands
far off is the missing surface, located analytically rather than stumbled on.
`creases::whether_the_envelope_cylinder_is_meshed_round_its_arc` does that, and it supersedes the
finer-grid step suggested above.

**Walked right round, it locates the gap — and its first labels repeated a mistake made earlier in
this very section.** Sampling `y²+z²=1` at θ every 15° and x = 2.5, 3.0, 3.5:

- **θ ∈ [60°,120°] and [240°,300°]**: the nearest mesh vertex is **0.000** at all three x. The mesh
  lies *on* the cylinder over a 60° arc each side — exactly the roll's span, and exactly sheets 5
  and 8 covering |z| = |sin θ| ≥ 0.866. The envelope patch is there and is meshed.
- **θ = 0° and 180°**: the field says **`on`** — the cylinder *is* the boundary — while the nearest
  mesh vertex stands **0.131 to 0.150** away. **That is uncovered boundary**, and it falls where the
  grid audit's farthest points already sat (y ≈ 0.926, z ≈ 0.077, i.e. θ ≈ 4.8°). Two unrelated
  instruments agree on the same place, which is the first such agreement in this investigation.
- θ = 0° is (y, z) = (1, 0), which is the **cap plane's own position at t = 0**. So the gap lies
  exactly where the envelope cylinder meets the **end-pose cap**: a narrow crease band, not a large
  missing patch.

The mistake to note, because it is the second of its kind here: the other rows printed "in", which
looks like "the cylinder is interior", but `sign` returns `Near` only within a band of **0.0025**,
so a point half a thousandth inside reads `Material`. The rows at θ = 60°–120° prove it — a vertex
0.000 away, labelled "in". Reading a sign as a position is exactly what made 278 thin triangles
look reversed earlier in this section. The instrument now prints the **depth**, so the θ = 15°–45°
rows can be believed or discarded rather than guessed at.

**Printed, they are not depths either — and that invalidates a conclusion recorded earlier in this
section.** The numbers come back **quantised**: 0.1340 and 0.2929, repeating at nearly every θ and
keyed to x rather than position — and 0.1340 = 1 − √3/2, 0.2929 = 1 − √2/2. Those are exact *field
values*, not distances. Decisively, at θ = 60°–120° a mesh vertex sits **0.000** away while the row
reads 0.1340, and a point cannot be 0.134 inside with a boundary vertex on top of it.

So `-enclosure[1]` is the field-value enclosure, which is what CLAUDE.md warns of: Boolean fields
"are not necessarily signed distances". Being one-Lipschitz, `|f| ≤ distance`, so the number is a
**lower bound** on depth. **Therefore the earlier claim in this section — "nothing lies deeper than
0.0105, so nothing is deeper than a chord explains" — is too strong**: small lower bounds do not
establish small depths. What was actually measured is that the *certified lower bounds* are small,
which is a weaker statement, and the "boundary inside the solid" question is reopened to that
extent.

**And one contradiction is now explicit rather than resolved.** If `f = −0.134` where a mesh vertex
lies, that vertex stands at least 0.134 from the true boundary — gross rather than chordal, and at
odds with the certificate passing on the closing cases. Either the enclosure is still being
misread, or those vertices really are off the surface.
`creases::what_the_field_reports_at_the_meshs_own_vertices` asks the vertices directly, which
separates the two.

**The lesson, since this is the third of its kind here:** the field's output has now been misread
three times in one investigation — a sign taken for a position, a sign taken for a depth, and a
value taken for a depth. Before reasoning from any number this field returns, establish what the
number *is*.

One precision, so the record does not overstate it: this is **not** a contradiction of CLAUDE.md's
"every edge used more than twice is now gone from every case". A same-way pair is exactly *two*
uses, both forward, which `hygiene` flags as its own fault class. Both statements can hold at once,
and they do.

The one loop the field refuses outright does so with `ReversedNormal` at [2.000337, 0.0887,
−0.1432] along very nearly **+x** — and at the plane x = 2 the solid lies at x > 2, so outward
there is −x. That is a triangle wound into the material, not a probe subtlety. Asking the
certificate's own question of **every** triangle (`sides(centroid, n, probe)` is `(Material,
Exterior)` outward and `(Exterior, Material)` reversed) gives:

| owner | outward | reversed | undecided |
|---|---|---|---|
| sheet 0 | 7 | 1 | 0 |
| sheet 1 | 241 | 1 | 16 |
| sheet 2 | 222 | 3 | 3 |
| sheet 3 | 265 | 4 | 16 |
| sheet 4 | 221 | 5 | 5 |
| sheet 6 | 129 | 3 | 0 |
| sheet 7 | 116 | 4 | 0 |
| sheet 9 | 1153 | 6 | 3 |
| sheet 10 | 338 | 8 | 0 |
| **the zip** | **228** | **243** | 47 |
| total | 2920 | 278 | 90 |

Every traced sheet is 1–3% reversed; **the zip is 52%**, which is a coin toss. So `rim_zip`'s
`wind` produces a *manifold-consistent* winding and says nothing about **outwardness**.

Why this was never reported: the closing cases all certify with **0 failed**, and
`Failure::Reversed` is one of the certificate's own verdicts, so their windings are sound. This
appears only where the zip does heavy work *and* the case refuses at `UnpairedRim` — before
`certify` ever runs.

**What not to conclude.** `wind`'s rule — walk a shared boundary edge opposite to the way its owner
walks it — *is* the standard consistency rule, and it inherits the owner's outwardness whenever the
owner is outward, which the sheets are 97–99% of the time. A 52% failure rate therefore is not that
rule failing; the specific suspect is its `unwrap_or(false)`, which leaves a triangle with **no**
edge on any boundary loop unflipped, i.e. arbitrarily wound. That is measurable, and it is measured
before anything is changed — the last speculative fix here cost a revert.

**That measurement came back null, by construction.** Replaying the passes cannot reach it: on the
finished mesh **no pass takes any of the seven loops**, so `lay` never runs and the fall-through is
never exercised. The suspect is neither confirmed nor cleared — the same staleness that mispredicted
the `loop_span` change, since these replications read the finished mesh while the bands were laid
in earlier rounds against different meshes.

**The test that needs neither replay nor theory** is how each reversed zip triangle sits against
its neighbours on the finished mesh. Consistency plus an outward neighbour *forces* outwardness, so
for a reversed triangle exactly one of four things holds, and counting them pins the mechanism
rather than narrowing suspects:

1. it shares an edge, walked oppositely, with an **outward** triangle — a contradiction, meaning
   the winding is not actually consistent there;
2. its only such neighbours are themselves **reversed** — propagation from a single bad seed;
3. some edge is walked the **same way** by another triangle — a fold;
4. some edge is walked by nothing else — it sits on the boundary and inherited nothing.

`creases::how_the_zips_reversed_triangles_sit_against_their_neighbours` asks exactly that.

The weighting matters: loops 0 and 1 carry **14.20 of the 14.26** total refused area. So the
dominant remainder is *not* surface the tracer must generate — it is two large holes lying on the
boundary that no centroid fan can close. That supersedes 5c's step 1 again, and it moves the work
back inside `swept_boundary`: a surface-following fill for a wide loop, and a trim that does not
leave a boundary inside the material. Neither is the bowtie, and neither is the rim fan.

**One tracer hypothesis already refuted by experiment.** `edge_strands` keeps a contact only where
`in_fan` holds — `a*b <= 0 && (a != 0 || b != 0)` over the two faces' normal-velocity classes — and
on the cap rim (3 + cos φ, 1, sin φ) the wall normal (cos φ, 0, sin φ) and the cap normal (0,1,0)
give normal velocities of exactly **+ω sin φ** and **−ω sin φ**. They are opposite-signed
everywhere and vanish *together* only at φ = 0, π, the fold points, where `in_fan` is false by the
clause written for an edge that slides along itself. That argument is clean and it is **wrong**:
admitting both-zero into the fan was tried and changed nothing at all — the tumbling cylinder came
back identical to the triangle (3255, the same 13 loops, volume 6.6563) and all five exports were
unmoved. So the rim's fan is not what cuts that surface. Reading the rule is not measuring it. None is `Spanned`, none encloses zero area, and none is consumed by a pass. So
every remaining loop on this case is surface for the tracer to *generate*, which is 5c, plus one
reversed-normal query to account for. The instruments are the four `#[ignore]`d diagnostics in
`creases.rs`: `whether_the_refused_loops_enclose_any_area`, `which_gate_refuses_each_unfilled_loop`,
`what_the_field_says_of_each_unfilled_loop` and `which_pass_consumes_each_loop_the_field_would_fill`.

**What must be produced.** The surface a generator sweeps as the motion carries it through a
fixed point: the bowtie. It shows in the mesh as a vertex of boundary degree 6 or 8 — v271 and
v195 on the tumbling cylinder — where four sectors meet at a point, and the loops between those
sectors are what the construction refuses.

**Why it must be a seed and cannot be recovered later.** The field reads those loops `Open`, so
nothing downstream may span them; and asked to contour their neighbourhoods it answers
`AmbiguousPoint`. Both ends are closed, so the surface has to enter as traced geometry.

**The construction.**
1. Find the motion's fixed points — where `Family::inverse_point_speed_bound_over` is zero across
   the whole declared interval. A rotation gives its axis, a screw none, the tumble the meeting of
   its two axes. Any motion whose fixed set is neither empty, a point, nor a line is refused by
   name rather than approximated.
2. Split every generator whose posed path passes within a sagitta of a fixed point **at** that
   point, so the fixed point is an exact column point and not a sampled near-miss.
3. Emit one patch per side of the split, its columns the generator's images at the traced
   parameters. Near the fixed point those columns collapse toward it, which is the pinch. Wind
   each side by its own normal-velocity sign, as `orientation` already does at a fold.
4. **The pinch vertex is one vertex, shared by identity between the two sides** — not two
   coincident ones. Every false boundary chased in this milestone came from two samplings of one
   place meeting at different vertices; emitting the pinch twice would manufacture the same
   defect at the hardest point in the mesh.

**The trap to expect.** A bowtie's halves face opposite ways, so at the pinch the sum of incident
normals is nearly zero — and `directions` judges a vertex along exactly that sum. The vertex
would read unresolved however much budget it is given. It needs its direction from **one** side,
which is the same rule milestone 1 already records for a tool edge, where one face's own normal
runs tangent to the other and reads zero forever.

**The gate.** The tumbling cylinder closes and certifies, its volume holding against sampled
membership; the eight closed-form cases, the turning prism, the turned lens and the turned box
stay byte-identical; and `seeds_moved_below_every_tolerance_leave_the_mesh_as_it_was` covers the
new patches, since a pinch is exactly where a knife-edge decision would hide.

### Rendered at last, and it resets where milestone 5 stands (2026-09-12)

The gate above is this design's acceptance criterion, not a report: as of this section the tumbling
cylinder refuses. What follows is the current state, and it came from **looking at the geometry** —
which nothing in this document had done until now.

**Draw it.** `creases::draw_the_refused_loops` writes an SVG of the mesh with every unpaired loop
in its own colour over it, in three orthographic views, and headless Chrome turns that into a
picture. For the STL itself, the repo's own dependency does the work: symlink
`web/node_modules/three` beside a small ES-module page using `STLLoader` with an import map, serve
it with `python3 -m http.server` (ES modules will **not** load over `file://`), and screenshot with
`--enable-unsafe-swiftshader --virtual-time-budget=20000`. Nothing else on this machine renders an
STL — no openscad, f3d, blender, meshlab, assimp, numpy-stl, matplotlib or trimesh, and `qlmanage`
hangs on one.

**With a negative control, because a single picture is still one instrument.** Through the
identical pipeline the **turned box** — which closes and certifies — renders as a clean closed
solid in all four views, crisp faces, nothing see-through. The **tumbling cylinder** renders as
**two separated bowls with a gap you can look straight through**, with cone-like fans converging on
the axis. Same material and `DoubleSide` in both, so it is not a backface artefact: that mesh is
missing its whole **equatorial band**.

**So "seven loops left" badly understates it.** Two of those loops bound one enormous missing
region — measured area **13.63** against a solid volume of 6.65. The stitch is measurably clean by
now (no folds, no over-used edges, no reversed winding, no boundary deeper than its own chord, no
gaps between abutting patches); what is absent is a large fraction of the boundary. **Do not read
the loop count as a progress bar.**

**And the loops' apexes are on the motion's fixed points.** Measured rather than seen, the nearest
approach of each loop to the tumble axis is:

| loop | nearest the axis | where |
|---|---|---|
| 0 | **0.0164** | [2.0, −0.016, 0.002] |
| 1 | **0.0164** | [2.0, −0.016, 0.002] |
| 2 | **0.0164** | [2.0, −0.016, 0.002] |
| 3 | 1.0000 | [2.0, 0.131, 0.991] |
| 4 | 1.0000 | [2.0, 0.131, 0.991] |
| 5 | 1.0000 | [4.0, −0.383, 0.924] |
| 6 | 1.0000 | [4.0, −0.383, −0.924] |

**Three loops share one apex vertex**, at x = 2.0 exactly and 0.0164 from the axis — the point
(2, 0, 0), where the tumble axis pierces the tool's wall and the motion holds it still. That also
settles an ambiguity this document read the wrong way earlier: `|r − 1|` of "most 0.9836" admits
radius 1.9836 *or* 0.0164, and it is the latter. The other four loops sit at radius exactly 1, on
the envelope cylinder, pairing at a shared vertex at each end.

**This substantially vindicates 5c's step 1 and narrows the refutation recorded above.** The design
says to find the motion's fixed points and split the generators passing near them. The refutation
holds only for the *crowded junctions* — those stand at radius 1 and the motion moves them
0.52–0.60, so they are not fixed points. But the loops' apexes are, and three refused loops meet
there. That is the plan's "bowtie sectors through fixed points", almost as written.

**The instrument lesson, plainly:** seven hours of statistics in this section did not show any of
this, and one render did. Draw the geometry first.

### A rough reference surface, and the number it puts on all of this (2026-09-12)

`tests/sweep_mesh/reference.rs` contours the field on a uniform grid and writes an STL
(`rough_reference_surfaces`, `#[ignore]`d, output to `SOLVENT_REFERENCE`). It is **not** the
certified extractor and claims nothing: `MaterialEvaluator::boundary` refuses on an ambiguous point
or a spent budget, exactly as it should, which is why it cannot draw a picture. This one cannot
refuse. It is blind to anything thinner than a cell — and this project's hard cases are the thin
ones — so **never assert on its surface**; read its volume and look at its shape.

**It reads signs, never values, and the control is what taught that.** The first version sampled the
midpoint of the field's value enclosure as a scalar and contoured the zero set. It gave the turned
box **15.8989**, then **15.8632** after the band was tightened, against a closed form of
**15.3237** — and located a box spanning 6.4 units in z where the object is 2 tall, having found
"material" far outside the solid. The cause is that for a sweep the enclosure is a **conservative
bracket, not an approximate value**: a point well outside can come back as [−20, +0.1], whose
midpoint is negative. Tightening the band changed nothing because the band was never the cause.
That was the fourth time in one session that a conservative interval was read as an approximate
number.

So it does what this project's first principle says: *a strict sign change brackets the boundary*.
Each grid point is classified by strict sign (material being the closure of `{f < 0}`, so `Near`
counts as material), and **every crossing is placed by bisection**. Marching **tetrahedra** — six
fanned about the 0–6 diagonal, so neighbours cut a shared face the same way and the surface has no
cracks — which is three cases rather than a 256-entry table that can be silently wrong. The field
also locates itself first on a grid eight times coarser, since `support_bounds` is conservative
(~20 × 34 × 32 where the turned box is ~2 × 3 × 2.5, which is 22 million points at h = 0.1).

**The control, at h = 0.1:** the turned box comes out **15.4157** against the closed form
**15.3237** — **+0.60%**, which is what marching tets with bisected crossings should give, and it
is the only thing that licenses reading the next number.

**And the next number is the alarm that was missing all along.** The tumbling cylinder's reference
volume is **9.4786**, against the **6.6563** the construction reports: a shortfall of about
**30%** — roughly 2.8 units of material simply absent, and ≈29% after allowing for the
instrument's own +0.6%. Two honest caveats: 6.6563 is a divergence-theorem volume of an *open*
mesh, so not strictly a volume; and the located box is still over-generous, which costs time rather
than correctness, since the contour is right to 0.6% and no spurious material was contoured.
Bisection dominates the cost — 79,912 edges in 100 s for the turned box, 52,672 in 27 s for the
tumbling cylinder.

The artefacts sit beside the constructed exports as
`rust/examples/swept_boundary/{turned_box,tumbling_cylinder}_reference.stl`. **This is the check to
run first on any case that refuses**, and had it existed on day one it would have said "a third of
the volume is missing" in one pass, instead of a dozen hypotheses about the stitch that was never
at fault.

**What the absence actually looks like.** Rendering the reference as a red cage *over* the
constructed mesh — the two STLs in one three.js page, the reference `wireframe`, the construction
solid — shows red with nothing behind it exactly where surface is missing. Down the tumble axis the
construction is present as **four lobes**, and the red-only region between them is a clear **X, a
bowtie through the centre**, the centre being the axis. From the side the same absence reads as a
**band across the middle**, the construction appearing only as an upper and a lower cap. Down z it
covers nearly the whole silhouette — which is why no single viewpoint would have revealed this, and
why the four-view render matters.

So the missing thirty per cent is not a vague equatorial band: it is **four bowtie sectors meeting
on the axis at the fixed points (2,0,0) and (4,0,0)**, with the construction holding the four lobes
between them. That is precisely what 5c names, and it is now visible rather than inferred — which
makes it the specification for what has to be generated.

### What the literature calls this, and the assumption we inherited (2026-09-12)

The degeneracy has a name and a general treatment, and the paper this construction follows
**excludes it by assumption**. Worth recording before anything is built, because it settles whether
a fix here would be a special case or a stratum of an existing taxonomy.

**Rossignac, Kim, Song, Suh & Joung, *Boundary of the volume swept by a free-form solid in screw
motion* (CAD 39(9), 2007)** — the source of the "generate, split, and test" paradigm and of the cap
rule implemented here — defines the generator as the grazing and silhouette points of ∂W, then
notes: *"In general, it is composed of loops of curves that separate the egress from the ingress
points. **Occasionally, it may contain two-dimensional regions or isolated points with tangential
velocity.**"* Ours is the second kind. And for planar faces: *"**When n×s = 0, no characteristic
curve exists on F.** Hence, we will now assume that n is not parallel to s."* At (2,0,0) the wall's
normal is −x̂ and the screw direction s is x̂, so **n×s = 0 exactly** — the framework assumes our
case away. A contact-curve tracer therefore finds a degenerate fan there rather than a surface, and
that is an inherited limitation, not a local defect. Their robustness contribution is worth noting
separately: cells are classified not by reachability from infinity (which fails when the swept
boundary has several shells — their example is a torus swept by a ball) but by **helix-shooting** a
witness point back along the motion.

**Abdel-Malek & Yeh's Jacobian rank deficiency / manifold stratification** (surveyed in
Abdel-Malek, Blackmore & Joy, *Swept Volumes: Foundations, Perspectives, and Applications*) is the
general frame. That survey's opening example is this problem in miniature: enumerating a sweep's
boundary curves leaves *"some curves that must be on the boundary still missing"*, and the missing
one is found by locating the **stationary point** (∇ξ = 0, equivalently det J = 0). The boundary is
then assembled by **successive stratification** — smooth grazing curves are the top-dimensional
stratum, a pinch a lower-dimensional one. So a bowtie is a *stratum*, which means a principled
treatment need not be a special case bolted on.

**Blackmore & Leu's SDE/SEDE** computes grazing points once at the initial pose and flows them by
the envelope ODE — an order of magnitude cheaper, and extended to piecewise-smooth solids — but it
presumes that flow is well defined, which is exactly what fails where the velocity vanishes.

**Adsul, Machchhar & Sohoni, *Incorporating Sharp Features in the General Solid Sweep Framework*
(arXiv:1405.7457)** addresses our case head-on, for G0 solids, through the **cone of normals** N_x
at a sharp point. Proposition 9 generalises the contact condition to non-smooth boundaries, and
§4.1 states the dichotomy exactly: at a sharp edge *"either there exists a **unique** n ∈ N_x such
that g(x,n,t) = 0 **or for all n ∈ N_x, g(x,n,t) = 0**"* — the second being our degeneracy, since a
vanishing velocity satisfies it for every normal. They show it *"leads to a singularity on C^E"*,
characterised by **Lemma 15**: a face of the swept edge is singular exactly where the velocity is
**tangent to the edge**. Each sharp-edge sweep is parametrised by a **funnel**, and the singularity
appears as that funnel pinching; their Fig. 11 draws the admissible-velocity region as a literal
bowtie. Implemented over ACIS and tested on more than fifty solids.

**A gap in our own notes, for the record.** `docs/implicit-meshing-methods.md` assesses six
contouring and meshing methods in detail but contains no mention of degeneracy, singular sets,
vanishing velocity or the grazing set. The contouring question was evaluated thoroughly and the
sweep-degeneracy question never written down, which is why it surfaced only after a long detour
through the stitch.

### The bowtie was the wrong target: the gap is the z = 0 band (2026-09-12)

Measured against the reference surface rather than inferred from a render, the missing surface is
**not** the bowtie sectors. Of 23.493 of reference area, **6.445 — 27.4%** lies farther than 0.15
from any constructed vertex, which matches the ~30% volume shortfall and confirms it really is
absent surface. But its distribution refutes the bowtie reading:

- **By x it is spread across the whole range**, and peaks in the *middle*: 2.00 → 1.04, 2.50 → 1.27,
  **3.00 → 1.81**, 3.50 → 1.27, 4.00 → 1.05. A bowtie at the pierce points would concentrate at
  x = 2 and x = 4; this does the opposite.
- **By angle about the tumble axis it is two wedges at ≈ 0° and ≈ 180°** (1.04 and 1.06 in the
  −30..0 and 0..30 bins, 1.03 and 1.06 at ±150..180), falling to 0.08 at ±90°. Since the angle is
  `atan2(z, y)`, angle ≈ 0 or 180 means **z ≈ 0**.
- Radius from the axis runs 0.517 to 1.412 — the second being √2, the cap rim's farthest distance
  from the axis.

That is the **third piece of the grazing set** derived above: on the wall `n·v = −yz`, so
`{yz = 0}` is the two lines `y = 0` at x = 2 and 4 (which sweep the bowties, and which sheets 6 and
7 do cover) *plus the circle* `z = 0`. It is the circle's sweep that is missing.

**And the sheets stop dead on the wedge boundary.** Their angular extents are sheet 1 at
15.0°..120.0°, sheet 2 at 67.5°..165.0°, sheet 3 at −120.0°..−15.0°, sheet 4 at −165.0°..−67.5° —
union −165°..−15° and 15°..165°, i.e. **exactly the complement of the uncovered wedges**. Sheet 6
is confirmed as the bowtie (x 2.00..2.02, spanning the angles) and sheet 7 its mirror at x ≈ 4.

Terminating that precisely on ±15° looks more like **clipping than absence**, which changes what
should be built: if the wedge is traced and then discarded, a new generator would fix nothing and
mask a trim defect that would also be corrupting other cases.
`reference::whether_the_missing_wedge_is_ever_traced` asks `seeds()` directly, before the pipeline
touches anything — points in the wedge mean trim, no points mean generation. **Do not build until
that reads.**

**It reads: the band is traced, and the pipeline destroys it.** Sheet 0 has **78 of its 153 points
in the wedge**, spanning angle −172.5°..180.0° and x 2.00..4.00 — the whole missing region. Every
other sheet has **zero**. And sheet 0's *finished* state, measured earlier, is **8 triangles in a
box `[3.942,−0.39,−0.018]..[3.995,0.036,0.013]`**: a scrap 0.05 across beside the pierce point
(4,0,0), one of whose triangles even reads reversed.

So sheet 0 is the `z = 0` grazing circle's sweep, it is produced correctly across the entire gap,
and somewhere between `seeds` and the finished mesh it goes from **153 points over x 2..4 and every
angle to 8 triangles in a 0.05 box**.

That looked like a pure trim defect, and it was written up as one. **The per-stage measurement
refuted that too** (`creases::which_pass_destroys_the_wedge`, sheet-0 triangles and — immune to
renumbering — triangles of any sheet whose centroid is in the wedge):

```
     clipped:  2089 tri; sheet 0  85 (x 2.002..4.000, angle -179.7..179.5); wedge  42  (baseline)
      merged:  2066       sheet 0  85                                       wedge  42  +0
        kept:  2066       sheet 0  85                                       wedge  42  +0
     unioned:  3198       sheet 0  84                                       wedge  40  -2
   uncovered:  2973       sheet 0  84                                       wedge  41  +1
      welded:  2353       sheet 0  37                                       wedge  20  -21
       split:  2923       sheet 0  37                                       wedge  20  +0
      zipped:  3288       sheet 0   8 (x 3.977..3.995)                      wedge  12  -8
```

**Then area settled it, and the count story was an artefact.** A triangle count was measuring
*slivers*. Adding area to the same walk gives sheet 0 **0.0102** at clipped and the whole wedge
**0.0088**, against the reference's **6.445** uncovered — short by a factor of about 700. The two
passes that looked guilty move that area by **0.002** in total (sheet 0: 0.0102 → 0.0080 at
welded → 0.0081 at zipped). Nothing is being destroyed in any quantity that matters, because there
was never any surface there to destroy.

An independent cross-check confirms it. The construction's **total** area is 17.5415 at clipped and
17.2205 at zipped, against the reference's 23.493 — short by 6.27, which is the uncovered 6.445 to
within the reference's own tolerance. The area is absent from the first stage onward, and the wedge
holds none of it at any stage.

**That "generation gap" was wrong too, and the fourth measurement settles what is actually
happening.** `which_pass_destroys_the_wedge` began at `Clipped`, so the two stages before it were
never looked at. Asked of the tracer's own patch and of `label_seeds`
(`creases::where_sheet_zeros_area_goes`):

```
sheet 0 raw from the tracer: 153 points, 272 triangles, area 4.1091
raw triangle areas: least 0.000e0, median 1.773e-2, greatest 2.551e-2
sheet 0 labels over 153 vertices: {"Inner": 122, "Kept": 27, "Moved": 4}
at the judged positions: all 272 triangles area 4.1222; the 20 all-Kept/Moved 0.0014
the judge moved sheet 0's vertices by at most 0.0200
```

**The tracer generates the band, at the right size.** 4.1091 against an independent closed form of
4 × π/3 = 4.1888 — the equator circle (x−3)² + y² = 1 turned about x̂ through ±30° sweeps
∮|y| ds × Δθ — agreeing to 2%. The triangles are not slivers either (median 1.8e-2). It is
`label_seeds` that empties it: **122 of 153 vertices are `Inner`**, and the all-`Kept`/`Moved` rule
keeps **0.0014 of 4.11**. Sheet 0 is then a curve's worth of slivers at `Clipped` because that is
all labelling left of it — not because the tracer made slivers.

**And the judge is right to do it.** The equator point (3,1,0) maps under the turn to
(3, cos θ, sin θ), which satisfies (x−3)² + y² = cos²θ ≤ 1 and |z| = |sin θ| ≤ 1 — so every image
of it lies *inside* the tool at θ = 0. The equator's sweep is genuinely **interior**, and only its
points near y = 0 (meeting the wall at x = 2 and x = 4, on the tumble axis itself) stay on the
boundary — which is exactly the 27 + 4 that are kept. So a generator is refuted for a fourth
reason: the surface is already generated, and it *should* be discarded. `welded`, `zipped`,
`clip_sheets`, `merge_creases`, `centroid_kept` and the coverage clip remain exonerated, and so now
does `label_seeds`.

**Which relocates the gap, with a prediction to check.** In the plane z = 0 the swept region is
∪_θ {(x−3)² + y² cos²θ ≤ 1}, which is largest at the roll's **ends** |θ| = 30°:
(x−3)² + 0.75 y² ≤ 1, reaching **|y| = 1/cos 30° = 1.1547** at x = 3 — outside the tool's own
equator radius of 1. So the true boundary near z = 0 is attained at the roll's *endpoints*, by no
interior contact at all: it is **cap-like** surface.

**The prediction holds.** Probed along y at x = 3, z = 0 with no pipeline involved
(`creases::whether_the_z_zero_band_is_cap_surface`), the field reads material at y = 1.15 and
exterior at y = 1.20 — the boundary is at 1.1547, not at the tool's equator radius of 1. So the
surface missing near z = 0 is end-pose surface, and the closed form is confirmed by the field
itself.

The same run asks which seed carries anything there:

```
  seed  0 traced: raw   4.1091, in the wedge  2.0546 (136 tris), surviving labelling  0.0013 (12 tris)
  seed  1 traced: raw   2.9780, in the wedge  0.0000 (0 tris)
  … seeds 2-8 traced: 0 in the wedge …
  seed  9    cap: raw   9.4172, in the wedge  0.0000 (0 tris)
  seed 10    cap: raw   9.4172, in the wedge  0.0000 (0 tris)
== the band over all seeds: raw 2.0546, surviving labelling 0.0013
```

Only sheet 0 reaches the band, and labelling empties it correctly. **The caps keep 9.4172 of area
each and have nothing there at all — zero *raw*, not emptied.**

**That last number is anomalous, and it is being treated as an open question rather than a
conclusion.** 9.4172 is exactly half the wall (2π·1·2 / 2 = 6.283) plus half the two end discs
(6.283 / 2 = 3.14), so a cap does contain wall. And the boundary point (3, 1.1547, 0) is precisely
the θ = 30° image of the tool-frame wall point (3, 1, −0.577) — whose n·v = −yz = +0.577 > 0 is
advancing, which `End::To` is meant to **keep**. So that facet should be present and in the band,
and it is not. Three possibilities, to be separated by measurement and not by argument: the cap
genuinely has no surface near z = 0 (the derivation is wrong somewhere), it has some at angles the
wedge predicate misses (the predicate is wrong), or it has some and the component keep rule drops
it (the defect). `creases::where_the_caps_surface_lies` histograms world |z| and angle over each
seed's own centroids, with no wedge predicate in the histogram, to tell them apart.

The candidate mechanism, if it proves to be the third: `caps` keeps a component when
`e.abs() < decisive || e.signum() == wanted`, where `e` is the **most decisive** normal velocity
over the whole connected component, and components are joined across every edge that is not a cut.
Cuts run only along the sheets' end columns. A band whose own sign is right but which is not
separated by a cut therefore inherits a neighbour's decisive sign and is dropped wholesale.

The histogram narrows it to two of the three. Per seed, over each patch's own centroids in world
coordinates, with no wedge predicate involved:

```
  seed  0 traced:  272 tris, z -0.449.. 0.449, 124 with |z|<0.1; bins [63, 1, 0, 0, 0,71,72, 1, 0, 0, 0,64]
  seed  1 traced:  128 tris, z  0.437.. 1.334,   0 with |z|<0.1; bins [ 0, 0, 0, 0, 0, 0,13,53,51,11, 0, 0]
  seed  6 traced:   80 tris, z -0.830.. 0.830,   8 with |z|<0.1; bins [ 0, 0,16,16, 0, 0, 0, 0,23,24, 1, 0]
  seed  9    cap: 2161 tris, z -1.362.. 1.362,   6 with |z|<0.1; bins [ 0, 0,335,453,293,0,0,0,333,455,292,0]
  seed 10    cap: 2161 tris, z -1.362.. 1.362,   6 with |z|<0.1; bins [ 0,292,455,331,0,0,0,293,454,336,0,0]
```

**Both caps avoid the band exactly**: the bins covering [−30°, 30°) are zero for both, and only 6 of
2161 triangles have |z| < 0.1. Sheet 0 is the only seed that reaches it (124 of 272 with |z| < 0.1).
The z span of ±1.362 also confirms the cap patches are posed into world, since max |z| at θ = 30° is
0.5·1 + 0.866·1 = 1.366 — so this is not a frame confusion, and the wedge predicate is not at fault
either, the hole being visible in raw 30° bins. That leaves "never there" against "dropped", which
`Cap::patch` cannot settle because it holds the kept facets only.
`creases::what_each_cap_kept_and_dropped` reads `Cap::components` — facets, extreme, kept, per
component — which settles it, and pins which cap is `From` and which `To`; every sign argument above
has *guessed* that pairing, and it is being measured rather than assumed a third time.

It reads:

```
  seed  9: end From, 8 components, 2161 facets kept, 2199 dropped
          862 facets, extreme   0.704322, DROPPED      843 facets, extreme  -0.704290, kept
          853 facets, extreme   0.704322, DROPPED      840 facets, extreme  -0.704322, kept
          243 facets, extreme   0.673057, DROPPED      241 facets, extreme  -0.673057, kept
          241 facets, extreme   0.673057, DROPPED      237 facets, extreme  -0.672859, kept
  seed 10: end To, 8 components, 2161 facets kept, 2199 dropped   (the same, signs reversed)
```

Eight components per cap — four large (~850 facets: the wall's quadrants) and four small (~240: the
discs' pieces) — every extreme a clean ±0.704322 or ±0.673057. So **the swallowing mechanism above
is refuted**: no component is mis-signed, none straddles by accident, and `e.abs() < decisive` never
fires, nothing being within 0.25 of zero. `From` keeps the four negative, `To` the four positive.

**And the defect is one step deeper, now proved.** It is a *design* limit rather than a bug: a cap
is reduced to whole connected components by **one sign each**, so a component whose interior contains
the divider `n·v = 0` cannot be half-kept. Here the band at world z = 0 lies along the curve
z_t = −y_t tan θ, which runs through a quadrant's **interior**, while the cuts run only along the
quadrant *boundaries* — the sheets' end columns, z_t = 0 and y_t = 0. So the quadrant is kept or
dropped whole and the band is lost either way, which is exactly the clean 60° hole centred on
angle 0.

`creases::whether_a_cap_facets_own_sign_disagrees_with_its_components` rebuilds the `To` cap
identically to `caps` — every constant read from the library, none guessed (sagitta 0.02,
`snap()` = `vertex_tolerance()` = 0.005, `longest` = `spacing` 0.5, `decisive` 0.25, `wanted` +1,
no grazing planes) — and asks the falsifiable question:

```
the To cap rebuilt as `caps` builds it: 4360 facets, 8 components, 483 cuts
370 facets with |z| < 0.1; 364 of them advance (own sign > 0) inside a DROPPED component
the clearest: own +0.555806, its component's extreme -0.704322, at [3.094,1.195,-0.080]
```

**364 of 370.** That facet's world radius is 1.198, right at the 1.1547 boundary the field
confirmed, at z ≈ 0. So the band is genuinely lost inside a component whose own sign is wrong for
it.

**The fix is therefore to cut each cap along its own `n·v = 0` divider at the end pose**, not only
along the sheets' end columns — which is the literature's rule (each cap cut by its own contact
condition) that the 5b plan recorded and never applied to this case. `CutMesh::cut_along` is the
seam and takes it: `insert` places an interior point outright (`Place::Face(t) => split_face`) and
pins it. The divider must be marched **per tool face**, since `n·v` is per-surface and a summed
normal at a crease says nothing about it; `CutMesh::faces` already carries the tool face of every
facet, split pieces included.

This is the fifth diagnosis of this band. The four before it — a bowtie generator, a trim defect at
`welded`/`zipped`, a generation gap wanting a funnel, and component swallowing — were each refuted
by the next measurement, every time because the conclusion rested on a quantity that merely looked
like the right one. Nothing is written until the baseline is captured, and the five export STLs and
a green suite are taken *before* `caps` is touched, since a change there touches every case.

**The baseline** (suite 1209 passed, 0 failed, 53 ignored; STLs checksummed in the scratchpad):
turning_prism 1436 triangles closed volume 11.1110; turned_lens 2786 closed 12.8368;
tumbling_cylinder 3288 open, `UnpairedRim { loops: [42,44,7,5,4,4,6] }`, 6.6540; turned_box 1304
closed 15.3167; sliding_dumbbell 35370 open, `UnpairedRim { loops: [4,4] }`, 9.5277.

**Which cases a divider cut would move, predicted before writing it**
(`creases::which_cases_a_divider_cut_would_move`, counting components that hold facets decisively
signed *both* ways, since one sign must then be dropped with the other):

```
       turning_prism From/To: 4 components, 0 straddling; 0 lost, area 0.0000
         turned_lens From/To: 2 components, 0 straddling; 0 lost, area 0.0000
   tumbling_cylinder From/To: 8 components, 4 straddling; 853 facets lost, area 3.6463
          turned_box   From: 1 component,  1 straddling; 0 lost, area 0.0000
          turned_box     To: 1 component,  1 straddling; 112 facets lost, area 7.0000
    sliding_dumbbell From/To: 2 components, 0 straddling; 0 lost, area 0.0000
```

The tumbling cylinder's row is trustworthy — its 8 components match `caps`' own 8 exactly, this tool
having no grazing face — and 3.6463 at each end is the right order for the reference's missing 6.445.

**The turned box's row is an artefact of the instrument, not a finding, and is recorded as such.**
`caps` marks facets lying on a grazing face `left[i]`, **skips** them, and adds the left/non-left
boundary edges as cuts. The instrument passed **no grazing planes** and never computed `left`, so for
the turned box — which does have grazing faces, the x = 2 and x = 4 planes that `swept_region`
handles — it built one uncut component where `caps` builds a cut one, giving the spurious "1
component, 7.0000 lost". That case closes and certifies today, so the omission must not be read as a
latent defect in it; the instrument needs the grazing treatment before its turned-box number means
anything, and the honest number is wanted *before* `caps` changes, a regression there landing on a
working case.

One feasibility question is settled: `locate` rejects a point only beyond `2·sagitta` (0.04), while a
divider marched on the analytic surface lies within one sagitta (0.02) of the facet plane — so
`cut_along` can take it. And the marching structure to reuse is `tool_faces::creases`: a
marching-squares zero-set tracer over a face's own parameter grid (96×384 on a revolved face), every
crossing bisected 50 times, saddles disambiguated by the cell centre's sign, segments chained by
shared grid edges. It is hard-wired to `b.implicit(...)` as its scalar, so the divider pass is the
same walk on the sign of `n·v` instead. `StationEquation::roots` is *not* the tool for it: it solves
`n·v = 0` for `v` at a fixed `u` station, and this divider crosses every station once, so stations
would return it as 96 isolated points rather than a curve.

**With the grazing treatment added, the prediction is clean — and the turned box was never
defective.** Passing the planes `construct_from` passes, computing `left`, inserting the left/non-left
boundary edges as cuts and excluding `left` from the counts turns the turned box's spurious row into
`10 components, 0 straddling, 192/198 grazing facets, 0 lost`:

```
       turning_prism From/To:  4 components, 0 straddling,   0 grazing; 0 lost,   area 0.0000
         turned_lens From/To:  2 components, 0 straddling,   0 grazing; 0 lost,   area 0.0000
   tumbling_cylinder From/To:  8 components, 4 straddling,   0 grazing; 853 lost, area 3.6463
          turned_box   From:  10 components, 0 straddling, 192 grazing; 0 lost,   area 0.0000
          turned_box     To:  10 components, 0 straddling, 198 grazing; 0 lost,   area 0.0000
    sliding_dumbbell From/To:  2 components, 0 straddling,   0 grazing; 0 lost,   area 0.0000
```

So the divider cut must move **the tumbling cylinder alone**, and the other four STLs must stay byte
for byte identical against the captured checksums. That is the regression gate, fixed in advance.

A useful sign question dissolves on inspection rather than needing a measurement: the zero set of
`n·v` is the same for `n` as for `−n`, so the outward-normal convention — `outward_sign`,
`face_outward` — cannot affect where a *cut* falls, only which side is later kept. The divider is
therefore orientation-free, which removes a whole class of error from the marcher.

**The divider cut is refuted as the fix — it would have been a no-op — and the real cause is
simpler.** Asked which branches are actually cut
(`creases::why_the_caps_cuts_fail_to_separate_the_sign_regions`), all nine end columns cut in and
**every branch is already covered**: sheet 0 is the equator (closed, tool-frame z ≈ 0,
`max|y·z|` = 0.0000), sheets 6–7 the wall's `y = 0` lines, sheets 5 and 8 the discs' own `y = 0`
dividers, sheets 1–4 the rim arcs. The divider here is `{z = 0} ∪ {sin φ = 0}`, since
`n·v = −z sin φ` on the wall, and it is *pose-independent*, the turn about x̂ leaving x̂ fixed in the
tool's frame. (`z_t = −y_t tan θ`, written above, is where the *missing band* lies — the preimage of
the world plane z = 0 — not the divider. Two different curves.)

What the per-component histogram shows instead:

```
     860 facets [0, 0, 860, 0], relative -0.7043..+0.6757  STRADDLES
     855 facets [0, 855, 0, 0], relative -0.7043..+0.6759  STRADDLES
     842 facets [842, 0, 0, 0], relative -0.6759..+0.7043  STRADDLES
     841 facets [0, 0, 0, 841], relative -0.6759..+0.7043  STRADDLES
     243 facets [0, 243, 0, 0], relative -0.6731..-0.0082
     241 facets [241, 0, 0, 0], relative +0.0082..+0.6731
```

Each big component lies in exactly **one** tool-frame quadrant, yet carries **both** signs. That is
impossible for one surface: on the wall `n·v/|v| = −z sin φ / √(z² + sin²φ)` has a fixed sign once
the signs of `z` and `y` are fixed. In the (−z, +y) quadrant the wall gives `−z·y > 0`, while the
**bottom disc** — normal −ẑ, so `n·v = −y` — gives `−y/√(1+y²)`, reaching −0.7071 at y = 1, which is
the −0.7043 measured.

**So the components span the tool's own sharp rim**, where the normal jumps and `n·v` changes sign
discontinuously, and a single extreme cannot speak for both sides.

**The fix is the rule `caps` already applies to grazing faces, applied to every tool edge** — its
own comment states the principle: *"a component must not reach through them to be judged by a normal
velocity that is not its own."* Adding every edge between facets of different `CutMesh::faces` as a
cut should give 4 wall quadrants plus 2 halves per disc = 8 uniformly-signed components, each
correctly kept or dropped. That is a few lines beside the pattern already there — **no divider
marcher, no `dividers` function, and no refactor of `creases`**, all of which this measurement has
just made unnecessary.

The A/B is in the same instrument: the tally runs once with only the contact columns cut, then again
with the face boundaries added. **It licenses the change** — `0 components straddle; 0 facets would
be dropped against their own sign`, and every component comes back single-face (`faces [0]`,
`faces [1]`, `faces [2]`) and uniformly signed, the wall in quadrants and each disc in halves.

One detail of the "after" tally is worth keeping, because it is the one way another case could still
move. Thirty-two components of exactly **12 facets** appear on the two disc faces, with narrow
monotone `relative` ranges. They are circular segments sliced off each disc by `cut_along`'s
straight **chords** between consecutive rim points — eight chords per half-rim, four half-rims.
They were always being cut; they merely stayed merged into the big components through the uncut rim,
and cutting the rim isolates them. Here each is uniformly signed and kept correctly, the smallest
maximum being 0.3049 against `decisive` = 0.25. But a sliver whose `|extreme|` fell *below* 0.25
would be kept as "grazing" whatever its sign, adding surface where a decisively-signed dropped
component used to swallow it. That is the failure mode to look for if a case other than the tumbling
cylinder moves.

**The change itself** is in `caps`, immediately after the contact-curve cuts and before the grazing
block so that both cut sets are in place before `components()`: every edge whose two facets have
different `CutMesh::faces` becomes a cut. Five lines, reusing the pattern beside it.

### What the two changes came to (2026-09-12)

**Landed, suite green at 1209 passed / 0 failed.** Two library changes, each measured before and
after, with the five-case export as the gate:

1. **`caps` cuts every tool-face boundary** (above). The band comes back: tumbling cylinder volume
   **6.6540 → 9.2929** against the reference's 9.4786 — from 30% short to 2.0% — and the reference's
   uncovered area **6.445 (27.4%) → 1.979 (8.4%)**, with the two peaks at angle 0° and 180° becoming
   its smallest bins. Predicted in advance from the straddling measurement and confirmed: prism,
   lens, turned box and dumbbell **byte-identical**, tumbling cylinder alone moved.
2. **`rim_zip` splits once more after its rounds.** The cap fix exposed six three-vertex loops —
   zero-area *gaps* between three triangles (no facet on their corners at all) whose vertices are
   collinear with one projecting inside the opposite edge, at +0.3333, +0.6603, +0.2731, +0.3408 and
   +0.4342, off the line by 1e-9 to 1e-16. Two of them the field called holes, so the property gate
   failed. They are T-junctions, and `split_at_vertices` ran only once, *before* the 32 rounds, so
   it never saw junctions the zip's own laying makes. Splitting once after the rounds takes them:
   **21 loops → 17**, 6661 → 6665 triangles, volume unchanged at 9.2929, and **no loop the field
   calls a hole**.

**One placement was tried, measured and reverted**, and is recorded so it is not tried again: the
same split *inside* the round loop. It interleaves with the zip and changes what the zip then
pairs — loops 21 → **23**, triangles 6661 → 6701, volume 9.2929 → **9.2581**, and still one loop the
field called a hole. Strictly worse than running it once at the end. The finished-mesh experiment
that suggested the fix was evidence, not proof, and the gap between the two placements is exactly
the caveat it carried.

**What still refuses, and it is no longer this band.** The tumbling cylinder is open with 17 loops
and `UnpairedRim`; every one passes `simple()`, the degeneracy check and the already-walked-edge
check, so each reaches the field pass and the *field* declines it — the honest `Open` category. The
remaining 8.4% uncovered is spread fairly evenly and peaks at x = 3.00 (0.98), which is a different
problem from the `z = 0` band and should be measured before it is guessed at.

**Five diagnoses of this band were refuted before the sixth held**, each by the next measurement: a
bowtie generator; a trim defect at `welded`/`zipped`; a generation gap wanting a funnel; component
swallowing in `caps`; and a divider cut that would have been a no-op, all three divider branches
already being cut. Plus one instrument flaw of mine that invented a defect in the turned box by
omitting the grazing treatment. The recurring cause was trusting a quantity that merely resembled
the one the conclusion rested on — most sharply, a **triangle count of slivers** where the question
was **area**.

### Facet shape: the tumbling cylinder is built of slivers, and that is peculiar to it

The finished mesh looks rough in a viewer, and `creases::what_makes_the_finished_mesh_rough`
measures why rather than guessing. It reports, per case and split by where each triangle came from
(a traced sheet, a cap, a grazing region, or a band the zip laid), the distribution of longest edge,
area and least altitude, and how many triangles fall under the certificate's own least probe (0.01).
A well-shaped triangle's altitude is a good fraction of its longest edge, so the **median
edge : altitude ratio** is the number to read:

| case | closes? | cap | sheet | zip | region |
| --- | --- | --- | --- | --- | --- |
| turning_prism | certifies | 2.3 (210) | 5.2 (1212) | 2.4 (14) | — |
| turned_lens | certifies | 2.0 (2042) | 2.0 (744) | — | — |
| turned_box | certifies | 2.2 (376) | 4.5 (256) | 19 (24) | 7.2 (648) |
| **tumbling_cylinder** | **open** | **11.3** (4189) | **8.5** (1371) | **8.6** (1105) | — |
| **sliding_dumbbell** | **open** | 3.8 (33934) | **11.4** (1322) | 7.1 (114) | — |

Triangle counts in brackets, since a ratio over two dozen triangles says little: the turned box's
zip is 19 but holds 24 triangles and *no* sliver by the probe, while its 648-triangle region at 7.2
is the substantial one.

**This is not a quality ceiling of the pipeline** — the three cases that certify keep every
substantial source at ratio 5.2 or better, two of them at 2. And a sharper pattern shows once all
five are in: **both cases that fail to close carry a major source at ratio 11 or worse**, at 23% to
63% slivers, while no closing case does. On the tumbling cylinder roughly 2000 of 6665 triangles are
thinner than the certificate's sliver threshold and over half are thinner than the sagitta; long
thin triangles shade badly and round badly into float32, which is the whole of the visible
roughness.

Five cases is a correlation and not a proof, and it suggested a hypothesis that would have inverted
the obvious ranking: that facet shape is *upstream* of the closure failure rather than cosmetic,
every stitch failure chased in this work having been a sliver of some kind.

**Measured within the case, that is refuted** (`creases::whether_the_open_loops_sit_in_slivery_neighbourhoods`).
The control is what makes it mean anything: "the open loops are surrounded by slivers" is empty when
28% of the mesh is slivers, so the comparison is against the seams the zip **closed** — it laid 1105
band triangles where it succeeded and left 17 loops where it did not, and both are seam regions. The
zip's own bands are excluded from both sides, or the test is circular, so what is measured is the
surface the stitch was *given*, never the output it made:

```
the whole non-zip surface: 5560 triangles, 28.3% slivers, median edge:altitude 7.8
radius 0.05:  near the OPEN loops  330 tri, 41.8% slivers, ratio 9.2
              near the CLOSED seams 1639 tri, 44.7% slivers, ratio 7.8
radius 0.15:  near the OPEN loops  700 tri, 41.0% slivers, ratio 9.5
              near the CLOSED seams 3233 tri, 37.6% slivers, ratio 8.6
```

Seam regions are slivery in general — 38% to 45% against a 28.3% baseline — which is simply where
the cutting and zipping happen. But the failed seams are not meaningfully worse than the succeeded
ones, and the sliver fraction **flips direction between the two radii**: at 0.05 the closed seams are
slivery-er. Two radii were reported precisely so the answer could not hinge on one choice, and the
three-point gap at 0.15 does not survive that.

The per-loop spread settles it: loops 0 and 1 fail in neighbourhoods **cleaner than the mesh
average** (25.3% and 21.2% slivers at ratios 6.4 and 6.8, against 28.3% and 7.8). If sliveriness
caused the failure, no loop should fail in a below-average neighbourhood. And the selection bias runs
*against* the refutation, since the zip closes seams where closing is easy, so the closed-seam
control is selected for closability and should have flattered the open loops by comparison.

So **facet shape is not upstream of closure**, the ranking below stands as written, and the
cross-case correlation is most parsimoniously a **common cause**: a case with nine traced sheets,
fixed points and degenerate strata produces both more slivers and more closure failures, with
neither causing the other.

Two distinct mechanisms, which should not be conflated. The tumbling cylinder's cap carries **4189
triangles where the turned box's carries 376**: nine sheets' end columns criss-cross it and
`CutMesh::connect` splits every facet edge a chord crosses, so the cutting itself shreds it. The
sliding dumbbell has *both* — an over-refined cap (median edge 0.0264, twenty times finer than the
prism's, 63% slivers) and a shredded sheet at ratio 11.4.

**The tool-face cut added by this work is not the cause.** Those edges already exist in the mesh —
`indexed_faces` groups facets per tool face, so a rim is already an edge — and the change only
inserts them into `cuts`, calling no `split_edge`. It adds cut marks, not geometry.

### What is left on the tumbling cylinder, in dependency order

1. **Closure** — 17 unpaired loops, every one the honest `Open` category, so each reaches the field
   pass and the field declines it. Surface is genuinely absent there.
2. **A certificate, which has never run on this case at all.** `construct` refuses at `UnpairedRim`
   before `certify`, so there is no per-triangle verification that the surface is *correct* — only
   that its volume is within 2.0% and its area within 8.4%. This is the real open question, and it
   is gated behind (1).
3. **Coverage** — 1.979 of 23.493 reference area still absent, spread evenly and peaking at
   x = 3.00 (0.98 there), over radius 0.322–1.412.
4. **Facet shape** — the table above. The most visible, and **measured to be the least dangerous**:
   the loops the zip left open sit in neighbourhoods no slivery-er than the seams it closed, and two
   of them sit in neighbourhoods cleaner than the mesh average. It briefly looked as though this
   belonged above (1); it does not. Worth fixing for the look of the thing and for float32, not as a
   route to closure.

Three reversals on one question, worth recording as a lesson in what to measure. The bowtie framing
was refuted by *where* the gap is; "traced, then annihilated" was refuted by *how much area* is
there; and the intermediate "both partly right" reading came of trusting a **triangle count** where
the quantity in question was **area**. A count of slivers reads exactly like a count of surface.
**Ask for area whenever the question is coverage** — and note that the sliver count was not merely
uninformative but actively misleading, since it fell by half at the very passes a trim defect would
have implicated.
