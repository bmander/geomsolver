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

## Refusals

Every refusal names its element. So far: `ReversedNormal { point, direction }` (material
outward and exterior inward), `Field(reason)` (the evaluator's own error). Unresolved
vertices are labelled and counted, with their offset and enclosure, and will refuse the
certificate in milestone 2.

## Next

Milestone 2: the triangle certificate (`c ± d·n` with `d = 2·sagitta`) and the closure audit
on open sheets. Milestone 3: caps and welds, the closed-form cases. See the plan for the rest.
