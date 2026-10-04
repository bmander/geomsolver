# Generating sweeps: the supported class

**Decision (2026-09-22).** The goal is a declarative hypoid gear pair. The two prerequisite
projects set on 2026-09-10 were an arbitrary spacetime sweep and reliable Boolean meshes.
The first has turned out to be a research problem. Five milestones and Phase 3a–3d of
`solid/swept_boundary` close the closed-form cases. The tumbling, slid and tilted cylinders
and the thin plate still refuse. The ordinary cylinder still fails 324 centroid checks
through the production constructor. We therefore stop pursuing arbitrary sweeps. Solvent
supports one class of sweep: the class that generates bevel and hypoid gears. It refuses
everything else, naming the condition that failed.

This is a scope reduction, not a relabelling. A sweep outside the class is refused even when
some construction happens to produce a plausible mesh for it.
[The execution plan](generating-sweeps-plan.md) holds the phases and exit gates.

## Why this class is tractable

For a full revolution under a rigid motion, the contact condition on one circle of latitude
is `A cos θ + B sin θ + C = 0` (`RevolvedSurface::contacts`). It has at most two roots, and
they are found in closed form. The swept boundary is then a small, known set of pieces:

- one envelope per profile segment (line, arc or circle), and
- one sharp-edge sweep (a fan of normals) per convex crease.

A rack (a prism under a translation seen from a rotation, 2026-10-03, issue #61) is simpler
still: the contact condition of a point of the prism's side is affine in the roll,
`s n·d − n·(ω×(p−o)) − s t n·(ω×d) = 0` (`NormalVelocity::Affine`), one root in closed form.

Nothing about the boundary has to be discovered by tracing an arbitrary field. If the tool
clears the blank at both roll limits, the end caps lie outside the blank and need not be
built. A general tumbling sweep has none of these properties, which is where the
general project spent its effort.

## The class

A sweep `removal := solid(tool, under: generating, from: a, to: b)` is **admitted** when all of
the following hold. Each condition is a predicate evaluated on the solved model, and each
failure is reported with its witness.

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| T1 | The tool is a full revolution with a line/arc/circle profile, a prism of lines and arcs whose caps stay clear of the blank over the roll, or an intersection (`bound`) of such, under fixed placements. | Every face has closed-form contacts. An intersection creates only convex creases. A prism cuts with its sides only (`ExtrudedSurface`), so a cap in the blank would be a face the class does not build. | The first operand outside the class, a union/cut in the tool, or the prism whose cap passes through the blank. |
| T2 | Every profile vertex inside the swept region is convex or tangent. | A convex corner sweeps a fan of normals. A concave corner sweeps nothing, but it trims two envelopes against each other, which is not supported. | The vertex and its turning angle. |
| M1 | The motion is one rotation about a fixed axis, or one rotation (or one translation) relative to another rotation (`motion(about: …)`, `motion(along: …)`, `motion(m, relative_to: n)`), at constant ratios, with a finite increasing roll. | This is the generating motion of a face-milled bevel or hypoid, including skew axes, and of a rack cutting a spur gear. | The motion node outside the class. |
| M2 | Each tool point's contact condition changes over the roll. | Where it does not, the point is on the boundary at every time or at none, and a contact time cannot parameterize the surface. A single rotation is like that everywhere (its tool-frame twist is constant), so the generator's motion is a relative one. This is the native path's existing refusal. It is checked at the samples, at a pole (whose normal is the axis), and between samples: the equation is C + a cos + b sin in the roll, and where (a, b) turns right round between two samples, or winds round a sample cell, with roots on every side, a point there has C and the amplitude both zero. That happens where the tool's normal passes through the point where two meeting axes cross. | A tool point whose path reaches the blank. |
| E1 | The tool at both roll limits has no common volume with the blank. | Then no end caps are needed inside the blank (the existing refusal). | The limit and the overlap volume. |
| E2 | Every source point whose contact reaches the blank has at most one contact time within the roll. | One sheet per placement, as the bevel pinion already satisfies. | The source point and both times. |
| E3 | The generated area factor J stays away from zero over the region reaching the blank. | A zero is a fold of the envelope: the cusp of undercut. J alone passes through infinity where a point's two contact times merge, which is a fold of the time chart and not of the surface. So the sign is read from J times the rate of the contact condition, which changes sign only at a true fold. | The contact point and the factor. |
| E4 | A placement's sheet does not cross itself inside the blank. | Sheets of different placements may cross, since the kernel split handles that. A self-crossing sheet is the defect the hypoid investigation found (`split_at_crossings`). | The pair of sheet points. |

### Screws: the constant-twist class

A screw (`motion(about: axis, advance: lead)`) has constant tool-frame twist: every tool point
moves the same way relative to the tool at every time, so M2 fails everywhere and a contact time
cannot parameterize the surface. Instead the contact set is fixed on the tool. Its **characteristic**
is, on each ring of a revolved face, the roots of the same `A cos θ + B sin θ + C = 0`, read once
under the screw's twist, and the boundary is that curve carried along the screw:
S(s, t) = M(t)·c(s), its normal the tool's. `admit_body` dispatches on `Family::screw()`. A screw's
sweep is asked T1, T2, M1, E0 and E1 as above, then (`solid::constant_twist`):

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | The characteristic is regular through the reach and a margin past it: no ring is about to lose its two roots (a double point, where the curve turns back on the ring), none is stationary, and the walk never jumps at a profile corner. | The sheet is one regular curve carried along. A convex corner's fan is not built for a screw yet. | The face and the ring. |
| S2 | At most one characteristic point of a ring reaches the blank, and every point that does lies on one stretch. | One sheet per placement, one point per station. | The second point. |
| S3 | The characteristic never runs along the screw's velocity: `(c′ × v)·n` keeps one sign and stays above `least_factor`. | A zero is a fold of the sheet. Signed, so a fold passed between two samples is seen by the sign it turned. | The face and the factor. |
| S4 | The section square to the axis (each characteristic point carried along its path to one height) does not cross itself. | The sheet is the same at every height, turned and raised, so it meets itself exactly where its section does. | The two points. |

The walk samples `rows` rings along every face, then samples the stretch it found `rows` times
across its reach, so a reach that is a sliver of a large round is asked as finely as one that is a
whole face. Each point's path through the blank is read a degree of turn apart. These rows are
sampled too.

The sheet (`brep::sweep::helical`) is exact at every node and every withheld point: rows run along
the sheet's section square to the axis, `Characteristic::on_section`. The characteristic itself has
a corner wherever the tool's profile changes curvature, by a step along the screw's path that the
sheet does not show, while its section is as smooth as the sheet. Columns run along the roll over
the stretch that carries the section past the blank. It is fitted, judged and refined by the loop
the traced sheet shares (`sheet::settle`). The split, cells, agreement and files are the generating
class's. `rust/examples/twist_drill/` is the worked case (`tests/twist_drill.rs`).

Translations and single rotations on their own also have constant twist. They are still refused (M1/M2): their
section is not a screw's (a translation's is square to its direction, a rotation's a half-plane),
which is the one piece the construction would need.

### A rack (issue #61)

A prism's stations are planes square to its extrusion, a station a distance along it, and its
section is the profile itself (`brep::sweep::cutter`, `Kind::Extruded`): each edge a face, the
walk's left turns its convex fans. The reach samples stations along the prism; the band runs
from the first hit to the last, and may not grow past the prism's own length (`Band::limit`).
A sector's flat caps join across its sides (`brep::pattern`, `Kept::Flat`): a plane square to
the axis is kept by the turn, its pcurves turned about the axis's foot, and a ring of it is an
annulus with no seam. A blank reaching its axis is built whole (neighbouring sectors' sides
would meet in it), so `rust/examples/generation/rack_cut_gear.sv` has a bore.
`tests/rack_cut.rs` is the gate: one space's volume against arithmetic (an arc of each circle per
roll, unioned over the roll) to 1e-4, the whole 20-tooth gear patterned from one sector, field
agreement, and the toleranced build. The OCCT oracle (`--kernel occt`) sections revolutions only.

### Undercut is excluded deliberately

A pinion with few teeth often has some undercut. There, the envelope of the blade tip's
round trims the flank envelope. A real gear has this, and E3/E4 refuse it. The first
version refuses undercut and requires the design to avoid it. Supporting it means one
*declared* intersection between two known envelopes of the same sweep. `BoundarySeam` and
`EnvelopeSeam` already perform that operation. Supporting it is a later, separately gated
extension. An undeclared crossing stays refused.

The classes are implemented as `solid::admission::admit_body`. `solventc --step/--stl` runs it
before building a body with swept cuts. Placements whose blank reads the same at every point
the checks read are checked once: the gear's 24 indexed cuts are one check.

## Refused, and how

Everything outside the class is refused at elaboration or export, never meshed on a
best-effort basis. The refusal names the condition row and the witness. The tumbling, slid
and tilted cylinders, the thin plate, the dumbbell and every prism sweep but a rack's with its
caps clear are outside the class. Their fixtures stay in the tree as **negative controls**: each must
produce its refusal.

## Acceptance

An admitted sweep is accepted only by evidence that caught the earlier failure. The
traced-sheet mesh arrangement passed its recorded volume and closed-shell gates while
19–37% of the hypoid pinion's triangles disagreed with the material field. So:

- **Field agreement is the gate.** Probe points 0.1 mm inside and outside every face or
  triangle against `MaterialEvaluator` (`a_hypoid_pinion_space_agrees_with_its_field`), with
  zero disagreement allowed.
- Recorded tooth-space volumes (`native_surfaces/gear_cells.rs`) and encoded shell/STEP
  round-trip checks remain necessary but are not sufficient.
- The admission predicates are **sampled** in the first version and must say so.
  Certifying them over whole regions uses the interval machinery
  (`docs/interval-geometry.md`), which already covers the circular crown. That
  certification is scheduled, not assumed.

## What was removed

These tracks were closed and deleted on 2026-09-25; git history keeps them.

- `solid/swept_boundary`, the general certified sweep closure, and its Phase 0–3 records
  (`docs/swept-boundary*.md` and `docs/fixtures/`).
- `gcs-cli/src/cad/mesh_sweep.rs`, the traced-sheet Manifold arrangement, with
  `solid::sweep_candidates` (`--stl-backend manifold`). It failed the field gate on 18% of a
  hypoid space.
- The Ju et al. reference experiment (`experiments/ju-sweep-reference`).
- The CGAL Mesh_3 backend (`--stl-backend cgal`), superseded by the core's own refinement
  (`docs/field-meshing.md`).

## Open decisions

1. **Backend.** The native OCCT split-and-classify path (`cad/native/sweep_boundary.rs`)
   already exports the common-apex bevel pair from source alone. It agrees with the field on
   both sides of every sheet node and reproduces the recorded volumes. It is native-only and
   took 6–11 minutes per member (12–20 s since docs/native-speed-plan.md). The one-core WASM target needs a kernel-free path
   eventually. The plan builds on the OCCT path first and keeps the admission predicates
   in the core, so a later backend reuses them.
2. **Supported offsets.** Whether the hypoid offsets the design needs satisfy E2–E4 is
   an unmeasured question. The investigation found crossing envelope branches at 15° to 45°,
   but in the mesh path, which also had construction defects. Phase 1 of the plan measures
   it. If the needed offsets fall outside, widening the class (declared branch trimming) is
   a decision for the user, not a default.
3. **Undercut.** It is refused for now, as described above.
