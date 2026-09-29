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

Nothing about the boundary has to be discovered by tracing an arbitrary field. If the tool
clears the blank at both roll limits, the end caps lie outside the blank and need not be
built. A general tumbling sweep has none of these properties, which is where the
general project spent its effort.

## The class

A sweep `solid removal(tool, under: generating, from: a, to: b)` is **admitted** when all of
the following hold. Each condition is a predicate evaluated on the solved model, and each
failure is reported with its witness.

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| T1 | The tool is a full revolution with a line/arc/circle profile, or an intersection (`bound`) of such revolutions, under fixed placements. | Every face has closed-form contacts. An intersection creates only convex creases. | The first operand that is not a revolution, or a union/cut in the tool. |
| T2 | Every profile vertex inside the swept region is convex or tangent. | A convex corner sweeps a fan of normals. A concave corner sweeps nothing, but it trims two envelopes against each other, which is not supported. | The vertex and its turning angle. |
| M1 | The motion is one rotation about a fixed axis, or one rotation relative to another (`motion turn`, `motion relative`), at constant ratios, with a finite increasing roll. | This is the generating motion of a face-milled bevel or hypoid, including skew axes. | The motion node outside the class. |
| M2 | Each tool point's contact condition changes over the roll. | Where it does not, the point is on the boundary at every time or at none, and a contact time cannot parameterize the surface. A single rotation is like that everywhere (its tool-frame twist is constant), so the generator's motion is a relative one. This is the native path's existing refusal. It is checked at the samples, at a pole (whose normal is the axis), and between samples: the equation is C + a cos + b sin in the roll, and where (a, b) turns right round between two samples, or winds round a sample cell, with roots on every side, a point there has C and the amplitude both zero. That happens where the tool's normal passes through the point where two meeting axes cross. | A tool point whose path reaches the blank. |
| E1 | The tool at both roll limits has no common volume with the blank. | Then no end caps are needed inside the blank (the existing refusal). | The limit and the overlap volume. |
| E2 | Every source point whose contact reaches the blank has at most one contact time within the roll. | One sheet per placement, as the bevel pinion already satisfies. | The source point and both times. |
| E3 | The generated area factor J stays away from zero over the region reaching the blank. | A zero is a fold of the envelope: the cusp of undercut. J alone passes through infinity where a point's two contact times merge, which is a fold of the time chart and not of the surface. So the sign is read from J times the rate of the contact condition, which changes sign only at a true fold. | The contact point and the factor. |
| E4 | A placement's sheet does not cross itself inside the blank. | Sheets of different placements may cross, since the kernel split handles that. A self-crossing sheet is the defect the hypoid investigation found (`split_at_crossings`). | The pair of sheet points. |

Translations, screws and single rotations have constant tool-frame twist. Their contact set is
fixed on the tool, so they are easier than relative rotations, but by a different construction
(`solid::constant_twist`, the tracer's characteristics). They are left out only because the
gear does not need them. Admitting them later is a separate, small extension, not a new
project.

### Undercut is excluded deliberately

A pinion with few teeth often has some undercut. There, the envelope of the blade tip's
round trims the flank envelope. A real gear has this, and E3/E4 refuse it. The first
version refuses undercut and requires the design to avoid it. Supporting it means one
*declared* intersection between two known envelopes of the same sweep. `BoundarySeam` and
`EnvelopeSeam` already perform that operation. Supporting it is a later, separately gated
extension. An undeclared crossing stays refused.

The class is implemented as `solid::admission::admit_body`. `solventc --step/--stl` runs it
before building a body with swept cuts. Placements whose blank reads the same at every point
the checks read are checked once: the gear's 24 indexed cuts are one check.

## Refused, and how

Everything outside the class is refused at elaboration or export, never meshed on a
best-effort basis. The refusal names the condition row and the witness. The tumbling, slid
and tilted cylinders, the thin plate, the dumbbell and every sweep whose tool is a prism
are outside the class. Their fixtures stay in the tree as **negative controls**: each must
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
